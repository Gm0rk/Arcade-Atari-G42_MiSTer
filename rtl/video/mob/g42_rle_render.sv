//============================================================================
//  Atari G42 for MiSTer
//  g42_rle_render.sv -- RLE row fetcher and zoom renderer
//
//  Draws one object set up by g42_rle_scaler. Port of the row loop of MAME
//  draw_rle_zoom[_hflip]():
//
//      for each destination row (sourcey += dy):
//          walk to source row sourcey >> 16    (row_start += 1 + *row_start)
//          sourcex = dx / 2, rle_end = 0
//          for each packet byte of the row, low byte first:
//              rle_end += run << 16
//              while sourcex < rle_end:
//                  if value: pixel = pen base + value
//                  dest += 1 (or -= 1 when flipped), sourcex += dx
//
//  Rows are variable length (a count word, then that many packet words)
//  with no index, so reaching a source row means reading the count word of
//  every row before it. The ROM is in SDRAM behind a one-request-at-a-time
//  port with tens of clocks of latency, which makes the word stream the
//  bottleneck (up to ~33,000 words in a heavy Guardians frame), so the work
//  is split in two:
//
//    Fetcher  walks the rows and copies each source row a destination row
//             needs into one of two row banks (256 words each, one M10K for
//             both), keeping an SDRAM request pending all the time. Several
//             destination rows mapping to one source row (upscaling) share
//             one copy: the fetcher counts them and hands over the bank with
//             the count. It does not fetch the tail of a row that lies past
//             the far clip edge (src_limit), where MAME's clipped loop stops.
//    Emitter  replays a bank once per destination row. Each clock it either
//             writes one destination pixel (and, when that pixel ends the
//             run, also takes the next packet byte) or takes a packet byte
//             that covers no destination pixel. It starts at the first pixel
//             inside the clip window (sourcex0, dest0) and stops after the
//             far edge (bound).
//
//  While the emitter draws from one bank the fetcher fills the other, so
//  SDRAM reads overlap pixel output. Replays and emitter never touch SDRAM.
//
//  Pixels leave as {priority[2:0], index[8:0]}: pix_base holds the priority
//  and the pen base (low bpp bits zero, MAME's alignment), the packet value
//  fills the low bits.
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_rle_render
	import g42_pkg::*;
(
	input  wire         clk,
	input  wire         rst_n,

	// ---- Object (held from start until done) ----------------------------------
	input  wire         start,          // pulse
	input  wire [22:0]  data_ptr,       // word offset of source row 0's count word
	input  wire [2:0]   mode,           // encoding mode
	input  wire [11:0]  pix_base,       // {priority[2:0], pen base[8:0]}
	input  wire         hflip,
	input  wire [7:0]   y0,             // first destination row
	input  wire [8:0]   nrows,          // destination rows, >= 1
	input  wire [31:0]  sourcey0,       // 16.16 source row of y0
	input  wire [25:0]  dy,             // 16.16 source rows per destination row
	input  wire [25:0]  dx,             // 16.16 source pixels per destination pixel
	input  wire [27:0]  sourcex0,       // 16.16 source column of dest0
	input  wire [8:0]   dest0,          // first destination column drawn
	input  wire [8:0]   bound,          // last destination column drawn
	input  wire [11:0]  src_limit,      // last source column that can be drawn

	// ---- SDRAM read port (RLE object ROM) ---------------------------------
	output logic [24:0] rom_addr,
	output logic        rom_req,
	input  wire [15:0]  rom_dout,
	input  wire         rom_ack,

	// ---- Framebuffer write port (g42_rle_fb) --------------------------------
	output logic [8:0]  fb_x,
	output logic [7:0]  fb_y,
	output logic [11:0] fb_pixel,       // {priority[2:0], index[8:0]}, non-zero
	output logic        fb_we,

	output logic        done            // pulse
);

	// The RLE decode tables' package is imported here rather than in the
	// header: Quartus 17.0 accepts only one import in a module header.
	import g42_rle_pkg::*;

	function automatic [24:0] word_addr(input [22:0] p);
		word_addr = SDR_RLE + {1'b0, p, 1'b0};
	endfunction

	//========================================================================
	//  Row banks: bank b, word i at {b, i}. Fetcher writes, emitter reads.
	//========================================================================
	logic [15:0] bankmem [512];
	logic [8:0]  bank_waddr, bank_raddr;
	logic [15:0] bank_wdata, bank_q;
	logic        bank_we;

	always_ff @(posedge clk) begin
		if (bank_we) bankmem[bank_waddr] <= bank_wdata;
		bank_q <= bankmem[bank_raddr];
	end

	// Handover: bank_full[b] is set by the fetcher with its job (words in
	// the bank, destination rows that use it) and cleared by the emitter
	// after the last of those rows.
	logic [1:0] bank_full;
	logic [8:0] job_nw  [2];
	logic [4:0] job_rep [2];
	logic       f_push, e_free;
	logic       f_bank, e_bank;

	typedef enum logic [1:0] { E_IDLE, E_START, E_RUN } estate_t;
	estate_t    estate;
	wire        estate_idle = (estate == E_IDLE);

	//========================================================================
	//  Fetcher
	//========================================================================
	typedef enum logic [2:0] {
		F_IDLE, F_NEXT, F_SKIP, F_HDR, F_WORDS, F_REP, F_END
	} fstate_t;

	fstate_t     fstate;
	logic [22:0] f_ptr;          // count word of source row f_row
	logic [15:0] f_row;
	logic        f_cnt_known;    // f_cnt is row f_row's count (row was fetched)
	logic [15:0] f_cnt;
	logic [31:0] f_sy;           // sourcey of the next destination row
	logic [8:0]  f_rows_left;    // destination rows not yet handed over
	logic [8:0]  f_widx;         // words stored
	logic [12:0] f_cum;          // run length of the stored words
	logic [4:0]  f_nrep;

	wire [15:0] f_target  = f_sy[31:16];
	wire [15:0] ack_count = rle_count(rom_dout);
	wire [22:0] skip_ptr  = f_ptr + 23'd1 + {7'd0, ack_count};
	wire [12:0] f_cum_n   = f_cum + {8'd0, rle_run(mode, rom_dout[7:0])}
	                              + {8'd0, rle_run(mode, rom_dout[15:8])};
	wire [31:0] f_sy_n    = f_sy + {6'd0, dy};

	always_ff @(posedge clk) begin
		f_push  <= 1'b0;
		bank_we <= 1'b0;

		if (!rst_n) begin
			fstate  <= F_IDLE;
			rom_req <= 1'b0;
		end
		else begin
			case (fstate)

			F_IDLE:
				if (start) begin
					f_ptr       <= data_ptr;
					f_row       <= 16'd0;
					f_cnt_known <= 1'b0;
					f_sy        <= sourcey0;
					f_rows_left <= nrows;
					f_bank      <= 1'b0;
					fstate      <= F_NEXT;
				end

			//---- Choose: skip a row, fetch the next row, or finish ---------------
			F_NEXT:
				if (f_rows_left == 9'd0) begin
					fstate <= F_END;
				end
				else if (f_row != f_target) begin
					if (f_cnt_known) begin
						// Leaving a fetched row: its count is known, no read.
						f_ptr       <= f_ptr + 23'd1 + {7'd0, f_cnt};
						f_row       <= f_row + 16'd1;
						f_cnt_known <= 1'b0;
					end else begin
						rom_addr <= word_addr(f_ptr);
						rom_req  <= 1'b1;
						fstate   <= F_SKIP;
					end
				end
				else if (!bank_full[f_bank]) begin
					rom_addr <= word_addr(f_ptr);
					rom_req  <= 1'b1;
					fstate   <= F_HDR;
				end

			//---- Skip rows: chained count reads ----------------------------------
			F_SKIP:
				if (rom_ack) begin
					f_ptr <= skip_ptr;
					f_row <= f_row + 16'd1;
					if (f_row + 16'd1 != f_target) begin
						rom_addr <= word_addr(skip_ptr);         // next row's count
					end else if (!bank_full[f_bank]) begin
						rom_addr <= word_addr(skip_ptr);         // the wanted row
						fstate   <= F_HDR;
					end else begin
						rom_req <= 1'b0;
						fstate  <= F_NEXT;
					end
				end

			//---- Fetch the wanted row: count, then packet words ------------------
			F_HDR:
				if (rom_ack) begin
					f_cnt       <= ack_count;
					f_cnt_known <= 1'b1;
					f_widx      <= 9'd0;
					f_cum       <= 13'd0;
					f_nrep      <= 5'd0;
					if (ack_count == 16'd0) begin
						rom_req <= 1'b0;                         // no packets
						fstate  <= F_REP;
					end else begin
						rom_addr <= word_addr(f_ptr + 23'd1);
						fstate   <= F_WORDS;
					end
				end

			F_WORDS:
				if (rom_ack) begin
					bank_waddr <= {f_bank, f_widx[7:0]};
					bank_wdata <= rom_dout;
					bank_we    <= 1'b1;
					f_widx     <= f_widx + 9'd1;
					f_cum      <= f_cum_n;
					// Stop after the last word, or once the words so far
					// reach past the last source column that can be drawn.
					if ({7'd0, f_widx} + 16'd1 >= f_cnt || f_cum_n > {1'b0, src_limit}
					        || f_widx == 9'd255) begin
						rom_req <= 1'b0;
						fstate  <= F_REP;
					end else begin
						rom_addr <= word_addr(f_ptr + 23'd2 + {14'd0, f_widx});
					end
				end

			//---- Destination rows that use this source row -----------------------
			F_REP: begin
				f_sy        <= f_sy_n;
				f_rows_left <= f_rows_left - 9'd1;
				f_nrep      <= f_nrep + 5'd1;
				if (f_rows_left == 9'd1 || f_sy_n[31:16] != f_row) begin
					job_nw[f_bank]  <= f_widx;
					job_rep[f_bank] <= f_nrep + 5'd1;
					f_push          <= 1'b1;
					f_bank          <= ~f_bank;
					fstate          <= F_NEXT;
				end
			end

			F_END:
				if (bank_full == 2'b00 && !f_push && estate_idle)
					fstate <= F_IDLE;

			default: fstate <= F_IDLE;
			endcase
		end
	end

	//========================================================================
	//  Emitter
	//========================================================================
	logic [4:0]  e_rep;          // rows left in the current job
	logic [8:0]  e_nw;           // words in the bank
	logic [7:0]  e_y;
	logic [27:0] e_sx;           // 16.16 sourcex
	logic [8:0]  e_dest;
	logic [12:0] e_cum;          // end of the current run, whole pixels
	logic [5:0]  e_val;          // value of the current run

	// Word queue from the bank: wa, wb, then bank_q when e_qv. e_hi: the
	// head word's low byte has been taken.
	logic [8:0]  e_rd;           // next bank word to read
	logic        e_qv;           // bank_q holds the word read last clock
	logic [15:0] e_wa, e_wb;
	logic [1:0]  e_n;
	logic        e_hi;

	wire [15:0] head_word = (e_n != 2'd0) ? e_wa : bank_q;
	wire        byte_v    = (e_n != 2'd0) || e_qv;
	wire [7:0]  cur_byte  = e_hi ? head_word[15:8] : head_word[7:0];
	wire        exhausted = !byte_v && (e_rd == e_nw);
	wire [4:0]  byte_run  = rle_run(mode, cur_byte);
	wire [5:0]  byte_val  = rle_value(mode, cur_byte[5:0]);

	wire        in_run    = {1'b0, e_sx[27:16]} < e_cum;
	wire [27:0] e_sx_n    = e_sx + {2'd0, dx};
	wire        run_ends  = {1'b0, e_sx_n[27:16]} >= e_cum;
	wire        at_bound  = (e_dest == bound);

	// Which byte to take this clock, and whether the row ends
	logic take, row_end;
	always_comb begin
		take    = 1'b0;
		row_end = 1'b0;
		if (estate == E_RUN) begin
			if (in_run) begin
				if (at_bound)
					row_end = 1'b1;
				else if (run_ends) begin
					if (byte_v)         take    = 1'b1;
					else if (exhausted) row_end = 1'b1;
				end
			end else begin
				if (byte_v)         take    = 1'b1;
				else if (exhausted) row_end = 1'b1;
			end
		end
	end

	wire pop = take && e_hi;       // the head word's second byte is taken

	// Bank reads: keep at most two words queued (see the queue update). A
	// row starts with an empty queue and word 0.
	wire [1:0] n_after  = e_n + {1'b0, e_qv} - {1'b0, pop};
	wire       rd_issue = (estate == E_START) ? (e_nw != 9'd0)
	                    : (estate == E_RUN) && !row_end && (e_rd != e_nw) && (n_after <= 2'd1);

	assign bank_raddr = {e_bank, (estate == E_START) ? 8'd0 : e_rd[7:0]};

	always_ff @(posedge clk) begin
		e_free <= 1'b0;
		fb_we  <= 1'b0;

		if (!rst_n) begin
			estate <= E_IDLE;
			e_bank <= 1'b0;
		end
		else begin
			case (estate)

			E_IDLE:
				if (start) begin
					e_bank <= 1'b0;
					e_y    <= y0;
				end
				else if (bank_full[e_bank] && !e_free) begin
					e_rep  <= job_rep[e_bank];
					e_nw   <= job_nw[e_bank];
					estate <= E_START;
				end

			// Row start: the first bank read goes out here.
			E_START: begin
				e_sx   <= sourcex0;
				e_dest <= dest0;
				e_cum  <= 13'd0;
				e_n    <= 2'd0;
				e_hi   <= 1'b0;
				e_qv   <= rd_issue;
				e_rd   <= {8'd0, rd_issue};
				estate <= E_RUN;
			end

			E_RUN: begin
				// Pixel: written if opaque; position and source advance
				// (a row end discards them, E_START reloads both)
				if (in_run) begin
					if (e_val != 6'd0) begin
						fb_x     <= e_dest;
						fb_y     <= e_y;
						fb_pixel <= pix_base | {6'd0, e_val};
						fb_we    <= 1'b1;
					end
					e_sx   <= e_sx_n;
					e_dest <= hflip ? e_dest - 9'd1 : e_dest + 9'd1;
				end

				// Byte
				if (take) begin
					e_cum <= e_cum + {8'd0, byte_run};
					e_val <= byte_val;
					e_hi  <= !e_hi;
				end

				// Word queue: drop the head if popped, append bank_q if it
				// arrived and was not the popped head itself.
				case ({pop, e_qv})
					2'b00: ;
					2'b01: begin
						if (e_n == 2'd0)      e_wa <= bank_q;
						else if (e_n == 2'd1) e_wb <= bank_q;
						e_n <= e_n + 2'd1;
					end
					2'b10: begin
						e_wa <= e_wb;
						e_n  <= e_n - 2'd1;
					end
					2'b11: begin
						if (e_n == 2'd0) ;                    // bank_q was the head
						else if (e_n == 2'd1) e_wa <= bank_q;
						else begin e_wa <= e_wb; e_wb <= bank_q; end
					end
				endcase
				e_qv <= rd_issue;
				if (rd_issue) e_rd <= e_rd + 9'd1;

				// Row end: next destination row, same bank or next job
				if (row_end) begin
					e_y <= e_y + 8'd1;
					if (e_rep == 5'd1) begin
						e_free <= 1'b1;
						e_bank <= ~e_bank;
						estate <= E_IDLE;
					end else begin
						e_rep  <= e_rep - 5'd1;
						estate <= E_START;
					end
				end
			end

			default: estate <= E_IDLE;
			endcase
		end
	end

	//========================================================================
	//  Bank handover
	//========================================================================
	// f_push marks the bank the fetcher just left (f_bank has already moved
	// on), e_free the bank the emitter just left.
	always_ff @(posedge clk) begin
		if (!rst_n || (start && fstate == F_IDLE)) begin
			bank_full <= 2'b00;
		end else begin
			if (f_push) bank_full[~f_bank] <= 1'b1;
			if (e_free) bank_full[~e_bank] <= 1'b0;
		end
	end

	always_ff @(posedge clk) begin
		if (!rst_n) done <= 1'b0;
		else        done <= (fstate == F_END) && bank_full == 2'b00 && !f_push && estate_idle;
	end

`ifdef SIMULATION
	// synthesis translate_off
	// The packet decode against MAME build_rle_tables(), every mode and
	// byte (the ROM sets use only modes 0, 2 and 5).
	initial begin : decode_check
		int t, run, val, errs;
		errs = 0;
		for (int m = 0; m < 8; m++)
			for (int i = 0; i < 256; i++) begin
				case (m)
					0:       t = (((i & 'hf0) + 'h10) << 4) | (i & 'h0f);
					1:       t = ((i & 'h0f) == 0) ? ((((i & 'hf0) + 'h10) << 4) | (i & 'h0f))
					                               : ((((i & 'he0) + 'h20) << 3) | (i & 'h1f));
					2, 3:    t = (((i & 'he0) + 'h20) << 3) | (i & 'h1f);
					4, 6:    t = ((i & 'h0f) == 0) ? ((((i & 'hf0) + 'h10) << 4) | (i & 'h0f))
					                               : ((((i & 'hc0) + 'h40) << 2) | (i & 'h3f));
					default: t = (((i & 'hc0) + 'h40) << 2) | (i & 'h3f);
				endcase
				run = t >> 8;
				val = t & 'hff;
				if (int'(rle_run(3'(m), 8'(i))) != run || int'(rle_value(3'(m), 6'(i))) != val) errs++;
			end
		if (errs) $fatal(1, "g42_rle_render: %0d packet decode mismatches", errs);
		else      $display("g42_rle_render: packet decode = MAME tables, 8 modes x 256 bytes");
	end

	always_ff @(posedge clk) begin
		if (rst_n && fstate == F_HDR && rom_ack && ack_count > 16'd256)
			$error("g42_rle_render: row with %0d entries exceeds the 256-word bank", ack_count);
		if (rst_n && estate == E_RUN && in_run && e_val != 0
		        && (e_dest > 9'd335 || e_y > 8'd239))
			$error("g42_rle_render: pixel outside the screen at %0d,%0d", e_dest, e_y);
	end
	// Stop on an object that never finishes.
	int guard;
	always_ff @(posedge clk) begin
		if (fstate != F_IDLE) begin
			guard <= guard + 1;
			if (guard > 20_000_000) $fatal(1, "g42_rle_render: object did not terminate");
		end else guard <= 0;
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
