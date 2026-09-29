//============================================================================
//  Atari G42 for MiSTer
//  g42_rle_fb.sv -- double-buffered motion object framebuffer and erase engine
//
//  Control register: io_latch ($E00050) bits 13:11 = {FRAME, ERASE, MOGO}.
//  MAME calls control_write((data >> 11) & 7) on every write that includes
//  the upper byte; ctrl_wr pulses for each, and the "nothing changed"
//  early return is made here:
//
//    control_write(data):
//      if data == bits: return                  (partial is not updated)
//      if old ERASE:    clear buffer[old FRAME] over lines
//                       max(0, partial+1) .. min(239, vpos)
//      bits = data
//      MOGO rising:     execute the command (render into buffer[~FRAME])
//      partial = vpos
//    VBLANK start:
//      if ERASE:        clear buffer[FRAME] over lines max(0, partial+1) .. 239
//      partial = -1
//
//  Guardians writes 0 then 3 (or 0 then 7) at the top of every VBLANK: the
//  0 clears the whole buffer just displayed and the MOGO renders into it.
//  Road Riot and Danger Express render every other frame.
//
//  MAME's erases are instant; here each request latches its buffer and line
//  range when made and waits in a 4-deep FIFO for the engine, which clears
//  4 pixels per clock (a full buffer in 20,160 clocks). rnd_hold is high
//  while any erase of the render target is queued or running; g42_rle waits
//  for it before drawing, so the erase lands first, as in MAME.
//
//  Storage: 12 bits per pixel, not 13. On Road Riot (MO base $200, colour
//  mask $1F) palette index bit 9 is always 1; on the $400 games (mask $3F)
//  colour bit 5 is descriptor bit 9, which is also priority bit 0. Either
//  way index bit 9 = mo_base[1] ^ (color_mask[5] & priority[0]), so it is
//  restored on the way out and {priority[2:0], index[8:0]} is stored:
//      lo  {priority[0], index[8:0]}   10 bits
//      hi  priority[2:1]               2 bits
//  A drawn pixel has a non-zero index (the packet value sits in the low
//  bits), so 0 still means "no object".
//
//  Each buffer is split four ways by x[1:0] (84 words per line, 20,160 per
//  quarter) so the erase engine can clear four pixels a clock through four
//  write ports. M10K: lo 4 x 20 (2048 x 5) + hi 4 x 5 (4096 x 2) = 100 per
//  buffer, 200 for both; one unsplit 12-bit buffer would be 99, a 13-bit
//  one 109.
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_rle_fb
	import g42_pkg::*;
(
	input  wire         clk,
	input  wire         rst_n,

	// ---- Control register ----------------------------------------------------
	input  wire [2:0]   ctrl,           // {FRAME, ERASE, MOGO}
	input  wire         ctrl_wr,        // one clock per 68000 write of the byte

	// ---- Raster --------------------------------------------------------------
	input  wire [8:0]   vpos,           // 0..239 visible, any value >= 240 in VBLANK
	input  wire         vblank_rise,

	// ---- Configuration (index bit 9) -----------------------------------------
	input  wire         base_bit9,      // palette base bit 9: 1 for $200, 0 for $400
	input  wire         color_bit5,     // colour mask bit 5 (= priority bit 0)

	// ---- Render write port -----------------------------------------------------
	input  wire [8:0]   rnd_x,
	input  wire [7:0]   rnd_y,
	input  wire [11:0]  rnd_pixel,      // {priority[2:0], index[8:0]}, non-zero
	input  wire         rnd_we,

	// ---- Display read port -----------------------------------------------------
	input  wire [8:0]   disp_x,
	input  wire [7:0]   disp_y,
	output logic [12:0] disp_pixel,     // {priority, index[9:0]} next clock; 0 = none

	// ---- Status ------------------------------------------------------------------
	output logic        mogo_rise,      // command trigger, one clock
	input  wire         rnd_accept,     // g42_rle took that MOGO as a DRAW
	output logic        rnd_hold,       // an erase of the render target is pending
	output logic        erase_busy
);

	localparam int      QW        = MO_FB_W / 4;              // 84 words per line
	localparam int      Q_WORDS   = QW * MO_FB_H;             // 20,160
	localparam [7:0]    LAST_LINE = 8'(MO_FB_H - 1);          // 239
	localparam [6:0]    LAST_QX   = 7'(QW - 1);               // 83

	// y * 84 + qx = y*64 + y*16 + y*4 + qx
	function automatic [14:0] q_addr(input [7:0] y, input [6:0] qx);
		q_addr = {1'b0, y, 6'd0} + {3'd0, y, 4'd0} + {5'd0, y, 2'd0} + {8'd0, qx};
	endfunction

	//========================================================================
	//  Register state
	//========================================================================
	logic [2:0] bits;           // MAME m_control_bits
	logic [8:0] part_top;       // max(0, m_partial_scanline + 1)
	logic       rnd_buf;        // buffer the current or last render writes

	wire  frame  = bits[2];
	wire  wr_chg = ctrl_wr && (ctrl != bits);

	//========================================================================
	//  Erase requests
	//========================================================================
	// VBLANK start and a register write can come in the same clock. Both use
	// the current FRAME (the write erases with the old bits), so they merge
	// into one range; the VBLANK request is evaluated first, as in MAME.
	//========================================================================
	wire [8:0] wr_bot9      = (vpos > 9'(LAST_LINE)) ? 9'(LAST_LINE) : vpos;
	wire [8:0] top_after_vb = vblank_rise ? 9'd0 : part_top;

	logic       req_v;
	logic [7:0] req_top, req_bot;

	always_comb begin
		req_v   = 1'b0;
		req_top = 8'd0;
		req_bot = 8'd0;
		if (bits[1]) begin
			if (vblank_rise && 9'(LAST_LINE) >= part_top) begin
				req_v   = 1'b1;
				req_top = part_top[7:0];
				req_bot = LAST_LINE;
			end
			if (wr_chg && wr_bot9 >= top_after_vb) begin
				if (req_v) begin
					if (top_after_vb[7:0] < req_top) req_top = top_after_vb[7:0];
					if (wr_bot9[7:0]      > req_bot) req_bot = wr_bot9[7:0];
				end else begin
					req_v   = 1'b1;
					req_top = top_after_vb[7:0];
					req_bot = wr_bot9[7:0];
				end
			end
		end
	end

	//========================================================================
	//  Request FIFO (4 deep) and engine
	//========================================================================
	// Requests are executed in arrival order; erases commute with each
	// other, and rnd_hold orders them against the renderer. Requests come
	// at most one per register write or VBLANK and a full erase takes about
	// 20,000 clocks, so the FIFO never fills in practice (checked in
	// simulation).
	//========================================================================
	logic [16:0] fifo [4];      // {buffer, top, bottom}
	logic [1:0]  f_rd, f_wr;
	logic [2:0]  f_cnt;

	logic        erasing, erase_buf;
	logic [7:0]  erase_line, erase_end;
	logic [6:0]  erase_qx;

	wire eng_last = erasing && (erase_qx == LAST_QX) && (erase_line >= erase_end);
	wire take     = (f_cnt != 3'd0) && (!erasing || eng_last);

	// rnd_hold: running erase or any queued request on the render target
	always_comb begin
		rnd_hold = erasing && (erase_buf == rnd_buf);
		for (int i = 0; i < 4; i++)
			if (3'(i) < f_cnt && fifo[2'(f_rd + 2'(i))][16] == rnd_buf)
				rnd_hold = 1'b1;
	end

	// Also covers the engine's last write, which lands a clock later.
	logic erasing_d;
	always_ff @(posedge clk) erasing_d <= erasing;
	assign erase_busy = erasing || erasing_d || (f_cnt != 3'd0);

	always_ff @(posedge clk) begin
		mogo_rise <= 1'b0;

		if (!rst_n) begin
			bits       <= 3'b000;
			part_top   <= 9'd0;           // MAME device_reset: partial = -1
			rnd_buf    <= 1'b1;
			f_rd       <= 2'd0;
			f_wr       <= 2'd0;
			f_cnt      <= 3'd0;
			erasing    <= 1'b0;
			erase_buf  <= 1'b0;
			erase_line <= 8'd0;
			erase_end  <= 8'd0;
			erase_qx   <= 7'd0;
		end
		else begin
			//---- Register update ------------------------------------------------
			if (wr_chg) begin
				bits     <= ctrl;
				part_top <= vpos + 9'd1;
				if (!bits[0] && ctrl[0]) mogo_rise <= 1'b1;
			end
			else if (vblank_rise) begin
				part_top <= 9'd0;
			end

			// Latch the render target when g42_rle starts a DRAW (the clock
			// after mogo_rise, when bits holds the new FRAME).
			if (rnd_accept) rnd_buf <= ~bits[2];

			//---- Engine --------------------------------------------------------
			if (erasing) begin
				if (erase_qx == LAST_QX) begin
					erase_qx <= 7'd0;
					if (erase_line >= erase_end) erasing <= 1'b0;
					else                         erase_line <= erase_line + 8'd1;
				end else begin
					erase_qx <= erase_qx + 7'd1;
				end
			end
			if (take) begin
				erasing    <= 1'b1;
				erase_buf  <= fifo[f_rd][16];
				erase_line <= fifo[f_rd][15:8];
				erase_end  <= fifo[f_rd][7:0];
				erase_qx   <= 7'd0;
				f_rd       <= f_rd + 2'd1;
			end

			//---- FIFO ------------------------------------------------------------
			if (req_v && f_cnt != 3'd4) begin
				fifo[f_wr] <= {frame, req_top, req_bot};
				f_wr       <= f_wr + 2'd1;
			end
			f_cnt <= f_cnt + {2'd0, req_v && f_cnt != 3'd4} - {2'd0, take};
		end
	end

	//========================================================================
	//  The two buffers, four quarters each
	//========================================================================
	// Every quarter has its own write port, driven by the erase engine while
	// it clears that buffer, otherwise by the renderer for its own x[1:0].
	// rnd_hold keeps the two off the same buffer. The write port is
	// registered for timing; the display never reads the buffer being
	// written, and rnd_hold drops long before the renderer's next write.
	//========================================================================
	wire [14:0] a_disp  = q_addr(disp_y, disp_x[8:2]);
	wire [14:0] a_rnd   = q_addr(rnd_y, rnd_x[8:2]);
	wire [14:0] a_erase = q_addr(erase_line, erase_qx);

	wire [95:0] q_all;          // 8 x {hi[1:0], lo[9:0]}, one slice per quarter

	// Written in the Verilog-2001 generate form (explicit genvar, generate /
	// endgenerate, g = g + 1): Quartus 17.0 does not accept a loop generate
	// with an inline genvar and g++. The memories themselves are g42_dpram
	// instances, not arrays declared in the loop: see g42_dpram.sv for why.
	// lo and hi are separate RAMs, packed as 2048 x 5 (g42_dpram_d2k, 20
	// M10K) and 4096 x 2 (5 M10K): 25 a quarter, 200 in all. One 12-bit RAM
	// would use 512 x 20 blocks (40 a quarter).
	genvar g;
	generate
	for (g = 0; g < 8; g = g + 1) begin : g_quarter
		localparam integer G_BUF = g / 4;         // quarters 0-3: buffer 0
		localparam integer G_QX  = g % 4;         // the pixel x[1:0] this quarter holds
		localparam [0:0]   BUF   = G_BUF[0:0];
		localparam [1:0]   QX    = G_QX[1:0];

		logic [14:0] waddr;
		logic [11:0] wdata;
		logic        we;

		wire er = erasing && (erase_buf == BUF);

		always_ff @(posedge clk) begin
			waddr <= er ? a_erase : a_rnd;
			wdata <= er ? 12'd0   : rnd_pixel;
			we    <= er || (rnd_we && rnd_buf == BUF && rnd_x[1:0] == QX);
		end

		wire [9:0] q_lo;
		wire [1:0] q_hi;

		// {priority[0], index[8:0]}
		g42_dpram_d2k #(.DW(10), .AW(15), .DEPTH(Q_WORDS)) u_lo
		(
			.clk   (clk),
			.we    (we),
			.waddr (waddr),
			.wdata (wdata[9:0]),
			.raddr (a_disp),
			.rdata (q_lo)
		);

		// priority[2:1]
		g42_dpram #(.DW(2), .AW(15), .DEPTH(Q_WORDS)) u_hi
		(
			.clk   (clk),
			.we    (we),
			.waddr (waddr),
			.wdata (wdata[11:10]),
			.raddr (a_disp),
			.rdata (q_hi)
		);

		assign q_all[g*12 +: 12] = {q_hi, q_lo};
	end
	endgenerate

	//========================================================================
	//  Display output
	//========================================================================
	// Select registered alongside the BRAM read so it matches the data. One
	// clock of latency, absorbed by the mixer's own register.
	//========================================================================
	logic [2:0] disp_sel;
	always_ff @(posedge clk) disp_sel <= {frame, disp_x[1:0]};

	wire [11:0] q12  = q_all[disp_sel*12 +: 12];
	wire        idx9 = base_bit9 ^ (color_bit5 & q12[9]);

	assign disp_pixel = (q12 == 12'd0) ? 13'd0 : {q12[11:9], idx9, q12[8:0]};

`ifdef SIMULATION
	// synthesis translate_off
	always_ff @(posedge clk) begin
		if (rst_n && req_v && f_cnt == 3'd4)
			$error("g42_rle_fb: erase request FIFO overflow");
		if (rst_n && rnd_we && erasing && erase_buf == rnd_buf)
			$error("g42_rle_fb: render pixel dropped during erase of buffer %0d", rnd_buf);
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
