//============================================================================
//  Atari G42 for MiSTer
//  g42_rle_checksum.sv -- RLE ROM checksums and ROM length, from the loader
//
//  The self-tests issue a CHECKSUM command (a 0 written to $FF7000, then a
//  MOGO) and read a table of RLE ROM checksums back from object RAM.
//  MAME atarirle.cpp:
//
//    device_start():
//      for each $20000-byte chunk of the RLE ROM:
//          checksums[chunk] = sum of the $10000 words in the chunk, mod $10000
//    compute_checksum():
//      reqsums = min(objram[0] + 1, 256)
//      for i in 0 .. reqsums-1:  objram[i] = checksums[i]
//
//  The sums are accumulated by snooping the ROM loader's SDRAM writes, so no
//  pass over SDRAM is needed (as Atari G1). Road Riot's ROM is 2 MB (16
//  chunks), Guardians' 6 MB (48), Danger Express' 8 MB (64); entries past
//  the ROM stay 0, as in MAME.
//
//  The same snoop measures the ROM: rom_words is one past the highest RLE
//  word written, which is MAME's m_rombase.length() for g42_rle_prescan
//  (the RLE ROM is the last SDRAM region, so the download ends with it).
//
//  The write-back runs independently of the renderer, so a CHECKSUM works
//  during the load-time prescan and during a render.
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_rle_checksum
	import g42_pkg::*;
(
	input  wire         clk,
	input  wire         rst_n,            // core reset: the write-back only
	// Accumulator reset: tie to PLL lock, not the core reset, which is held
	// for the whole ROM download and would keep the table cleared.
	input  wire         acc_rst_n,

	// ---- Snoop of the ROM loader's SDRAM write port -----------------------
	// load_active: ROM (index 0) download only. Its rising edge restarts the
	// clear, so later config/nvram downloads must not assert it.
	input  wire         load_active,
	input  wire [24:0]  load_addr,
	input  wire [15:0]  load_data,
	input  wire         load_we,          // level, held until the SDRAM takes the word
	output logic [22:0] rom_words,        // RLE ROM length in words (0 before a load)

	// ---- Write-back on a CHECKSUM command ---------------------------------
	input  wire         cksum_start,      // pulse
	input  wire [15:0]  objram_word0,     // the game's requested count, minus 1
	output logic [14:0] wr_addr,          // work RAM word address
	output logic [15:0] wr_data,
	output logic        wr_en,            // one clock per word
	output logic        busy
);

	//========================================================================
	//  Accumulation during the load
	//========================================================================
	// Chunks are $20000 bytes: the chunk index is (load_addr - SDR_RLE)
	// bits 22:17. Words are summed with 16-bit wraparound. One muxed read
	// and one muxed write port, so sums[] infers as one M10K. Accumulation
	// (ROM download) and write-back (CHECKSUM) never overlap.
	//========================================================================
	localparam [24:0] RLE_MAX_BYTES = 25'h800000;   // 8 MB

	logic [15:0] sums [256];
	logic [7:0]  sum_raddr, sum_waddr;
	logic [15:0] sum_wdata, sum_q;
	logic        sum_we;

	always_ff @(posedge clk) begin
		if (sum_we) sums[sum_waddr] <= sum_wdata;
		sum_q <= sums[sum_raddr];
	end

	wire [24:0] rle_offset    = load_addr - SDR_RLE;
	wire        in_rle_region = load_active && (load_addr >= SDR_RLE)
	                         && (rle_offset < RLE_MAX_BYTES);
	wire [7:0]  chunk         = {2'd0, rle_offset[22:17]};

	// 256-clock clear at the start of each ROM download and after acc_rst_n.
	logic [7:0] clear_i;
	logic       clearing;
	logic       load_active_d;

	always_ff @(posedge clk) begin
		load_active_d <= load_active;
		if (!acc_rst_n || (load_active && !load_active_d)) begin
			clearing <= 1'b1;
			clear_i  <= 8'd0;
		end
		else if (clearing) begin
			if (clear_i == 8'hFF) clearing <= 1'b0;
			else                  clear_i  <= clear_i + 1'b1;
		end
	end

	// load_we is a level, held with stable address and data until the SDRAM
	// arbiter takes the word (three clocks or more), so each word is added
	// once, on the rising edge of load_we:
	//   edge clock       read sums[chunk]; capture chunk and data
	//   next clock       sums[chunk] <= sum_q + data
	// sum_q is the current total: load_we falls between words, so the
	// previous write lands at least one clock before this read.
	logic        load_we_d, acc_v;
	logic [7:0]  acc_chunk;
	logic [15:0] acc_data;

	always_ff @(posedge clk) begin
		if (!acc_rst_n) begin
			load_we_d <= 1'b0;
			acc_v     <= 1'b0;
			rom_words <= '0;
		end else begin
			load_we_d <= load_we;
			acc_v     <= in_rle_region && load_we && !load_we_d;
			if (load_active && !load_active_d)
				rom_words <= '0;
			else if (in_rle_region && load_we && ({1'b0, rle_offset[22:1]} >= rom_words))
				rom_words <= {1'b0, rle_offset[22:1]} + 23'd1;
		end
		acc_chunk <= chunk;
		acc_data  <= load_data;
	end

	//========================================================================
	//  Write-back
	//========================================================================
	typedef enum logic [1:0] { W_IDLE, W_READ, W_WRITE } wstate_t;
	wstate_t wstate;

	logic [8:0] req_count;     // objram[0] + 1, capped at 256
	logic [8:0] wr_i;

	assign busy = (wstate != W_IDLE);

	always_comb begin
		sum_raddr = (wstate == W_IDLE) ? chunk : wr_i[7:0];
		if (clearing) begin
			sum_waddr = clear_i;
			sum_wdata = 16'd0;
			sum_we    = 1'b1;
		end else begin
			sum_waddr = acc_chunk;
			sum_wdata = sum_q + acc_data;
			sum_we    = acc_v;
		end
	end

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			wstate <= W_IDLE;
			wr_en  <= 1'b0;
		end
		else begin
			wr_en <= 1'b0;

			case (wstate)
			W_IDLE:
				if (cksum_start) begin
					req_count <= (objram_word0 >= 16'd255) ? 9'd256
					                                       : ({1'b0, objram_word0[7:0]} + 9'd1);
					wr_i   <= 9'd0;
					wstate <= W_READ;
				end

			// One clock of sums[] read latency (sum_raddr = wr_i here).
			W_READ: wstate <= W_WRITE;

			W_WRITE: begin
				wr_addr <= {6'd0, wr_i};         // object RAM is work RAM word 0
				wr_data <= sum_q;
				wr_en   <= 1'b1;
				if (wr_i + 9'd1 >= req_count) begin
					wstate <= W_IDLE;
				end else begin
					wr_i   <= wr_i + 9'd1;
					wstate <= W_READ;
				end
			end

			default: wstate <= W_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
