//============================================================================
//  Atari G42 for MiSTer
//  g42_mainram.sv -- 64 KB work RAM at $FF0000-$FFFFFF
//
//  One contiguous RAM, as on the board; the video hardware reads parts of it:
//    $FF0000-$FF0FFF  motion object RAM  (256 objects x 8 words)
//    $FF2000-$FF5FFF  playfield tile map (128 x 64)
//    $FF6000-$FF6FFF  alpha tile map (64 x 32) and the per-line scroll words
//    $FF7000          MO command latch (ordinary RAM, snooped by g42_top)
//  everything else is plain work RAM, including the 68000 stack.
//
//  True dual-port block RAM (g42_tdpram), one per byte lane so the 68000's
//  byte writes are byte enables:
//    port A  the 68000, read and write; for one clock at a time the RLE
//            engine's CHECKSUM write-back takes it over (ext_we). The 68000
//            holds its data strobes for several clocks and samples read data
//            late in the bus cycle, so a stolen clock is invisible to it.
//    port B  read only, for the video fetchers and the RLE object snapshot
//            (arbitrated in g42_video).
//  Unlike the G1 core, port A reads at the address it writes, so the two
//  lanes fit in 64 M10K as true dual-port memories instead of 128 as
//  duplicated simple dual-port ones.
//
//  Clock domain: clk_sys (both ports).
//============================================================================

`default_nettype none

module g42_mainram (
	input  wire         clk,

	// ---- Port A: 68000 -----------------------------------------------------
	input  wire [14:0]  cpu_addr,   // word address from $FF0000
	input  wire [15:0]  cpu_din,
	input  wire         cpu_wr_hi,  // write D[15:8] (/UDS)
	input  wire         cpu_wr_lo,  // write D[7:0]  (/LDS)
	output logic [15:0] cpu_dout,   // one clock after cpu_addr

	// ---- Port A override: RLE CHECKSUM write-back (word writes) ------------
	input  wire [14:0]  ext_addr,
	input  wire [15:0]  ext_din,
	input  wire         ext_we,

	// ---- Port B: video, read only ------------------------------------------
	input  wire [14:0]  vid_addr,
	output logic [15:0] vid_dout    // one clock after vid_addr
);

	wire [14:0] a_addr = ext_we ? ext_addr      : cpu_addr;
	wire [7:0]  a_hi   = ext_we ? ext_din[15:8] : cpu_din[15:8];
	wire [7:0]  a_lo   = ext_we ? ext_din[7:0]  : cpu_din[7:0];
	wire        a_whi  = ext_we | cpu_wr_hi;
	wire        a_wlo  = ext_we | cpu_wr_lo;

	wire [7:0]  qa_hi, qa_lo, qb_hi, qb_lo;

	// One explicit true dual-port RAM per byte lane (32 M10K each): Quartus
	// inferred this coding as two simple dual-port copies (see g42_tdpram).
	// On a write, port A returns the written byte instead of the old one;
	// the 68000 does not sample data on a write cycle, so nothing sees it.
	g42_tdpram #(.DW(8), .AW(15)) u_hi
	(
		.clk    (clk),
		.addr_a (a_addr),
		.din_a  (a_hi),
		.we_a   (a_whi),
		.q_a    (qa_hi),
		.addr_b (vid_addr),
		.din_b  (8'h00),
		.we_b   (1'b0),
		.q_b    (qb_hi)
	);

	g42_tdpram #(.DW(8), .AW(15)) u_lo
	(
		.clk    (clk),
		.addr_a (a_addr),
		.din_a  (a_lo),
		.we_a   (a_wlo),
		.q_a    (qa_lo),
		.addr_b (vid_addr),
		.din_b  (8'h00),
		.we_b   (1'b0),
		.q_b    (qb_lo)
	);

	assign cpu_dout = {qa_hi, qa_lo};
	assign vid_dout = {qb_hi, qb_lo};

endmodule

`default_nettype wire
