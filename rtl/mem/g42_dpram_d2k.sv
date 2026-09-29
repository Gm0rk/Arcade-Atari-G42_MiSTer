//============================================================================
//  Atari G42 for MiSTer
//  g42_dpram_d2k.sv -- simple dual-port block RAM, M10K capped at 2048 words
//
//  g42_dpram (same ports, same coding template) with the synthesis
//  attribute max_depth = 2048, so Quartus builds it from M10K blocks no
//  deeper than 2048 words: 2048 x 5, or narrower shapes of 2048 or fewer.
//  Where Quartus picks the shape itself it favours deep, narrow blocks:
//    framebuffer quarter, 20,160 x 10   4096 x 2: 25 M10K; capped: 20
//    prescan table,        5120 x 19    8192 x 1: 19 M10K; capped: 12
//  The read side then selects between up to 10 blocks instead of 5 or 3.
//  With the chip's block RAM nearly full, those 47 blocks decide whether
//  the design's small memories are built as RAM or from registers.
//
//  Clock domain: clk
//============================================================================

`default_nettype none

module g42_dpram_d2k #(
	parameter int DW    = 8,              // data width
	parameter int AW    = 10,             // address width
	parameter int DEPTH = 1 << AW         // words; may be less than 2^AW
)(
	input  wire          clk,

	// ---- Write port ------------------------------------------------------
	input  wire          we,              // write enable, one clock per word
	input  wire [AW-1:0] waddr,
	input  wire [DW-1:0] wdata,

	// ---- Read port -------------------------------------------------------
	input  wire [AW-1:0] raddr,
	output logic [DW-1:0] rdata           // mem[raddr], one clock later
);

	(* max_depth = 2048 *) logic [DW-1:0] mem [0:DEPTH-1];

	always_ff @(posedge clk) begin
		if (we) mem[waddr] <= wdata;
		rdata <= mem[raddr];
	end

endmodule

`default_nettype wire
