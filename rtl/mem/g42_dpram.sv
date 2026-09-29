//============================================================================
//  Atari G42 for MiSTer
//  g42_dpram.sv -- simple dual-port block RAM, one clock
//
//  One write port and one registered read port on the same clock, written
//  exactly as Intel's "simple dual-port RAM, single clock" coding template,
//  so Quartus maps it to M10K blocks.
//
//  Why a module of its own: Quartus 17.0's Analysis & Synthesis stalled right
//  after elaboration, where it infers block RAM. The motion-object
//  framebuffer (g42_rle_fb) had the design's only memories declared inside a
//  generate loop, 1.9 Mbit of them; were Quartus not to recognise those as
//  RAM, it would build them from registers and never finish. A memory
//  declared at module scope, one per instance, is the form every Quartus
//  version infers, so the generate loop now instantiates this module.
//
//  With the module in place, Analysis & Synthesis finished in 3 minutes.
//
//  Read-during-write to the same address returns the old contents (the
//  read is a non-blocking assignment in the same process as the write), as
//  the arrays this module replaces did, so simulation is unchanged.
//
//  Quartus picks the M10K shape; g42_dpram_d2k.sv is the same RAM with
//  the shape capped at 2048 words.
//
//  Clock domain: clk
//============================================================================

`default_nettype none

module g42_dpram #(
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

	logic [DW-1:0] mem [0:DEPTH-1];

	always_ff @(posedge clk) begin
		if (we) mem[waddr] <= wdata;
		rdata <= mem[raddr];
	end

endmodule

`default_nettype wire
