//============================================================================
//  Atari G42 for MiSTer
//  g42_palette.sv -- 2048-entry palette RAM, IRGB-1555 to RGB888
//
//  CPU $FC0000-$FC0FFF: 2048 16-bit entries in MAME's IRGB_1555 format
//  (PALETTE(config, "palette").set_format(palette_device::IRGB_1555, 2048)).
//  Dual-port BRAM as in G1's g1_palette: port A is the 68000 (read/write,
//  byte lanes), port B the video scan-out (read only). A CPU write and a
//  video read of the same entry in one clock give the video port the old
//  value.
//
//  Decode (MAME raw_to_rgb_converter::IRRRRRGGGGGBBBBB_decoder): the
//  intensity bit is the shared LSB of three 6-bit channels, not a brightness
//  flag; pal6bit expands 6 to 8 bits by repeating the top two:
//      r6 = {raw[14:10], raw[15]}     r8 = {r6, r6[5:4]}   (g, b alike)
//  so $7C00 -> R = $FB and $FC00 -> R = $FF.
//
//  Clock domain: clk_sys (both ports).
//============================================================================

`default_nettype none

module g42_palette
	import g42_pkg::*;
(
	input  wire         clk,

	// ---- Port A: 68000 -----------------------------------------------------
	input  wire [10:0]  cpu_addr,           // entry 0..2047 (68000 word address [11:1])
	input  wire [15:0]  cpu_din,
	input  wire         cpu_wr_hi,          // write D[15:8] (/UDS)
	input  wire         cpu_wr_lo,          // write D[7:0]  (/LDS)
	output logic [15:0] cpu_dout,           // one clock after cpu_addr

	// ---- Port B: video scan-out -------------------------------------------
	input  wire [10:0]  vid_index,
	output logic [7:0]  vid_r,              // two clocks after vid_index
	output logic [7:0]  vid_g,
	output logic [7:0]  vid_b
);

	//========================================================================
	//  Storage: byte-wide arrays so the CPU byte enables infer reliably
	//========================================================================
	logic [7:0] ram_hi [PAL_ENTRIES];
	logic [7:0] ram_lo [PAL_ENTRIES];

	logic [7:0] cpu_q_hi, cpu_q_lo;
	logic [7:0] vid_q_hi, vid_q_lo;

	// Port A: CPU read/write.
	always_ff @(posedge clk) begin
		if (cpu_wr_hi) ram_hi[cpu_addr] <= cpu_din[15:8];
		if (cpu_wr_lo) ram_lo[cpu_addr] <= cpu_din[7:0];
		cpu_q_hi <= ram_hi[cpu_addr];
		cpu_q_lo <= ram_lo[cpu_addr];
	end

	assign cpu_dout = {cpu_q_hi, cpu_q_lo};

	// Port B: video read.
	always_ff @(posedge clk) begin
		vid_q_hi <= ram_hi[vid_index];
		vid_q_lo <= ram_lo[vid_index];
	end

	//========================================================================
	//  IRGB-1555 to RGB888
	//========================================================================
	wire [15:0] entry = {vid_q_hi, vid_q_lo};
	wire [5:0]  r6    = {entry[14:10], entry[15]};
	wire [5:0]  g6    = {entry[9:5],   entry[15]};
	wire [5:0]  b6    = {entry[4:0],   entry[15]};

	always_ff @(posedge clk) begin
		vid_r <= {r6, r6[5:4]};
		vid_g <= {g6, g6[5:4]};
		vid_b <= {b6, b6[5:4]};
	end

endmodule

`default_nettype wire
