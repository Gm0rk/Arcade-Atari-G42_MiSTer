//============================================================================
//  Atari G42 for MiSTer
//  g42_rle_pkg.sv -- RLE packet decoding shared by the motion object engine
//
//  MAME build_rle_tables() fills one 256-entry table per encoding mode with
//  (run << 8) | value, but every entry is a bit slice of its index, e.g. for
//  4bpp: table[i] = ((i & 0xF0) + 0x10) << 4 | (i & 0x0F), so value = i[3:0]
//  and run = i[7:4] + 1. The tables reduce to the functions below. Modes
//  (object header word 2, bits 10:8; attributes from g42_pkg):
//
//    mode  bpp  special   value       run
//      0    4     no      b[3:0]      b[7:4] + 1   (1..16)
//      1    5    yes      4bpp form if b[3:0] == 0, else 5bpp form
//      2,3  5     no      b[4:0]      b[7:5] + 1   (1..8)
//      4,6  6    yes      4bpp form if b[3:0] == 0, else 6bpp form
//      5,7  6     no      b[5:0]      b[7:6] + 1   (1..4)
//
//  A special mode's 4bpp form has a zero value: a transparent run of up to
//  16 in a 5 or 6bpp object. The three G42 ROM sets use modes 0, 2 and 5
//  only; the special modes are decoded anyway.
//
//  Also: the row count word. MAME's prescan_rle() rewrites any count with
//  bit 15 set as its complement ("inverted"), and the renderer only ever
//  reads counts the prescan has visited, so undoing it on the fly is exact.
//
//  No clock: functions only.
//============================================================================

`default_nettype none

`ifndef G42_RLE_PKG_SV
`define G42_RLE_PKG_SV

package g42_rle_pkg;

	// Packet byte -> run length, 1..16
	function automatic [4:0] rle_run(input [2:0] mode, input [7:0] b);
		if ((g42_pkg::rle_mode_special(mode) && b[3:0] == 4'h0) || g42_pkg::rle_mode_bpp(mode) == 3'd4)
			rle_run = {1'b0, b[7:4]} + 5'd1;
		else if (g42_pkg::rle_mode_bpp(mode) == 3'd5)
			rle_run = {2'd0, b[7:5]} + 5'd1;
		else
			rle_run = {3'd0, b[7:6]} + 5'd1;
	endfunction

	// Packet byte (its low six bits) -> pixel value, 0 = transparent
	function automatic [5:0] rle_value(input [2:0] mode, input [5:0] b);
		if ((g42_pkg::rle_mode_special(mode) && b[3:0] == 4'h0) || g42_pkg::rle_mode_bpp(mode) == 3'd4)
			rle_value = {2'd0, b[3:0]};
		else if (g42_pkg::rle_mode_bpp(mode) == 3'd5)
			rle_value = {1'b0, b[4:0]};
		else
			rle_value = b[5:0];
	endfunction

	// Row count word as MAME's renderer sees it after prescan_rle()
	function automatic [15:0] rle_count(input [15:0] w);
		rle_count = w[15] ? ~w : w;
	endfunction

endpackage

`endif

`default_nettype wire
