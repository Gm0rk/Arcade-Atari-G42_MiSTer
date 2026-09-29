//============================================================================
//  Atari G42 for MiSTer
//  g42_mixer.sv -- layer priority and the colour MUX
//
//  MAME atarig42_state::screen_update, with the palette address as the
//  colour-MUX PALs build it (equations in the comments of init_roadriot /
//  init_guardian in atarig42.cpp; CRAn = colour RAM address bit n):
//
//    alpha visible (pen != 0, or the opaque bit)   -> alpha
//    else MO pixel != 0 and MO priority >= PF priority -> motion object
//    else                                          -> playfield
//
//  Palette index of each layer:
//    alpha      {3'b000, colour[3:0], pen[3:0]}                    base $000
//    MO         {CRA10, MO pixel[9:0]}: CRA10 = 1 only for Guardians / Danger
//               Express, i.e. bit 10 of the game's MO base ($400, $200)
//    playfield  {CRA10, PF.VID[9:0]}: CRA10 = 1 only for Road Riot (base
//               $400). PF.VID[12:8] = colour bank (priority = bank[4:2]),
//               [7:5] = tile colour, [4:0] = pen; Road Riot's 6bpp tiles OR
//               pen bit 5 into tile colour bit 0.
//
//  MAME computes the playfield index as 32 * colour + pen: blend_gfx keeps
//  the 5bpp element's granularity (32) although pens are 6 bits, so pen bit
//  5 adds into the tile colour and a sum can carry into bit 10 and beyond.
//  The two forms agree unless tile colour bit 0 and pen bit 5 are both set or
//  the sum passes $7FF; over 1.66 M visible playfield pixels captured from
//  all three games, neither happens (sim/video/README.md). The colour bank's
//  bit 1 is palette bit 9 in both forms: Road Riot shows it (banks 22 and 26
//  select entries $600-$6FF, which the game fills separately from $400-$4FF).
//  PF_MAME_FORM = 1 selects MAME's sum, truncated to 11 bits.
//
//  Clock domain: clk_sys; one registered stage.
//============================================================================

`default_nettype none

module g42_mixer #(
	parameter bit PF_MAME_FORM = 1'b0     // 0: colour-MUX PAL form, 1: MAME's 32 * colour + pen
)(
	input  wire         clk,
	input  wire         de,                 // active display area

	input  wire  [2:0]  pf_base,            // playfield palette base bits 10:8 (cfg.pf_base[2:0])
	input  wire         mo_base10,          // MO palette base bit 10 (cfg.mo_base[2])

	// ---- Layer enables (OSD / debug; 1 = shown) ---------------------------
	input  wire         pf_enable,          // off: entry 0, and MOs ignore priority
	input  wire         mo_enable,          // off: MO transparent
	input  wire         al_enable,          // off: alpha transparent

	// ---- Playfield --------------------------------------------------------
	input  wire  [5:0]  pf_pen,
	input  wire  [2:0]  pf_color,           // tile colour
	input  wire  [4:0]  pf_bank,            // colour bank of the line

	// ---- Motion objects (framebuffer: {priority[2:0], index[9:0]}, 0 = none)
	input  wire  [12:0] mo_pixel,

	// ---- Alpha ------------------------------------------------------------
	input  wire  [3:0]  al_pen,
	input  wire  [3:0]  al_color,
	input  wire         al_opaque,

	output logic [10:0] pal_index           // registered
);

	wire [2:0] pf_pri = pf_enable ? pf_bank[4:2] : 3'd0;

	logic [10:0] pf_idx;
	always_comb begin
		if (PF_MAME_FORM)
			pf_idx = {pf_base, 8'd0} + {1'b0, pf_bank[1:0], 8'd0}
			       + {3'd0, pf_color, 5'd0} + {5'd0, pf_pen};
		else
			pf_idx = {pf_base[2], pf_bank[1:0], pf_color | {2'b00, pf_pen[5]}, pf_pen[4:0]};
		if (!pf_enable)
			pf_idx = 11'd0;
	end

	wire al_visible = al_enable && ((al_pen != 4'd0) || al_opaque);
	wire mo_visible = mo_enable && (mo_pixel != 13'd0) && (mo_pixel[12:10] >= pf_pri);

	always_ff @(posedge clk) begin
		if (!de)
			// Entry 0 outside the active area, as G1: a scaler sampling in
			// blanking must not see a stale colour.
			pal_index <= 11'd0;
		else if (al_visible)
			pal_index <= {3'b000, al_color, al_pen};
		else if (mo_visible)
			pal_index <= {mo_base10, mo_pixel[9:0]};
		else
			pal_index <= pf_idx;
	end

endmodule

`default_nettype wire
