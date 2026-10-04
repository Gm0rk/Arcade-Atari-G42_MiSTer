//============================================================================
//  Atari G42 for MiSTer
//  g42_video_timing.sv -- raster timing generator
//
//  456 x 262 raster, 336 x 240 visible, from a 7.159090 MHz dot enable
//  (MAME set_raw(14.318181 MHz / 2, 456, 0, 336, 262, 0, 240)); the counters
//  and strobes are those of the hardware-proven G1 timing generator.
//
//  One change from G1: vis_x / vis_y describe the pixel of the current dot,
//  the same pixel the registered hblank / vblank / de describe. The blanking
//  outputs are decoded from the counters and registered, so they lag hcnt by
//  one dot; G1 used hcnt itself as vis_x, which showed x = 1..336 while de
//  was high (never pixel 0, and 336 is outside the line buffers). Here vis_x
//  is the previous dot's hcnt, so de = 1 exactly for x = 0..335.
//
//  The sync is centred for a 15 kHz screen (g42_pkg), and VSync starts and
//  ends on HSync leading edges, as in broadcast sync.
//
//  Clock domain: clk_sys, advanced by ce_pix (one pulse per dot).
//============================================================================

`default_nettype none

module g42_video_timing
	import g42_pkg::*;
#(
	// Porch/sync split. MAME's totals come from published specs; the split is
	// not documented (the board's SOS chip), so it is a parameter. The defaults
	// centre the picture for broadcast-calibrated 15 kHz sets (see g42_pkg).
	parameter int P_H_FRONT = H_FRONT,
	parameter int P_H_SYNC  = H_SYNC,
	parameter int P_V_FRONT = V_FRONT,
	parameter int P_V_SYNC  = V_SYNC
)(
	input  wire                 clk,            // clk_sys
	input  wire                 rst_n,
	input  wire                 ce_pix,         // 7.159090 MHz dot enable

	output logic [HCNT_W-1:0]   hcnt,           // 0..455, dot counter
	output logic [VCNT_W-1:0]   vcnt,           // 0..261, line counter

	output logic                hblank,         // registered, describe the pixel
	output logic                vblank,         //   at (vis_x, vis_y)
	output logic                hsync,
	output logic                vsync,
	output logic                de,             // active display area

	output logic                line_start,     // one clk_sys: hcnt wrapped to 0
	output logic                frame_start,    // one clk_sys: vcnt wrapped to 0
	output logic                vblank_rise,    // one clk_sys: entering VBLANK
	output logic                vblank_fall,    // one clk_sys: leaving VBLANK

	output logic [8:0]          vis_x,          // 0..335 while de, else 0
	output logic [7:0]          vis_y           // 0..239 while de, else 0
);

	// Active video first, then front porch, sync, back porch; same vertically.
	localparam int HS_START = H_VISIBLE + P_H_FRONT;
	localparam int HS_END   = HS_START  + P_H_SYNC;
	localparam int VS_START = V_VISIBLE + P_V_FRONT;
	localparam int VS_END   = VS_START  + P_V_SYNC;

	//========================================================================
	//  Raster counters
	//========================================================================
	logic h_wrap, v_wrap;

	always_comb begin
		h_wrap = (hcnt == HCNT_W'(H_TOTAL - 1));
		v_wrap = h_wrap && (vcnt == VCNT_W'(V_TOTAL - 1));
	end

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			hcnt <= '0;
			vcnt <= '0;
		end
		else if (ce_pix) begin
			if (h_wrap) begin
				hcnt <= '0;
				vcnt <= v_wrap ? '0 : (vcnt + 1'b1);
			end
			else begin
				hcnt <= hcnt + 1'b1;
			end
		end
	end

	//========================================================================
	//  Blanking, sync, display enable and the matching pixel position
	//========================================================================
	// Decoded from the counters and registered on ce_pix, so they lag
	// hcnt/vcnt by one dot. The position registers take the same dot's
	// counters, so all of them describe one pixel. The *_n names mean "next".
	//------------------------------------------------------------------------
	logic hblank_n, vblank_n, hsync_n, vsync_n;
	logic [VCNT_W-1:0] hs_line;
	logic [8:0] hpos_d;
	logic [7:0] vpos_d;

	// VSync's edges sit on HSync leading edges, as broadcast sync and the
	// handoff's it8_sync_center put them: hs_line is the line the latest
	// HSync began (HSync comes late in the line before its picture), and
	// VSync covers the three lines HSync begins from VS_START on.
	always_comb begin
		hblank_n = (hcnt >= HCNT_W'(H_VISIBLE));
		vblank_n = (vcnt >= VCNT_W'(V_VISIBLE));
		hsync_n  = (hcnt >= HCNT_W'(HS_START)) && (hcnt < HCNT_W'(HS_END));
		hs_line  = (hcnt < HCNT_W'(HS_START))    ? vcnt
		         : (vcnt == VCNT_W'(V_TOTAL - 1)) ? '0 : vcnt + 1'b1;
		vsync_n  = (hs_line >= VCNT_W'(VS_START)) && (hs_line < VCNT_W'(VS_END));
	end

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			hblank <= 1'b1;
			vblank <= 1'b1;
			hsync  <= 1'b0;
			vsync  <= 1'b0;
			de     <= 1'b0;
			hpos_d <= '0;
			vpos_d <= '0;
		end
		else if (ce_pix) begin
			hblank <= hblank_n;
			vblank <= vblank_n;
			hsync  <= hsync_n;
			vsync  <= vsync_n;
			de     <= !hblank_n && !vblank_n;
			hpos_d <= hcnt[8:0];
			vpos_d <= vcnt[7:0];
		end
	end

	// Forced to 0 in blanking, so a consumer that ignores de reads pixel 0
	// rather than indexing a buffer out of range.
	always_comb begin
		vis_x = hblank ? 9'd0 : hpos_d;
		vis_y = vblank ? 8'd0 : vpos_d;
	end

	//========================================================================
	//  Strobes
	//========================================================================
	// line_start comes one clk_sys after the ce_pix that wrapped hcnt, i.e.
	// during dot 0, one dot before the line's first visible pixel.
	// vblank_rise raises the 68000's IRQ4 and cues the MO end-of-frame erase.
	//------------------------------------------------------------------------
	logic vblank_d;

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			vblank_d    <= 1'b1;
			line_start  <= 1'b0;
			frame_start <= 1'b0;
			vblank_rise <= 1'b0;
			vblank_fall <= 1'b0;
		end
		else begin
			line_start  <= 1'b0;
			frame_start <= 1'b0;
			vblank_rise <= 1'b0;
			vblank_fall <= 1'b0;

			if (ce_pix) begin
				vblank_d    <= vblank_n;
				line_start  <= h_wrap;
				frame_start <= v_wrap;
				vblank_rise <=  vblank_n && !vblank_d;
				vblank_fall <= !vblank_n &&  vblank_d;
			end
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	// Front porch and sync must fit in the blanking period; the back porch is
	// the remainder, so the totals (and 59.923 Hz) never change.
	initial begin
		if (HS_END >= H_TOTAL)
			$fatal(1, "g42_video_timing: horizontal front porch + sync leave no back porch");
		if (VS_END >= V_TOTAL)
			$fatal(1, "g42_video_timing: vertical front porch + sync leave no back porch");
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
