//============================================================================
//  Atari G42 for MiSTer
//  g42_crt_path.sv -- the CRT Adjust output path (analog 15 kHz)
//
//  rmonic79's crt_adjust (rtl/crt/crt_adjust.sv) with its glue, built to the
//  "CRT Adjust and Auto-Width: Porting Handoff" (Arcade-ITech8, Oct 2026).
//  In ITech8 the glue is in the top level; here it is a module so
//  sim/timing can test it. The parts, in the handoff's order:
//
//  * Centred sync. The handoff's it8_sync_center measures where a board's
//    picture is and makes new sync around it. The G42 raster is fixed, so
//    g42_video_timing makes the centred sync itself (g42_pkg: HSync 4.75 us,
//    its leading edge 35.75 us before the picture's middle; VSync 3 lines,
//    138 lines before the middle of the active lines, its edges on HSync
//    leading edges). What the sync module gives CRT Adjust besides is made
//    here: VSync a line late ("vlate") and VBLANK for the line each HSync
//    begins (its "vb_out").
//  * CRT Adjust glue: H-Size grows the picture about the screen's middle,
//    less the dot crt_adjust's read pipeline adds; the read clock restarts
//    on hs_ref_out; the OSD/HDMI window covers both pictures.
//  * The negative H-Position fix is in crt_adjust.sv itself.
//  * CRT Auto-Width: the H-Size that makes the picture 48.75 us wide; the
//    OSD's H-Size trims from there.
//
//  With every amount at 0 the picture is exactly where it is with CRT
//  Adjust off. sim/timing/tb_crt.cpp checks the handoff's list.
//
//  Per-core constants (the handoff's porting checklist), for 7.159 MHz dots
//  and this path's clock, clk_sys at 57.272727 MHz (8 clocks a dot):
//    base (RD_BASE)        4 x 57.27 / 7.159 = 32 quarter clocks a dot
//    Auto-Width constant   48.75 us x 4 x 57.27 MHz = 11168
//    centring middle m     35.75 us x 7.159 MHz = 256 dots
//    crt_adjust HTOTAL     456 dots, VTOTAL 262 lines
//
//  Different from Arcade-ITech8, because of this core's clock and line:
//    * Each H-Size step is 1/32 of the width (1/48 there, from its 96 MHz
//      video clock), so Auto-Width lands on +1, 48.40 us.
//    * The H-Size in use is kept to -16..+5, not -16..+31: +5 (54.3 us) is
//      the widest picture that fits this line, held about its middle.
//    * The output is black while HSync is high, so no H-Size and H-Position
//      combination can put picture on the sync pulse.
//
//  CRT Adjust is forced off while a scandoubler option is on: the read rate
//  is built on the native 15 kHz pixel clock.
//
//  Clock domain: clk_sys (57.272727 MHz); ce_pix is the 7.159 MHz dot.
//============================================================================

`default_nettype none

module g42_crt_path
	import g42_pkg::*;
(
	input  wire        clk,                // clk_sys
	input  wire        ce_pix,             // native dot enable, one in 8 clocks

	// OSD settings, as the status bits hold them
	input  wire        enable,             // CRT Adjust: On
	input  wire        autowidth,          // CRT Auto-Width: On
	input  wire  [4:0] hsize,              // H-Size: 5-bit two's complement, -16..+15
	input  wire  [6:0] hpos,               // H-Position option: 0..48 = +0..+48, 49..96 = -48..-1
	input  wire  [4:0] vshift,             // V-Shift: 5-bit two's complement, -16..+15
	input  wire        scandoubler,        // a Scandoubler Fx option, or forced

	// Native video from g42_core, registered on ce_pix; the sync is already
	// centred (g42_video_timing)
	input  wire  [7:0] r_in,
	input  wire  [7:0] g_in,
	input  wire  [7:0] b_in,
	input  wire        hblank,
	input  wire        vblank,
	input  wire        hsync,
	input  wire        vsync,

	// Adjusted video: the top level selects these while active is high
	output logic       active,             // CRT Adjust in use
	output wire        ce_out,             // CE_PIXEL: the read rate
	output wire  [7:0] r_out,
	output wire  [7:0] g_out,
	output wire  [7:0] b_out,
	output wire        hs_out,
	output wire        vs_out,
	output logic       de_out              // VGA_DE for video_freak (HDMI, OSD)
);

	//========================================================================
	//  Settings, latched on ce_pix (handoff step 5)
	//========================================================================
	logic              width_on;
	logic signed [4:0] hsize_s;
	logic signed [5:0] vshift_s;
	logic        [6:0] hpos_d;

	always_ff @(posedge clk) begin
		if (ce_pix) begin
			active   <= enable && !scandoubler;
			width_on <= autowidth;
			hsize_s  <= $signed(hsize);
			vshift_s <= $signed({vshift[4], vshift});
			hpos_d   <= hpos;
		end
	end

	// H-Position option list: 0, +1..+48, then -48..-1 (entry 49 = -48)
	wire signed [10:0] hpos_usr = (hpos_d <= 7'd48) ? $signed({4'd0, hpos_d})
	                                                : $signed({4'd0, hpos_d}) - 11'sd97;

	//========================================================================
	//  CRT Auto-Width and the H-Size in use (handoff step 7)
	//========================================================================
	// crt_adjust reads each source dot every (base + H-Size) quarter clocks:
	// a native dot is 8 clk_sys, 32 quarters (RD_BASE), and each H-Size step
	// is 1/32 of the width.
	localparam int RD_BASE = 32;

	// The read period for a 48.75 us picture, about 93 % of a 52.66 us
	// broadcast line (SMPTE's safe action area), as the handoff works it out:
	// P = round(48.75 us x 4 x clk MHz / pic_dots). Its constant, 48.75 x 4 x
	// 57.272727 (= 630/11 MHz), is 491400/44 = 11168.2: AW_CONST = 11168.
	// ITech8 divides by its measured pic_dots with a restoring divider; the
	// G42 picture is always 336 dots, so the division is done here, at
	// elaboration: (11168 + 168) / 336 = 33, H-Size +1 (48.40 us; +2 would be
	// 49.87 us). A different target T (us) is AW_CONST = T x 4 x 630/11.
	localparam int AW_CONST = (195 * 2520 + 22) / 44;                          // 11168
	localparam int AW_PER   = (AW_CONST + H_VISIBLE / 2) / H_VISIBLE;           // 33
	localparam int H_AUTO   = AW_PER - RD_BASE;                                 // +1

	// Auto-Width's amount, if on, plus the OSD's. The handoff keeps the sum to
	// -16..+31 (steps of 1/48 on a 510-dot line there); here it is kept to
	// -16..+5: held about its middle (256 dots after HSync), the +5 picture
	// ends 5.75 dots before the next HSync, while +6 would reach it and +7
	// run through it. Larger OSD values act as +5.
	localparam int H_MIN = -16;
	localparam int H_MAX = 5;

	wire signed [7:0] h_sum = (width_on ? 8'(H_AUTO) : 8'sd0) + {{3{hsize_s[4]}}, hsize_s};
	logic signed [6:0] h_eff;
	always_ff @(posedge clk)
		h_eff <= (h_sum < 8'(H_MIN)) ? 7'(H_MIN) : (h_sum > 8'(H_MAX)) ? 7'(H_MAX) : h_sum[6:0];

	//========================================================================
	//  H-Size about the middle of the screen (handoff step 8)
	//========================================================================
	// crt_adjust stretches the line from HSync, which would carry the
	// picture's middle right as it grows. HMID holds it: -m x H / (base + H)
	// dots for each H-Size H, m the picture's middle after HSync (35.75 us
	// x 7.159 MHz = 256 dots), base 32, rounded; the handoff's generator,
	// hmid(256, 32), for H -16..+5. Indexed by H + 16, H-Size +5 first. The
	// amounts reach +256 at H = -16, so 10 bits (9 in ITech8, whose base is
	// 48). If g42_pkg's porches change, m and the table change with them.
	localparam [219:0] HMID = {     // H-Size +5 .. -16
		 -10'sd35,  -10'sd28,  -10'sd22,  -10'sd15,   -10'sd8,    10'sd0,
		   10'sd8,   10'sd17,   10'sd26,   10'sd37,   10'sd47,   10'sd59,   10'sd72,   10'sd85,
		 10'sd100,  10'sd116,  10'sd134,  10'sd154,  10'sd175,  10'sd199,  10'sd226,  10'sd256
	};

	// hpos_off = hpos_usr + HMID[h_eff + 16] - 1: the -1 makes up for the dot
	// crt_adjust's read pipeline adds. crt_adjust takes 9 bits (-256..+255);
	// only H-Size -16 or -15 with a positive H-Position reaches past that,
	// and is clamped there.
	wire        [5:0] h_idx = 6'(h_eff + 7'sd16);
	logic signed [9:0] hmid;
	logic signed [8:0] hpos_off;
	wire  signed [10:0] off_sum = hpos_usr + {hmid[9], hmid} - 11'sd1;

	always_ff @(posedge clk) begin
		hmid     <= $signed(HMID[h_idx * 10 +: 10]);
		hpos_off <= (off_sum < -11'sd256) ? -9'sd256 : (off_sum > 11'sd255) ? 9'sd255 : off_sum[8:0];
	end

	//========================================================================
	//  Read clock (handoff step 6)
	//========================================================================
	// One dot every (base + h_eff) quarter clocks: an accumulator in steps of
	// 4, reset on the rise of crt_adjust's hs_ref_out, never the raw HSync, so
	// the width does not drift line to line.
	wire  hs_ref;
	logic hs_ref_d;
	always_ff @(posedge clk) hs_ref_d <= hs_ref;
	wire  hs_ref_rise = hs_ref & ~hs_ref_d;

	wire  [7:0] rd_period = 8'(RD_BASE) + {h_eff[6], h_eff};   // 16..37
	logic [7:0] rd_acc;
	wire        rd_tick = (rd_acc + 8'd4) >= rd_period;

	always_ff @(posedge clk) begin
		if      (hs_ref_rise) rd_acc <= 8'd0;
		else if (rd_tick)     rd_acc <= rd_acc + 8'd4 - rd_period;
		else                  rd_acc <= rd_acc + 8'd4;
	end

	assign ce_out = active ? rd_tick : ce_pix;

	//========================================================================
	//  What the handoff's sync module gives CRT Adjust (vlate, vb_out)
	//========================================================================
	// VSync one line late: crt_adjust sends the picture a line late, so
	// VSync follows it (the sync module's vlate, tied to CRT Adjust on). The
	// native VSync's edges are on HSync leading edges; c_vs takes, at each
	// HSync leading edge, the VSync of the dot before it, so it changes one
	// line later, on HSync edges too.
	//
	// VBLANK for the line each HSync begins (the sync module's vb_out):
	// crt_adjust samples vb_in at HSync, which comes late in the line before
	// its picture, so plain VBLANK there is the previous line's and the first
	// picture line was lost. vline counts raster lines, cleared where VBLANK
	// ends (line 0, dot 0) and advanced at each line's first dot; at HSync,
	// late in line vline, the line it begins is in VBLANK when vline is
	// 239..260.
	logic       hs_d, vs_d, vs_late, hblank_d, vblank_d;
	logic [8:0] vline;                      // the current raster line, 0..261

	wire hs_rise = hsync && !hs_d;          // during HSync's first dot

	always_ff @(posedge clk) begin
		if (ce_pix) begin
			hs_d     <= hsync;
			vs_d     <= vsync;
			hblank_d <= hblank;
			vblank_d <= vblank;
			if (hs_rise) vs_late <= vs_d;
			if (!vblank && vblank_d)
				vline <= 9'd0;
			else if (!hblank && hblank_d)
				vline <= (vline == 9'(V_TOTAL - 1)) ? 9'd0 : vline + 9'd1;
		end
	end

	wire c_vs = hs_rise ? vs_d : vs_late;
	wire c_vb = (vline >= 9'(V_VISIBLE - 1)) && (vline <= 9'(V_TOTAL - 2));

	//========================================================================
	//  crt_adjust (handoff step 4)
	//========================================================================
	// CONTENTSHIFT: H-Position moves the content and HSync stays native. The
	// module's hsize input is documentation only (the read rate comes in on
	// pxl2_cen), so it gets the OSD's value.
	wire [7:0] mod_r, mod_g, mod_b;
	wire       hb_mod;

	/* verilator lint_off PINCONNECTEMPTY */
	crt_adjust #(
		.VTOTAL    (V_TOTAL),
		.HTOTAL    (H_TOTAL),
		.HPOS_MODE (1)
	) u_crt_adjust (
		.clk        (clk),
		.pxl_cen    (ce_pix),
		.pxl2_cen   (ce_out),
		.active     (active),
		.hsize      (hsize_s),
		.hoffset    (hpos_off),
		.voffset    (vshift_s),
		.r_in       (r_in),
		.g_in       (g_in),
		.b_in       (b_in),
		.hs_in      (hsync),
		.vs_in      (c_vs),
		.hb_in      (hblank | vblank),
		.vb_in      (c_vb),
		.r_out      (mod_r),
		.g_out      (mod_g),
		.b_out      (mod_b),
		.hs_out     (hs_out),
		.vs_out     (vs_out),
		.hb_out     (hb_mod),
		.vb_out     (),
		.hs_ref_out (hs_ref)
	);
	/* verilator lint_on PINCONNECTEMPTY */

	// Black while HSync is high (a G42 addition): a large negative H-Position
	// with a wide picture would otherwise start it inside the sync pulse.
	assign r_out = hs_out ? 8'd0 : mod_r;
	assign g_out = hs_out ? 8'd0 : mod_g;
	assign b_out = hs_out ? 8'd0 : mod_b;

	//========================================================================
	//  OSD/HDMI window (handoff step 9)
	//========================================================================
	// The union of the native active area (VBLANK a line late, as the
	// picture leaves the line buffer a line late; sampled as each line's
	// active part ends) and CRT Adjust's own (~hb_out), so HDMI and the OSD
	// keep the whole picture whichever way H-Size or H-Position moves it.
	//
	// crt_adjust's output runs 9 clocks behind its input (a register on
	// pxl_cen, then one on pxl2_cen: hs_in to hs_out). The native area goes
	// through the same two registers, so that with every amount at 0 the
	// window is exactly the picture, dot for dot, and moves with it; taken
	// straight from the input it opened one dot early, a black column at the
	// left of the HDMI picture.
	logic vblank_1l, nat_q, nat_out;
	always_ff @(posedge clk)
		if (ce_pix && hblank && !hblank_d) vblank_1l <= vblank;

	wire native_active = ~(hblank | vblank_1l);
	wire str_active    = ~hb_mod;

	always_ff @(posedge clk) begin
		if (ce_pix) nat_q   <= native_active;
		if (ce_out) nat_out <= nat_q;
		de_out <= nat_out | str_active;
	end

endmodule

`default_nettype wire
