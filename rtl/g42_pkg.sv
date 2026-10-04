//============================================================================
//  Atari G42 for MiSTer
//  g42_pkg.sv -- shared constants, types and SDRAM map
//
//  Everything more than one module has to agree on: raster timing, palette
//  and framebuffer geometry, the SDRAM region map, the RLE encoding modes and
//  the MRA configuration bytes. Per-game behaviour comes from those bytes at
//  runtime (g42_cfg_t), so one .rbf runs all three games.
//============================================================================

`ifndef G42_PKG_SV
`define G42_PKG_SV

package g42_pkg;

	//------------------------------------------------------------------------
	// Raster timing
	//------------------------------------------------------------------------
	// MAME set_raw(14.318181 MHz / 2, 456, 0, 336, 262, 0, 240): 7.159 MHz dot
	// clock (139.7 ns), 15.700 kHz line, 59.923 Hz frame. The totals are
	// published specs. Where the sync sits inside the blanking is not: the
	// board's SOS chip is undocumented, and an arcade monitor is centred with
	// its own H/V position controls anyway. So the sync is placed as the
	// "CRT Adjust and Auto-Width" handoff (Arcade-ITech8's it8_sync_center)
	// places it, the broadcast placement (SMPTE 170M) a TV or PVM on the
	// analog output is set up for:
	//
	//   Horizontal: the HSync leading edge 35.75 us before the middle of the
	//   picture (4.7 us sync + 4.7 us back porch + half of a 52.7 us picture),
	//   HSync 4.75 us wide. In 7.159 MHz dots: middle 256 dots after the edge
	//   (35.76 us, a twentieth of a dot off), sync 34 dots. The 336-dot picture
	//   (46.9 us) then has 32 dots front porch, 34 sync and 54 back porch. The
	//   handoff's sync module measures the picture to find the middle; here
	//   the raster is fixed, so the split is the result. (The G1 split,
	//   16/36/68, put the picture 16 dots, 2.2 us, right of centre.)
	//
	//   Vertical: the VSync leading edge 138 lines before the middle of the
	//   active lines (3 sync + 15 back porch + 120), VSync 3 lines, its edges
	//   on HSync leading edges (see g42_video_timing). Here: 4 front, 3 sync,
	//   15 back. (The G1 split, 6/3/13, sat 2 lines high.)
	//
	// Only VGA_HS/VGA_VS move. Game timing (VBLANK at line 240, the IRQ, the
	// MO erase), DE, HDMI and the OSD do not depend on the split. CRT Adjust
	// (OSD) works outward from this centre; sim/timing measures both.
	//------------------------------------------------------------------------
	localparam int H_TOTAL   = 456;
	localparam int H_VISIBLE = 336;
	localparam int H_FRONT   = 32;
	localparam int H_SYNC    = 34;
	localparam int H_BACK    = H_TOTAL - H_VISIBLE - H_FRONT - H_SYNC;   // 54

	localparam int V_TOTAL   = 262;
	localparam int V_VISIBLE = 240;
	localparam int V_FRONT   = 4;
	localparam int V_SYNC    = 3;
	localparam int V_BACK    = V_TOTAL - V_VISIBLE - V_FRONT - V_SYNC;   // 15

	localparam int HCNT_W    = 9;
	localparam int VCNT_W    = 9;

	//------------------------------------------------------------------------
	// Palette: 2048 entries of IRGB-1555 at $FC0000-$FC0FFF.
	//------------------------------------------------------------------------
	localparam int PAL_ENTRIES = 2048;
	localparam int PAL_AW      = 11;

	//------------------------------------------------------------------------
	// Motion object framebuffer: 336 x 240, double buffered, 13 bits a pixel:
	//     [12:10]  object priority (MVID9-11), compared with the playfield's
	//     [9:0]    palette index bits 9:0 of the object pixel; bit 10 comes
	//              from the game's MO palette base ($200 or $400)
	// 0 means no object pixel: every drawn pixel has a non-zero index.
	//------------------------------------------------------------------------
	localparam int MO_FB_W      = H_VISIBLE;
	localparam int MO_FB_H      = V_VISIBLE;
	localparam int MO_FB_BPP    = 13;
	localparam int MO_FB_PIXELS = MO_FB_W * MO_FB_H;    // 80,640
	localparam int MO_FB_AW     = 17;

	//------------------------------------------------------------------------
	// Build number, shown as BLD on the diagnostic overlay so a screenshot can
	// be tied to its sources. Incremented with every set of changed files.
	//------------------------------------------------------------------------
	localparam [15:0] G42_BUILD = 16'd1;

	//------------------------------------------------------------------------
	// SDRAM region map (byte addresses). One layout for every game; the MRA
	// zero-fills what a game does not use. The object ROM is last because its
	// size varies (2, 6 or 8 MB), so no set pays for padding after it.
	//------------------------------------------------------------------------
	localparam [24:0] SDR_PROG  = 25'h000000;   // 512 KB  68000 program
	localparam [24:0] SDR_OKI   = 25'h080000;   // 512 KB  OKI6295 samples
	localparam [24:0] SDR_ALPHA = 25'h100000;   // 128 KB  alphanumerics
	localparam [24:0] SDR_JSA   = 25'h120000;   //  64 KB  JSA III 6502 program
	localparam [24:0] SDR_EEP   = 25'h130000;   //   2 KB  EEPROM factory image
	localparam [24:0] SDR_DSP   = 25'h131000;   //   8 KB  ASIC65 program (Road Riot)
	localparam [24:0] SDR_PFA   = 25'h140000;   // 512 KB  playfield, first third
	localparam [24:0] SDR_PFB   = 25'h1C0000;   // 512 KB  playfield, second third
	localparam [24:0] SDR_PFC   = 25'h240000;   // 512 KB  playfield, last third
	localparam [24:0] SDR_RLE   = 25'h2C0000;   // <= 8 MB RLE motion objects

	localparam [24:0] SDR_EEP_END = SDR_EEP + 25'h000800;
	localparam [24:0] SDR_DSP_END = SDR_DSP + 25'h002000;

	//------------------------------------------------------------------------
	// RLE encoding modes: object header word 2, bits 10:8. MAME's lookup
	// tables reduce to bit slices (g42_rle_decode). All three G42 ROM sets use
	// only modes 0, 2 and 5; the special modes are there for completeness.
	//
	//   mode  bpp  special
	//     0    4     no      value = b[3:0], run = b[7:4]+1
	//     1    5    yes      low nibble 0 -> 4bpp form (long transparent run)
	//     2    5     no      value = b[4:0], run = b[7:5]+1
	//     3    5     no      (as 2)
	//     4    6    yes
	//     5    6     no      value = b[5:0], run = b[7:6]+1
	//     6    6    yes      (as 4)
	//     7    6     no      (as 5)
	//------------------------------------------------------------------------
	function automatic [2:0] rle_mode_bpp(input [2:0] mode);
		case (mode)
			3'd0:           rle_mode_bpp = 3'd4;
			3'd1,3'd2,3'd3: rle_mode_bpp = 3'd5;
			default:        rle_mode_bpp = 3'd6;
		endcase
	endfunction

	function automatic rle_mode_special(input [2:0] mode);
		rle_mode_special = (mode == 3'd1) || (mode == 3'd4) || (mode == 3'd6);
	endfunction

	//------------------------------------------------------------------------
	// MRA configuration bytes (ioctl_index 1)
	//------------------------------------------------------------------------
	localparam [7:0] GAME_ROADRIOT = 8'd0;
	localparam [7:0] GAME_DANGEREX = 8'd1;
	localparam [7:0] GAME_GUARDIAN = 8'd2;

	localparam [7:0] SLOOP_NONE     = 8'd0;
	localparam [7:0] SLOOP_ROADRIOT = 8'd1;
	localparam [7:0] SLOOP_GUARDIAN = 8'd2;

	typedef struct packed {
		logic [7:0] game_id;        // GAME_*
		logic [7:0] sloop;          // SLOOP_*: which bank controller is fitted
		logic [7:0] flags;          // [0] ADC0809 fitted (Road Riot)
		                            // [1] ASIC65 is ROM-based: run the TMS32010
		                            //     (Road Riot); else high-level (Guardians)
		                            // [2] RTS at $080000 (Guardians, as MAME)
		                            // [3] 32768 playfield tiles (else 16384)
		logic [7:0] mo_base;        // MO palette base >> 8: $02 or $04
		logic [7:0] pf_base;        // playfield palette base >> 8: $04 or $00
		logic [7:0] mo_color_mask;  // MO colour field width: $1F or $3F
		logic [7:0] obj_count_h;    // RLE objects in the ROM (MAME
		logic [7:0] obj_count_l;    //   count_objects), high and low byte
	} g42_cfg_t;

	localparam int CFG_BYTES = 8;

endpackage

`endif
