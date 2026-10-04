//============================================================================
//  Atari G42 for MiSTer
//  Arcade-Atari-G42.sv -- top level: MiSTer framework glue
//
//  Road Riot 4WD, Guardians of the 'Hood and Danger Express. The game system
//  itself is g42_core (rtl/g42_core.sv); this module connects it to the
//  framework: hps_io (OSD, ROM download, controllers, NVRAM), the PLL, the
//  SDRAM controller, the video output path and the audio.
//
//  Video: the core's native 15 kHz stream goes through arcade_video (the
//  scandoubler, HQ2x and scanline options, gamma) or, when CRT Adjust is on,
//  through g42_crt_path: rmonic79's crt_adjust (H-Size, H-Position and
//  V-Shift for an analog CRT) with CRT Auto-Width, built to the "CRT Adjust
//  and Auto-Width" porting handoff from Arcade-ITech8 (rtl/crt/
//  g42_crt_path.sv). CRT Adjust is forced off while a scandoubler option is
//  active: its read-rate base assumes the native pixel clock. video_freak sets the aspect ratio and
//  integer scaling for HDMI.
//
//  The module must be named "emu" and use sys/emu_ports.vh; the framework
//  binds to those names.
//
//  Clock domain: clk_sys, 57.272727 MHz; the SDRAM controller runs on
//  clk_ram = 2 x clk_sys from the same PLL.
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

	import g42_pkg::*;

	//========================================================================
	//  Unused framework ports
	//========================================================================
	assign ADC_BUS  = 'Z;
	assign USER_OUT = '1;
	assign {UART_RTS, UART_TXD, UART_DTR} = 0;
	assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
	assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;

	assign VGA_F1         = 0;
	assign VGA_SCALER     = 0;
	assign VGA_DISABLE    = 0;
	assign HDMI_FREEZE    = 0;
	assign HDMI_BLACKOUT  = 0;
	assign HDMI_BOB_DEINT = 0;

	assign LED_DISK  = 0;
	assign LED_POWER = 0;
	assign BUTTONS   = 0;

	//========================================================================
	//  OSD
	//========================================================================
	// Two builds share this file:
	//   Arcade-Atari-G42        release: the video options and the Service
	//                           Menu on the main page, a CRT Adjust page, and
	//                           a Controls page for Road Riot only
	//   Arcade-Atari-G42_debug  the same plus a Debug page; its .qsf defines
	//                           the Verilog macro G42_DEBUG
	//
	// The CRT Adjust page is laid out as the "CRT Adjust and Auto-Width"
	// handoff (Arcade-ITech8) has it, on the same status bits.
	//
	// Hidden entries (status_menumask, the H<n> prefixes):
	//   H1  CRT Auto-Width, H-Size, H-Position, V-Shift: while CRT Adjust is
	//       off
	//   H2  the Controls page: unless the loaded game is Road Riot (its
	//       wheel sensitivity is the only control option). The game comes
	//       from the MRA's configuration bytes, so the page appears once a
	//       Road Riot MRA has loaded.
	//
	// Pages: P1 CRT Adjust, P2 Controls, P3 Debug (debug build).
	//
	// Status bits:
	//   0        reset                    23       Service Menu
	//   4:2      scandoubler fx           24       diagnostic overlay (debug)
	//   7:5      scale                    101      CRT Adjust
	//   10       68000 clock (debug)      102      CRT Auto-Width
	//   11       watchdog (debug)         100:96   CRT H-Size
	//   13:12    wheel sensitivity        85:79    CRT H-Position
	//   16:14    layers off (debug)       78:74    CRT V-Shift
	//   20:17    SDRAM capture (debug)    122:121  aspect ratio
	// The bits keep their places in both builds, so a game's saved settings
	// read the same in either; the release build ignores the debug ones
	// (dstatus below). CRT Auto-Width was "CRT Auto-Fill" on the same bit.
	// build_id.v provides `BUILD_DATE; sys/build_id.tcl writes it before
	// every compile.
	//------------------------------------------------------------------------
	`include "build_id.v"

	// CRT Adjust amounts, as in the handoff: H-Size and V-Shift are 5-bit
	// two's complement (0, +1..+15, -16..-1); the H-Position list is 0..+48
	// then -48..-1 (entry 49 = -48), decoded in g42_crt_path.
	localparam CRT_S5 = "0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1";
	localparam CRT_HP = "0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1";

	localparam CONF_STR = {
		"Atari-G42;;",
		"-;",
		"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
		"O[4:2],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
		"O[7:5],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
		"-;",
		// CRT Adjust (analog 15 kHz output). H1 hides the amounts while it is
		// off. Auto-Width makes the picture 48.75 us wide; H-Size trims from
		// there about the middle of the screen; H-Position and V-Shift move
		// it from the centre.
		"P1,CRT Adjust;",
		"P1-;",
		"P1O[101],CRT Adjust,Off,On;",
		"H1P1O[102],CRT Auto-Width,Off,On;",
		"H1P1O[100:96],CRT H-Size,", CRT_S5, ";",
		"H1P1O[85:79],CRT H-Position,", CRT_HP, ";",
		"H1P1O[78:74],CRT V-Shift,", CRT_S5, ";",
		"-;",
		// The game's own test menu: set On, then reset.
		"O[23],Service Menu,Off,On;",
		// Road Riot's wheel from an analog stick. Medium (100%) is first so
		// it is the default.
		"H2P2,Controls;",
		"P2-;",
		"P2O[13:12],Wheel sensitivity,Medium,High,Low;",
`ifdef G42_DEBUG
		"P3,Debug;",
		"P3-;",
		"P3O[24],Diagnostic overlay,Off,On;",
		"P3O[11],Watchdog,Enabled,Disabled;",
		"P3O[10],68000 clock,14.318MHz,7.159MHz;",
		"P3-;",
		"P3O[14],Motion objects,On,Off;",
		"P3O[15],Playfield,On,Off;",
		"P3O[16],Alphanumerics,On,Off;",
		"P3-;",
		"P3O[20],SDRAM capture,Auto (self-test),Manual;",
		"P3O[18:17],Manual read phase,t8,t9,t10,t7;",
		"P3O[19],Manual sample edge,Falling,Rising;",
`endif
		"-;",
		"T[0],Reset;",
		"R[0],Reset and close OSD;",
		// Overridden by each MRA's <buttons>; Road Riot's list for a bare .rbf
		"J1,Left Trigger,Right Trigger,Start,Coin,Pedal;",
		"jn,A,B,Start,Select,R;",
`ifdef G42_DEBUG
		"V,v",`BUILD_DATE,"-debug"
`else
		"V,v",`BUILD_DATE
`endif
	};

	wire         forced_scandoubler;
	wire  [1:0]  buttons;
	wire [127:0] status;
	wire [21:0]  gamma_bus;
	wire         direct_video;

	wire         ioctl_download, ioctl_upload, ioctl_wr, ioctl_rd;
	wire [26:0]  ioctl_addr;
	wire  [7:0]  ioctl_dout, ioctl_din;
	wire [15:0]  ioctl_index;
	wire         ioctl_wait;
	wire         nvram_dirty;

	wire [31:0]  joystick_0, joystick_1, joystick_2;
	wire [15:0]  joystick_l_analog_0, joystick_r_analog_0;
	wire  [7:0]  paddle_0;

	wire         game_rr;                // from g42_core: Road Riot is loaded (H2)

	hps_io #(.CONF_STR(CONF_STR)) hps_io
	(
		.clk_sys             (clk_sys),
		.HPS_BUS             (HPS_BUS),
		.EXT_BUS             (),
		.gamma_bus           (gamma_bus),

		.forced_scandoubler  (forced_scandoubler),
		.direct_video        (direct_video),
		.video_rotated       (1'b0),
		.new_vmode           (1'b0),

		.buttons             (buttons),
		.status              (status),
		.status_menumask     ({13'd0, ~game_rr, ~status[101], 1'b0}),

		.ioctl_download      (ioctl_download),
		.ioctl_upload        (ioctl_upload),
		.ioctl_upload_req    (nvram_dirty),
		.ioctl_upload_index  (8'd2),
		.ioctl_wr            (ioctl_wr),
		.ioctl_rd            (ioctl_rd),
		.ioctl_addr          (ioctl_addr),
		.ioctl_dout          (ioctl_dout),
		.ioctl_din           (ioctl_din),
		.ioctl_index         (ioctl_index),
		.ioctl_wait          (ioctl_wait),

		.joystick_0          (joystick_0),
		.joystick_1          (joystick_1),
		.joystick_2          (joystick_2),
		.joystick_l_analog_0 (joystick_l_analog_0),
		.joystick_r_analog_0 (joystick_r_analog_0),
		.paddle_0            (paddle_0)
	);

	//========================================================================
	//  Clocks and reset
	//========================================================================
	wire clk_sys, clk_ram, clk_ram_ps, pll_locked;

	pll pll
	(
		.refclk   (CLK_50M),
		.rst      (1'b0),
		.outclk_0 (clk_sys),      //  57.272727 MHz
		.outclk_1 (clk_ram),      // 114.545454 MHz
		.outclk_2 (clk_ram_ps),   // 114.545454 MHz, -90 degrees, to the SDRAM chip
		.locked   (pll_locked)
	);

	assign SDRAM_CLK = clk_ram_ps;

	// g42_core adds the download and self-test terms (see its Resets section)
	wire reset_in = RESET | status[0] | buttons[1];

	//========================================================================
	//  SDRAM controller
	//========================================================================
	wire [24:0] sdr_addr;
	wire [15:0] sdr_din, sdr_dout;
	wire        sdr_rd, sdr_we, sdr_ready;
	wire [2:0]  cap_sel;

	/* verilator lint_off PINCONNECTEMPTY */
	sdram u_sdram
	(
		.SDRAM_DQ          (SDRAM_DQ),
		.SDRAM_A           (SDRAM_A),
		.SDRAM_BA          (SDRAM_BA),
		.SDRAM_DQML        (SDRAM_DQML),
		.SDRAM_DQMH        (SDRAM_DQMH),
		.SDRAM_nCS         (SDRAM_nCS),
		.SDRAM_nRAS        (SDRAM_nRAS),
		.SDRAM_nCAS        (SDRAM_nCAS),
		.SDRAM_nWE         (SDRAM_nWE),
		.SDRAM_CKE         (SDRAM_CKE),
		.init              (~pll_locked),
		.rd_phase          (cap_sel[1:0]),
		.rd_half           (cap_sel[2]),
		.dbg_refresh_count (),
		.dbg_dq7           (),
		.dbg_dq8           (),
		.dbg_dq9           (),
		.dbg_dq10          (),
		.clk               (clk_ram),
		.addr              (sdr_addr),
		.din               (sdr_din),
		.dout              (sdr_dout),
		.rd                (sdr_rd),
		.we                (sdr_we),
		.ready             (sdr_ready)
	);
	/* verilator lint_on PINCONNECTEMPTY */

	//========================================================================
	//  The game system
	//========================================================================
	wire        ce_pix;
	wire [7:0]  vid_r, vid_g, vid_b;
	wire        vid_hblank, vid_vblank, vid_hsync, vid_vsync;
	wire signed [15:0] audio;
	wire        led;

	// Debug options: the status bits in the debug build; zero, which is every
	// option's default, in the release build. A setting left over from the
	// debug build (the two share each game's saved settings) then has no
	// effect, and the overlay's logic is optimised away.
`ifdef G42_DEBUG
	wire [127:0] dstatus = status;
`else
	wire [127:0] dstatus = 128'd0;
`endif

	g42_core #(.BUILD_DATE(`BUILD_DATE)) u_core
	(
		.clk                 (clk_sys),
		.pll_locked          (pll_locked),
		.reset_in            (reset_in),

		.service             (status[23]),
		.cpu_div2            (dstatus[10]),
		.disable_wd          (dstatus[11]),
		.sensitivity         (status[13:12]),
		.layer_off           (dstatus[16:14]),
		// {rd_half, rd_phase}: "Falling" (0) samples half a clock early
		.cap_manual          ({~dstatus[19], dstatus[18:17]}),
		.cap_auto            (~dstatus[20]),
		.dbg_page            ({1'b0, dstatus[24]}),

		.ioctl_download      (ioctl_download),
		.ioctl_upload        (ioctl_upload),
		.ioctl_index         (ioctl_index),
		.ioctl_wr            (ioctl_wr),
		.ioctl_addr          (ioctl_addr),
		.ioctl_dout          (ioctl_dout),
		.ioctl_din           (ioctl_din),
		.ioctl_wait          (ioctl_wait),
		.nvram_dirty         (nvram_dirty),

		.joystick_0          (joystick_0),
		.joystick_1          (joystick_1),
		.joystick_2          (joystick_2),
		.joystick_l_analog_0 (joystick_l_analog_0),
		.joystick_r_analog_0 (joystick_r_analog_0),
		.paddle_0            (paddle_0),

		.sdr_addr            (sdr_addr),
		.sdr_din             (sdr_din),
		.sdr_rd              (sdr_rd),
		.sdr_we              (sdr_we),
		.sdr_dout            (sdr_dout),
		.sdr_ready           (sdr_ready),
		.cap_sel             (cap_sel),

		.ce_pix              (ce_pix),
		.vid_r               (vid_r),
		.vid_g               (vid_g),
		.vid_b               (vid_b),
		.vid_hblank          (vid_hblank),
		.vid_vblank          (vid_vblank),
		.vid_hsync           (vid_hsync),
		.vid_vsync           (vid_vsync),

		.audio               (audio),
		.led                 (led),
		.game_rr             (game_rr)
	);

	//========================================================================
	//  Video: arcade_video path (scandoubler, HQ2x, scanlines, gamma)
	//========================================================================
	wire [7:0] av_r, av_g, av_b;
	wire       av_hs, av_vs, av_de, av_ce;
	wire [1:0] av_sl;
	wire       av_clk;

	arcade_video #(.WIDTH(H_VISIBLE), .DW(24)) u_arcade_video
	(
		.clk_video          (clk_sys),
		.ce_pix             (ce_pix),
		.RGB_in             ({vid_r, vid_g, vid_b}),
		.HBlank             (vid_hblank),
		.VBlank             (vid_vblank),
		.HSync              (vid_hsync),
		.VSync              (vid_vsync),
		.CLK_VIDEO          (av_clk),
		.CE_PIXEL           (av_ce),
		.VGA_R              (av_r),
		.VGA_G              (av_g),
		.VGA_B              (av_b),
		.VGA_HS             (av_hs),
		.VGA_VS             (av_vs),
		.VGA_DE             (av_de),
		.VGA_SL             (av_sl),
		.fx                 (status[4:2]),
		.forced_scandoubler (forced_scandoubler),
		.gamma_bus          (gamma_bus)
	);

	assign CLK_VIDEO = clk_sys;

	//========================================================================
	//  Video: CRT Adjust path (rmonic79/MiSTer-CRT-Adjust, with Auto-Width)
	//========================================================================
	// Off while a scandoubler option is on: the read rate is built for the
	// native 15 kHz pixel clock. See rtl/crt/g42_crt_path.sv.
	wire scandoubler = (status[4:2] != 3'd0) || forced_scandoubler;

	wire       crt_on, crt_ce, crt_hs, crt_vs, crt_de;
	wire [7:0] crt_r, crt_g, crt_b;

	g42_crt_path u_crt_path
	(
		.clk         (clk_sys),
		.ce_pix      (ce_pix),

		.enable      (status[101]),
		.autowidth   (status[102]),
		.hsize       (status[100:96]),
		.hpos        (status[85:79]),
		.vshift      (status[78:74]),
		.scandoubler (scandoubler),

		.r_in        (vid_r),
		.g_in        (vid_g),
		.b_in        (vid_b),
		.hblank      (vid_hblank),
		.vblank      (vid_vblank),
		.hsync       (vid_hsync),
		.vsync       (vid_vsync),

		.active      (crt_on),
		.ce_out      (crt_ce),
		.r_out       (crt_r),
		.g_out       (crt_g),
		.b_out       (crt_b),
		.hs_out      (crt_hs),
		.vs_out      (crt_vs),
		.de_out      (crt_de)
	);

	//========================================================================
	//  Video outputs
	//========================================================================
	assign VGA_R    = crt_on ? crt_r  : av_r;
	assign VGA_G    = crt_on ? crt_g  : av_g;
	assign VGA_B    = crt_on ? crt_b  : av_b;
	assign VGA_HS   = crt_on ? crt_hs : av_hs;
	assign VGA_VS   = crt_on ? crt_vs : av_vs;
	assign VGA_SL   = crt_on ? 2'd0   : av_sl;
	assign CE_PIXEL = crt_on ? crt_ce : av_ce;

	// Aspect ratio and integer scaling for the HDMI scaler
	wire [1:0] ar = status[122:121];

	video_freak u_video_freak
	(
		.CLK_VIDEO   (CLK_VIDEO),
		.CE_PIXEL    (CE_PIXEL),
		.VGA_VS      (VGA_VS),
		.HDMI_WIDTH  (HDMI_WIDTH),
		.HDMI_HEIGHT (HDMI_HEIGHT),
		.VGA_DE      (VGA_DE),
		.VIDEO_ARX   (VIDEO_ARX),
		.VIDEO_ARY   (VIDEO_ARY),
		.VGA_DE_IN   (crt_on ? crt_de : av_de),
		.ARX         ((!ar) ? 12'd4 : {10'd0, ar - 2'd1}),
		.ARY         ((!ar) ? 12'd3 : 12'd0),
		.CROP_SIZE   (12'd0),
		.CROP_OFF    (5'd0),
		.SCALE       (status[7:5])
	);

	//========================================================================
	//  Audio: the JSA III is mono and signed
	//========================================================================
	assign AUDIO_L   = audio;
	assign AUDIO_R   = audio;
	assign AUDIO_S   = 1'b1;
	assign AUDIO_MIX = 2'd0;

	//========================================================================
	//  Activity LED: download, SDRAM self-test and object prescan; a fast
	//  blink afterwards means the SDRAM self-test failed.
	//========================================================================
	assign LED_USER = led;

endmodule
