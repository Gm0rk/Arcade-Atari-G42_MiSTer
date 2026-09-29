//============================================================================
//  Atari G42 for MiSTer
//  g42_core.sv -- the whole game system, between the MiSTer framework glue
//                 (Arcade-Atari-G42.sv) and the SDRAM controller
//
//  Everything that is the arcade board lives here, so the same module runs
//  in the Verilator full-system simulation (sim/system) with an SDRAM model
//  in place of sdram.sv:
//      g42_rom_loader       MRA download, configuration, EEPROM/DSP copies
//      g42_sdram_arb        one SDRAM port shared by five requesters
//      g42_sdram_selftest   capture sweep and readback check after a load
//      g42_top              68000 board: CPU, memory map, SLOOP, ASIC65
//      g42_jsa3             JSA III sound board
//      g42_video            timing, playfield, alpha, mixer
//      g42_rle              motion object engine
//      g42_palette          2048-entry palette
//      g42_controls / g42_adc0809   Road Riot wheel and pedal
//      g42_dbg_text         diagnostic overlay
//
//  Resets:
//    rst_n_sys  the object engine: held during any download, the SDRAM
//               self-test and a framework reset
//    rst_n      the game (CPUs, sound, board, video): as rst_n_sys, and also
//               until the object prescan after a ROM download has finished
//    rst_n_mem  the loader, arbiter and self-test path: must run during a
//               download, which writes through them
//    dbg_clear  PLL unlock only: probes that must survive every download
//
//  Clock domain: clk_sys, 57.272727 MHz.
//============================================================================

`default_nettype none

module g42_core
	import g42_pkg::*;
#(
	parameter bit SELFTEST_FULL = 1'b1,      // 0: sweep only (simulation)
	parameter bit ROM_CACHE     = 1'b1,      // 68000 program ROM cache (g42_top)
	parameter int ROM_CACHE_BITS = 13,       // its size, 2^n words: 13 = 16 KB (22 M10K), 12 = 8 KB
	parameter [47:0] BUILD_DATE = "000000"   // "YYMMDD" from build_id.v
)(
	input  wire         clk,                 // clk_sys
	input  wire         pll_locked,
	input  wire         reset_in,            // framework reset, OSD reset, user button

	// ---- OSD options ------------------------------------------------------------
	input  wire         service,             // Service Menu: test switch on
	input  wire         cpu_div2,            // Debug: 68000 at 7.159 MHz
	input  wire         disable_wd,          // Debug: watchdog off
	input  wire [1:0]   sensitivity,         // Controls: wheel sensitivity
	input  wire [2:0]   layer_off,           // Debug: {alpha, playfield, objects} hidden
	input  wire [2:0]   cap_manual,          // Debug: SDRAM capture {rd_half, rd_phase}
	input  wire         cap_auto,            // Debug: 1 = capture from the self-test
	input  wire [1:0]   dbg_page,            // Debug: 0 off, 1 overlay panel

	// ---- ioctl (hps_io) -------------------------------------------------------------
	input  wire         ioctl_download,
	input  wire         ioctl_upload,
	input  wire [15:0]  ioctl_index,
	input  wire         ioctl_wr,
	input  wire [26:0]  ioctl_addr,
	input  wire [7:0]   ioctl_dout,
	output logic [7:0]  ioctl_din,
	output logic        ioctl_wait,
	output logic        nvram_dirty,         // -> ioctl_upload_req

	// ---- Controls -------------------------------------------------------------------
	input  wire [31:0]  joystick_0,
	input  wire [31:0]  joystick_1,
	input  wire [31:0]  joystick_2,
	input  wire [15:0]  joystick_l_analog_0,
	input  wire [15:0]  joystick_r_analog_0,
	input  wire [7:0]   paddle_0,

	// ---- SDRAM controller port (sdram.sv) ---------------------------------------------
	output logic [24:0] sdr_addr,
	output logic [15:0] sdr_din,
	output logic        sdr_rd,
	output logic        sdr_we,
	input  wire  [15:0] sdr_dout,
	input  wire         sdr_ready,
	output logic [2:0]  cap_sel,             // {rd_half, rd_phase} for the controller

	// ---- Video ---------------------------------------------------------------------------
	output logic        ce_pix,
	output logic [7:0]  vid_r,
	output logic [7:0]  vid_g,
	output logic [7:0]  vid_b,
	output logic        vid_hblank,
	output logic        vid_vblank,
	output logic        vid_hsync,
	output logic        vid_vsync,

	// ---- Audio -------------------------------------------------------------------------------
	output logic signed [15:0] audio,

	// ---- Status --------------------------------------------------------------------------
	output logic        led
);

	//========================================================================
	//  Resets
	//========================================================================
	wire st_busy;

	// rst_n_sys: the object engine, which must run the object prescan before
	// the game starts. rst_n: the game itself and the video, also held until
	// that prescan has finished, so the game boots with its objects ready, as
	// the MAME driver does (its prescan runs at start-up). The video waits
	// too, which leaves the prescan the whole SDRAM bus (0.2-0.6 s).
	logic ps_wait;
	wire rst_n_sys = !(reset_in || ioctl_download || !pll_locked || st_busy);
	wire rst_n     = rst_n_sys && !ps_wait;
	// The framework asserts its reset during a download: without the
	// ioctl_download term the loader would be held and ioctl_wait never raised.
	wire rst_n_mem = !(reset_in || !pll_locked) || ioctl_download;
	wire dbg_clear = !pll_locked;

	//========================================================================
	//  Clock enables (free running, so the timing never stops)
	//========================================================================
	wire ce_cpu_p1, ce_cpu_p2, ce_ym, ce_6502, ce_oki;

	g42_ce u_ce (
		.clk       (clk),
		.rst_n     (pll_locked),
		.cpu_div2  (cpu_div2),
		.ce_pix    (ce_pix),
		.ce_cpu_p1 (ce_cpu_p1),
		.ce_cpu_p2 (ce_cpu_p2),
		.ce_ym     (ce_ym),
		.ce_6502   (ce_6502),
		.ce_oki    (ce_oki)
	);

	//========================================================================
	//  ROM loading
	//========================================================================
	g42_cfg_t   cfg;
	wire        cfg_valid, rom_loading, rom_loaded;
	wire [24:0] ldr_addr;
	wire [15:0] ldr_din;
	wire        ldr_we, ldr_ack;
	wire        eep_def_we, dsp_prog_we;
	wire [10:0] eep_def_addr;
	wire [7:0]  eep_def_din;
	wire [11:0] dsp_prog_addr;
	wire [15:0] dsp_prog_data;

	g42_rom_loader u_loader (
		.clk            (clk),
		.rst_n          (rst_n_mem),
		.ioctl_download (ioctl_download),
		.ioctl_index    (ioctl_index),
		.ioctl_wr       (ioctl_wr),
		.ioctl_addr     (ioctl_addr),
		.ioctl_dout     (ioctl_dout),
		.ioctl_wait     (ioctl_wait),
		.sdr_addr       (ldr_addr),
		.sdr_din        (ldr_din),
		.sdr_we         (ldr_we),
		.sdr_ack        (ldr_ack),
		.eep_we         (eep_def_we),
		.eep_addr       (eep_def_addr),
		.eep_din        (eep_def_din),
		.dsp_we         (dsp_prog_we),
		.dsp_addr       (dsp_prog_addr),
		.dsp_din        (dsp_prog_data),
		.cfg            (cfg),
		.cfg_valid      (cfg_valid),
		.rom_loading    (rom_loading),
		.rom_loaded     (rom_loaded)
	);

	// One pulse when a ROM download completes. It comes while the consumers
	// may still be held in reset, so it is latched until each one starts.
	logic rom_loaded_d;
	always_ff @(posedge clk) rom_loaded_d <= rom_loaded;
	wire load_complete = rom_loaded && !rom_loaded_d;

	// The object prescan needs the object count from the configuration,
	// which the MRA sends after the ROM image: cfg_fresh says it has arrived
	// since the last ROM download (cfg_valid may still be the previous
	// game's). Between the two downloads the game briefly leaves reset.
	logic cfg_dl_d, cfg_fresh;
	wire  cfg_dl = ioctl_download && (ioctl_index[5:0] == 6'd1);
	always_ff @(posedge clk) begin
		cfg_dl_d <= cfg_dl;
		if (dbg_clear || rom_loading)  cfg_fresh <= 1'b0;
		else if (cfg_dl_d && !cfg_dl)  cfg_fresh <= 1'b1;
	end

	// ps_req asks the object engine for a prescan; it is withdrawn as soon as
	// the engine is busy (a request still up when the prescan ends would start
	// a second one). ps_wait holds the game in reset until a prescan has
	// completed: busy fell while the engine was not being reset. One aborted
	// by a reset (the NVRAM download that may follow) is requested again.
	wire  prescan_busy;
	logic st_pending, ps_req, ps_busy_d, rst_n_d;
	always_ff @(posedge clk) begin
		ps_busy_d <= prescan_busy;
		rst_n_d   <= rst_n_sys;
		if (dbg_clear) begin
			st_pending <= 1'b0;
			ps_req     <= 1'b0;
			ps_wait    <= 1'b0;
		end else begin
			if (load_complete) st_pending <= 1'b1;
			else if (st_busy)  st_pending <= 1'b0;

			if (load_complete) begin
				ps_req  <= 1'b1;
				ps_wait <= 1'b1;
			end else begin
				if (prescan_busy) ps_req <= 1'b0;
				if (ps_busy_d && !prescan_busy) begin
					if (rst_n_d) ps_wait <= 1'b0;
					else         ps_req  <= 1'b1;
				end
			end
		end
	end

	wire [7:0] game_id = cfg.game_id;
	wire       is_rr   = (game_id == GAME_ROADRIOT);
	wire       is_gh   = (game_id == GAME_GUARDIAN);

	//========================================================================
	//  SDRAM channels
	//========================================================================
	wire [24:0] m68k_rom_addr, jsa_prog_addr, tile_rom_addr, rle_rom_addr, oki_rom_addr;
	wire        m68k_rom_req,  jsa_prog_req,  tile_rom_req,  rle_rom_req,  oki_rom_req;
	wire [15:0] cpu_rom_dout,  tile_rom_dout, rle_rom_dout,  oki_rom_dout;
	wire        cpu_rom_ack,   tile_rom_ack,  rle_rom_ack,   oki_rom_ack;
	wire        tile_rom_gnt;

	wire [24:0] st_addr;
	wire        st_req, st_solo;

	//------------------------------------------------------------------------
	// CPU channel: the 68000 and the 6502 share it (the 6502 at 1.79 MHz has
	// a whole bus cycle of slack). Owner and address latch together before
	// the request becomes visible, and the request drops for a clock at each
	// handover, so the arbiter never sees one owner's address with the other's
	// request. The self-test owns the channel outright while it runs, with
	// both CPUs in reset.
	//------------------------------------------------------------------------
	logic        cpu_arb_req, cpu_owner_68k;
	logic [24:0] cpu_arb_addr;

	always_ff @(posedge clk) begin
		if (!rst_n_mem) begin
			cpu_arb_req   <= 1'b0;
			cpu_owner_68k <= 1'b0;
			cpu_arb_addr  <= '0;
		end
		else if (!cpu_arb_req) begin
			if (!st_busy) begin
				if (m68k_rom_req) begin
					cpu_owner_68k <= 1'b1;
					cpu_arb_addr  <= m68k_rom_addr;
					cpu_arb_req   <= 1'b1;
				end
				else if (jsa_prog_req) begin
					cpu_owner_68k <= 1'b0;
					cpu_arb_addr  <= jsa_prog_addr;
					cpu_arb_req   <= 1'b1;
				end
			end
		end
		else if (cpu_rom_ack) begin
			cpu_arb_req <= 1'b0;
		end
	end

	wire m68k_rom_ack = cpu_rom_ack &&  cpu_owner_68k && !st_busy;
	// The address compare drops the ack of a fetch the 6502 no longer wants:
	// a sound reset cannot withdraw a request already latched here.
	wire jsa_prog_ack = cpu_rom_ack && !cpu_owner_68k && !st_busy
	                 && (cpu_arb_addr == jsa_prog_addr);

	g42_sdram_arb u_arb (
		.clk        (clk),
		.rst_n      (rst_n_mem),
		.ld_addr    (ldr_addr),     .ld_din   (ldr_din),     .ld_we   (ldr_we),
		.ld_ack     (ldr_ack),
		.cpu_addr   (st_busy ? st_addr : cpu_arb_addr),
		.cpu_req    (st_busy ? st_req  : cpu_arb_req),
		.cpu_dout   (cpu_rom_dout), .cpu_ack  (cpu_rom_ack),
		.tile_addr  (tile_rom_addr), .tile_req (tile_rom_req), .tile_gnt (tile_rom_gnt),
		.tile_dout  (tile_rom_dout), .tile_ack (tile_rom_ack),
		.rle_addr   (rle_rom_addr),  .rle_req  (rle_rom_req),
		.rle_dout   (rle_rom_dout),  .rle_ack  (rle_rom_ack),
		.oki_addr   (oki_rom_addr),  .oki_req  (oki_rom_req),
		.oki_dout   (oki_rom_dout),  .oki_ack  (oki_rom_ack),
		.sdr_addr   (sdr_addr),
		.dbg_cpu_done_addr (),
		.solo_cpu   (st_busy && st_solo),
		.sdr_din    (sdr_din),
		.sdr_we     (sdr_we),
		.sdr_rd     (sdr_rd),
		.sdr_dout   (sdr_dout),
		.sdr_ready  (sdr_ready)
	);

	//========================================================================
	//  SDRAM self-test and read-capture setting
	//========================================================================
	wire        st_done, st_pass, st_sw_on, st_best_valid, st_best_clean;
	wire [31:0] st_sum_w, st_sum_r, st_rep_err;
	wire [2:0]  st_sw_sel, st_best_sel;
	wire [16*8-1:0] st_sw_err;

	g42_sdram_selftest #(.FULL(SELFTEST_FULL)) u_selftest (
		.clk           (clk),
		// Not rst_n_mem: the written sums are taken during the download and the
		// framework's reset at its end would clear them.
		.rst_n         (!dbg_clear),
		.load_active   (rom_loading),
		.load_addr     (ldr_addr),
		.load_data     (ldr_din),
		.load_we       (ldr_we),
		.load_complete (st_pending),
		.rd_addr       (st_addr),
		.rd_req        (st_req),
		.rd_dout       (cpu_rom_dout),
		.rd_ack        (cpu_rom_ack),
		.busy          (st_busy),
		.solo          (st_solo),
		.done          (st_done),
		.pass          (st_pass),
		.sum_written   (st_sum_w),
		.sum_read      (st_sum_r),
		.rep_err       (st_rep_err),
		.sweep_sel     (st_sw_sel),
		.sweep_on      (st_sw_on),
		.sweep_err     (st_sw_err),
		.best_sel      (st_best_sel),
		.best_valid    (st_best_valid),
		.best_clean    (st_best_clean)
	);

	// During the sweep, the setting under test; then the measured best (Auto)
	// or the OSD's setting (Manual).
	always_comb begin
		if (st_sw_on)                       cap_sel = st_sw_sel;
		else if (cap_auto && st_best_valid) cap_sel = st_best_sel;
		else                                cap_sel = cap_manual;
	end

	//========================================================================
	//  Inputs
	//========================================================================
	// MAME port layouts (atarig42.cpp). Every button is active low. MiSTer
	// joystick bits: [0] right, [1] left, [2] down, [3] up, then the MRA's
	// <buttons> in order from bit 4:
	//   Road Riot 4WD    4 Left Trigger  5 Right Trigger  6 Start  7 Coin  8 Pedal
	//   Danger Express   4 Fire  5 Jump  6 Duck  7 Coin  8 Start
	//   Guardians        4 Punch 1  5 Kick 1  6 Defend  7 Punch 2  8 Kick 2
	//                    9 Coin  10 Start
	// Danger Express and Guardians have no start buttons: Jump and Defend
	// start a game. The pad's Start is wired to the same switch.
	//------------------------------------------------------------------------
	wire vblank;
	wire snd_m2s_full, snd_s2m_full;
	wire adc_eoc;

	// Danger Express: one player port. Bits 7:4 are the prototype's debug
	// switches (Toggle 0/1, Step, Freeze; Test colours on IN1), released.
	function automatic [15:0] dx_port(input [31:0] j);
		dx_port = {~j[3], ~j[2], ~j[1], ~j[0],   // 15:12 up, down, left, right
		           ~(j[5] | j[8]),               // 11 Button 2: Jump (and Start)
		           ~j[4],                        // 10 Button 1: Fire
		           ~j[6],                        //  9 Button 3: Duck
		           1'b1,                         //  8 not used
		           8'hFF};
	endfunction

	// Guardians: Button 1..5 of a player
	function automatic [4:0] gh_btn(input [31:0] j);
		gh_btn = {j[8], j[7], j[6] | j[10], j[5], j[4]};   // {B5 B4 B3 B2 B1}
	endfunction

	wire [4:0] g1b = gh_btn(joystick_0);
	wire [4:0] g2b = gh_btn(joystick_1);
	wire [4:0] g3b = gh_btn(joystick_2);

	wire [15:0] gh_in0 = {~joystick_0[3], ~joystick_0[2], ~joystick_0[1], ~joystick_0[0],
	                      ~g1b[3], ~g1b[0], ~g1b[4],       // 11 P1 B4, 10 P1 B1, 9 P1 B5
	                      1'b1,                            //  8
	                      ~joystick_2[3], ~joystick_2[2], ~joystick_2[1], ~joystick_2[0],
	                      ~g3b[3], ~g3b[0], ~g3b[4],       //  3 P3 B4, 2 P3 B1, 1 P3 B5
	                      1'b1};                           //  0

	wire [15:0] gh_in1 = {~joystick_1[3], ~joystick_1[2], ~joystick_1[1], ~joystick_1[0],
	                      ~g2b[3], ~g2b[0], ~g2b[4],       // 11 P2 B4, 10 P2 B1, 9 P2 B5
	                      3'b111,                          //  8:6
	                      ~g3b[1], ~g3b[2],                //  5 P3 B2, 4 P3 B3
	                      ~g2b[1], ~g2b[2],                //  3 P2 B2, 2 P2 B3
	                      ~g1b[1], ~g1b[2]};               //  1 P1 B2, 0 P1 B3

	// Road Riot: IN0 10 left trigger (MAME Button 2), 9 right trigger
	// (Button 3), 8 start.
	wire [15:0] rr_in0 = {5'b11111, ~joystick_0[4], ~joystick_0[5], ~joystick_0[6], 8'hFF};

	logic [15:0] in0, in1;
	always_comb begin
		case (game_id)
			GAME_ROADRIOT: begin in0 = rr_in0;              in1 = 16'hFFFF;            end
			GAME_DANGEREX: begin in0 = dx_port(joystick_0); in1 = dx_port(joystick_1); end
			default:       begin in0 = gh_in0;              in1 = gh_in1;              end
		endcase
	end

	// IN2: 7 VBLANK (low during VBLANK), 6 test switch, 5 command latch full,
	// 4 response latch full (all active low), 3 ADC end of conversion (Road
	// Riot, active high; unused and high on the others).
	wire [15:0] in2 = {8'hFF, ~vblank, ~service, ~snd_m2s_full, ~snd_s2m_full,
	                   is_rr ? adc_eoc : 1'b1, 3'b111};

	// Coins go to the JSA III (Coin 1 = left mech, Coin 2 = right). Player 3's
	// coin drops into the right mech, as on the G1 core.
	wire coin_btn_p1 = is_gh ? joystick_0[9] : joystick_0[7];
	wire coin_btn_p2 = is_gh ? joystick_1[9] : joystick_1[7];
	wire coin_btn_p3 = is_gh ? joystick_2[9] : 1'b0;

	//------------------------------------------------------------------------
	// Road Riot wheel and pedal
	//------------------------------------------------------------------------
	wire vblank_rise;
	wire [7:0] wheel, pedal, adc_data;
	wire [2:0] adc_chan_sel;
	wire       adc_start;

	g42_controls u_controls (
		.clk         (clk),
		.rst_n       (rst_n),
		.frame_tick  (vblank_rise),
		.dpad        (joystick_0[1:0]),
		.pedal_btn   (joystick_0[8]),
		.l_analog    (joystick_l_analog_0),
		.r_analog    (joystick_r_analog_0),
		.paddle      (paddle_0),
		.sensitivity (sensitivity),
		.wheel       (wheel),
		.pedal       (pedal)
	);

	g42_adc0809 u_adc (
		.clk      (clk),
		.rst_n    (rst_n),
		.wheel    (wheel),
		.pedal    (pedal),
		.chan_sel (adc_chan_sel),
		.start    (adc_start),
		.data     (adc_data),
		.eoc      (adc_eoc)
	);

	//========================================================================
	//  The main board
	//========================================================================
	wire [10:0] pal_addr;
	wire [15:0] pal_din, pal_dout;
	wire        pal_wr_hi, pal_wr_lo;

	wire [14:0] vram_addr, vram_din_addr;
	wire [15:0] vram_dout, vram_din;
	wire        vram_we;

	wire [2:0]  rle_ctrl;
	wire        rle_ctrl_wr, rle_cmd_wr;
	wire [15:0] rle_cmd, rle_objram_w0;

	wire [7:0]  snd_cmd, snd_resp;
	wire        snd_cmd_wr, snd_resp_rd, snd_int, snd_reset;
	wire [7:0]  jsaiii_port;

	wire [7:0]  nv_dout;
	wire        nv_access = (ioctl_index[7:0] == 8'd2) && (ioctl_download || ioctl_upload);

	wire [23:0] dbg_cpu_addr, dbg_wd_addr;
	wire [7:0]  dbg_hit, dbg_sloop_changes, dbg_asic_cmds;
	wire [15:0] dbg_wd_count, dbg_iack_cnt, dbg_io_latch, dbg_asic_pc;
	wire [23:0] dbg_rom_wait;
	wire [15:0] dbg_rom_lookups, dbg_rom_misses;
	wire [1:0]  dbg_sloop_bank;

	g42_top #(.ROM_CACHE(ROM_CACHE), .ROM_CACHE_BITS(ROM_CACHE_BITS)) u_board (
		.clk            (clk),
		.rst_n          (rst_n),
		.ce_cpu_p1      (ce_cpu_p1),
		.ce_cpu_p2      (ce_cpu_p2),
		.cfg            (cfg),
		.disable_wd     (disable_wd),
		.dbg_clr        (dbg_clear),
		.vblank_rise    (vblank_rise),

		.in0            (in0),
		.in1            (in1),
		.in2            (in2),
		.jsaiii_port    (jsaiii_port),

		.adc_data       (adc_data),
		.adc_chan_sel   (adc_chan_sel),
		.adc_start      (adc_start),

		.rom_addr       (m68k_rom_addr),
		.rom_req        (m68k_rom_req),
		.rom_dout       (cpu_rom_dout),
		.rom_ack        (m68k_rom_ack),

		.pal_addr       (pal_addr),
		.pal_din        (pal_din),
		.pal_wr_hi      (pal_wr_hi),
		.pal_wr_lo      (pal_wr_lo),
		.pal_dout       (pal_dout),

		.vram_addr      (vram_addr),
		.vram_dout      (vram_dout),
		.vram_din_addr  (vram_din_addr),
		.vram_din       (vram_din),
		.vram_we        (vram_we),

		.snd_cmd        (snd_cmd),
		.snd_cmd_wr     (snd_cmd_wr),
		.snd_resp_rd    (snd_resp_rd),
		.snd_resp       (snd_resp),
		.snd_int        (snd_int),
		.snd_reset      (snd_reset),

		.rle_ctrl       (rle_ctrl),
		.rle_ctrl_wr    (rle_ctrl_wr),
		.rle_cmd        (rle_cmd),
		.rle_cmd_wr     (rle_cmd_wr),
		.rle_objram_w0  (rle_objram_w0),

		.dsp_prog_we    (dsp_prog_we),
		.dsp_prog_addr  (dsp_prog_addr),
		.dsp_prog_data  (dsp_prog_data),

		.eep_def_we     (eep_def_we),
		.eep_def_addr   (eep_def_addr),
		.eep_def_din    (eep_def_din),
		.nv_wr          (nv_access && ioctl_download && ioctl_wr),
		.nv_addr        (ioctl_addr[10:0]),
		.nv_din         (ioctl_dout),
		.nv_dout        (nv_dout),
		.nv_dirty       (nvram_dirty),

		.dbg_cpu_addr   (dbg_cpu_addr),
		.dbg_hit        (dbg_hit),
		.dbg_wd_count   (dbg_wd_count),
		.dbg_wd_addr    (dbg_wd_addr),
		.dbg_iack_cnt   (dbg_iack_cnt),
		.dbg_io_latch   (dbg_io_latch),
		.dbg_sloop_bank (dbg_sloop_bank),
		.dbg_sloop_changes (dbg_sloop_changes),
		.dbg_rom_wait   (dbg_rom_wait),
		.dbg_rom_lookups (dbg_rom_lookups),
		.dbg_rom_misses (dbg_rom_misses),
		.dbg_asic_pc    (dbg_asic_pc),
		.dbg_asic_cmds  (dbg_asic_cmds)
	);

	assign ioctl_din = nv_dout;

	//========================================================================
	//  JSA III sound board
	//========================================================================
	wire [15:0] jsa_pc;
	wire        jsa_ce;

	g42_jsa3 u_jsa3 (
		.clk                 (clk),
		.rst_n               (rst_n),
		.board_reset         (snd_reset),
		.ce_6502             (ce_6502),
		.ce_ym               (ce_ym),
		.ce_oki              (ce_oki),

		.main_din            (snd_cmd),
		.main_wr             (snd_cmd_wr),
		.main_rd             (snd_resp_rd),
		.main_dout           (snd_resp),
		.main_irq            (snd_int),
		.main_to_sound_ready (snd_m2s_full),
		.sound_to_main_ready (snd_s2m_full),

		.coin_l              (coin_btn_p1),
		.coin_r              (coin_btn_p2 | coin_btn_p3),
		.tilt                (1'b0),
		.service             (1'b0),
		// The test switch reaches the JSA III too (MAME main_test_read_line).
		.self_test           (service),
		.jsaiii_port         (jsaiii_port),

		.prog_addr           (jsa_prog_addr),
		.prog_req            (jsa_prog_req),
		.prog_dout           (cpu_rom_dout),
		.prog_ack            (jsa_prog_ack),

		.oki_addr            (oki_rom_addr),
		.oki_req             (oki_rom_req),
		.oki_dout            (oki_rom_dout),
		.oki_ack             (oki_rom_ack),

		.audio               (audio),
		.dbg_pc              (jsa_pc),
		.dbg_cpu_ce          (jsa_ce)
	);

	//========================================================================
	//  Video
	//========================================================================
	wire [10:0] pal_index;
	wire        hblank, hsync, vsync, de;
	wire [8:0]  vis_x, mo_x, vpos;
	wire [7:0]  vis_y, mo_y;
	wire [12:0] mo_pixel;
	wire [7:0]  vid_fetch_ovr;
	wire [11:0] vid_fetch_max;

	wire [14:0] rle_vaddr;
	wire        rle_vreq, rle_vack, rle_vwe;
	wire [15:0] rle_vdin;

	g42_video u_video (
		.clk                (clk),
		.rst_n              (rst_n),
		.ce_pix             (ce_pix),
		.cfg                (cfg),
		.mo_enable          (!layer_off[0]),
		.pf_enable          (!layer_off[1]),
		.al_enable          (!layer_off[2]),

		.vram_addr          (vram_addr),
		.vram_dout          (vram_dout),
		.vram_din_addr      (vram_din_addr),
		.vram_din           (vram_din),
		.vram_we            (vram_we),

		.ext_vram_addr      (rle_vaddr),
		.ext_vram_req       (rle_vreq),
		.ext_vram_ack       (rle_vack),
		.ext_vram_din       (rle_vdin),
		.ext_vram_we        (rle_vwe),

		.tile_rom_addr      (tile_rom_addr),
		.tile_rom_req       (tile_rom_req),
		.tile_rom_gnt       (tile_rom_gnt),
		.tile_rom_dout      (tile_rom_dout),
		.tile_rom_ack       (tile_rom_ack),

		.mo_x               (mo_x),
		.mo_y               (mo_y),
		.mo_pixel           (mo_pixel),

		.pal_index          (pal_index),

		.hblank             (hblank),
		.vblank             (vblank),
		.hsync              (hsync),
		.vsync              (vsync),
		.de                 (de),
		.vblank_rise        (vblank_rise),
		.vis_x              (vis_x),
		.vis_y              (vis_y),
		.vpos               (vpos),

		.dbg_fetch_overruns (vid_fetch_ovr),
		.dbg_fetch_max      (vid_fetch_max)
	);

	//========================================================================
	//  Motion objects
	//========================================================================
	wire        render_busy, erase_busy, cksum_busy;
	wire [12:0] stat_valid;
	wire [9:0]  stat_max_w;
	wire [8:0]  stat_max_h;
	wire [7:0]  mogo_drops;
	wire [23:0] render_clocks;
	wire [22:0] rle_rom_words;

	wire [15:0] obj_count16 = {cfg.obj_count_h, cfg.obj_count_l};

	g42_rle u_rle (
		.clk           (clk),
		.rst_n         (rst_n_sys),
		.obj_count     (obj_count16[12:0]),
		.mo_base       (cfg.mo_base),
		.color_mask    (cfg.mo_color_mask[5:0]),

		.ctrl          (rle_ctrl),
		.ctrl_wr       (rle_ctrl_wr),
		.cmd           (rle_cmd),
		.cmd_wr        (rle_cmd_wr),
		.objram_word0  (rle_objram_w0),

		.vpos          (vpos),
		.vblank_rise   (vblank_rise),

		.vram_addr     (rle_vaddr),
		.vram_req      (rle_vreq),
		.vram_dout     (vram_dout),
		.vram_ack      (rle_vack),
		.vram_din      (rle_vdin),
		.vram_we       (rle_vwe),

		.rom_addr      (rle_rom_addr),
		.rom_req       (rle_rom_req),
		.rom_dout      (rle_rom_dout),
		.rom_ack       (rle_rom_ack),

		// The checksum accumulator and ROM length span the download: reset only
		// by the PLL, restarted by the ROM download itself.
		.load_active   (rom_loading),
		.load_rst_n    (!dbg_clear),
		.load_addr     (ldr_addr),
		.load_data     (ldr_din),
		.load_we       (ldr_we),
		.load_complete (ps_req && cfg_fresh && !ioctl_download),

		.disp_x        (mo_x),
		.disp_y        (mo_y),
		.disp_pixel    (mo_pixel),

		.prescan_busy  (prescan_busy),
		.render_busy   (render_busy),
		.erase_busy    (erase_busy),
		.cksum_busy    (cksum_busy),
		.stat_valid    (stat_valid),
		.stat_max_w    (stat_max_w),
		.stat_max_h    (stat_max_h),
		.mogo_drops    (mogo_drops),
		.render_clocks (render_clocks),
		.rom_words     (rle_rom_words)
	);

	//========================================================================
	//  Palette
	//========================================================================
	wire [7:0] pal_r, pal_g, pal_b;

	g42_palette u_palette (
		.clk       (clk),
		.cpu_addr  (pal_addr),
		.cpu_din   (pal_din),
		.cpu_wr_hi (pal_wr_hi),
		.cpu_wr_lo (pal_wr_lo),
		.cpu_dout  (pal_dout),
		.vid_index (pal_index),
		.vid_r     (pal_r),
		.vid_g     (pal_g),
		.vid_b     (pal_b)
	);

	//========================================================================
	//  Diagnostic overlay (OSD: Debug -> Diagnostic overlay)
	//========================================================================
	// Drawn from the raster counters over the game picture, so it works when
	// the 68000 never runs. Rows (see g42_dbg_text):
	//   BLD   compile date YYMMDD and build number (G42_BUILD), decimal
	//   CPU   68000 bus address of the current cycle
	//   BOOT  HH GG LLLL: HH boot progress (g42_top dbg_hit: 80 palette,
	//         40 watchdog, 20 IRQ ack, 10 I/O latch, 08 sound, 04 EEPROM,
	//         02 ASIC65, 01 work RAM), GG game id, LLLL I/O latch
	//   WDOG  CC AAAAAA: watchdog resets, address of the last cycle before one
	//   IRQ   interrupt acknowledges | frames since reset
	//   ROMW  WWWWWW XX: clk_sys the 68000 waited for ROM last frame (a frame
	//         is 954,480 = $E9070); XX = SLOOP bank changes x 4 + bank
	//   CACH  program ROM reads last frame | ... that missed the ROM cache
	//   SND   commands written | responses read | last response
	//   JSA   6502 PC | 6502 bus cycles per frame / 256
	//   ASIC  DSP PC | command writes
	//   MOGO  DRAWs dropped | clk_sys of the last DRAW
	//   RLE   {prescan, render, erase, checksum busy} | objects with data | widest
	//   VFET  line fetch overruns last frame | longest line fetch (budget $E40)
	//   PALW  palette writes | largest palette index shown last frame
	//   SUMW  SDRAM self-test: sum written
	//   SUMR  ... sum read back
	//   REPE  ... repeat-test errors
	//   PHAS  0000ACMU: U setting in use, M manual setting, C best was clean,
	//         A auto; plus P (bit 28) pass, D (bit 24) done
	//   SW03  sweep errors of capture settings 0-3, 8 bits each (saturated)
	//   SW47  ... settings 4-7
	//   LOAD  {config valid, ROM loaded} | loader's highest address / 64 KB
	//------------------------------------------------------------------------
	localparam [47:0] BLD_DATE_ASCII = BUILD_DATE;
	localparam [23:0] BLD_DATE_BCD   = {BLD_DATE_ASCII[43:40], BLD_DATE_ASCII[35:32],
	                                    BLD_DATE_ASCII[27:24], BLD_DATE_ASCII[19:16],
	                                    BLD_DATE_ASCII[11:8],  BLD_DATE_ASCII[3:0]};
	localparam [3:0]  BLD_H = 4'((G42_BUILD / 100) % 10);
	localparam [3:0]  BLD_T = 4'((G42_BUILD / 10)  % 10);
	localparam [3:0]  BLD_O = 4'( G42_BUILD        % 10);
	localparam [35:0] BLD_BCD = {BLD_DATE_BCD, BLD_H, BLD_T, BLD_O};

	// Per-frame counters
	logic [15:0] frame_cnt;
	logic [15:0] jsa_cyc, jsa_cyc_l;
	logic [15:0] pal_wr_cnt;
	logic [10:0] pal_max, pal_max_l;
	logic [11:0] snd_wr_cnt, snd_rd_cnt;
	logic [7:0]  snd_last;
	logic [7:0]  ldr_max;
	logic        pal_wr_d;

	always_ff @(posedge clk) begin
		pal_wr_d <= pal_wr_hi | pal_wr_lo;
		if (dbg_clear) begin
			frame_cnt  <= '0;
			jsa_cyc    <= '0;
			jsa_cyc_l  <= '0;
			pal_wr_cnt <= '0;
			pal_max    <= '0;
			pal_max_l  <= '0;
			snd_wr_cnt <= '0;
			snd_rd_cnt <= '0;
			snd_last   <= '0;
			ldr_max    <= '0;
		end else begin
			if (vblank_rise) begin
				frame_cnt <= frame_cnt + 1'b1;
				jsa_cyc_l <= jsa_cyc;
				jsa_cyc   <= '0;
				pal_max_l <= pal_max;
				pal_max   <= '0;
			end else begin
				if (jsa_ce) jsa_cyc <= jsa_cyc + 1'b1;
				if (de && pal_index > pal_max) pal_max <= pal_index;
			end
			if ((pal_wr_hi | pal_wr_lo) && !pal_wr_d) pal_wr_cnt <= pal_wr_cnt + 1'b1;
			if (snd_cmd_wr) snd_wr_cnt <= snd_wr_cnt + 1'b1;
			if (snd_resp_rd) begin
				snd_rd_cnt <= snd_rd_cnt + 1'b1;
				snd_last   <= snd_resp;
			end
			if (rom_loading && ldr_we && ldr_addr[24:17] > ldr_max) ldr_max <= ldr_addr[24:17];
		end
	end

	function automatic [7:0] sat8(input [15:0] v);
		sat8 = (v > 16'd255) ? 8'd255 : v[7:0];
	endfunction

	localparam int DBG_ROWS = 21;
	wire [32*DBG_ROWS-1:0] dbg_vals = {
		BLD_BCD[31:0],                                                          // BLD
		{8'd0, dbg_cpu_addr},                                                   // CPU
		{dbg_hit, game_id, dbg_io_latch},                                       // BOOT
		{dbg_wd_count[7:0], dbg_wd_addr},                                       // WDOG
		{dbg_iack_cnt, frame_cnt},                                              // IRQ
		{dbg_rom_wait, dbg_sloop_changes[5:0], dbg_sloop_bank},                 // ROMW
		{dbg_rom_lookups, dbg_rom_misses},                                      // CACH
		{snd_wr_cnt, snd_rd_cnt, snd_last},                                     // SND
		{jsa_pc, jsa_cyc_l[15:8], 8'd0},                                        // JSA
		{dbg_asic_pc, dbg_asic_cmds, 8'd0},                                     // ASIC
		{mogo_drops, render_clocks},                                            // MOGO
		{prescan_busy, render_busy, erase_busy, cksum_busy, 3'd0, stat_valid,
		 2'd0, stat_max_w},                                                     // RLE
		{8'd0, vid_fetch_ovr, 4'd0, vid_fetch_max},                             // VFET
		{pal_wr_cnt, 5'd0, pal_max_l},                                          // PALW
		st_sum_w,                                                               // SUMW
		st_sum_r,                                                               // SUMR
		st_rep_err,                                                             // REPE
		{3'd0, st_pass, 3'd0, st_done, 8'd0, 3'd0, cap_auto, 3'd0, st_best_clean,
		 1'b0, cap_manual, 1'b0, cap_sel},                                      // PHAS
		{sat8(st_sw_err[16*0 +: 16]), sat8(st_sw_err[16*1 +: 16]),
		 sat8(st_sw_err[16*2 +: 16]), sat8(st_sw_err[16*3 +: 16])},             // SW03
		{sat8(st_sw_err[16*4 +: 16]), sat8(st_sw_err[16*5 +: 16]),
		 sat8(st_sw_err[16*6 +: 16]), sat8(st_sw_err[16*7 +: 16])},             // SW47
		{3'd0, cfg_valid, 3'd0, rom_loaded, 16'd0, ldr_max}                     // LOAD
	};

	wire dbg_pix, dbg_box;

	g42_dbg_text #(.N_ROWS(DBG_ROWS)) u_dbg_text (
		.clk      (clk),
		.vis_x    (vis_x),
		.vis_y    (vis_y),
		.vals     (dbg_vals),
		.row0_msn (BLD_BCD[35:32]),
		.pix      (dbg_pix),
		.in_box   (dbg_box)
	);

	//========================================================================
	//  Video output
	//========================================================================
	// g42_palette's RGB is complete four clk_sys into the dot, and the sync
	// and blanking describe the same dot (g42_video), so they are registered
	// together at the next ce_pix with no extra delay. The overlay is two
	// clocks behind vis_x, also inside the dot.
	always_ff @(posedge clk) begin
		if (ce_pix) begin
			if (dbg_page != 2'd0 && dbg_box) begin
				vid_r <= dbg_pix ? 8'hFF : 8'h00;
				vid_g <= dbg_pix ? 8'hFF : 8'h00;
				vid_b <= dbg_pix ? 8'hFF : 8'h00;
			end else begin
				vid_r <= pal_r;
				vid_g <= pal_g;
				vid_b <= pal_b;
			end
			vid_hblank <= hblank;
			vid_vblank <= vblank;
			vid_hsync  <= hsync;
			vid_vsync  <= vsync;
		end
	end

	//========================================================================
	//  Activity LED
	//========================================================================
	// On during the download, the SDRAM self-test and the object prescan; a
	// fast blink afterwards means the self-test failed.
	logic [23:0] blink;
	always_ff @(posedge clk) blink <= blink + 1'b1;

	assign led = rom_loading | st_busy | prescan_busy | (st_done & ~st_pass & blink[22]);

`ifdef SIMULATION
	// synthesis translate_off
	logic st_busy_d;
	int   sim_frame = 0;
	int   ps_reads, ps_clocks;
	always_ff @(posedge clk) begin
		st_busy_d <= st_busy;
		// One line per frame: the 68000's bus address at VBLANK, for
		// comparison with a MAME PC trace
		if (vblank_rise && rst_n) begin
			sim_frame <= sim_frame + 1;
			$display("FRAME %0d ADDR %06X", sim_frame + 1, dbg_cpu_addr);
		end
		if (load_complete)              $display("g42_core: ROM download complete");
		if (st_busy && !st_busy_d)      $display("g42_core: SDRAM self-test started");
		if (!st_busy && st_busy_d)      $display("g42_core: SDRAM self-test finished");
		if (prescan_busy && !ps_busy_d) begin
			$display("g42_core: RLE prescan started (%0d objects)", obj_count16);
			ps_reads  <= 0;
			ps_clocks <= 0;
		end
		if (prescan_busy) begin
			ps_clocks <= ps_clocks + 1;
			if (rle_rom_ack) ps_reads <= ps_reads + 1;
		end
		if (!prescan_busy && ps_busy_d) $display("g42_core: RLE prescan finished: %0d objects with data, max %0d x %0d, ROM %0d words; %0d reads in %0d clocks",
		                                         stat_valid, stat_max_w, stat_max_h, rle_rom_words, ps_reads, ps_clocks);
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
