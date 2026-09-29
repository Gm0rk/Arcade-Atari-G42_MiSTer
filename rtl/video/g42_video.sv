//============================================================================
//  Atari G42 for MiSTer
//  g42_video.sv -- video subsystem: timing, line sequencer, port arbitration,
//                  layers and mixing
//
//  Owns the raster timebase and runs the per-line fetch engines against it,
//  arbitrating the two resources they share:
//    - the work RAM video port (control words, alpha map, playfield map, and
//      the RLE engine's object RAM reads at the lowest priority)
//    - the SDRAM tile channel (alpha character ROM, playfield tile ROM)
//  The architecture is the G1 core's (g1_video.sv): line N+1 is fetched into
//  ping-pong line buffers during line N; the playfield and alpha fetchers run
//  concurrently after the line's control words are latched.
//
//  Line sequencer. MAME latches the control words of lines 0..255 (lines
//  240-255 are in VBLANK but still latch, and what they latch carries into
//  the next frame's line 0), so the sequencer runs one job per line:
//    line_start of v    buffers   control words    tile fetch
//    0 .. 238           swap      line v+1         line v+1
//    239                swap      line 240         -
//    240 .. 254         -         line v+1         -
//    255 .. 260         -         -                -
//    261                -         line 0           line 0
//  Every line's words are latched once a frame, in MAME's order, each just
//  before its own fetch (so line 0 only after lines 240-255). G1 instead
//  started a fetch of line 0 at line 239 to get its buffer swap, latching
//  line 0's words early. Here the swap is its own event at each displayed
//  line's start, and it carries the line's fine X scroll and colour bank.
//  MAME reads the words of lines 8k..8k+7 at line 8k; this reads line L's at
//  the start of line L-1. The two differ only if the CPU rewrites a line's
//  words between those moments (one word in 34 captured frames).
//
//  A fetch still running at the next line start is an overrun: no swap (the
//  previous line shows again), the next fetch is skipped, and the control
//  words are still latched so the scroll state stays exact. dbg_* count them.
//
//  Budget: a line is 456 dots = 3648 clk_sys; the fetch needs 213 SDRAM
//  words (43 x 3 + 42 x 2, as G1) and 87 work RAM reads. Simulated line jobs
//  (sim/video/README.md): 1,595 clk_sys on an idle bus, 2,600 with a 68000
//  fetching from SDRAM without pause and the RLE engine taking every free
//  slot, 3,500 under a heavier synthetic load.
//
//  Output timing: hblank/vblank/hsync/vsync/de, vis_x/vis_y and vpos come
//  from g42_video_timing and change on the clock after ce_pix; they all
//  describe the same dot. pal_index for that dot is valid two clk_sys later
//  (line buffer / MO framebuffer read, mixer register) and g42_palette adds
//  two more, so RGB is complete four clk_sys into the dot, well before the
//  next ce_pix. The sync/blank outputs therefore need no delay to line up
//  with g42_palette's RGB (see the README; G1's three-dot delay would shift
//  the picture).
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_video
	import g42_pkg::*;
#(
	parameter bit PF_MAME_FORM = 1'b0       // playfield colour: 0 PAL form, 1 MAME form (g42_mixer)
)(
	input  wire         clk,                // clk_sys
	input  wire         rst_n,
	input  wire         ce_pix,             // dot enable, one clk_sys in 8

	// Only pf_base[2:0], mo_base[2] and flags[3] matter here; the struct is
	// shared by the whole core.
	/* verilator lint_off UNUSEDSIGNAL */
	input  g42_cfg_t    cfg,                // pf_base, mo_base, flags[3]
	/* verilator lint_on UNUSEDSIGNAL */

	// ---- OSD / debug layer enables (1 = shown) -----------------------------
	input  wire         mo_enable,
	input  wire         pf_enable,
	input  wire         al_enable,

	// ---- Work RAM video port (1-clock read latency) ------------------------
	output logic [14:0] vram_addr,          // read address, word offset from $FF0000
	input  wire  [15:0] vram_dout,          // data for last clock's vram_addr
	output logic [14:0] vram_din_addr,      // RLE write-back, passed through
	output logic [15:0] vram_din,
	output logic        vram_we,

	// ---- RLE engine work RAM requester (lowest read priority) -------------
	input  wire  [14:0] ext_vram_addr,
	input  wire         ext_vram_req,
	output logic        ext_vram_ack,       // one clock after the grant, with vram_dout
	input  wire  [15:0] ext_vram_din,
	input  wire         ext_vram_we,

	// ---- SDRAM tile channel (G1 protocol with grant) -----------------------
	output logic [24:0] tile_rom_addr,      // byte address, valid with tile_rom_req
	output logic        tile_rom_req,
	input  wire         tile_rom_gnt,       // the arbiter took the presented request
	input  wire  [15:0] tile_rom_dout,      // big-endian word, valid with tile_rom_ack
	input  wire         tile_rom_ack,

	// ---- Motion object framebuffer read ------------------------------------
	output logic [8:0]  mo_x,               // = vis_x
	output logic [7:0]  mo_y,               // = vis_y
	input  wire  [12:0] mo_pixel,           // one clock after mo_x/mo_y

	// ---- To g42_palette -----------------------------------------------------
	output logic [10:0] pal_index,          // 2 clk_sys after vis_x changes

	// ---- Raster -------------------------------------------------------------
	output logic        hblank,
	output logic        vblank,
	output logic        hsync,
	output logic        vsync,
	output logic        de,
	output logic        vblank_rise,        // one clk_sys at VBLANK start (line 240)
	output logic [8:0]  vis_x,              // 0..335 while de
	output logic [7:0]  vis_y,              // 0..239 while de
	output logic [8:0]  vpos,               // current raster line 0..261

	// ---- Debug, per frame ---------------------------------------------------
	output logic [7:0]  dbg_fetch_overruns, // line starts that found a fetch still running
	output logic [11:0] dbg_fetch_max       // longest line job (words + fetch), clk_sys
);

	//========================================================================
	//  Raster timing
	//========================================================================
	wire [VCNT_W-1:0] vcnt;
	wire line_start, frame_start;

	// hcnt and vblank_fall are not needed here (left open on purpose).
	/* verilator lint_off PINCONNECTEMPTY */
	g42_video_timing u_timing (
		.clk         (clk),
		.rst_n       (rst_n),
		.ce_pix      (ce_pix),
		.hcnt        (),
		.vcnt        (vcnt),
		.hblank      (hblank),
		.vblank      (vblank),
		.hsync       (hsync),
		.vsync       (vsync),
		.de          (de),
		.line_start  (line_start),
		.frame_start (frame_start),
		.vblank_rise (vblank_rise),
		.vblank_fall (),
		.vis_x       (vis_x),
		.vis_y       (vis_y)
	);
	/* verilator lint_on PINCONNECTEMPTY */

	assign vpos = vcnt;

	//========================================================================
	//  Line sequencer
	//========================================================================
	// line_start arrives with vcnt already on the new line v (see the table
	// in the header).
	//------------------------------------------------------------------------
	wire       disp_line  = (vcnt < VCNT_W'(V_VISIBLE));
	wire       last_line  = (vcnt == VCNT_W'(V_TOTAL - 1));
	wire [7:0] next_line  = last_line ? 8'd0 : 8'(vcnt + 9'd1);  // used for v <= 254 and 261
	wire       words_due  = (vcnt <= VCNT_W'(254)) || last_line;
	wire       fetch_due  = (vcnt <= VCNT_W'(V_VISIBLE - 2)) || last_line;

	typedef enum logic [1:0] { L_IDLE, L_SCROLL, L_FETCH } lstate_t;
	lstate_t lstate;

	logic       sc_start, pf_start, al_start, swap;
	logic [7:0] sc_line, fetch_line;
	wire        sc_done, pf_busy, al_busy;

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			lstate     <= L_IDLE;
			sc_start   <= 1'b0;
			pf_start   <= 1'b0;
			al_start   <= 1'b0;
			swap       <= 1'b0;
			sc_line    <= 8'd0;
			fetch_line <= 8'd0;
		end
		else begin
			sc_start <= 1'b0;
			pf_start <= 1'b0;
			al_start <= 1'b0;
			swap     <= 1'b0;

			if (line_start) begin
				// Show the buffer filled during the last line, if it is complete.
				swap <= disp_line && (lstate == L_IDLE);
				// The words are latched even when the fetch must be skipped.
				sc_start <= words_due;
				sc_line  <= next_line;
			end

			case (lstate)
			L_IDLE:
				if (line_start && fetch_due) begin
					fetch_line <= next_line;
					lstate     <= L_SCROLL;
				end

			// The fetch uses this line's scroll state: start after the words.
			L_SCROLL:
				if (sc_done) begin
					pf_start <= 1'b1;
					al_start <= 1'b1;
					lstate   <= L_FETCH;
				end

			// The fetchers go busy the clock after their start pulse.
			L_FETCH:
				if (!pf_busy && !al_busy && !pf_start && !al_start)
					lstate <= L_IDLE;

			default: lstate <= L_IDLE;
			endcase
		end
	end

	//------------------------------------------------------------------------
	// Debug: overruns and the longest line job, latched once a frame
	//------------------------------------------------------------------------
	logic [7:0]  ovr_cnt;
	logic [11:0] len_cnt, len_max;

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			ovr_cnt            <= '0;
			len_cnt            <= '0;
			len_max            <= '0;
			dbg_fetch_overruns <= '0;
			dbg_fetch_max      <= '0;
		end
		else begin
			if (lstate != L_IDLE) begin
				if (len_cnt != 12'hFFF) len_cnt <= len_cnt + 1'b1;
			end
			else begin
				len_cnt <= '0;
				if (len_cnt > len_max) len_max <= len_cnt;
			end
			if (line_start && (disp_line || last_line) && lstate != L_IDLE && ovr_cnt != 8'hFF)
				ovr_cnt <= ovr_cnt + 1'b1;
			if (frame_start) begin
				dbg_fetch_overruns <= ovr_cnt;
				dbg_fetch_max      <= len_max;
				ovr_cnt            <= '0;
				len_max            <= '0;
			end
		end
	end

	//========================================================================
	//  Control words
	//========================================================================
	wire [9:0]  xscroll;
	wire [8:0]  yscroll;
	wire [4:0]  color_bank;
	wire [2:0]  tile_bank;

	wire [14:0] sc_vaddr, pf_vaddr, al_vaddr;
	wire        sc_vreq,  pf_vreq,  al_vreq;
	logic       sc_vack,  pf_vack,  al_vack;

	g42_scroll u_scroll (
		.clk        (clk),
		.rst_n      (rst_n),
		.start      (sc_start),
		.line       (sc_line),
		.vram_addr  (sc_vaddr),
		.vram_req   (sc_vreq),
		.vram_dout  (vram_dout),
		.vram_ack   (sc_vack),
		.xscroll    (xscroll),
		.yscroll    (yscroll),
		.color_bank (color_bank),
		.tile_bank  (tile_bank),
		.done       (sc_done)
	);

	//========================================================================
	//  Work RAM port arbitration
	//========================================================================
	// Fixed read priority: control words, playfield, alpha, then the RLE
	// engine (its object RAM snapshot has a frame of slack; the tile fetchers
	// must finish within a line). The RAM has one clock of latency, so the
	// ack is the grant delayed by one clock. The RLE write-back uses the
	// write port (vram_din_addr), not this arbiter.
	//------------------------------------------------------------------------
	logic sc_sel_d, pf_sel_d, al_sel_d, ext_sel_d;

	always_comb begin
		if      (sc_vreq) vram_addr = sc_vaddr;
		else if (pf_vreq) vram_addr = pf_vaddr;
		else if (al_vreq) vram_addr = al_vaddr;
		else              vram_addr = ext_vram_addr;

		vram_din_addr = ext_vram_addr;
		vram_din      = ext_vram_din;
		vram_we       = ext_vram_we;
	end

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			sc_sel_d  <= 1'b0;
			pf_sel_d  <= 1'b0;
			al_sel_d  <= 1'b0;
			ext_sel_d <= 1'b0;
		end
		else begin
			sc_sel_d  <= sc_vreq;
			pf_sel_d  <= pf_vreq && !sc_vreq;
			al_sel_d  <= al_vreq && !sc_vreq && !pf_vreq;
			ext_sel_d <= ext_vram_req && !sc_vreq && !pf_vreq && !al_vreq;
		end
	end

	assign sc_vack      = sc_sel_d;
	assign pf_vack      = pf_sel_d;
	assign al_vack      = al_sel_d;
	assign ext_vram_ack = ext_sel_d;

	//========================================================================
	//  Layers
	//========================================================================
	wire [24:0] pf_raddr, al_raddr;
	wire        pf_rreq,  al_rreq;
	logic       pf_rgnt,  al_rgnt;
	logic       pf_rack,  al_rack;

	wire [5:0] pf_pen;   wire [2:0] pf_color;   wire [4:0] pf_bank;
	wire [3:0] al_pen;   wire [3:0] al_color;   wire       al_opaque;

	g42_playfield u_pf (
		.clk        (clk),
		.rst_n      (rst_n),
		.tiles32k   (cfg.flags[3]),
		.start      (pf_start),
		.line       (fetch_line),
		.swap       (swap),
		.xscroll    (xscroll),
		.yscroll    (yscroll),
		.tile_bank  (tile_bank),
		.color_bank (color_bank),
		.vram_addr  (pf_vaddr),
		.vram_req   (pf_vreq),
		.vram_dout  (vram_dout),
		.vram_ack   (pf_vack),
		.rom_addr   (pf_raddr),
		.rom_req    (pf_rreq),
		.rom_gnt    (pf_rgnt),
		.rom_dout   (tile_rom_dout),
		.rom_ack    (pf_rack),
		.disp_x     (vis_x),
		.disp_pen   (pf_pen),
		.disp_color (pf_color),
		.disp_bank  (pf_bank),
		.busy       (pf_busy)
	);

	g42_alpha u_al (
		.clk         (clk),
		.rst_n       (rst_n),
		.start       (al_start),
		.line        (fetch_line),
		.swap        (swap),
		.vram_addr   (al_vaddr),
		.vram_req    (al_vreq),
		.vram_dout   (vram_dout),
		.vram_ack    (al_vack),
		.rom_addr    (al_raddr),
		.rom_req     (al_rreq),
		.rom_gnt     (al_rgnt),
		.rom_dout    (tile_rom_dout),
		.rom_ack     (al_rack),
		.disp_x      (vis_x),
		.disp_pen    (al_pen),
		.disp_color  (al_color),
		.disp_opaque (al_opaque),
		.busy        (al_busy)
	);

	//========================================================================
	//  Tile ROM channel: two fetchers, one arbiter port
	//========================================================================
	// G1's grant handshake: the arbiter raises tile_rom_gnt in the clock it
	// takes the presented request and latches tile_rom_addr; at that edge the
	// owner is recorded and the owning fetcher advances, so a granted request
	// is never presented again, and owner and arbiter address come from one
	// selection at one edge. The arbiter can grant again in the ack clock.
	// One access is in flight at a time, so one owner register routes every
	// ack; an ack and a grant in the same clock route by the old owner.
	//
	// Unlike G1's fetchers, which dropped the request for a clock after each
	// word (G1 marked granted requests "taken" here instead), g42_playfield and
	// g42_alpha present their next word while the current one is in flight, so
	// the request is already up in the ack clock and the RLE engine gets no
	// slot between two tile words. The playfield goes first (more words).
	//------------------------------------------------------------------------
	logic inflight_pf;

	wire pick_pf = pf_rreq;

	assign tile_rom_req  = pf_rreq || al_rreq;
	assign tile_rom_addr = pick_pf ? pf_raddr : al_raddr;

	assign pf_rgnt = tile_rom_gnt &&  pick_pf;
	assign al_rgnt = tile_rom_gnt && !pick_pf;
	assign pf_rack = tile_rom_ack &&  inflight_pf;
	assign al_rack = tile_rom_ack && !inflight_pf;

	always_ff @(posedge clk) begin
		if (!rst_n)
			inflight_pf <= 1'b0;
		else if (tile_rom_gnt)
			inflight_pf <= pick_pf;
	end

	//========================================================================
	//  Motion object framebuffer address and mixer
	//========================================================================
	assign mo_x = vis_x;
	assign mo_y = vis_y;

	g42_mixer #(
		.PF_MAME_FORM (PF_MAME_FORM)
	) u_mixer (
		.clk       (clk),
		.de        (de),
		.pf_base   (cfg.pf_base[2:0]),
		.mo_base10 (cfg.mo_base[2]),
		.pf_enable (pf_enable),
		.mo_enable (mo_enable),
		.al_enable (al_enable),
		.pf_pen    (pf_pen),
		.pf_color  (pf_color),
		.pf_bank   (pf_bank),
		.mo_pixel  (mo_pixel),
		.al_pen    (al_pen),
		.al_color  (al_color),
		.al_opaque (al_opaque),
		.pal_index (pal_index)
	);

`ifdef SIMULATION
	// synthesis translate_off
	always_ff @(posedge clk) begin
		if (rst_n && sc_start && sc_vreq)
			$display("%t g42_video: control-word job started while the last one runs", $time);
		if (rst_n && swap && (pf_start || al_start))
			$display("%t g42_video: swap and fetch start in one clock", $time);
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
