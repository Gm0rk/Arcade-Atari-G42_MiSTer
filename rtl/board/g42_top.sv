//============================================================================
//  Atari G42 for MiSTer
//  g42_top.sv -- the main board: 68000, memory map, interrupts, bus
//
//  The 68000 (fx68k) with its program ROM behind the SLOOP (fetched from
//  SDRAM through the arbiter), work RAM, the 4 KB RAM at $E80000, EEPROM,
//  watchdog, the ASIC65 coprocessor, the I/O latch and the register
//  interfaces to the palette, the RLE object engine, the ADC and the JSA III
//  sound board, which live in the top level. Follows MAME atarig42.cpp.
//
//  Interrupts: IRQ4 = VBLANK start, cleared by a write to $E03000; IRQ5 =
//  JSA III. The acknowledge cycle is ended with DTACK and the autovector
//  number on the data bus (as the Atari G1 core, which found VPA-style
//  autovectoring did not terminate on hardware).
//
//  I/O latch ($E00050), reset to 0 like the board's latch:
//    D14    ASIC65 reset, active low      D4     JSA III reset, active low
//    D13:11 MO control {FRAME,ERASE,MOGO} D3, D0 coin counters (not used)
//  So after reset the sound board and the ASIC65 stay in reset until the game
//  releases them (Guardians within its first frames; Road Riot keeps the
//  sound board in reset through its five-second start-up tests, as in MAME).
//
//  The dbg_* outputs are diagnostic probes for the OSD overlay. The sticky
//  ones are cleared only by dbg_clr, so they survive watchdog resets.
//
//  Clock domain: clk_sys, the 68000 gated by clock enables from g42_ce.
//============================================================================

`default_nettype none

module g42_top
	import g42_pkg::*;
#(
	parameter bit ROM_CACHE      = 1'b1, // 0: every program ROM read goes to SDRAM
	parameter int ROM_CACHE_BITS = 13    // cache size: 2^n words (13 = 16 KB, 22 M10K)
)(
	input  wire         clk,
	input  wire         rst_n,

	// ---- Clock enables ------------------------------------------------------
	input  wire         ce_cpu_p1,
	input  wire         ce_cpu_p2,

	// ---- Configuration ------------------------------------------------------
	input  g42_cfg_t    cfg,
	input  wire         disable_wd,          // OSD: watchdog off
	input  wire         dbg_clr,             // clears the sticky probes

	// ---- Raster --------------------------------------------------------------
	input  wire         vblank_rise,         // one clock at VBLANK start

	// ---- Input ports (assembled in the top level) ----------------------------
	input  wire [15:0]  in0,
	input  wire [15:0]  in1,
	input  wire [15:0]  in2,
	input  wire [7:0]   jsaiii_port,

	// ---- ADC0809 (Road Riot) --------------------------------------------------
	input  wire [7:0]   adc_data,
	output logic [2:0]  adc_chan_sel,
	output logic        adc_start,           // one clock per access to $E00020-$E0002F

	// ---- Program ROM fetch (SDRAM, through the arbiter) -----------------------
	output logic [24:0] rom_addr,            // SDRAM byte address, held with rom_req
	output logic        rom_req,             // held until rom_ack
	input  wire [15:0]  rom_dout,
	input  wire         rom_ack,

	// ---- Palette (in the top level, shared with the video scan-out) -----------
	output logic [10:0] pal_addr,
	output logic [15:0] pal_din,
	output logic        pal_wr_hi,
	output logic        pal_wr_lo,
	input  wire [15:0]  pal_dout,

	// ---- Work RAM video port (g42_video) --------------------------------------
	// Port B read (vram_addr/vram_dout) and the RLE CHECKSUM write-back, which
	// shares the 68000's port for a clock at a time (see g42_mainram).
	input  wire [14:0]  vram_addr,
	output logic [15:0] vram_dout,
	input  wire [14:0]  vram_din_addr,
	input  wire [15:0]  vram_din,
	input  wire         vram_we,

	// ---- JSA III ----------------------------------------------------------------
	output logic [7:0]  snd_cmd,             // $E00041 command byte
	output logic        snd_cmd_wr,          // one clock per command write
	output logic        snd_resp_rd,         // one clock per read of $E00031
	input  wire [7:0]   snd_resp,
	input  wire         snd_int,             // JSA III -> IRQ5
	output logic        snd_reset,           // level: I/O latch bit 4 is 0

	// ---- RLE engine ---------------------------------------------------------------
	output logic [2:0]  rle_ctrl,            // I/O latch bits 13:11
	output logic        rle_ctrl_wr,         // one clock per write of the latch's upper byte
	output logic [15:0] rle_cmd,             // value written to $FF7000
	output logic        rle_cmd_wr,          // one clock per write to $FF7000
	output logic [15:0] rle_objram_w0,       // work RAM word 0, snooped

	// ---- ASIC65 program download (Road Riot) -------------------------------------
	input  wire         dsp_prog_we,
	input  wire [11:0]  dsp_prog_addr,
	input  wire [15:0]  dsp_prog_data,

	// ---- EEPROM: factory image and MiSTer NVRAM -------------------------------------
	input  wire         eep_def_we,
	input  wire [10:0]  eep_def_addr,
	input  wire [7:0]   eep_def_din,
	input  wire         nv_wr,
	input  wire [10:0]  nv_addr,
	input  wire [7:0]   nv_din,
	output logic [7:0]  nv_dout,
	output logic        nv_dirty,

	// ---- Debug ------------------------------------------------------------------------
	output logic [23:0] dbg_cpu_addr,        // address of the current bus cycle
	// Boot progress, sticky: 7 palette write, 6 watchdog kick, 5 IRQ4 ack,
	// 4 I/O latch, 3 sound command, 2 EEPROM, 1 ASIC65, 0 work RAM
	output logic [7:0]  dbg_hit,
	output logic [15:0] dbg_wd_count,        // watchdog resets
	output logic [23:0] dbg_wd_addr,         // last bus cycle before the latest one
	output logic [15:0] dbg_iack_cnt,        // interrupt acknowledge cycles
	output logic [15:0] dbg_io_latch,        // last I/O latch value
	output logic [1:0]  dbg_sloop_bank,
	output logic [7:0]  dbg_sloop_changes,   // bank changes, wraps
	output logic [23:0] dbg_rom_wait,        // clk_sys the 68000 waited on ROM last frame
	output logic [15:0] dbg_rom_lookups,     // program ROM reads last frame (saturating)
	output logic [15:0] dbg_rom_misses,      // ... of which cache misses
	output logic [15:0] dbg_asic_pc,         // ASIC65 DSP program counter
	output logic [7:0]  dbg_asic_cmds        // ASIC65 command writes, wraps
);

	//========================================================================
	//  68000
	//========================================================================
	wire [23:1] cpu_a;
	wire [15:0] cpu_do;
	logic [15:0] cpu_di;
	wire        cpu_rw_n, cpu_as_n, cpu_uds_n, cpu_lds_n;
	wire [2:0]  cpu_fc;
	logic       cpu_dtack_n;

	wire wd_reset;

	// Interrupt levels (vectors from iack_vector):
	//   IRQ4 -- VBLANK, cleared by a write to $E03000
	//   IRQ5 -- JSA III sound board
	logic irq4;
	wire  irq5 = snd_int;

	// Registered: a combinational encoding glitches through other levels when
	// it changes, and fx68k needs IPL stable across successive samples.
	logic [2:0] ipl;
	always_ff @(posedge clk) ipl <= irq5 ? 3'd5 : (irq4 ? 3'd4 : 3'd0);

	// Interrupt acknowledge: FC = 111 while AS is low. The AS term matters: FC
	// can read 111 between bus cycles, and is_iack overrides the read data.
	wire is_iack = (cpu_fc == 3'b111) && !cpu_as_n;

	// Autovector number 24 + level on D7:0: 28 ($70) VBLANK, 29 ($74) sound.
	wire [15:0] iack_vector = {8'h00, 5'b00011, cpu_a[3:1]};

	/* verilator lint_off PINCONNECTEMPTY */
	fx68k u_cpu (
		.clk        (clk),
		.extReset   (!rst_n || wd_reset),
		.pwrUp      (!rst_n),
		.enPhi1     (ce_cpu_p1),
		.enPhi2     (ce_cpu_p2),

		.eab        (cpu_a),
		.oEdb       (cpu_do),
		.iEdb       (cpu_di),

		.eRWn       (cpu_rw_n),
		.ASn        (cpu_as_n),
		.LDSn       (cpu_lds_n),
		.UDSn       (cpu_uds_n),
		.E          (),
		.VMAn       (),
		.FC0        (cpu_fc[0]),
		.FC1        (cpu_fc[1]),
		.FC2        (cpu_fc[2]),

		.BGn        (),
		.oRESETn    (),
		.oHALTEDn   (),

		.VPAn       (1'b1),               // vector supplied on the data bus
		.DTACKn     (cpu_dtack_n),
		.BERRn      (1'b1),
		.HALTn      (1'b1),
		.BRn        (1'b1),
		.BGACKn     (1'b1),

		.IPL0n      (~ipl[0]),
		.IPL1n      (~ipl[1]),
		.IPL2n      (~ipl[2])
	);
	/* verilator lint_on PINCONNECTEMPTY */

	assign dbg_cpu_addr = {cpu_a, 1'b0};

	//========================================================================
	//  Bus cycle events
	//========================================================================
	// cyc_start: first clock of a bus cycle (AS falling edge; the address is
	// stable). wr_start: first clock a write's data strobes are active; the
	// data is valid from then on. rd_start: first clock of a read with its
	// strobes (they assert with AS on a read). Devices with side effects act
	// on these one-clock events, never on the multi-clock select levels.
	logic as_n_d, wstb_d, rstb_d;
	wire  wstb = !cpu_as_n && !cpu_rw_n && (!cpu_uds_n || !cpu_lds_n);
	wire  rstb = !cpu_as_n &&  cpu_rw_n && (!cpu_uds_n || !cpu_lds_n);

	always_ff @(posedge clk) begin
		as_n_d <= cpu_as_n;
		wstb_d <= wstb;
		rstb_d <= rstb;
	end

	wire cyc_start = as_n_d && !cpu_as_n;
	wire wr_start  = wstb && !wstb_d;
	wire rd_start  = rstb && !rstb_d;

	//========================================================================
	//  Address decode
	//========================================================================
	wire sel_rom, sel_rom_wr, sel_rom_x, sel_in0, sel_in1, sel_in2, sel_jsaiii;
	wire sel_adc, sel_snd_resp, sel_snd_cmd, sel_io_latch, sel_eeprom_unlock;
	wire sel_irq_ack, sel_watchdog, sel_ram2, sel_asic_stat, sel_asic_read;
	wire sel_asic_write, sel_eeprom, sel_palette, sel_ram, sel_unmapped;
	wire [14:0] dec_ram_addr;
	wire [10:0] dec_ram2_addr, dec_pal_addr, dec_eeprom_addr;
	wire [2:0]  dec_adc_chan;
	wire        dec_asic_cmd;

	g42_addr_decode u_decode (
		.addr              (cpu_a),
		// No selects during IACK: the CPU drives A23-A4 high, which decodes as
		// work RAM and would put RAM data on the bus instead of the vector.
		.as_n              (cpu_as_n | is_iack),
		.rw_n              (cpu_rw_n),
		.uds_n             (cpu_uds_n),
		.lds_n             (cpu_lds_n),

		.sel_rom           (sel_rom),
		.sel_rom_wr        (sel_rom_wr),
		.sel_rom_x         (sel_rom_x),
		.sel_in0           (sel_in0),
		.sel_in1           (sel_in1),
		.sel_in2           (sel_in2),
		.sel_jsaiii        (sel_jsaiii),
		.sel_adc           (sel_adc),
		.sel_snd_resp      (sel_snd_resp),
		.sel_snd_cmd       (sel_snd_cmd),
		.sel_io_latch      (sel_io_latch),
		.sel_eeprom_unlock (sel_eeprom_unlock),
		.sel_irq_ack       (sel_irq_ack),
		.sel_watchdog      (sel_watchdog),
		.sel_ram2          (sel_ram2),
		.sel_asic_stat     (sel_asic_stat),
		.sel_asic_read     (sel_asic_read),
		.sel_asic_write    (sel_asic_write),
		.sel_eeprom        (sel_eeprom),
		.sel_palette       (sel_palette),
		.sel_ram           (sel_ram),
		.sel_unmapped      (sel_unmapped),

		.ram_addr          (dec_ram_addr),
		.ram2_addr         (dec_ram2_addr),
		.pal_addr          (dec_pal_addr),
		.eeprom_addr       (dec_eeprom_addr),
		.adc_chan          (dec_adc_chan),
		.asic_cmd          (dec_asic_cmd)
	);

	// Per-lane write levels, for the plain memories (repeating a write is
	// harmless there).
	wire wr_hi = !cpu_rw_n && !cpu_uds_n;
	wire wr_lo = !cpu_rw_n && !cpu_lds_n;

	//========================================================================
	//  SLOOP and program ROM
	//========================================================================
	// The SLOOP sees every cycle in $000000-$07FFFF, reads (including opcode
	// fetches) and writes, as MAME's handler does. Counted at AS, when the
	// address is stable; a write's strobes are not needed.
	wire in_rom_space = (cpu_a[23:19] == 5'd0) && !is_iack;
	wire [1:0] sloop_bank;

	g42_sloop u_sloop (
		.clk        (clk),
		.rst_n      (rst_n),
		.sloop_type (cfg.sloop),
		.cpu_addr   (cpu_a[18:1]),
		.acc_strobe (cyc_start && in_rom_space),
		.bank       (sloop_bank)
	);

	assign dbg_sloop_bank = sloop_bank;

	logic [1:0] sloop_bank_d;
	always_ff @(posedge clk) begin
		sloop_bank_d <= sloop_bank;
		if (dbg_clr) dbg_sloop_changes <= 8'd0;
		else if (sloop_bank != sloop_bank_d) dbg_sloop_changes <= dbg_sloop_changes + 1'b1;
	end

	// Banked window $078000-$07FFFF -> ROM $078000 + bank x $2000, the 8 KB
	// bank mirrored four times (MAME: 0x78000/2 + bank * 0x1000 + (offset & 0xfff)).
	wire        in_window = (cfg.sloop != SLOOP_NONE) && (cpu_a[18:15] == 4'hF);
	wire [18:1] rom_word  = in_window ? {4'hF, sloop_bank, cpu_a[12:1]} : cpu_a[18:1];

	//------------------------------------------------------------------------
	// Program ROM cache
	//------------------------------------------------------------------------
	// Every 68000 read of the program ROM would otherwise wait for the SDRAM:
	// about ten clk_sys from the strobes to the word, which costs one or two
	// wait states a cycle where the real board's EPROMs need none. A 16 KB
	// direct-mapped cache (one word per line, indexed by the physical ROM
	// address, so the SLOOP banks need no special case) answers a hit in
	// three clk_sys, inside the zero-wait window. It holds only ROM, which
	// nothing writes, so it is never flushed while a game runs; while rst_n
	// is low it clears itself (a download holds reset far longer than the
	// 8,192 clocks this takes).
	//
	// Sequential prefetch. When a read of word X is answered from SDRAM (or
	// from a prefetch, or hits the word the last prefetch brought in), word
	// X+1 is fetched into the cache in the background. Straight-line code and
	// the games' ROM table copies then find their next word in the cache, or
	// arriving, so a stream of reads costs one SDRAM latency, not one each.
	//
	// Timing of one read cycle (T = first clock its strobes are active):
	//   T+1  rom_go: rom_word carries the bank this cycle may have selected
	//        (the SLOOP updates the clock after AS falls); cache lookup
	//   T+2  rom_lk: tag compare. A miss joins a prefetch of the same word
	//        already in flight, or requests the word
	//   T+3  hit: rom_have and DTACK
	//   miss: the word's arrival writes the cache and gives DTACK
	// One fetch is in flight at a time (f_busy): g42_core's channel takes one
	// request and says nothing until its ack, so a presented request and its
	// address are held until the ack. A request is raised only by a cycle's
	// own lookup or by a prefetch, never by a level that could re-arm in the
	// clock a fetch completes.
	localparam int CW = ROM_CACHE_BITS;      // index bits
	localparam int TW = 18 - CW;             // tag bits: the rest of rom_word[18:1]

	logic rom_go, rom_lk;
	always_ff @(posedge clk) begin
		rom_go <= rd_start && sel_rom && rst_n;
		rom_lk <= rom_go;
	end

	wire [CW-1:0] c_idx    = rom_word[CW:1];
	wire [TW-1:0] c_tag_in = rom_word[18:CW+1];

	logic [15:0]   c_data [2**CW];
	logic [TW:0]   c_tags [2**CW];           // {valid, tag}
	logic [15:0]   c_q;
	logic [TW:0]   c_tq;
	logic [CW-1:0] c_clr;

	// ---- The fetch in flight ----
	logic        f_busy;                     // presented, not yet acknowledged
	logic        f_pf;                       // it is a prefetch
	logic [18:1] f_word;                     // its word address
	logic        d_wait;                     // this cycle missed and waits for its word
	logic [18:1] pf_last;                    // word the last prefetch brought in

	wire f_ack   = rom_ack && f_busy;
	wire f_match = (f_word == rom_word);

	wire c_hit  = ROM_CACHE && (c_tq == {1'b1, c_tag_in});
	wire c_miss = rom_lk && !c_hit;

	// The word for this cycle arrives now: a demand fetch, or a prefetch of
	// the same word, acknowledged while the cycle waits (or in its lookup clock).
	wire deliver = f_ack && f_match && (d_wait || c_miss);

	// A missed cycle presents its own request once no other fetch is out; the
	// request goes out combinationally in that clock and is then held by
	// f_busy.
	wire d_start = (c_miss || d_wait) && !deliver && !f_busy;

	assign rom_req  = f_busy || d_start;
	assign rom_addr = SDR_PROG + {6'd0, (f_busy ? f_word : rom_word), 1'b0};

	// Next prefetch: word X+1 when X is delivered from a fetch, or hits the
	// word the last prefetch brought in. Launched only into a free slot.
	wire        pf_hit   = rom_lk && c_hit && (rom_word == pf_last);
	wire        pf_start = ROM_CACHE && (deliver || pf_hit) && (!f_busy || f_ack);
	wire [18:1] pf_word  = rom_word + 18'd1;

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			f_busy  <= 1'b0;
			f_pf    <= 1'b0;
			d_wait  <= 1'b0;
			pf_last <= '0;
		end else begin
			if (f_ack) begin
				f_busy <= 1'b0;
				if (f_pf) pf_last <= f_word;
			end
			if (d_start) begin
				f_busy <= 1'b1;
				f_pf   <= 1'b0;
				f_word <= rom_word;
			end
			if (pf_start) begin
				f_busy <= 1'b1;
				f_pf   <= 1'b1;
				f_word <= pf_word;
			end

			if (c_miss && !deliver) d_wait <= 1'b1;
			if (deliver || cpu_as_n || cyc_start) d_wait <= 1'b0;
		end
	end

	// Cache writes: every fetch that arrives, demand or prefetch
	wire [CW-1:0] f_idx = f_word[CW:1];
	wire [TW-1:0] f_tag = f_word[18:CW+1];

	always_ff @(posedge clk) begin
		if (f_ack) c_data[f_idx] <= rom_dout;
		c_q <= c_data[c_idx];
	end

	wire [CW-1:0] t_wa = rst_n ? f_idx : c_clr;
	wire [TW:0]   t_wd = rst_n ? {1'b1, f_tag} : '0;

	always_ff @(posedge clk) begin
		if (!rst_n || f_ack) c_tags[t_wa] <= t_wd;
		c_tq  <= c_tags[c_idx];
		c_clr <= rst_n ? '0 : c_clr + 1'b1;
	end

	// rom_have: rom_data belongs to the cycle in progress. Cleared while AS is
	// high and at every cycle start, after the set terms, so a new cycle can
	// never be acknowledged with the previous cycle's word. On a miss DTACK
	// also comes in the delivery clock itself, with the word straight from the
	// arbiter (its output register holds it).
	logic        rom_have;
	logic [15:0] rom_data;

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			rom_have <= 1'b0;
			rom_data <= 16'h0000;
		end else begin
			if (rom_lk && c_hit) begin
				rom_data <= c_q;
				rom_have <= 1'b1;
			end
			if (deliver) begin
				rom_data <= rom_dout;
				rom_have <= 1'b1;
			end
			if (cpu_as_n || cyc_start) rom_have <= 1'b0;
		end
	end

	wire        rom_ready   = rom_have || deliver;
	wire [15:0] rom_word_in = rom_have ? rom_data : rom_dout;

	// Hit statistics for the overlay: lookups and misses per frame
	logic [15:0] c_look_cnt, c_miss_cnt;
	always_ff @(posedge clk) begin
		if (vblank_rise) begin
			dbg_rom_lookups <= c_look_cnt;
			dbg_rom_misses  <= c_miss_cnt;
			c_look_cnt <= '0;
			c_miss_cnt <= '0;
		end else begin
			if (rom_lk && c_look_cnt != 16'hFFFF) c_look_cnt <= c_look_cnt + 1'b1;
			if (c_miss && c_miss_cnt != 16'hFFFF) c_miss_cnt <= c_miss_cnt + 1'b1;
		end
	end

	// The word at $080000: MAME's region is $80004 bytes, zero past the
	// program, and Guardians' init patches an RTS in (the game calls $080000).
	wire [15:0] rom_x_data = cfg.flags[2] ? 16'h4E75 : 16'h0000;

	//========================================================================
	//  Work RAM ($FF0000) and its snoops
	//========================================================================
	wire [15:0] ram_dout;

	g42_mainram u_ram (
		.clk       (clk),
		.cpu_addr  (dec_ram_addr),
		.cpu_din   (cpu_do),
		.cpu_wr_hi (sel_ram && wr_hi),
		.cpu_wr_lo (sel_ram && wr_lo),
		.cpu_dout  (ram_dout),
		.ext_addr  (vram_din_addr),
		.ext_din   (vram_din),
		.ext_we    (vram_we),
		.vid_addr  (vram_addr),
		.vid_dout  (vram_dout)
	);

	// $FF7000 (word $3800) is the MO command: ordinary RAM that the MO
	// hardware reacts to (MAME mo_command_w: 0 = CHECKSUM, else DRAW). A byte
	// write carries the byte on both halves, so the zero test holds for byte
	// writes too. Word 0 is snooped for the CHECKSUM count.
	always_ff @(posedge clk) begin
		rle_cmd_wr <= 1'b0;
		if (!rst_n) begin
			rle_cmd       <= 16'd0;
			rle_objram_w0 <= 16'd0;
		end
		else if (wr_start && sel_ram) begin
			if (dec_ram_addr == 15'h3800) begin
				rle_cmd    <= cpu_do;
				rle_cmd_wr <= 1'b1;
			end
			if (dec_ram_addr == 15'h0000) begin
				if (!cpu_uds_n) rle_objram_w0[15:8] <= cpu_do[15:8];
				if (!cpu_lds_n) rle_objram_w0[7:0]  <= cpu_do[7:0];
			end
		end
	end

	//========================================================================
	//  4 KB RAM ($E80000)
	//========================================================================
	// Plain RAM in MAME. One array per byte lane so byte writes infer as
	// byte enables.
	logic [7:0] ram2_hi [2048];
	logic [7:0] ram2_lo [2048];
	logic [7:0] ram2_q_hi, ram2_q_lo;

	always_ff @(posedge clk) begin
		if (sel_ram2 && wr_hi) ram2_hi[dec_ram2_addr] <= cpu_do[15:8];
		ram2_q_hi <= ram2_hi[dec_ram2_addr];
	end

	always_ff @(posedge clk) begin
		if (sel_ram2 && wr_lo) ram2_lo[dec_ram2_addr] <= cpu_do[7:0];
		ram2_q_lo <= ram2_lo[dec_ram2_addr];
	end

	//========================================================================
	//  Palette
	//========================================================================
	assign pal_addr  = dec_pal_addr;
	assign pal_din   = cpu_do;
	assign pal_wr_hi = sel_palette && wr_hi;
	assign pal_wr_lo = sel_palette && wr_lo;

	//========================================================================
	//  EEPROM
	//========================================================================
	wire [7:0] eeprom_dout;

	g42_eeprom u_eeprom (
		.clk        (clk),
		.rst_n      (rst_n),
		.cpu_addr   (dec_eeprom_addr),
		.cpu_din    (cpu_do[7:0]),
		.cpu_wr     (wr_start && sel_eeprom && !cpu_lds_n),
		.cpu_unlock (wr_start && sel_eeprom_unlock),
		.cpu_dout   (eeprom_dout),
		.def_we     (eep_def_we),
		.def_addr   (eep_def_addr),
		.def_din    (eep_def_din),
		.nv_wr      (nv_wr),
		.nv_addr    (nv_addr),
		.nv_din     (nv_din),
		.nv_dout    (nv_dout),
		.nv_dirty   (nv_dirty)
	);

	//========================================================================
	//  I/O latch ($E00050)
	//========================================================================
	// Written per byte lane (MAME io_latch_w with mem_mask). The MO control
	// bits are passed on every upper-byte write; g42_rle ignores a write that
	// does not change them, as MAME's control_write does.
	logic [15:0] io_latch;

	always_ff @(posedge clk) begin
		rle_ctrl_wr <= 1'b0;
		if (!rst_n) begin
			io_latch <= 16'h0000;
			rle_ctrl <= 3'd0;
		end
		else if (wr_start && sel_io_latch) begin
			if (!cpu_uds_n) begin
				io_latch[15:8] <= cpu_do[15:8];
				rle_ctrl       <= cpu_do[13:11];
				rle_ctrl_wr    <= 1'b1;
			end
			if (!cpu_lds_n) io_latch[7:0] <= cpu_do[7:0];
		end
	end

	assign snd_reset    = !io_latch[4];
	assign dbg_io_latch = io_latch;

	wire asic_reset = !io_latch[14];

	//========================================================================
	//  ASIC65
	//========================================================================
	wire [15:0] asic_rdata, asic_status;
	wire [15:0] asic_pc;
	wire [7:0]  asic_cmds;

	/* verilator lint_off PINCONNECTEMPTY */
	g42_asic65 #(.DRAM_WORDS(256)) u_asic65 (
		.clk         (clk),
		.rst_n       (rst_n),
		.rom_based   (cfg.flags[1]),
		.asic_reset  (asic_reset),
		.host_wr     (wr_start && sel_asic_write),
		.host_wr_cmd (dec_asic_cmd),
		.host_wdata  (cpu_do),
		.host_rd     (rd_start && sel_asic_read),
		.host_rdata  (asic_rdata),
		.host_status (asic_status),
		.prog_we     (dsp_prog_we),
		.prog_addr   (dsp_prog_addr),
		.prog_data   (dsp_prog_data),
		.dsp_pc      (asic_pc),
		.dsp_running (),
		.cmd_count   (asic_cmds),
		.irq_count   ()
	);
	/* verilator lint_on PINCONNECTEMPTY */

	assign dbg_asic_pc   = asic_pc;
	assign dbg_asic_cmds = asic_cmds;

	// The data read is registered two clocks after host_rd: hold DTACK until
	// then. The 68000 would sample it later anyway, but this does not rely on it.
	logic [1:0] asic_rd_wait;
	always_ff @(posedge clk) begin
		if (cpu_as_n)                         asic_rd_wait <= 2'd0;
		else if (sel_asic_read && asic_rd_wait != 2'd3) asic_rd_wait <= asic_rd_wait + 2'd1;
	end
	wire asic_rd_ready = (asic_rd_wait == 2'd3);

	//========================================================================
	//  Sound board interface
	//========================================================================
	// $E00041 and $E00031 are byte handlers on the odd address (D7:0, /LDS).
	always_ff @(posedge clk) begin
		snd_cmd_wr <= 1'b0;
		if (!rst_n) begin
			snd_cmd <= 8'h00;
		end
		else if (wr_start && sel_snd_cmd && !cpu_lds_n) begin
			snd_cmd    <= cpu_do[7:0];
			snd_cmd_wr <= 1'b1;
		end
	end

	assign snd_resp_rd = rd_start && sel_snd_resp && !cpu_lds_n;

	//========================================================================
	//  ADC0809 (Road Riot)
	//========================================================================
	// A read or a write of $E00020-$E0002F starts a conversion of the channel
	// it addresses (MAME a2d_data_r / a2d_select_w). Without an ADC the reads
	// return $FF and nothing starts.
	wire has_adc = cfg.flags[0];

	assign adc_chan_sel = dec_adc_chan;
	assign adc_start    = has_adc && sel_adc && (rd_start || wr_start);

	//========================================================================
	//  Interrupts
	//========================================================================
	always_ff @(posedge clk) begin
		if (!rst_n)                          irq4 <= 1'b0;
		else if (wr_start && sel_irq_ack)    irq4 <= 1'b0;
		else if (vblank_rise)                irq4 <= 1'b1;
	end

	//========================================================================
	//  Watchdog
	//========================================================================
	g42_watchdog u_watchdog (
		.clk        (clk),
		.rst_n      (rst_n),
		.disable_wd (disable_wd),
		.kick       (wr_start && sel_watchdog),
		.frame_tick (vblank_rise),
		.wd_reset   (wd_reset)
	);

	//========================================================================
	//  Read data multiplexer and DTACK
	//========================================================================
	// Everything but ROM and the ASIC65 data port answers within a clock
	// (block RAM or a register); the 68000 samples data several clk_sys after
	// DTACK, so their DTACK is immediate. Unmapped reads return $0000 (MAME).
	always_comb begin
		cpu_di = 16'h0000;
		if      (is_iack)       cpu_di = iack_vector;
		else if (sel_rom)       cpu_di = rom_word_in;
		else if (sel_rom_x)     cpu_di = rom_x_data;
		else if (sel_ram)       cpu_di = ram_dout;
		else if (sel_ram2)      cpu_di = {ram2_q_hi, ram2_q_lo};
		else if (sel_palette)   cpu_di = pal_dout;
		else if (sel_eeprom)    cpu_di = {8'h00, eeprom_dout};
		else if (sel_in0)       cpu_di = in0;
		else if (sel_in1)       cpu_di = in1;
		else if (sel_in2)       cpu_di = in2;
		else if (sel_jsaiii)    cpu_di = {8'h00, jsaiii_port};
		else if (sel_adc)       cpu_di = {has_adc ? adc_data : 8'hFF, 8'h00};
		else if (sel_snd_resp)  cpu_di = {8'h00, snd_resp};
		else if (sel_asic_stat) cpu_di = asic_status;
		else if (sel_asic_read) cpu_di = asic_rdata;
	end

	always_comb begin
		if      (sel_rom)       cpu_dtack_n = !rom_ready;
		else if (sel_asic_read) cpu_dtack_n = !asic_rd_ready;
		else                    cpu_dtack_n = cpu_as_n;
	end

	//========================================================================
	//  Diagnostic probes
	//========================================================================
	// Boot progress, sticky
	always_ff @(posedge clk) begin
		if (dbg_clr) dbg_hit <= 8'd0;
		else begin
			if (sel_palette && !cpu_rw_n)   dbg_hit[7] <= 1'b1;
			if (sel_watchdog)               dbg_hit[6] <= 1'b1;
			if (sel_irq_ack)                dbg_hit[5] <= 1'b1;
			if (sel_io_latch)               dbg_hit[4] <= 1'b1;
			if (sel_snd_cmd)                dbg_hit[3] <= 1'b1;
			if (sel_eeprom)                 dbg_hit[2] <= 1'b1;
			if (sel_asic_write)             dbg_hit[1] <= 1'b1;
			if (sel_ram)                    dbg_hit[0] <= 1'b1;
		end
	end

	// Interrupt acknowledges
	logic iack_d;
	always_ff @(posedge clk) begin
		iack_d <= is_iack;
		if (dbg_clr)                  dbg_iack_cnt <= 16'd0;
		else if (is_iack && !iack_d)  dbg_iack_cnt <= dbg_iack_cnt + 1'b1;
	end

	// Watchdog resets, and the bus cycle in progress when the latest fired
	logic wd_reset_d;
	logic [23:0] last_addr;
	always_ff @(posedge clk) begin
		if (cyc_start) last_addr <= {cpu_a, 1'b0};
		wd_reset_d <= wd_reset;
		if (dbg_clr) dbg_wd_count <= 16'd0;
		else if (wd_reset && !wd_reset_d) begin
			dbg_wd_count <= dbg_wd_count + 1'b1;
			dbg_wd_addr  <= last_addr;
		end
	end

	// Clocks per frame the 68000 was held for program ROM, out of 954,480:
	// clocks without DTACK from the fifth after the strobes on. A 68000 read
	// needs DTACK about five clk_sys after its strobes to run without wait
	// states, so this counts the wait states the SDRAM costs (cache misses).
	logic [23:0] rom_wait_cnt;
	logic [2:0]  rom_cyc_clk;
	always_ff @(posedge clk) begin
		if (rd_start)                rom_cyc_clk <= 3'd1;
		else if (rom_cyc_clk != 3'd7) rom_cyc_clk <= rom_cyc_clk + 1'b1;

		if (vblank_rise) begin
			dbg_rom_wait <= rom_wait_cnt;
			rom_wait_cnt <= 24'd0;
		end
		else if (sel_rom && !rom_ready && rom_cyc_clk >= 3'd5) begin
			rom_wait_cnt <= rom_wait_cnt + 1'b1;
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	// Unmapped accesses are legal (MAME ignores them) but worth seeing in a
	// simulation log.
	always_ff @(posedge clk)
		if ((rd_start || wr_start) && sel_unmapped)
			$display("g42_top: unmapped %s at $%06X", cpu_rw_n ? "read" : "write", {cpu_a, 1'b0});

	// With +iolog: every access to the sound board, the ASIC65 and the I/O
	// latch, with the frame number, for comparison with a MAME tap log.
	// The raster line is counted from VBLANK (line 240), 3,648 clk_sys a line.
	bit   iolog;
	int   sim_frame, since_vbl, line;
	initial iolog = $test$plusargs("iolog");
	always_ff @(posedge clk) begin
		if (!rst_n)           sim_frame <= 0;
		else if (vblank_rise) sim_frame <= sim_frame + 1;
		since_vbl <= vblank_rise ? 0 : since_vbl + 1;
	end
	assign line = (240 + since_vbl / 3648) % 262;

	// With +buslog: per frame, the number of 68000 read cycles and the clk_sys
	// AS was low for, split into program ROM and everything else, to see
	// whether ROM reads run at the speed of zero-wait RAM reads.
	bit buslog;
	initial buslog = $test$plusargs("buslog");
	int as_len, rom_cyc, rom_clk, oth_cyc, oth_clk;
	logic cyc_rom, cyc_rd;
	always_ff @(posedge clk) begin
		if (cyc_start) begin as_len <= 1; cyc_rom <= 1'b0; cyc_rd <= 1'b0; end
		else if (!cpu_as_n) as_len <= as_len + 1;
		if (rd_start) begin cyc_rom <= sel_rom; cyc_rd <= 1'b1; end
		if (!as_n_d && cpu_as_n && cyc_rd) begin     // AS rose: cycle over
			if (cyc_rom) begin rom_cyc <= rom_cyc + 1; rom_clk <= rom_clk + as_len; end
			else         begin oth_cyc <= oth_cyc + 1; oth_clk <= oth_clk + as_len; end
		end
		if (vblank_rise && rst_n) begin
			if (buslog) $display("BUS %0d rom %0d cycles %0d clk  other %0d cycles %0d clk", sim_frame,
			                     rom_cyc, rom_clk, oth_cyc, oth_clk);
			rom_cyc <= 0; rom_clk <= 0; oth_cyc <= 0; oth_clk <= 0;
		end
	end
	always_ff @(posedge clk) begin
		if (iolog && rst_n) begin
			if (wr_start && sel_snd_cmd)    $display("IO %0d %0d W E00040 %04X", sim_frame, line, cpu_do);
			if (wr_start && sel_io_latch)   $display("IO %0d %0d W E00050 %04X", sim_frame, line, cpu_do);
			if (wr_start && sel_irq_ack)    $display("IO %0d %0d W E03000", sim_frame, line);
			if (wr_start && sel_asic_write) $display("IO %0d %0d W %06X %04X", sim_frame, line, {cpu_a, 1'b0}, cpu_do);
			if (rd_start && sel_snd_resp)   $display("IO %0d %0d R E00030 %04X", sim_frame, line, {8'h00, snd_resp});
			if (rd_start && sel_asic_read)  $display("IO %0d %0d R F60000", sim_frame, line);
		end
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
