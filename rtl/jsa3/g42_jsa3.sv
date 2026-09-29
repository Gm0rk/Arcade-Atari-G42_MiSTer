//============================================================================
//  Atari G42 for MiSTer
//  g42_jsa3.sv -- Atari JSA III sound board
//
//  6502 @ 1.789773 MHz (T65), YM2151 @ 3.579545 MHz (JT51) and OKI6295 @
//  1.193182 MHz (JT6295) with banked ADPCM ROM, mono output. Follows MAME's
//  atari_jsa_iii_device (atarijsa.cpp, "full map verified from schematics
//  and Batman GALs") and is derived from the Atari G1 JSA II (g1_jsa2.sv),
//  whose hardware-verified bus timing, JT51/JT6295 hookup and comm latches it
//  keeps. MAME 0.264 and current MAME behave the same for everything here.
//
//  6502 memory map (MAME atarijsa3_map). The games use the mirrors: $2808
//  (OKI status), $280A/$280C/$280E, $2900 (volume), $2A08 and $2B00 (OKI).
//    $0000-$1FFF   RAM (8 KB)
//    $2000-$2001   YM2151                         mirror $07FE ($2000-$27FF)
//    $2800-$2801   read  OKI6295 status           mirror $05F8
//                  write overall volume           mirror $05F8
//    $2802         read  sound command from 68000 mirror $05F9
//    $2804         read  RDIO                     mirror $05F9
//    $2806         read/write IRQ acknowledge     mirror $05F9
//    $2A00-$2A01   write OKI6295                  mirror $05F8
//    $2A02         write sound response to 68000  mirror $05F9
//    $2A04         write WRIO                     mirror $05F9
//    $2A06         write MIX                      mirror $05F9
//    $3000-$3FFF   program ROM window, WRIO bits 7:6 x $1000 into the first
//                  16 KB of the 64 KB image (Guardians boots from $3000 and
//                  relies on bank 0 after reset)
//    $4000-$FFFF   fixed program ROM
//  So $2800-$2FFF decodes on a[9] (read/control block vs write block) and
//  a[2:1] (register); a[10], a[8:3] and a[0] are don't-care. Reads of the
//  write block and of $2806 return 0, as MAME's unmapped/ack reads.
//
//  RDIO ($2804) and the 68000's raw port $E00012 (jsaiii_port):
//    0x80 self test, 1 = on          0x08 service (coin door), active high
//    0x40 command latch empty        0x04 tilt, active high
//    0x20 response latch full        0x02 coin L, active high
//    0x10 self test, 1 = on          0x01 coin R, active high
//  MAME's rdio_r() XORs 0x90 when the test switch is on, so its 6502 always
//  reads 0 in bits 4 and 7 while the 68000 sees 1. As on the G1 JSA II, whose
//  schematic shows the 6502 reading the test line through an inverting
//  buffer, the 6502 here reads test-on as 1: at reset all three programs test
//  bit 7 of $280C and run their RAM/ROM diagnostics (bank walk $45/$85/$C5,
//  about 2 s) before the normal start-up, which MAME never does. Verified
//  against MAME with that bit forced on (sim/jsa3/README.md).
//
//  WRIO ($2A04, MAME wrio_w):
//    0xC0 program ROM bank           0x04 OKI6295 reset, active low
//    0x30 coin counters (unused)     0x02 OKI bank bit 0
//    0x08 OKI6295 PIN7: 1 = /132     0x01 YM2151 reset, active low
//
//  MIX ($2A06, MAME mix_w):
//    0x20 low-pass filter enable (not emulated, nor by MAME)
//    0x10 OKI bank bit 1
//    0x0E YM2151 volume, 0-7
//    0x01 OKI6295 volume, 0 = half, 1 = full
//  Overall volume ($2800 write): gain = data / 127 on YM and OKI. The games
//  write 0-$7F (a 33-step log table); Guardians never writes it and relies
//  on the reset value 1.0.
//
//  OKI address map (18 bits, 256 KB), into the 512 KB sample ROM:
//    $00000-$1FFFF  banked, entry {MIX4, WRIO1}: 0,1 -> ROM $00000,
//                   2 -> ROM $20000, 3 -> ROM $40000 (the phrase table too)
//    $20000-$3FFFF  fixed, ROM $60000
//
//  6502 IRQ = periodic timer (249.7 Hz) OR YM2151 IRQ, as MAME's
//  update_sound_irq(); $2806 read or write acknowledges the timer.
//
//  Clock domain: clk_sys, gated by ce_6502 / ce_ym / ce_oki.
//============================================================================

`default_nettype none

module g42_jsa3
	import g42_pkg::*;
(
	input  wire        clk,                 // clk_sys, 57.272727 MHz
	input  wire        rst_n,               // global reset
	input  wire        board_reset,         // level: 1 while io_latch bit 4 is 0 (whole board held in reset)

	//------------------------------------------------------------------------
	// Clock enables
	//------------------------------------------------------------------------
	input  wire        ce_6502,             // 1.789773 MHz
	input  wire        ce_ym,               // 3.579545 MHz
	input  wire        ce_oki,              // 1.193182 MHz

	//------------------------------------------------------------------------
	// 68000 interface
	//------------------------------------------------------------------------
	input  wire [7:0]  main_din,            // command byte from the 68000
	input  wire        main_wr,             // one clock per write to $E00041
	input  wire        main_rd,             // one clock per read of $E00031 (start of the bus cycle)
	output logic [7:0] main_dout,           // response latch
	output logic       main_irq,            // -> 68000 IRQ5
	output logic       main_to_sound_ready, // raw flag, 1 = command latch full
	output logic       sound_to_main_ready, // raw flag, 1 = response latch full

	//------------------------------------------------------------------------
	// Coin door, read by the 6502 (and raw by the 68000 at $E00012)
	//------------------------------------------------------------------------
	input  wire        coin_l,              // active high
	input  wire        coin_r,              // active high
	input  wire        tilt,                // active high
	input  wire        service,             // active high
	input  wire        self_test,           // 1 = test switch on
	output logic [7:0] jsaiii_port,         // raw JSAIII port as the 68000 reads it at $E00012

	//------------------------------------------------------------------------
	// Program ROM (SDRAM byte address, 16-bit big-endian data)
	//------------------------------------------------------------------------
	output logic [24:0] prog_addr,          // word address (bit 0 = 0), held with prog_req
	output logic        prog_req,           // held until prog_ack
	input  wire  [15:0] prog_dout,          // valid with prog_ack
	input  wire         prog_ack,           // one clock

	//------------------------------------------------------------------------
	// OKI6295 sample ROM (SDRAM byte address, 16-bit big-endian data)
	//------------------------------------------------------------------------
	output logic [24:0] oki_addr,           // word address (bit 0 = 0), held with oki_req
	output logic        oki_req,            // held until oki_ack
	input  wire  [15:0] oki_dout,           // valid with oki_ack
	input  wire         oki_ack,            // one clock

	//------------------------------------------------------------------------
	// Audio and debug
	//------------------------------------------------------------------------
	output logic signed [15:0] audio,       // mono, half of MAME's level (see Mixer)
	output logic [15:0] dbg_pc,             // 6502 program counter
	output logic        dbg_cpu_ce          // one clock per 6502 bus cycle (stall monitor)
);

	// The 6502 and everything the board reset clears. Registered, as it also
	// drives T65's asynchronous reset input (Res_n), which must not glitch.
	logic board_rst;
	always_ff @(posedge clk) board_rst <= !rst_n || board_reset;

	//========================================================================
	//  Communication latches
	//========================================================================
	wire [7:0] snd_from_main;
	wire       snd_nmi;
	logic      snd_rd_cmd, snd_wr_resp;
	wire [7:0] cpu_do;

	g42_sound_comm u_comm (
		.clk                 (clk),
		.rst_n               (rst_n),
		.sound_reset         (board_reset),
		.main_din            (main_din),
		.main_wr             (main_wr),
		.main_rd             (main_rd),
		.main_dout           (main_dout),
		.main_irq            (main_irq),
		.snd_din             (cpu_do),
		.snd_wr              (snd_wr_resp),
		.snd_rd              (snd_rd_cmd),
		.snd_dout            (snd_from_main),
		.snd_nmi             (snd_nmi),
		.main_to_sound_ready (main_to_sound_ready),
		.sound_to_main_ready (sound_to_main_ready)
	);

	//========================================================================
	//  6502
	//========================================================================
	logic [7:0] cpu_di;
	wire        cpu_rw_n;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [23:0] cpu_a;                      // bits 23:16 are the 65816 bank
	wire [63:0] cpu_regs;                   // PC, S, P, Y, X, A: only the PC is used
	/* verilator lint_on UNUSEDSIGNAL */

	logic sound_irq;

	// Bus-cycle enable: the ce_6502 grid, held back while a program-ROM byte
	// is still in flight from SDRAM (see Program ROM). All bus-cycle strobes
	// in this module use it.
	logic cpu_ce;

	// Unused outputs are left open. DEBUG is a VHDL record port that cannot be
	// named from SystemVerilog (the GHDL conversion flattens it to DEBUG_*).
	/* verilator lint_off PINCONNECTEMPTY */
	/* verilator lint_off PINMISSING */
	T65 u_cpu (
		.Mode    (2'b00),                   // 6502
		.BCD_en  (1'b1),
		.Res_n   (!board_rst),
		.Enable  (cpu_ce),
		.Clk     (clk),
		.Rdy     (1'b1),
		.Abort_n (1'b1),
		.IRQ_n   (!sound_irq),
		.NMI_n   (!snd_nmi),
		.SO_n    (1'b1),
		.R_W_n   (cpu_rw_n),
		.Sync    (),
		.EF      (), .MF (), .XF (), .ML_n (), .VP_n (), .VDA (), .VPA (),
		.A       (cpu_a),
		.DI      (cpu_di),
		.DO      (cpu_do),
		.Regs    (cpu_regs),
		.NMI_ack ()
	);
	/* verilator lint_on PINMISSING */
	/* verilator lint_on PINCONNECTEMPTY */

	assign dbg_pc     = cpu_regs[63:48];
	assign dbg_cpu_ce = cpu_ce;

	wire [15:0] a  = cpu_a[15:0];
	wire        wr = !cpu_rw_n;
	wire        rd =  cpu_rw_n;

	//========================================================================
	//  Address decode
	//========================================================================
	wire sel_ram  = (a[15:13] == 3'b000);                     // $0000-$1FFF
	wire sel_ym   = (a[15:11] == 5'b00100);                   // $2000-$27FF
	wire sel_io   = (a[15:11] == 5'b00101);                   // $2800-$2FFF
	wire sel_bank = (a[15:12] == 4'h3);                       // $3000-$3FFF
	wire sel_rom  = (a[15:14] != 2'b00);                      // $4000-$FFFF

	wire io_28    = sel_io && !a[9];                          // $2800 block
	wire io_2a    = sel_io &&  a[9];                          // $2A00 block
	wire [1:0] io_sel = a[2:1];

	wire rd_oki   = io_28 && rd && (io_sel == 2'd0);          // $2800
	wire wr_vol   = io_28 && wr && (io_sel == 2'd0);          // $2800
	wire rd_cmd   = io_28 && rd && (io_sel == 2'd1);          // $2802
	wire rd_rdio  = io_28 && rd && (io_sel == 2'd2);          // $2804
	wire irq_ack  = io_28 &&       (io_sel == 2'd3);          // $2806
	wire wr_oki   = io_2a && wr && (io_sel == 2'd0);          // $2A00
	wire wr_resp  = io_2a && wr && (io_sel == 2'd1);          // $2A02
	wire wr_wrio  = io_2a && wr && (io_sel == 2'd2);          // $2A04
	wire wr_mix   = io_2a && wr && (io_sel == 2'd3);          // $2A06

	// Comm latch accesses are single-clock strobes: the 6502 holds its address
	// for a whole bus cycle.
	logic rd_cmd_d, wr_resp_d;
	always_ff @(posedge clk) begin
		if (cpu_ce) begin
			rd_cmd_d  <= rd_cmd;
			wr_resp_d <= wr_resp;
		end
	end
	assign snd_rd_cmd  = cpu_ce && rd_cmd  && !rd_cmd_d;
	assign snd_wr_resp = cpu_ce && wr_resp && !wr_resp_d;

	//========================================================================
	//  WRIO / MIX / overall volume
	//========================================================================
	logic [1:0] rom_bank;
	logic       oki_pin7;      // "voice frequency": 1 = /132, 0 = /165
	logic       oki_reset_n;
	logic       ym_reset_n;
	logic [1:0] oki_bank;      // {MIX bit 4, WRIO bit 1}
	logic [2:0] ym_volume;
	logic       oki_volume;    // 0 = half, 1 = full
	logic [7:0] overall_vol;   // gain = overall_vol / 127

	// YM2151 CT1 output pin; gates the OKI as in MAME (see Mixer).
	wire        ym_ct1;

	// Reset values: WRIO as cleared (YM and OKI held in reset until the 6502
	// releases them, bank 0), PIN7 high and the volumes at MAME's reset
	// values (1.0), which Guardians depends on.
	always_ff @(posedge clk) begin
		if (board_rst) begin
			rom_bank    <= 2'd0;
			oki_pin7    <= 1'b1;
			oki_reset_n <= 1'b0;
			ym_reset_n  <= 1'b0;
			oki_bank    <= 2'd0;
			ym_volume   <= 3'd7;
			oki_volume  <= 1'b1;
			overall_vol <= 8'd127;
		end
		else if (cpu_ce) begin
			if (wr_wrio) begin
				rom_bank    <= cpu_do[7:6];
				oki_pin7    <= cpu_do[3];
				oki_reset_n <= cpu_do[2];
				oki_bank[0] <= cpu_do[1];
				ym_reset_n  <= cpu_do[0];
				// Bits 5:4 (coin counters) have nothing to drive.
			end
			if (wr_mix) begin
				oki_bank[1] <= cpu_do[4];
				ym_volume   <= cpu_do[3:1];
				oki_volume  <= cpu_do[0];
				// Bit 5 (low-pass filter enable) is not emulated.
			end
			if (wr_vol) overall_vol <= cpu_do;
		end
	end

	//========================================================================
	//  Interrupts
	//========================================================================
	// Periodic IRQ: JSA_MASTER_CLOCK / 4 / 16 / 16 / 14 = 3579545 / 14336
	// = 249.69 Hz, counted on ce_ym so it tracks the sound clock exactly. The
	// count restarts when the board leaves reset, as MAME re-arms its periodic
	// timer when the 6502 is reset.
	localparam int IRQ_DIV = 14336;

	logic [13:0] irq_cnt;
	logic        timed_int;
	wire         ym_irq_n;

	always_ff @(posedge clk) begin
		if (board_rst) begin
			irq_cnt   <= '0;
			timed_int <= 1'b0;
		end
		else begin
			if (ce_ym) begin
				if (irq_cnt == 14'(IRQ_DIV - 1)) begin
					irq_cnt   <= '0;
					timed_int <= 1'b1;
				end else begin
					irq_cnt <= irq_cnt + 1'b1;
				end
			end
			if (cpu_ce && irq_ack) timed_int <= 1'b0;
		end
	end

	always_comb sound_irq = timed_int | !ym_irq_n;

	//========================================================================
	//  Program ROM
	//========================================================================
	// $3000-$3FFF is a 4 KB window into the first 16 KB of the 64 KB image
	// (WRIO bits 7:6); $4000-$FFFF maps straight through. The image is the
	// ROM file as-is for all three games (MAME's dangerex region moves the
	// file's first 16 KB to $10000, which its 6502 never reads).
	//
	// The SDRAM returns 16-bit words; the last one is kept, tagged with its
	// word index, and serves the other byte of the word without a new
	// request. Straight-line code hits it on every other fetch, which halves
	// the requests and the cycles that wait on SDRAM. The ROM never changes
	// while the core runs, so the tag alone keeps the word valid; it is
	// cleared by the global reset (a new ROM download).
	wire [15:0] rom_off  = sel_bank ? {2'b00, rom_bank, a[11:0]} : a;
	wire [14:0] rom_widx = rom_off[15:1];

	logic [15:0] buf_word;
	logic [14:0] buf_widx;
	logic        buf_valid;
	logic        req_busy;     // a request is out and not yet acknowledged
	logic [14:0] req_widx;     // its word index

	wire buf_hit = buf_valid && (buf_widx == rom_widx);

	//------------------------------------------------------------------------
	// Fetch timing (as G1). T65 drives a new address just after an Enable
	// clock and samples DI at the next one, so the SDRAM request goes out in
	// the clock after the enable and holds until the ack. The enable ending a
	// ROM-read cycle waits for the byte (ce_pend): a slow grant stretches that
	// cycle, and the following one ends on the grid again.
	//
	// A bus cycle is at least two clocks: cpu_ce_q blocks an enable in the
	// clock after one (the tick waits in ce_pend). Otherwise a stretched
	// enable just before a grid tick makes a one-clock cycle whose RAM read
	// returns ram_q for the previous address. It also lets Enable use the
	// registered rom_cycle_q and rom_have, keeping T65's Enable (which fans out
	// to the whole CPU) off T65's address adder, the decode and the tag
	// compare.
	//
	// A request is never dropped: a board reset during a fetch lets it finish
	// (the SDRAM protocol holds req and address until ack) and the new
	// request waits for it. Its word still lands in the buffer under its own
	// tag, and rom_have ignores the buffer while any request is out, so a late
	// ack can never replace a word the CPU is about to read. req drops for at
	// least the clock after every ack, as it always does on G1 (only a late
	// ack could otherwise be followed at once by a request for a new address).
	//------------------------------------------------------------------------
	logic prog_ack_q;
	always_ff @(posedge clk) prog_ack_q <= prog_ack;

	wire  rom_cycle  = (sel_bank | sel_rom) && rd;
	wire  need_fetch = rom_cycle && !buf_hit && !board_rst && !prog_ack_q;
	wire [14:0] prog_widx = req_busy ? req_widx : rom_widx;

	assign prog_req  = req_busy || need_fetch;
	assign prog_addr = SDR_JSA + {9'd0, prog_widx, 1'b0};

	logic rom_have;            // the buffer holds the byte for the current address
	logic ce_pend;             // a ce_6502 tick is waiting for that byte
	logic rom_cycle_q;
	logic cpu_ce_q;

	always_comb cpu_ce = (ce_6502 || ce_pend) && !cpu_ce_q && (!rom_cycle_q || rom_have);

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			buf_valid <= 1'b0;
			req_busy  <= 1'b0;
		end
		else if (prog_ack) begin
			buf_word  <= prog_dout;
			buf_widx  <= prog_widx;
			buf_valid <= 1'b1;
			req_busy  <= 1'b0;
		end
		else if (need_fetch && !req_busy) begin
			req_busy <= 1'b1;
			req_widx <= rom_widx;
		end
	end

	always_ff @(posedge clk) begin
		if (board_rst) begin
			rom_have    <= 1'b0;
			ce_pend     <= 1'b0;
			cpu_ce_q    <= 1'b0;
			rom_cycle_q <= 1'b0;
		end
		else begin
			cpu_ce_q    <= cpu_ce;
			rom_cycle_q <= rom_cycle;
			if (cpu_ce) begin
				rom_have <= 1'b0;
				ce_pend  <= 1'b0;
			end
			else begin
				// The word for this address is arriving now, or is already
				// held with no request out.
				rom_have <= rom_cycle && ((prog_ack && prog_widx == rom_widx) ||
				                          (buf_hit && !req_busy));
				if (ce_6502) ce_pend <= 1'b1;
			end
		end
	end

	wire [7:0] rom_data = rom_off[0] ? buf_word[7:0] : buf_word[15:8];

	//========================================================================
	//  Work RAM
	//========================================================================
	// Registered read: ram_q follows the address one clock later.
	logic [7:0] ram [8192];
	logic [7:0] ram_q;

	always_ff @(posedge clk) begin
		if (cpu_ce && sel_ram && wr) ram[a[12:0]] <= cpu_do;
		ram_q <= ram[a[12:0]];
	end

	//========================================================================
	//  YM2151 (jotego JT51)
	//========================================================================
	wire signed [15:0] ym_left, ym_right;
	wire [7:0] ym_dout;

	// JT51 needs a second enable, cen_p1, at half the cen rate. Some tools tie
	// it low silently if unconnected, and the chip then produces nothing.
	logic ce_ym_p1_tog;
	always_ff @(posedge clk) begin
		if (!rst_n) ce_ym_p1_tog <= 1'b0;
		else if (ce_ym) ce_ym_p1_tog <= ~ce_ym_p1_tog;
	end
	wire ce_ym_p1 = ce_ym & ce_ym_p1_tog;

	//------------------------------------------------------------------------
	// A 6502 write is held and handed to JT51 in the next cen_p1 clock. JT51
	// takes a register write in any clock but starts its busy flag (32 cen_p1
	// cycles, the chip's 64 master clocks) only if the write falls in a cen_p1
	// clock, and on the ce_6502 grid it never would: G1 reads busy as always
	// clear. The programs poll busy before every write ($511F in Road Riot),
	// so this keeps their write timing to the chip's (and MAME's). The pending
	// write reads as busy, so the flag is set from the write on.
	//------------------------------------------------------------------------
	logic       ym_wr_pend;
	logic       ym_a0_q;
	logic [7:0] ym_din_q;

	always_ff @(posedge clk) begin
		if (board_rst) begin
			ym_wr_pend <= 1'b0;
		end
		else begin
			if (ce_ym_p1) ym_wr_pend <= 1'b0;
			if (cpu_ce && sel_ym && wr) begin
				ym_wr_pend <= 1'b1;
				ym_a0_q    <= a[0];
				ym_din_q   <= cpu_do;
			end
		end
	end

	/* verilator lint_off PINCONNECTEMPTY */
	jt51 u_ym (
		.rst    (!(rst_n && ym_reset_n)),
		.clk    (clk),
		.cen    (ce_ym),
		.cen_p1 (ce_ym_p1),
		.cs_n   (!(ym_wr_pend && ce_ym_p1)),
		.wr_n   (1'b0),
		.a0     (ym_a0_q),
		.din    (ym_din_q),
		.dout   (ym_dout),
		.ct1    (ym_ct1), .ct2 (),          // CT1 gates the OKI, see Mixer
		.irq_n  (ym_irq_n),
		.sample (),
		.left   (), .right (),
		.xleft  (ym_left), .xright (ym_right)
	);
	/* verilator lint_on PINCONNECTEMPTY */

	wire [7:0] ym_status = {ym_dout[7] | (ym_wr_pend & ym_a0_q), ym_dout[6:0]};

	//========================================================================
	//  OKI6295 (jotego JT6295)
	//========================================================================
	wire signed [13:0] oki_snd;
	wire [7:0]  oki_status;
	wire [17:0] oki_rom_addr;
	wire [7:0]  oki_rom_byte;
	wire        oki_rom_ok;

	// INTERPOL=0: the 4x upsampling FIR (INTERPOL=1) needs jtframe_fir_mono.v
	// from the JTFRAME repository. 0 keeps JTFRAME out of the build, at the
	// cost of a slightly harsher ADPCM high end.
	/* verilator lint_off PINCONNECTEMPTY */
	jt6295 #(.INTERPOL(0)) u_oki (
		.rst      (!(rst_n && oki_reset_n)),
		.clk      (clk),
		.cen      (ce_oki),
		.ss       (oki_pin7),               // PIN7: 1 = /132, 0 = /165
		.wrn      (!(cpu_ce && wr_oki)),
		.din      (cpu_do),
		.dout     (oki_status),
		.rom_addr (oki_rom_addr),
		.rom_data (oki_rom_byte),
		.rom_ok   (oki_rom_ok),
		.sound    (oki_snd),
		.sample   ()
	);
	/* verilator lint_on PINCONNECTEMPTY */

	//------------------------------------------------------------------------
	// Banking: the OKI's 18-bit address becomes a 19-bit address in the
	// 512 KB sample ROM (MAME atari_jsa_oki_base_device::device_start).
	//------------------------------------------------------------------------
	logic [1:0] oki_rom_hi;
	always_comb begin
		if (oki_rom_addr[17])   oki_rom_hi = 2'b11;      // fixed, ROM $60000
		else if (!oki_bank[1])  oki_rom_hi = 2'b00;      // entries 0, 1
		else if (!oki_bank[0])  oki_rom_hi = 2'b01;      // entry 2
		else                    oki_rom_hi = 2'b10;      // entry 3
	end
	wire [18:0] oki_rom_phys = {oki_rom_hi, oki_rom_addr[16:0]};

	//------------------------------------------------------------------------
	// OKI sample fetch (as G1, one word held). JT6295 switches between the
	// ADPCM stream and its control reads every ~200 clocks, so the held word
	// is tagged with its physical word address and the byte is valid only
	// while the tag matches what JT6295 asks for now. Tagging after banking
	// makes a bank switch drop the held word at once. Kept combinational: a
	// registered flag could stay low when the address returns to the held
	// word, stalling the control reads until the next switch. The ADPCM reads
	// take no rom_ok and assume data within two cen32 periods (~400 clocks),
	// well above the SDRAM latency.
	//------------------------------------------------------------------------
	logic [15:0] oki_word;
	logic [17:0] oki_wtag;     // physical word address of oki_word
	logic        oki_wvalid;
	logic [17:0] oki_req_w;    // word address of the fetch in flight
	logic        oki_pend;

	assign oki_rom_ok   = oki_wvalid && (oki_wtag == oki_rom_phys[18:1]);
	assign oki_rom_byte = oki_rom_phys[0] ? oki_word[7:0] : oki_word[15:8];

	assign oki_req  = oki_pend;
	assign oki_addr = SDR_OKI + {6'd0, oki_req_w, 1'b0};

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			oki_wvalid <= 1'b0;
			oki_pend   <= 1'b0;
		end
		else if (oki_ack) begin
			oki_word   <= oki_dout;
			oki_wtag   <= oki_req_w;
			oki_wvalid <= 1'b1;
			oki_pend   <= 1'b0;
		end
		else if (!oki_pend && !oki_rom_ok) begin
			oki_req_w <= oki_rom_phys[18:1];
			oki_pend  <= 1'b1;
		end
	end

	//========================================================================
	//  RDIO / JSAIII port
	//========================================================================
	// The raw port as MAME builds it (jsa_iii_ioports). The 6502 reads the
	// same value: bits 4 and 7 are the test switch as the 68000 sees it,
	// without MAME's 0x90 XOR (see header).
	assign jsaiii_port = {
		self_test,               // 0x80 self test, 1 = on
		!main_to_sound_ready,    // 0x40 NMI line state, active low
		sound_to_main_ready,     // 0x20 sound output full
		self_test,               // 0x10 self test, 1 = on
		service,                 // 0x08 service (coin door)
		tilt,                    // 0x04 tilt
		coin_l,                  // 0x02 coin L (IPT_COIN1)
		coin_r                   // 0x01 coin R (IPT_COIN2)
	};

	//========================================================================
	//  Read multiplexer
	//========================================================================
	always_comb begin
		cpu_di = 8'h00;

		// T65 requires DO fed back to DI while R_W_n = 0: some (undocumented)
		// read-modify-write opcodes re-read the value being written.
		if (!cpu_rw_n)                 cpu_di = cpu_do;
		else if (sel_ram)              cpu_di = ram_q;
		else if (sel_bank || sel_rom)  cpu_di = rom_data;
		else if (sel_ym)               cpu_di = ym_status;
		else if (rd_oki)               cpu_di = oki_status;
		else if (rd_cmd)               cpu_di = snd_from_main;
		else if (rd_rdio)              cpu_di = jsaiii_port;
	end

	//========================================================================
	//  Mixer
	//========================================================================
	// MAME (atarijsa.cpp JSA III + atarig42.cpp), mono, full scale = 1.0:
	//     out = 0.8 * ov * ( 0.60 * (L + R) * vol/7  +  0.75 * g * ct1 * oki )
	// ov = overall volume / 127, vol = MIX bits 3:1, g = 1.0 or 0.5 from MIX
	// bit 0, ct1 = YM2151 CT1; L, R = YM outputs (1.0 = 32768) and oki = the
	// sum of the four voices, one full-scale voice = 1.0 (okim6295.cpp: 12-bit
	// sample x volume / 2048). JT6295 sums four 12-bit voices into 14 bits, so
	// one voice = 2048.
	//
	// In 16-bit units that is
	//     out = (0.48/7) * ov * ( (L + R) * vol  +  140 * g * ct1 * oki_snd )
	// (0.75 * 16 * 7 / 0.60 = 140). Halved for headroom, as G1:
	//     audio = pre * overall * 283 / 2^20
	// (0.24 / (7 * 127) = 283.08 / 2^20, -0.03 %). MAME peaks at 29089 in the
	// Road Riot attract (overall $63) and would reach ~37300 at the top
	// setting $7F; here that is ~18650. Four loud OKI voices can still clip,
	// as in MAME. Saturating: a wrapped mix cracks on every peak.
	//
	// OKI gated by YM2151 CT1, as MAME's update_all_volumes(). CT1 is register
	// $1B bit 6, reset to 0; all three games write $C0 there during start-up.
	//------------------------------------------------------------------------
	wire signed [16:0] ym_sum = {ym_left[15], ym_left} + {ym_right[15], ym_right};
	wire signed [20:0] ym_v   = ym_sum * $signed({1'b0, ym_volume});

	// oki_snd * 140 = (oki_snd << 7) + (oki_snd << 3) + (oki_snd << 2); it is
	// even, so the half-volume product 70 * oki_snd is exact.
	wire signed [21:0] oki_x    = 22'(oki_snd);
	wire signed [21:0] oki_140  = (oki_x <<< 7) + (oki_x <<< 3) + (oki_x <<< 2);
	wire signed [21:0] oki_v    = !ym_ct1     ? 22'sd0
	                            : oki_volume  ? oki_140
	                            :               (oki_140 >>> 1);

	// overall * 283 = overall * (256 + 16 + 8 + 2 + 1), at most 72165
	wire [16:0] vol_k = {1'b0, overall_vol, 8'd0} + {5'd0, overall_vol, 4'd0}
	                  + {6'd0, overall_vol, 3'd0} + {8'd0, overall_vol, 1'b0}
	                  + {9'd0, overall_vol};

	logic signed [22:0] pre_q;
	logic signed [40:0] mix_q;

	always_ff @(posedge clk) begin
		pre_q <= 23'(ym_v) + 23'(oki_v);
		mix_q <= pre_q * $signed({1'b0, vol_k}) + 41'sd524288;   // + 0.5 LSB: round
	end

	wire signed [20:0] mixed = 21'(mix_q >>> 20);

	// Saturate to 16 bits. The negative limit is written as 16'sh8000
	// (-32768): 16'sd32768 does not fit in 16 signed bits.
	always_ff @(posedge clk) begin
		if      (mixed >  21'sd32767) audio <= 16'sh7FFF;
		else if (mixed < -21'sd32768) audio <= 16'sh8000;
		else                          audio <= mixed[15:0];
	end

endmodule

`default_nettype wire
