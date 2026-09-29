//============================================================================
//  Atari G42 for MiSTer
//  tms32010.sv -- TMS32010 / TMS320C15 DSP core (ASIC65 of Road Riot 4WD)
//
//  Instruction-exact model of MAME's tms320c1x.cpp (0.264: tms32010.cpp,
//  functionally identical): same architectural results for every opcode,
//  including MAME's quirks, and the same cycle count per execute_run()
//  iteration. Quirks reproduced on purpose:
//    * the PC is 16 bits and never masked (branch operands are loaded whole);
//      program addresses (fetch, TBLR/TBLW, CALA) and the stack use 12 bits
//    * ST bits 12-9 and 7-1 read as 1; LST keeps INTM and ignores next-ARP
//    * SAR stores the auxiliary register after its own *+/*- update
//    * SST direct always writes page 1 and never changes ARP
//    * DMOV/LTD write to (address + 1) mod 256
//    * ADDH detects overflow only for a non-negative high word turning
//      negative (MAME takes the addend's sign from the zero-extended ALU)
//    * SUBC never sets OV (MAME compares against the unchanged ACC)
//    * MPY turns a product of 0x40000000 into 0xC0000000
//    * TBLR/TBLW copy stack level 1 into level 0
//    * illegal opcodes take 0 cycles; the interrupt is not taken after MPY,
//      MPYK or the exact word 7F82 (EINT) and costs 3 cycles
//
//  Timing. One machine cycle per ce (5 MHz for a 20 MHz part). An iteration
//  of MAME's loop (optional interrupt entry + one instruction) runs in four
//  clocks, S_IDLE -> S_DEC -> S_OPR -> S_EXE, and all its effects (register
//  writes, the data RAM write, the I/O strobe, the BIO sample) land in S_EXE.
//  A signed budget counts ce pulses minus the cycles MAME charges: an
//  iteration starts whenever the budget is positive, exactly like MAME's
//  do { } while (icount > 0), so a 0-cycle illegal opcode is followed at once
//  by the next instruction and a 2-cycle one leaves the core idle for one
//  extra ce. With 11 or more clocks per ce the core is idle most of the time.
//
//  Program memory is external: synchronous, one clock of read latency. The
//  opcode is fetched in S_IDLE, the second word (branch target) or the TBLR
//  data in S_DEC. TBLW writes are dropped (MAME maps the space as ROM).
//
//  Data memory is internal (256 x 16 block RAM). DRAM_WORDS selects the map:
//  144 = TMS320C10 (0x00-0x8F, what MAME instantiates), 256 = TMS320C15 (the
//  part Road Riot actually uses). Unmapped words read 0 and ignore writes.
//  The array powers up as zero and is not cleared by rst_n.
//
//  Resets. rst_n: power-on, every register as MAME's device_start() followed
//  by device_reset(). reset: the RS pin (MAME INPUT_LINE_RESET), held while
//  1: PC, ACC, the pending interrupt and ST (= 0x7EFE) are reset, the other
//  registers keep their values, nothing runs. Like MAME, which acts on the
//  line between loop iterations, an iteration already in flight completes
//  first (at most four clocks).
//
//  Clock domain: clk (clk_sys), advanced by ce.
//============================================================================

`default_nettype none

module tms32010 #(
	parameter int DRAM_WORDS = 256                  // 144 (TMS320C10 map) or 256 (TMS320C15)
) (
	input  wire         clk,
	input  wire         rst_n,                      // power-on reset
	input  wire         reset,                      // RS: hold in reset while 1 (at an iteration boundary)
	input  wire         ce,                         // one pulse per machine cycle

	// Program memory (synchronous ROM/RAM, data one clock after the address)
	output logic [11:0] pm_addr,                    // fetch / operand / TBLR address
	input  wire  [15:0] pm_rdata,                   // word addressed in the previous clock

	// I/O ports (IN/OUT), strobes are one clock long, in S_EXE
	output logic [2:0]  io_addr,                    // port number (opcode bits 10-8)
	output logic [15:0] io_dout,                    // OUT data, valid with io_wr
	output logic        io_wr,                      // OUT strobe
	output logic        io_rd,                      // IN strobe: io_din is sampled in this clock
	input  wire  [15:0] io_din,                     // IN data, combinational for io_addr
	input  wire         bio,                        // 1 = BIO pin asserted (BIOZ branches)
	input  wire         int_req,                    // one-clock pulse: latch a pending interrupt

	// Debug
	output logic [15:0] dbg_pc,                     // program counter (16 bits, as MAME)
	output logic        dbg_retire,                 // one-clock pulse after each iteration
	output logic [2:0]  dbg_cycles,                 // cycles that iteration cost (with dbg_retire)
	output logic        dbg_irq                     // with dbg_retire: the iteration took the interrupt
);

	//========================================================================
	//  State
	//========================================================================
	typedef enum logic [1:0] {
		S_IDLE,                                     // wait for budget, interrupt entry, fetch
		S_DEC,                                      // opcode in, data RAM and 2nd word addressed
		S_OPR,                                      // operands in, shifted operand registered
		S_EXE                                       // execute: every architectural effect
	} state_t;

	state_t       state;
	logic signed [3:0] budget;                      // ce pulses not yet spent (MAME's icount)

	logic [15:0]  pc;
	logic [31:0]  acc;
	logic [31:0]  preg;
	logic [15:0]  treg;
	logic [15:0]  ar0, ar1;
	logic [11:0]  stk0, stk1, stk2, stk3;          // stk3 is the top (MAME m_STACK[3])
	logic         st_ov, st_ovm, st_intm, st_arp, st_dp;
	logic         intf;                             // pending interrupt
	logic         irq_block;                        // previous opcode was MPY, MPYK or 7F82

	logic [15:0]  ir;                               // opcode of the current iteration
	logic [15:0]  arg;                              // second word or TBLR data
	logic [15:0]  mem;                              // data RAM operand (0 if unmapped)
	logic [31:0]  opnd;                             // shifted memory operand for the ALU
	logic [7:0]   daddr;                            // data address of the current iteration
	logic         irq_taken;                        // interrupt entry done in S_IDLE

	wire [15:0] st_word = {st_ov, st_ovm, st_intm, 4'b1111, st_arp, 7'b1111111, st_dp};

	//========================================================================
	//  Data RAM: 256 x 16, registered read in S_DEC, write in S_EXE
	//========================================================================
	logic [15:0] dram [0:255];
	logic [7:0]  dram_raddr, dram_waddr;
	logic [15:0] dram_q, dram_wdata;
	logic        dram_we;
	logic        dram_rmapped;                      // the registered read address is mapped

`ifdef SIMULATION
	// synthesis translate_off
	initial for (int i = 0; i < 256; i++) dram[i] = 16'h0000;
	// synthesis translate_on
`endif

	always_ff @(posedge clk) begin
		if (dram_we) dram[dram_waddr] <= dram_wdata;
		dram_q <= dram[dram_raddr];
	end

	localparam logic [8:0] DW = DRAM_WORDS[8:0];

	function automatic logic mapped(input logic [7:0] a);
		mapped = ({1'b0, a} < DW);
	endfunction

	//========================================================================
	//  Decode (of the opcode register; S_DEC decodes pm_rdata directly for
	//  the few things it needs)
	//========================================================================
	wire [7:0] bh  = ir[15:8];
	wire [7:0] bl  = ir[7:0];
	wire       ind = bl[7];                         // indirect addressing

	wire op_add  = (bh[7:4] == 4'h0);
	wire op_sub  = (bh[7:4] == 4'h1);
	wire op_lac  = (bh[7:4] == 4'h2);
	wire op_sar  = (bh[7:1] == 7'b0011_000);        // 30-31
	wire op_lar  = (bh[7:1] == 7'b0011_100);        // 38-39
	wire op_in   = (bh[7:3] == 5'b0100_0);          // 40-47
	wire op_out  = (bh[7:3] == 5'b0100_1);          // 48-4F
	wire op_sacl = (bh == 8'h50);
	wire op_sach = (bh[7:3] == 5'b0101_1);          // 58-5F
	wire op_addh = (bh == 8'h60);
	wire op_adds = (bh == 8'h61);
	wire op_subh = (bh == 8'h62);
	wire op_subs = (bh == 8'h63);
	wire op_subc = (bh == 8'h64);
	wire op_zalh = (bh == 8'h65);
	wire op_zals = (bh == 8'h66);
	wire op_tblr = (bh == 8'h67);
	wire op_mar  = (bh == 8'h68);                   // MAR / LARP
	wire op_dmov = (bh == 8'h69);
	wire op_lt   = (bh == 8'h6A);
	wire op_ltd  = (bh == 8'h6B);
	wire op_lta  = (bh == 8'h6C);
	wire op_mpy  = (bh == 8'h6D);
	wire op_ldpk = (bh == 8'h6E);
	wire op_ldp  = (bh == 8'h6F);
	wire op_lark = (bh[7:1] == 7'b0111_000);        // 70-71
	wire op_xor  = (bh == 8'h78);
	wire op_and  = (bh == 8'h79);
	wire op_or   = (bh == 8'h7A);
	wire op_lst  = (bh == 8'h7B);
	wire op_sst  = (bh == 8'h7C);
	wire op_tblw = (bh == 8'h7D);
	wire op_lack = (bh == 8'h7E);
	wire op_7f   = (bh == 8'h7F);
	wire op_mpyk = (bh[7:5] == 3'b100);             // 80-9F
	wire op_banz = (bh == 8'hF4);
	wire op_bv   = (bh == 8'hF5);
	wire op_bioz = (bh == 8'hF6);
	wire op_call = (bh == 8'hF8);
	wire op_b    = (bh == 8'hF9);
	wire op_blz  = (bh == 8'hFA);
	wire op_blez = (bh == 8'hFB);
	wire op_bgz  = (bh == 8'hFC);
	wire op_bgez = (bh == 8'hFD);
	wire op_bnz  = (bh == 8'hFE);
	wire op_bz   = (bh == 8'hFF);

	// 7Fxx group: MAME dispatches on the low five bits only
	wire [4:0] fn = bl[4:0];
	wire x_dint = op_7f && fn == 5'h01;
	wire x_eint = op_7f && fn == 5'h02;
	wire x_abs  = op_7f && fn == 5'h08;
	wire x_zac  = op_7f && fn == 5'h09;
	wire x_rovm = op_7f && fn == 5'h0A;
	wire x_sovm = op_7f && fn == 5'h0B;
	wire x_cala = op_7f && fn == 5'h0C;
	wire x_ret  = op_7f && fn == 5'h0D;
	wire x_pac  = op_7f && fn == 5'h0E;
	wire x_apac = op_7f && fn == 5'h0F;
	wire x_spac = op_7f && fn == 5'h10;
	wire x_push = op_7f && fn == 5'h1C;
	wire x_pop  = op_7f && fn == 5'h1D;
	wire x_legal = op_7f && (fn <= 5'h02 || (fn >= 5'h08 && fn <= 5'h10) || fn == 5'h1C || fn == 5'h1D);

	wire op_bcond = op_banz | op_bv | op_bioz | op_blz | op_blez | op_bgz | op_bgez | op_bnz | op_bz;

	// Instructions that address data memory (getdata/putdata in MAME); their
	// indirect forms update AR[ARP] and, unless noted, ARP.
	wire memop = op_add | op_sub | op_lac | op_sar | op_lar | op_in | op_out | op_sacl | op_sach
	           | op_addh | op_adds | op_subh | op_subs | op_subc | op_zalh | op_zals | op_tblr
	           | op_mar | op_dmov | op_lt | op_ltd | op_lta | op_mpy | op_ldp
	           | op_xor | op_and | op_or | op_lst | op_sst | op_tblw;

	wire legal = memop | op_ldpk | op_lark | op_lack | x_legal | op_mpyk | op_bcond | op_call | op_b;

	// s_opcode_main / s_opcode_7F cycle counts
	logic [2:0] base_cycles;
	always_comb begin
		if (!legal)
			base_cycles = 3'd0;
		else if (op_tblr | op_tblw)
			base_cycles = 3'd3;
		else if (op_in | op_out | op_call | op_b | x_cala | x_ret | x_push | x_pop)
			base_cycles = 3'd2;
		else
			base_cycles = 3'd1;
	end

	//========================================================================
	//  Auxiliary register update (UPDATE_AR / UPDATE_ARP)
	//========================================================================
	wire [15:0] ar_cur = st_arp ? ar1 : ar0;
	wire [8:0]  ar_step = ar_cur[8:0] + {8'd0, bl[5]} - {8'd0, bl[4]};
	wire [15:0] ar_upd = {ar_cur[15:9], ar_step};   // 9-bit wrap, bits 15-9 kept
	wire        ar_upd_en  = memop && ind && (bl[5] || bl[4]);
	wire        arp_upd_en = memop && ind && !bl[3] && !op_sst && !op_lst;

	//========================================================================
	//  ALU
	//========================================================================
	// Add/subtract with MAME's CALCULATE_ADD/SUB_OVERFLOW
	wire        use_p   = x_apac | x_spac | op_lta | op_ltd;
	wire        do_sub  = op_sub | op_subh | op_subs | x_spac | op_subc;
	wire [31:0] as_b    = use_p ? preg : opnd;
	wire [31:0] as_r    = do_sub ? (acc - as_b) : (acc + as_b);
	wire        as_ovf  = do_sub ? ((acc[31] ^ as_b[31]) & (acc[31] ^ as_r[31]))
	                             : (~(acc[31] ^ as_b[31]) & (acc[31] ^ as_r[31]));
	wire [31:0] as_sat  = acc[31] ? 32'h8000_0000 : 32'h7FFF_FFFF;

	// ADDH: 16-bit add to the high word; overflow only positive -> negative
	wire [15:0] addh_r   = acc[31:16] + mem;
	wire        addh_ovf = ~acc[31] & addh_r[15];

	// ABS
	wire [31:0] abs_neg = -acc;

	// Multiplier: MPY = mem x T, MPYK = T x 13-bit constant
	wire signed [15:0] mul_a = treg;
	wire signed [15:0] mul_b = op_mpyk ? {{3{ir[12]}}, ir[12:0]} : mem;
	wire signed [31:0] mul_p = mul_a * mul_b;

	// SACH: high word of ACC shifted left by 0-7 (MAME allows all eight)
	logic [15:0] sach_v;
	always_comb begin
		case (bh[2:0])
			3'd0: sach_v = acc[31:16];
			3'd1: sach_v = acc[30:15];
			3'd2: sach_v = acc[29:14];
			3'd3: sach_v = acc[28:13];
			3'd4: sach_v = acc[27:12];
			3'd5: sach_v = acc[26:11];
			3'd6: sach_v = acc[25:10];
			3'd7: sach_v = acc[24:9];
		endcase
	end

	// Branch condition
	logic cond;
	always_comb begin
		unique case (1'b1)
			op_banz: cond = (ar_cur[8:0] != 9'd0);
			op_bv:   cond = st_ov;
			op_bioz: cond = bio;
			op_blz:  cond = acc[31];
			op_blez: cond = acc[31] || (acc == 32'd0);
			op_bgz:  cond = !acc[31] && (acc != 32'd0);
			op_bgez: cond = !acc[31];
			op_bnz:  cond = (acc != 32'd0);
			op_bz:   cond = (acc == 32'd0);
			default: cond = 1'b0;
		endcase
	end

	wire [2:0] exe_cycles = base_cycles + {2'd0, op_bcond & cond} + (irq_taken ? 3'd3 : 3'd0);

	// getdata() operand: data RAM word (0 if unmapped), shifted and extended
	// for the ALU a clock early (registered in S_OPR) for timing
	wire [15:0] mem_rd = dram_rmapped ? dram_q : 16'h0000;
	logic [31:0] opnd_nx;
	always_comb begin
		if (op_add || op_sub || op_lac)
			opnd_nx = {{16{mem_rd[15]}}, mem_rd} << bh[3:0];
		else if (op_subh)
			opnd_nx = {mem_rd, 16'h0000};
		else if (op_subc)
			opnd_nx = {1'b0, mem_rd, 15'h0000};
		else
			opnd_nx = {16'h0000, mem_rd};
	end

	wire [15:0] pc_inc = pc + 16'd1;

	//========================================================================
	//  Iteration start: interrupt check (MAME Ext_IRQ) and budget
	//========================================================================
	wire idle     = (state == S_IDLE);
	wire start    = idle && (budget > 4'sd0) && !reset;
	wire irq_take = start && intf && !irq_block && !st_intm;
	wire exe      = (state == S_EXE);

	// budget: +1 per ce, minus the cycles charged in S_EXE (saturates at +7)
	logic signed [4:0] budget_sum;
	always_comb
		budget_sum = $signed({budget[3], budget}) + (ce ? 5'sd1 : 5'sd0)
		           - (exe ? $signed({2'b00, exe_cycles}) : 5'sd0);

	//========================================================================
	//  Memory and port addressing
	//========================================================================
	// S_DEC decodes the fresh opcode word straight from pm_rdata
	wire [7:0]  dec_bh = pm_rdata[15:8];
	wire [7:0]  dec_bl = pm_rdata[7:0];

	always_comb begin
		// program memory
		unique case (state)
			S_IDLE:  pm_addr = irq_take ? 12'h002 : pc[11:0];
			S_DEC:   pm_addr = (dec_bh == 8'h67) ? acc[11:0] : pc[11:0];   // TBLR data / 2nd word
			default: pm_addr = pc[11:0];
		endcase

		// data RAM read: IND = AR[ARP] & 0xFF, DMA = DP:7 bits
		dram_raddr = dec_bl[7] ? ar_cur[7:0] : {st_dp, dec_bl[6:0]};

		// data RAM write, all in S_EXE
		if (op_sst && !ind)
			dram_waddr = {1'b1, bl[6:0]};                           // DMA_DP1: page 1
		else if (op_dmov || op_ltd)
			dram_waddr = daddr + 8'd1;
		else
			dram_waddr = daddr;

		unique case (1'b1)
			op_sacl:         dram_wdata = acc[15:0];
			op_sach:         dram_wdata = sach_v;
			op_sar:          dram_wdata = (ar_upd_en && (bh[0] == st_arp)) ? ar_upd : (bh[0] ? ar1 : ar0);
			op_in:           dram_wdata = io_din;
			op_tblr:         dram_wdata = arg;
			op_sst:          dram_wdata = st_word;
			default:         dram_wdata = mem;                      // DMOV, LTD
		endcase

		dram_we = exe && mapped(dram_waddr)
		        && (op_sacl | op_sach | op_sar | op_in | op_tblr | op_sst | op_dmov | op_ltd);

		// ports
		io_addr = bh[2:0];
		io_dout = mem;
		io_wr   = exe && op_out;
		io_rd   = exe && op_in;
	end

	//========================================================================
	//  Sequencer and registers
	//========================================================================
	always_ff @(posedge clk) begin
		dbg_retire <= 1'b0;

		budget <= (budget_sum > 5'sd7) ? 4'sd7 : budget_sum[3:0];

		// an interrupt stays pending until taken (MAME: cannot be cleared)
		if (int_req)
			intf <= 1'b1;

		unique case (state)
			//----------------------------------------------------------------
			S_IDLE: if (start) begin
				irq_taken <= irq_take;
				if (irq_take) begin
					// Ext_IRQ: INTF=0, INTM=1, push PC, PC=2
					intf    <= int_req;
					st_intm <= 1'b1;
					stk0    <= stk1;
					stk1    <= stk2;
					stk2    <= stk3;
					stk3    <= pc[11:0];
					pc      <= 16'h0003;
				end else begin
					pc      <= pc_inc;
				end
				state <= S_DEC;
			end

			//----------------------------------------------------------------
			S_DEC: begin
				ir    <= pm_rdata;
				daddr <= dram_raddr;
				dram_rmapped <= mapped(dram_raddr);
				state <= S_OPR;
			end

			//----------------------------------------------------------------
			S_OPR: begin
				mem   <= mem_rd;
				arg   <= pm_rdata;
				opnd  <= opnd_nx;
				state <= S_EXE;
			end

			//----------------------------------------------------------------
			S_EXE: begin
				state      <= S_IDLE;
				dbg_retire <= 1'b1;
				dbg_cycles <= exe_cycles;
				dbg_irq    <= irq_taken;
				irq_block  <= op_mpy | op_mpyk | (ir == 16'h7F82);

				// ---- auxiliary registers -----------------------------------
				if (ar_upd_en) begin
					if (st_arp) ar1 <= ar_upd;
					else        ar0 <= ar_upd;
				end
				if (op_banz) begin
					if (st_arp) ar1 <= {ar1[15:9], ar1[8:0] - 9'd1};
					else        ar0 <= {ar0[15:9], ar0[8:0] - 9'd1};
				end
				if (op_lar) begin
					if (bh[0]) ar1 <= mem;
					else       ar0 <= mem;
				end
				if (op_lark) begin
					if (bh[0]) ar1 <= {8'h00, bl};
					else       ar0 <= {8'h00, bl};
				end
				if (arp_upd_en)
					st_arp <= bl[0];

				// ---- status ------------------------------------------------
				if (op_add | op_sub | op_adds | op_subh | op_subs | x_apac | x_spac | op_lta | op_ltd) begin
					if (as_ovf) st_ov <= 1'b1;
				end
				if (op_addh && addh_ovf)
					st_ov <= 1'b1;
				if (op_bv && st_ov)
					st_ov <= 1'b0;
				if (x_sovm) st_ovm <= 1'b1;
				if (x_rovm) st_ovm <= 1'b0;
				if (x_dint) st_intm <= 1'b1;
				if (x_eint) st_intm <= 1'b0;
				if (op_ldpk) st_dp <= bl[0];
				if (op_ldp)  st_dp <= mem[0];
				if (op_lst) begin
					st_ov  <= mem[15];
					st_ovm <= mem[14];
					st_arp <= mem[8];
					st_dp  <= mem[0];
				end

				// ---- accumulator -------------------------------------------
				unique case (1'b1)
					op_add | op_sub | op_adds | op_subh | op_subs | x_apac | x_spac | op_lta | op_ltd:
						acc <= (as_ovf && st_ovm) ? as_sat : as_r;
					op_lac:
						acc <= opnd;
					op_addh:
						acc <= {(addh_ovf && st_ovm) ? (acc[31] ? 16'h8000 : 16'h7FFF) : addh_r, acc[15:0]};
					op_subc:
						acc <= !as_r[31] ? {as_r[30:0], 1'b1} : {acc[30:0], 1'b0};
					op_zalh:
						acc <= {mem, 16'h0000};
					op_zals:
						acc <= {16'h0000, mem};
					op_xor:
						acc <= {acc[31:16], acc[15:0] ^ mem};
					op_and:
						acc <= {16'h0000, acc[15:0] & mem};
					op_or:
						acc <= {acc[31:16], acc[15:0] | mem};
					op_lack:
						acc <= {24'h000000, bl};
					x_zac:
						acc <= 32'h0000_0000;
					x_pac:
						acc <= preg;
					x_abs:
						if (acc[31])
							acc <= (st_ovm && abs_neg == 32'h8000_0000) ? 32'h7FFF_FFFF : abs_neg;
					x_pop:
						acc <= {20'h00000, stk3};
					default: ;
				endcase

				// ---- T and P -----------------------------------------------
				if (op_lt | op_lta | op_ltd)
					treg <= mem;
				if (op_mpy)
					preg <= (mul_p == 32'sh4000_0000) ? 32'hC000_0000 : mul_p;
				if (op_mpyk)
					preg <= mul_p;

				// ---- program counter and stack -----------------------------
				if (op_bcond) begin
					pc <= cond ? arg : pc_inc;
				end
				if (op_b)
					pc <= arg;
				if (op_call) begin
					pc   <= arg;
					stk0 <= stk1;
					stk1 <= stk2;
					stk2 <= stk3;
					stk3 <= pc_inc[11:0];
				end
				if (x_cala) begin
					pc   <= {4'h0, acc[11:0]};
					stk0 <= stk1;
					stk1 <= stk2;
					stk2 <= stk3;
					stk3 <= pc[11:0];
				end
				if (x_push) begin
					stk0 <= stk1;
					stk1 <= stk2;
					stk2 <= stk3;
					stk3 <= acc[11:0];
				end
				if (x_ret)
					pc <= {4'h0, stk3};
				if (x_ret | x_pop) begin
					stk3 <= stk2;
					stk2 <= stk1;
					stk1 <= stk0;
				end
				if (op_tblr | op_tblw)
					stk0 <= stk1;
			end

			default: state <= S_IDLE;
		endcase

		//--------------------------------------------------------------------
		// RS pin: MAME device_reset(), applied while held and idle
		//--------------------------------------------------------------------
		if (reset && idle) begin
			state     <= S_IDLE;
			budget    <= 4'sd0;
			pc        <= 16'h0000;
			acc       <= 32'h0000_0000;
			intf      <= 1'b0;
			st_ov     <= 1'b0;
			st_ovm    <= 1'b1;
			st_intm   <= 1'b1;
			st_arp    <= 1'b0;
			st_dp     <= 1'b0;
			irq_taken <= 1'b0;
		end

		//--------------------------------------------------------------------
		// power-on: device_start() + device_reset()
		//--------------------------------------------------------------------
		if (!rst_n) begin
			state      <= S_IDLE;
			budget     <= 4'sd0;
			pc         <= 16'h0000;
			acc        <= 32'h0000_0000;
			preg       <= 32'h0000_0000;
			treg       <= 16'h0000;
			ar0        <= 16'h0000;
			ar1        <= 16'h0000;
			stk0       <= 12'h000;
			stk1       <= 12'h000;
			stk2       <= 12'h000;
			stk3       <= 12'h000;
			intf       <= 1'b0;
			irq_block  <= 1'b0;
			irq_taken  <= 1'b0;
			st_ov      <= 1'b0;
			st_ovm     <= 1'b1;
			st_intm    <= 1'b1;
			st_arp     <= 1'b0;
			st_dp      <= 1'b0;
			ir         <= 16'h0000;
			dbg_retire <= 1'b0;
			dbg_cycles <= 3'd0;
			dbg_irq    <= 1'b0;
		end
	end

	assign dbg_pc = pc;

endmodule

`default_nettype wire
