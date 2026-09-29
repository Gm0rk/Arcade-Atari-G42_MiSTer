//============================================================================
//  Atari G42 for MiSTer
//  g42_asic65.sv -- ASIC65 math coprocessor (68000 side, $F40000-$F80003)
//
//  Follows MAME asic65.cpp in its two G42 flavours, selected at runtime:
//
//  rom_based = 1 (Road Riot 4WD, ASIC65_ROMBASED): a TMS32010-family DSP
//  (tms32010.sv) runs the 136089-1012 program from a 2K x 16 block RAM that
//  the ROM loader fills (words 0x800-0xFFF of the 4K program space read 0, as
//  MAME's zero-filled region does). The DSP clock is 20 MHz / 4 = 5 M machine
//  cycles per second: a phase accumulator gives 11 ce pulses every 126
//  clocks (gaps of 11 or 12), exactly 5.000 MHz from 57.272727 MHz.
//  Host latches, as MAME's synced_write / read / io_r / m68k_r/w / stat_r/w:
//    68000 write  $F80000/2 : TDATA = data, CMD = offset, TFULL = 1, and a
//                             DSP interrupt request (pending until taken)
//    68000 read   $F60000   : 68DATA, clears 68FULL
//    68000 read   $F40000   : TFULL<<15 | 68FULL<<14 | XFLG<<13
//    DSP IN  port 0 (2,4,6) : TDATA, clears TFULL
//    DSP OUT port 0         : 68DATA = data, 68FULL = 1
//    DSP IN  port 1 (3,5,7) : 68FULL<<15 | TFULL<<14 | CMD<<13 | 0x1000
//                             (bit 12 is a jumper; MAME and the program's
//                             normal path want it set)
//    DSP OUT port 1         : XFLG = data & 1
//    BIO                    : asserted while TFULL == 0
//  asic_reset holds the DSP in reset and does not touch the latches: the
//  game writes its first command while the DSP is in reset and releases it
//  afterwards. When a latch is set and cleared in the same clock (a write
//  or read racing the DSP), setting wins: the new word stays pending.
//
//  rom_based = 0 (Guardians of the 'Hood, Danger Express, ASIC65_GUARDIANS):
//  the DSP is held in reset and the answers come from MAME's C++ (command_map
//  row 2): REFLECT, CHECKSUM (0x0027), VERSION (0x0013), RAMTEST (0), RESET,
//  INITBANKS, SETBANK (banklist/bankaddr), VERIFYBANK (bankverify). $F40000
//  reads 0x4000. Parameters go to a 32-word array, a command write resets
//  both indices, and read() applies its side effects in MAME's order. The
//  reset line latches the command on assertion (m_command = -1) and
//  re-issues it on release. Undefined behaviour in the C++ is replaced by
//  its sane equivalent: m_command == -1 decodes as OP_UNKNOWN (result 0);
//  parameters past the 32nd are dropped (MAME writes past the array into
//  m_yorigin, unused in this mode); SETBANK with m_param[0] >= 34 leaves
//  the bank unchanged.
//
//  host_rdata is registered two clocks after host_rd (the side effects happen
//  exactly once, at host_rd) and held until the next host_rd.
//
//  Clock domain: clk (clk_sys, 57.272727 MHz).
//============================================================================

`default_nettype none

module g42_asic65 #(
	parameter int DRAM_WORDS = 256                  // DSP data RAM: 256 = TMS320C15 (real part), 144 = MAME
) (
	input  wire         clk,                        // clk_sys, 57.272727 MHz
	input  wire         rst_n,                      // core reset: latches, HLE state, DSP registers
	input  wire         rom_based,                  // 1 = Road Riot (DSP), 0 = Guardians HLE
	input  wire         asic_reset,                 // 1 = ASIC65 held in reset (io latch bit 14 = 0)

	// ---- 68000 interface ---------------------------------------------------
	input  wire         host_wr,                    // one clock per write to $F80000-$F80003
	input  wire         host_wr_cmd,                // 0 = parameter ($F80000), 1 = command ($F80002)
	input  wire  [15:0] host_wdata,                 // write data, valid with host_wr
	input  wire         host_rd,                    // one clock at the start of a read of $F60000
	output logic [15:0] host_rdata,                 // $F60000 data, valid 2 clocks after host_rd
	output logic [15:0] host_status,                // $F40000 data (no side effects)

	// ---- program download (Road Riot 136089-1012, word k at address k) ----
	input  wire         prog_we,                    // one clock per word
	input  wire  [11:0] prog_addr,                  // word address; 0x800-0xFFF are ignored
	input  wire  [15:0] prog_data,                  // big-endian word from the ROM

	// ---- debug -------------------------------------------------------------
	output logic [15:0] dsp_pc,                     // DSP program counter (16 bits, as MAME)
	output logic        dsp_running,                // DSP out of reset (rom_based and not asic_reset)
	output logic [7:0]  cmd_count,                  // command writes ($F80002), wraps
	output logic [7:0]  irq_count                   // DSP interrupts taken, wraps (0 with Road Riot: never enabled)
);

	//========================================================================
	//  5 MHz machine-cycle enable: 11 pulses per 126 clocks
	//========================================================================
	logic [6:0] ce_ph;
	logic       dsp_ce;

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			ce_ph  <= 7'd0;
			dsp_ce <= 1'b0;
		end else if (ce_ph >= 7'd115) begin
			ce_ph  <= ce_ph - 7'd115;                // + 11 - 126
			dsp_ce <= 1'b1;
		end else begin
			ce_ph  <= ce_ph + 7'd11;
			dsp_ce <= 1'b0;
		end
	end

	//========================================================================
	//  Program memory: 2K x 16, written by the loader, read by the DSP
	//========================================================================
	logic [15:0] prom [0:2047];
	logic [15:0] prom_q;
	logic        prom_hi;                           // address was 0x800-0xFFF
	wire  [11:0] pm_addr;

`ifdef SIMULATION
	// synthesis translate_off
	initial for (int i = 0; i < 2048; i++) prom[i] = 16'h0000;
	// synthesis translate_on
`endif

	always_ff @(posedge clk) begin
		if (prog_we && !prog_addr[11])
			prom[prog_addr[10:0]] <= prog_data;
		prom_q  <= prom[pm_addr[10:0]];
		prom_hi <= pm_addr[11];
	end

	wire [15:0] pm_rdata = prom_hi ? 16'h0000 : prom_q;

	//========================================================================
	//  DSP
	//========================================================================
	// MAME's io map mirrors ports 0/1 at 2/4/6 and 3/5/7: bits 2-1 are not decoded
	/* verilator lint_off UNUSEDSIGNAL */
	wire [2:0]  io_addr;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [15:0] io_dout;
	wire        io_wr, io_rd;
	logic [15:0] io_din;
	wire        dsp_retire, dsp_irq;

	logic tfull, full68, cmd_bit, xflg;
	logic [15:0] tdata, data68;

	tms32010 #(
		.DRAM_WORDS (DRAM_WORDS)
	) u_dsp (
		.clk        (clk),
		.rst_n      (rst_n),
		.reset      (asic_reset || !rom_based),
		.ce         (dsp_ce),
		.pm_addr    (pm_addr),
		.pm_rdata   (pm_rdata),
		.io_addr    (io_addr),
		.io_dout    (io_dout),
		.io_wr      (io_wr),
		.io_rd      (io_rd),
		.io_din     (io_din),
		.bio        (!tfull),
		.int_req    (host_wr && rom_based),
		.dbg_pc     (dsp_pc),
		.dbg_retire (dsp_retire),
		/* verilator lint_off PINCONNECTEMPTY */
		.dbg_cycles (),                             // testbench only
		/* verilator lint_on PINCONNECTEMPTY */
		.dbg_irq    (dsp_irq)
	);

	// asic65_io_map: port 0 mirrored at 2/4/6, port 1 at 3/5/7
	always_comb begin
		if (io_addr[0])
			io_din = {full68, tfull, cmd_bit, 1'b1, 12'h000};   // stat_r
		else
			io_din = tdata;                                     // m68k_r
	end

	//========================================================================
	//  Host latches (ROM-based mode)
	//========================================================================
	always_ff @(posedge clk) begin
		// DSP side first, the 68000 side after: a set in the same clock wins
		if (io_rd && !io_addr[0])
			tfull <= 1'b0;
		if (io_wr && io_addr[0])
			xflg <= io_dout[0];

		if (host_rd && rom_based)
			full68 <= 1'b0;
		if (io_wr && !io_addr[0]) begin
			full68 <= 1'b1;
			data68 <= io_dout;
		end

		if (host_wr && rom_based) begin
			tfull   <= 1'b1;
			cmd_bit <= host_wr_cmd;
			tdata   <= host_wdata;
		end

		if (!rst_n) begin
			tfull   <= 1'b0;
			full68  <= 1'b0;
			cmd_bit <= 1'b0;
			xflg    <= 1'b0;
			tdata   <= 16'h0000;
			data68  <= 16'h0000;
		end
	end

	//========================================================================
	//  High-level mode (ASIC65_GUARDIANS)
	//========================================================================
	typedef enum logic [3:0] {
		OP_UNKNOWN, OP_REFLECT, OP_CHECKSUM, OP_VERSION, OP_RAMTEST, OP_RESET,
		OP_INITBANKS, OP_SETBANK, OP_VERIFYBANK
	} hle_op_t;

	logic [15:0] command;                           // m_command
	logic        command_neg;                       // m_command == -1
	logic [5:0]  pidx;                              // m_param_index, 0..32
	logic [7:0]  ridx;                              // m_result_index (u8, wraps)
	logic [1:0]  last_bank;                         // m_last_bank, always < 4
	logic        reset_state;                       // m_reset_state
	logic [15:0] param0;                            // copy of m_param[0] for SETBANK

	// m_param[32]: written by parameter writes, read by REFLECT
	logic [15:0] pram [0:31];
	logic [4:0]  pram_raddr;
	logic [15:0] pram_q;

	always_ff @(posedge clk) begin
		if (host_wr && !host_wr_cmd && !pidx[5])
			pram[pidx[4:0]] <= host_wdata;
		pram_q <= pram[pram_raddr];
	end

	hle_op_t hle_op;
	always_comb begin
		hle_op = OP_UNKNOWN;
		if (!command_neg) begin
			unique case (command)
				16'h0001: hle_op = OP_REFLECT;
				16'h0002: hle_op = OP_CHECKSUM;
				16'h0003: hle_op = OP_VERSION;
				16'h0004: hle_op = OP_RAMTEST;
				16'h0007: hle_op = OP_RESET;
				16'h000E: hle_op = OP_INITBANKS;
				16'h000F: hle_op = OP_SETBANK;
				16'h0010: hle_op = OP_VERIFYBANK;
				default:  hle_op = OP_UNKNOWN;
			endcase
		end
	end

	// banklist[34]: bank for SETBANK's first parameter; 4 = leave unchanged
	function automatic logic [2:0] banklist(input logic [15:0] p);
		unique case (p)
			16'd0:  banklist = 3'd1;
			16'd2:  banklist = 3'd0;
			16'd5:  banklist = 3'd3;
			16'd7:  banklist = 3'd2;
			16'd16: banklist = 3'd3;
			16'd17: banklist = 3'd3;
			16'd20: banklist = 3'd1;
			16'd21: banklist = 3'd1;
			16'd22: banklist = 3'd0;
			16'd23: banklist = 3'd0;
			16'd28: banklist = 3'd2;
			16'd29: banklist = 3'd2;
			default: banklist = 3'd4;               // listed 4s and out of range
		endcase
	endfunction

	// bankaddr[bank][i] = 0x77C0 + 16 * bank + {0,E,2,C,4,A,6,8}[i]
	function automatic logic [15:0] bankaddr(input logic [1:0] bank, input logic [2:0] i);
		logic [3:0] off;
		unique case (i)
			3'd0: off = 4'h0;
			3'd1: off = 4'hE;
			3'd2: off = 4'h2;
			3'd3: off = 4'hC;
			3'd4: off = 4'h4;
			3'd5: off = 4'hA;
			3'd6: off = 4'h6;
			3'd7: off = 4'h8;
		endcase
		bankaddr = {10'b0111_0111_11, bank, off};
	endfunction

	function automatic logic [15:0] bankverify(input logic [1:0] bank);
		unique case (bank)
			2'd0: bankverify = 16'h0EB2;
			2'd1: bankverify = 16'h1000;
			2'd2: bankverify = 16'h171B;
			2'd3: bankverify = 16'h3D28;
		endcase
	endfunction

	// read() of the current command, as seen at host_rd
	wire  [4:0]  pidx_m1 = pidx[4:0] - 5'd1;        // REFLECT reads m_param[--m_param_index] (32 -> 31)
	logic [15:0] hle_result;
	logic [2:0]  setbank_list;
	logic [1:0]  setbank_bank;
	always_comb begin
		setbank_list = banklist(param0);
		setbank_bank = setbank_list[2] ? last_bank : setbank_list[1:0];
		unique case (hle_op)
			OP_CHECKSUM:   hle_result = 16'h0027;
			OP_VERSION:    hle_result = 16'h0013;
			OP_SETBANK:    hle_result = (pidx != 6'd0)
			                          ? bankaddr(setbank_bank, (ridx < 8'd8) ? ridx[2:0] : 3'd7) : 16'h0000;
			OP_VERIFYBANK: hle_result = bankverify(last_bank);
			default:       hle_result = 16'h0000;   // REFLECT comes from pram, the rest are 0
		endcase
		pram_raddr = pidx_m1;
	end

	always_ff @(posedge clk) begin
		// reset_line(): latch on assertion, re-issue on release
		reset_state <= asic_reset;
		if (asic_reset && !reset_state)
			command_neg <= 1'b1;
		if (!asic_reset && reset_state && !command_neg) begin
			ridx <= 8'd0;
			pidx <= 6'd0;
		end

		// data_w()
		if (host_wr && !rom_based) begin
			if (host_wr_cmd) begin
				command     <= host_wdata;
				command_neg <= 1'b0;
				ridx        <= 8'd0;
				pidx        <= 6'd0;
			end else if (!pidx[5]) begin
				if (pidx == 6'd0)
					param0 <= host_wdata;
				pidx <= pidx + 6'd1;
			end
		end

		// read() side effects
		if (host_rd && !rom_based) begin
			unique case (hle_op)
				OP_REFLECT:
					if (pidx != 6'd0)
						pidx <= pidx - 6'd1;
				OP_RESET: begin
					ridx <= 8'd0;
					pidx <= 6'd0;
				end
				OP_INITBANKS:
					last_bank <= 2'd0;
				OP_SETBANK:
					if (pidx != 6'd0) begin
						last_bank <= setbank_bank;
						ridx      <= ridx + 8'd1;
					end
				default: ;
			endcase
		end

		if (!rst_n) begin
			command     <= 16'h0000;
			command_neg <= 1'b0;
			pidx        <= 6'd0;
			ridx        <= 8'd0;
			last_bank   <= 2'd0;
			reset_state <= 1'b0;
			param0      <= 16'h0000;
		end
	end

	//========================================================================
	//  68000 read data: sampled at host_rd, registered one clock later
	//========================================================================
	logic        rd_pend, rd_reflect;
	logic [15:0] rd_val;

	always_ff @(posedge clk) begin
		rd_pend <= host_rd;
		if (host_rd) begin
			rd_val     <= rom_based ? data68 : hle_result;
			rd_reflect <= !rom_based && hle_op == OP_REFLECT && pidx != 6'd0;
		end
		if (rd_pend)
			host_rdata <= rd_reflect ? pram_q : rd_val;

		if (!rst_n) begin
			rd_pend    <= 1'b0;
			rd_reflect <= 1'b0;
			rd_val     <= 16'h0000;
			host_rdata <= 16'h0000;
		end
	end

	assign host_status = rom_based ? {tfull, full68, xflg, 13'h0000} : 16'h4000;

	//========================================================================
	//  Debug
	//========================================================================
	assign dsp_running = rom_based && !asic_reset;

	always_ff @(posedge clk) begin
		if (host_wr && host_wr_cmd)
			cmd_count <= cmd_count + 8'd1;
		if (dsp_retire && dsp_irq)
			irq_count <= irq_count + 8'd1;
		if (!rst_n) begin
			cmd_count <= 8'd0;
			irq_count <= 8'd0;
		end
	end

endmodule

`default_nettype wire
