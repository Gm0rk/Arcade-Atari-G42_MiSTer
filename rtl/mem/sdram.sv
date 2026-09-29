//============================================================================
//  Atari G42 for MiSTer
//  sdram.sv -- SDRAM controller for the DE10-Nano 32 MB module
//
//  Single port, one 16-bit word per access, auto precharge on every access,
//  no bursts. Written from the JEDEC SDR SDRAM specification, -7 grade timings.
//
//  Organisation: 16M x 16, four banks of 8192 rows x 512 columns. The port
//  takes a 25-bit byte address (bit 0 ignored); the word address maps as
//      addr[23:22] bank    addr[21:9] row    addr[8:0] column
//
//  Timing at clk = 114.545448 MHz (8.73 ns):
//      tRC 60 ns (7 clk)  tRCD 18 ns (3)  tRP 18 ns (3)  tRAS 42 ns (5)  CL 3
//
//  Access slot (tN = N clocks after ACTIVE):
//      t0  ACTIVE (row, bank)
//      t4  READ or WRITE with auto precharge (tRCD: 4 clk = 35 ns)
//      t8  read data captured (nominal; t7-t10 selectable, see rd_phase)
//      t9  earliest next ACTIVE: tRC = 9 clk = 78.6 ns, ~12.7 M accesses/s
//  READ/WRITE cannot move before t4: auto precharge starts one clock after
//  it, and tRAS must still be met.
//
//  Read capture: the command outputs are registered, so a command written at
//  edge N is sampled by the chip at N+1 and CL 3 puts the word on DQ at N+4
//  (t8). t7 samples while the bus is still turning around and returns
//  plausible but unstable words. The usable window also depends on the -90
//  degree SDRAM_CLK phase and on fitter placement, so the capture step
//  (rd_phase) and edge (rd_half) are runtime inputs; the top level sets them
//  from a boot-time self-test sweep or the OSD. Margins: Arcade-Atari-G42.sdc.
//
//  Clock domain: clk (clk_ram). The requester runs on clk_sys = clk_ram / 2
//  from the same PLL, phase-aligned, so the crossing is synchronous; ready is
//  held for two clk cycles so clk_sys sees it once. If clk_ram stops being an
//  exact 2x clk_sys, this handshake needs a synchroniser.
//============================================================================

`default_nettype none

module sdram (
	// ---- SDRAM pins -------------------------------------------------------
	inout  wire [15:0] SDRAM_DQ,
	output logic [12:0] SDRAM_A,
	output logic [1:0]  SDRAM_BA,
	output logic        SDRAM_DQML,
	output logic        SDRAM_DQMH,
	output logic        SDRAM_nCS,
	output logic        SDRAM_nRAS,
	output logic        SDRAM_nCAS,
	output logic        SDRAM_nWE,
	output logic        SDRAM_CKE,

	// ---- Control ----------------------------------------------------------
	input  wire         init,          // hold high until the PLL is locked
	// Read capture step: 0 = t8, 1 = t9, 2 = t10, 3 = t7.
	input  wire  [1:0]  rd_phase,
	// Sample DQ on the falling edge half a clock (4.36 ns) before the selected
	// step. With rd_phase this gives eight sample points ~4.4 ns apart.
	input  wire         rd_half,

	// ---- Debug ------------------------------------------------------------
	// AUTO REFRESH commands issued; about 147,000 per second when healthy.
	output logic [31:0] dbg_refresh_count,
	// DQ at t7, t8, t9 and t10 of the latest read (a step is sampled only if
	// the slot runs that long, i.e. up to cap_end).
	output logic [15:0] dbg_dq7,
	output logic [15:0] dbg_dq8,
	output logic [15:0] dbg_dq9,
	output logic [15:0] dbg_dq10,
	input  wire         clk,       // clk_ram, 114.545448 MHz

	// ---- Request port -----------------------------------------------------
	// Assert rd or we with addr (and din) held stable until ready pulses. Both
	// must then drop before the next request is accepted (see req_armed).
	input  wire [24:0]  addr,      // byte address; bit 0 ignored
	input  wire [15:0]  din,
	output logic [15:0] dout,
	input  wire         rd,
	input  wire         we,
	output logic        ready      // 2 clk cycles, so clk_sys sees it once
);

	//------------------------------------------------------------------------
	// Command encoding: {nCS, nRAS, nCAS, nWE}
	//------------------------------------------------------------------------
	// DESELECT (nCS high) is used during power-up. It also keeps SDRAM_nCS
	// from being constant: Quartus would remove a constant pin, and a missing
	// port voids the whole set_output_delay constraint that names it.
	localparam [3:0] CMD_DESELECT   = 4'b1111;
	localparam [3:0] CMD_NOP        = 4'b0111;
	localparam [3:0] CMD_ACTIVE     = 4'b0011;
	localparam [3:0] CMD_READ       = 4'b0101;
	localparam [3:0] CMD_WRITE      = 4'b0100;
	localparam [3:0] CMD_PRECHARGE  = 4'b0010;
	localparam [3:0] CMD_REFRESH    = 4'b0001;
	localparam [3:0] CMD_LOADMODE   = 4'b0000;

	logic [3:0] cmd;
	assign {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = cmd;

	//------------------------------------------------------------------------
	// Mode register
	//------------------------------------------------------------------------
	//   A[2:0]   = 000   burst length 1
	//   A[3]     = 0     sequential burst
	//   A[6:4]   = 011   CAS latency 3
	//   A[8:7]   = 00    standard operation
	//   A[9]     = 1     single-location write
	//   A[12:10] = 0
	//
	// CL 3, not 2: a -7 part is rated for CL 2 only up to about 100 MHz.
	//------------------------------------------------------------------------
	localparam [12:0] MODE_REG = 13'b0_00_1_00_011_0_000;

	//------------------------------------------------------------------------
	// Address decomposition
	//------------------------------------------------------------------------
	wire [23:0] word_addr = addr[24:1];
	wire [1:0]  a_bank    = word_addr[23:22];
	wire [12:0] a_row     = word_addr[21:9];
	wire [8:0]  a_col     = word_addr[8:0];

	//------------------------------------------------------------------------
	// Bidirectional data bus
	//------------------------------------------------------------------------
	logic [15:0] dq_out;
	logic        dq_oe;
	assign SDRAM_DQ = dq_oe ? dq_out : 16'hZZZZ;

	// DQM is low in normal operation: every access is a full 16-bit word. A
	// byte-write path would need DQML/DQMH driven in the WRITE cycle.
	logic dqm;
	assign SDRAM_DQML = dqm;
	assign SDRAM_DQMH = dqm;

	//------------------------------------------------------------------------
	// Clock enable
	//------------------------------------------------------------------------
	// Registered, not constant: JEDEC requires CKE low through the power-up
	// wait. A constant would also be optimised away, removing the SDRAM_CKE
	// port and voiding the set_output_delay constraint for every pin listed
	// with it.
	logic cke;
	assign SDRAM_CKE = cke;

	//------------------------------------------------------------------------
	// Refresh
	//------------------------------------------------------------------------
	// 8192 rows every 64 ms: one AUTO REFRESH per 7.8 us = 894 clocks.
	// Requested every 780 clocks (6.8 us) to leave margin for waiting on an
	// access in flight. Refresh is only issued between access slots.
	//------------------------------------------------------------------------
	localparam int REFRESH_INTERVAL = 780;
	localparam int REFRESH_SLOT     = 8;   // covers tRFC (60 ns)

	logic [11:0] refresh_cnt;
	logic        refresh_pending;

	always_ff @(posedge clk) begin
		if (init) begin
			refresh_cnt     <= '0;
			refresh_pending <= 1'b0;
		end else begin
			if (refresh_cnt >= REFRESH_INTERVAL) begin
				refresh_cnt     <= '0;
				refresh_pending <= 1'b1;
			end else begin
				refresh_cnt <= refresh_cnt + 1'b1;
			end
			if (state == S_REFRESH && step == 0) refresh_pending <= 1'b0;
		end
	end

	//------------------------------------------------------------------------
	// Main state machine
	//------------------------------------------------------------------------
	typedef enum logic [2:0] {
		S_INIT_WAIT, S_INIT_PRE, S_INIT_REF, S_INIT_MODE,
		S_IDLE, S_ACCESS, S_REFRESH
	} state_t;

	state_t     state;
	logic [3:0] step;
	logic [14:0] init_cnt;
	logic [3:0] init_refs;

	logic       req_we;

	// One access per request. rd and we are levels, held by the arbiter until
	// it sees ready, but the access returns to S_IDLE before ready is
	// asserted. Without req_armed, S_IDLE would start a second access to the
	// same address, whose ready and data would be taken as the answer to the
	// next request. req_armed clears when an access is accepted and re-arms
	// once rd and we are both low; the arbiter drops them for at least one
	// clk_sys cycle (two clk) between grants.
	logic       req_armed;
	logic [1:0] ready_sr;
	logic       ready_pend;

	// Falling-edge copy of DQ, half a period (4.36 ns) ahead of the
	// rising-edge sample; used when rd_half is set.
	logic [15:0] dq_negedge;
	always_ff @(negedge clk) dq_negedge <= SDRAM_DQ;

	// dq_negedge is packed into the I/O cell (DDIOINCELL). A direct
	// half-cycle path from there to dout is mostly I/O-to-core routing and
	// fails timing, so the sample moves into a core register on the next
	// falling edge and dout loads it one step later (cap_end = cap_step + 1).
	// The sample time is unchanged: half a clock before step cap_step.
	logic [15:0] dq_neg2;
	always_ff @(negedge clk) dq_neg2 <= dq_negedge;

	// Capture step: rd_phase 0 -> t8, 1 -> t9, 2 -> t10, 3 -> t7, so the
	// default is the nominal CL 3 cycle.
	//
	// rd_phase and rd_half come from clk_sys and change only between
	// accesses. They are registered here, and the SDC declares the paths into
	// rd_phase_q / rd_half_q false: the value is quasi-static and the first
	// access to use a new setting starts several clocks later. Unregistered,
	// they would form a clk_sys -> clk_ram path (mux, add, compare, 16-bit
	// capture enable) timed against the 8.73 ns edge spacing.
	logic [1:0] rd_phase_q;
	logic       rd_half_q;
	logic [3:0] cap_step;     // step whose edge (or the falling edge
							  // half a clock before it) is sampled
	logic [3:0] cap_end;      // step at which dout is loaded
	always_ff @(posedge clk) begin
		rd_phase_q <= rd_phase;
		rd_half_q  <= rd_half;
		cap_step   <= (rd_phase_q == 2'd3) ? 4'd6                   // t7
										   : 4'd7 + {2'd0, rd_phase_q};
		cap_end    <= cap_step + {3'd0, rd_half_q};
	end

	assign ready = |ready_sr;

	always_ff @(posedge clk) begin
		// Defaults every cycle, overridden below, so a stale ACTIVE or WRITE
		// is never held on the command bus.
		cmd      <= CMD_NOP;
		dq_oe    <= 1'b0;
		dqm      <= 1'b0;
		ready_sr <= {ready_sr[0], 1'b0};
		if (!rd && !we) req_armed <= 1'b1;

		// ready is asserted one clock after dout is loaded. Every clk_sys edge
		// is also a clk_ram edge, so changing both on the same edge could let
		// the arbiter capture the new ready with the old data; the delay gives
		// dout a full clk period of stability first.
		if (ready_pend) begin
			ready_sr   <= 2'b11;
			ready_pend <= 1'b0;
		end

		if (init) begin
			state     <= S_INIT_WAIT;
			init_cnt  <= '0;
			init_refs <= '0;
			step      <= '0;
			ready_sr   <= '0;
			ready_pend <= 1'b0;
			req_armed  <= 1'b1;       // a request already waiting is taken
			dqm       <= 1'b1;
			cke       <= 1'b0;          // held low until the wait completes
			cmd       <= CMD_DESELECT;  // keep nCS high while unconfigured
			SDRAM_A   <= '0;
			SDRAM_BA  <= '0;
		end
		else begin
			case (state)

			//================================================================
			//  Power-up initialisation
			//================================================================
			// 100 us of stable clock before any command: 11455 clocks at
			// 114.5 MHz, rounded up to 16384.
			S_INIT_WAIT: begin
				dqm <= 1'b1;
				// CKE low and the part deselected for the whole wait; CKE
				// rises on the final cycle so it is stable for PRECHARGE ALL.
				cmd <= CMD_DESELECT;
				if (init_cnt == 15'd16383) begin
					cke   <= 1'b1;
					state <= S_INIT_PRE;
					step  <= '0;
				end else begin
					cke      <= 1'b0;
					init_cnt <= init_cnt + 1'b1;
				end
			end

			// PRECHARGE ALL BANKS (A10 = 1)
			S_INIT_PRE: begin
				dqm <= 1'b1;
				if (step == 0) begin
					cmd     <= CMD_PRECHARGE;
					SDRAM_A <= 13'b0010000000000;
				end
				if (step == 3) begin
					state <= S_INIT_REF;
					step  <= '0;
				end else begin
					step <= step + 1'b1;
				end
			end

			// Eight AUTO REFRESH cycles, as the spec requires.
			S_INIT_REF: begin
				dqm <= 1'b1;
				if (step == 0) cmd <= CMD_REFRESH;
				if (step == REFRESH_SLOT - 1) begin
					step <= '0;
					if (init_refs == 4'd7) state <= S_INIT_MODE;
					else                   init_refs <= init_refs + 1'b1;
				end else begin
					step <= step + 1'b1;
				end
			end

			// LOAD MODE REGISTER, then tMRD (2 clocks) before any command.
			S_INIT_MODE: begin
				dqm <= 1'b1;
				if (step == 0) begin
					cmd      <= CMD_LOADMODE;
					SDRAM_A  <= MODE_REG;
					SDRAM_BA <= 2'b00;
				end
				if (step == 3) begin
					state <= S_IDLE;
					step  <= '0;
				end else begin
					step <= step + 1'b1;
				end
			end

			//================================================================
			//  Idle: pick refresh or an access
			//================================================================
			// Refresh wins: it is bounded to one slot, and an access can wait.
			S_IDLE: begin
				step <= '0;
				if (refresh_pending) begin
					state <= S_REFRESH;
				end
				else if ((rd || we) && req_armed) begin
					req_armed <= 1'b0;
					req_we   <= we;
					state    <= S_ACCESS;
					// ACTIVE is issued here, at t0 of the slot.
					cmd      <= CMD_ACTIVE;
					SDRAM_A  <= a_row;
					SDRAM_BA <= a_bank;

				end
			end

			//================================================================
			//  Refresh slot
			//================================================================
			S_REFRESH: begin
				if (step == 0) begin
					cmd               <= CMD_REFRESH;
					dbg_refresh_count <= dbg_refresh_count + 1'b1;
				end
				if (step == REFRESH_SLOT - 1) begin
					state <= S_IDLE;
					step  <= '0;
				end else begin
					step <= step + 1'b1;
				end
			end

			//================================================================
			//  Access slot
			//================================================================
			// step 0 is t1; t0 was the ACTIVE issued in S_IDLE.
			S_ACCESS: begin
				step <= step + 1'b1;

				case (step)
				4'd3: begin
					// t4: READ or WRITE. A10 = 1 selects auto precharge; A9 and
					// A[12:11] must be zero, the column is on A[8:0].
					SDRAM_A  <= {3'b001, 1'b0, a_col};
					SDRAM_BA <= a_bank;
					if (req_we) begin
						cmd    <= CMD_WRITE;
						dq_out <= din;
						dq_oe  <= 1'b1;
					end else begin
						cmd <= CMD_READ;
					end
				end

				default: begin
					// dbg_dq probes: DQ at t7..t10, as far as the slot runs.
					if (!req_we) begin
						if (step == 4'd6) dbg_dq7  <= SDRAM_DQ;
						if (step == 4'd7) dbg_dq8  <= SDRAM_DQ;
						if (step == 4'd8) dbg_dq9  <= SDRAM_DQ;
						if (step == 4'd9) dbg_dq10 <= SDRAM_DQ;
					end

					// Capture at the runtime-selected step (see cap_end). The
					// slot also ends here for writes.
					if (step == cap_end) begin
						if (!req_we)
							dout <= rd_half_q ? dq_neg2 : SDRAM_DQ;
						ready_pend <= 1'b1;   // ready follows dout by a clock
						state      <= S_IDLE;
						step       <= '0;
					end
				end
				endcase
			end

			default: state <= S_IDLE;
			endcase
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	//------------------------------------------------------------------------
	// Command-sequence assertions: tRCD, and no command during the power-up
	// wait. They catch structural mistakes, not wrong timing constants.
	//------------------------------------------------------------------------
	logic [3:0] last_cmd;
	int         since_active;

	always_ff @(posedge clk) begin
		last_cmd <= cmd;

		if (cmd == CMD_ACTIVE) since_active <= 0;
		else                   since_active <= since_active + 1;

		if (cmd == CMD_READ || cmd == CMD_WRITE) begin
			if (since_active < 3)
				$fatal(1, "sdram: tRCD violated -- READ/WRITE %0d clocks after ACTIVE",
					   since_active + 1);
		end

		if (cmd != CMD_NOP && state inside {S_INIT_WAIT})
			$fatal(1, "sdram: command issued during the power-up wait");
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
