//============================================================================
//  Atari G42 for MiSTer
//  g42_sdram_selftest.sv -- SDRAM read-capture sweep and readback check
//
//  From the Atari G1 core (g1_sdram_selftest.sv), where it found the SDRAM
//  read-capture point that works on a given build. Runs after every ROM
//  download, while the game is held in reset:
//    1. SWEEP   each of the eight capture settings reads 4,096 words (the
//               first 8 KB of the image, copied to block RAM as it is written)
//               and counts the ones that come back wrong;
//    2. PICK    fewest errors, ties broken towards the middle of the clean
//               window; the controller uses this setting in Auto;
//    3. IMAGE   the whole image read back once, with the video and object
//               engines running (contended), summed two ways;
//    4. REPEAT  65,536 reads of word 0 with the bus to itself.
//  The written sums come from snooping the loader during the download. The
//  end of the image is wherever the loader wrote last, so each game's image
//  (6.3 to 10.8 MB) is tested in full. A plain sum catches wrong data; the
//  address-mixed sum also catches swapped words and stuck address lines.
//
//  A pass shows the data path was clean for these reads; a marginal timing
//  can still fail under a game's access pattern. A fail is conclusive.
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_sdram_selftest #(
	parameter bit FULL = 1'b1            // 0: sweep only (simulation shortcut)
)(
	input  wire         clk,
	input  wire         rst_n,

	// ---- Snoop of the ROM loader's write port ---------------------------------
	input  wire         load_active,     // ROM download (index 0) in progress
	input  wire [24:0]  load_addr,
	input  wire [15:0]  load_data,
	input  wire         load_we,         // held until the arbiter acks
	input  wire         load_complete,   // start request, held until busy

	// ---- Read port (muxed onto the CPU channel while busy) ----------------------
	output logic [24:0] rd_addr,
	output logic        rd_req,          // held until rd_ack
	input  wire  [15:0] rd_dout,
	input  wire         rd_ack,

	// ---- Result -------------------------------------------------------------------
	output logic        busy,            // running: game in reset, test owns the channel
	output logic        solo,            // the other SDRAM channels must stay idle
	output logic        done,            // a test has completed since reset
	output logic        pass,            // all checks matched (valid with done)
	output logic [31:0] sum_written,
	output logic [31:0] sum_read,
	output logic [31:0] rep_err,         // repeat-test mismatches

	// ---- Capture setting -------------------------------------------------------------
	output logic [2:0]  sweep_sel,       // setting under test while sweep_on
	output logic        sweep_on,        // sweep_sel overrides Auto / OSD
	output logic [16*8-1:0] sweep_err,   // 16-bit error count per setting
	output logic [2:0]  best_sel,        // {rd_half, rd_phase} picked for Auto
	output logic        best_valid,      // the sweep has finished
	output logic        best_clean       // best_sel read every word correctly
);

	//========================================================================
	//  Write-side accumulation
	//========================================================================
	// Restarted by the first write of each ROM download, not by its start, so
	// the configuration download that follows cannot clear it. One word per
	// edge of load_we: the loader holds it until the arbiter acknowledges.
	logic        load_we_d, arm;
	logic [31:0] mix_written, mix_read;
	logic [24:0] image_end;              // one word past the highest address written

	always_ff @(posedge clk) begin
		load_we_d <= load_we;
		if (!rst_n) begin
			sum_written <= 32'd0;
			mix_written <= 32'd0;
			image_end   <= 25'd2;
			arm         <= 1'b1;
		end
		else if (!load_active) begin
			arm <= 1'b1;
		end
		else if (load_we && !load_we_d) begin
			arm <= 1'b0;
			if (arm) begin
				sum_written <= {16'd0, load_data};
				mix_written <= {16'd0, load_data} + {7'd0, load_addr};
				image_end   <= load_addr + 25'd2;
			end else begin
				sum_written <= sum_written + {16'd0, load_data};
				mix_written <= {mix_written[30:0], mix_written[31]}
				             ^ ({16'd0, load_data} + {7'd0, load_addr});
				if (load_addr + 25'd2 > image_end) image_end <= load_addr + 25'd2;
			end
		end
	end

	//========================================================================
	//  Ground truth for the sweep: the first 8 KB of the image
	//========================================================================
	localparam [24:0] TRUTH_END = 25'h002000;

	logic [15:0] truth [4096];
	logic [15:0] truth_q;
	logic [11:0] sw_n;                   // word being read at this setting

	always_ff @(posedge clk) begin
		if (load_active && load_we && !load_we_d && load_addr < TRUTH_END)
			truth[load_addr[12:1]] <= load_data;
		truth_q <= truth[sw_n];
	end

	//========================================================================
	//  Test sequence
	//========================================================================
	typedef enum logic [3:0] {
		T_IDLE, T_SW_REQ, T_SW_WAIT, T_PICK, T_PICKED,
		T_IMG_REQ, T_IMG_WAIT, T_REP_REQ, T_REP_WAIT
	} state_t;

	state_t      state;
	logic [2:0]  sw_i;
	logic [16:0] rep_n;
	logic [24:0] cur;

	assign busy = (state != T_IDLE);

	//------------------------------------------------------------------------
	// Pick: settings walked in sample-time order (tord: position -> setting)
	//     sample time  t6.5  t7.0  t7.5  t8.0  t8.5  t9.0  t9.5  t10.0
	//     setting        7     3     4     0     5     1     6     2
	// Score = {own errors, both neighbours' errors}; a missing neighbour at
	// either end counts as 4,096.
	//------------------------------------------------------------------------
	function automatic [2:0] tord(input [2:0] k);
		case (k)
			3'd0: tord = 3'd7;   3'd1: tord = 3'd3;
			3'd2: tord = 3'd4;   3'd3: tord = 3'd0;
			3'd4: tord = 3'd5;   3'd5: tord = 3'd1;
			3'd6: tord = 3'd6;   default: tord = 3'd2;
		endcase
	endfunction

	logic [2:0]  pk;
	logic [32:0] best_score;
	wire  [2:0]  pk_s  = tord(pk);
	wire  [2:0]  pk_sl = tord(pk - 3'd1);
	wire  [2:0]  pk_sr = tord(pk + 3'd1);
	wire  [15:0] e_c   = sweep_err[16*pk_s +: 16];
	wire  [15:0] e_l   = (pk == 3'd0) ? 16'd4096 : sweep_err[16*pk_sl +: 16];
	wire  [15:0] e_r   = (pk == 3'd7) ? 16'd4096 : sweep_err[16*pk_sr +: 16];
	wire  [32:0] score = {e_c, {1'b0, e_l} + {1'b0, e_r}};

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			state      <= T_IDLE;
			rd_req     <= 1'b0;
			rd_addr    <= '0;
			done       <= 1'b0;
			pass       <= 1'b0;
			sweep_on   <= 1'b0;
			sweep_sel  <= 3'd0;
			sweep_err  <= '0;
			solo       <= 1'b0;
			best_valid <= 1'b0;
			best_clean <= 1'b0;
			best_sel   <= 3'd4;          // t7.5 until measured
			sw_n       <= '0;
			sw_i       <= 3'd0;
			rep_err    <= 32'd0;
			sum_read   <= 32'd0;
			mix_read   <= 32'd0;
		end
		else begin
			case (state)

			T_IDLE: begin
				// Every download starts a new test: each game has its own image.
				if (load_complete) begin
					sweep_err  <= '0;
					sw_i       <= 3'd0;
					sw_n       <= '0;
					sweep_on   <= 1'b1;
					solo       <= 1'b1;
					best_valid <= 1'b0;
					done       <= 1'b0;
					state      <= T_SW_REQ;
				end
			end

			// ---- 1. Sweep; the setting changes only while the controller is idle
			T_SW_REQ: begin
				sweep_sel <= sw_i;
				rd_addr   <= {12'd0, sw_n, 1'b0};
				rd_req    <= 1'b1;
				state     <= T_SW_WAIT;
			end

			T_SW_WAIT: begin
				if (rd_ack) begin
					rd_req <= 1'b0;
					if (rd_dout != truth_q)
						sweep_err[16*sw_i +: 16] <= sweep_err[16*sw_i +: 16] + 1'b1;
					sw_n <= sw_n + 1'b1;
					if (sw_n == 12'hFFF) begin
						if (sw_i == 3'd7) begin
							sweep_on <= 1'b0;
							pk       <= 3'd0;
							state    <= T_PICK;
						end else begin
							sw_i  <= sw_i + 1'b1;
							state <= T_SW_REQ;
						end
					end else begin
						state <= T_SW_REQ;
					end
				end
			end

			// ---- 2. Pick, one setting per clock
			T_PICK: begin
				if (pk == 3'd0 || score < best_score) begin
					best_score <= score;
					best_sel   <= pk_s;
				end
				if (pk == 3'd7) state <= T_PICKED;
				else            pk    <= pk + 1'b1;
			end

			T_PICKED: begin
				best_valid <= 1'b1;
				best_clean <= (best_score[32:17] == 16'd0);
				cur        <= 25'd0;
				sum_read   <= 32'd0;
				mix_read   <= 32'd0;
				solo       <= 1'b0;          // the image pass runs contended
				state      <= FULL ? T_IMG_REQ : T_REP_REQ;
				rep_n      <= '0;
				rep_err    <= 32'd0;
				sw_n       <= '0;            // truth_q -> word 0 for the repeat test
				if (!FULL) begin
					sum_read <= sum_written;
					mix_read <= mix_written;
				end
			end

			// ---- 3. Whole image, at the setting the game will use
			T_IMG_REQ: begin
				rd_addr <= cur;
				rd_req  <= 1'b1;
				state   <= T_IMG_WAIT;
			end

			T_IMG_WAIT: begin
				if (rd_ack) begin
					rd_req   <= 1'b0;
					sum_read <= sum_read + {16'd0, rd_dout};
					mix_read <= {mix_read[30:0], mix_read[31]}
					          ^ ({16'd0, rd_dout} + {7'd0, cur});
					if (cur + 25'd2 >= image_end) begin
						solo  <= 1'b1;
						state <= T_REP_REQ;
					end else begin
						cur   <= cur + 25'd2;
						state <= T_IMG_REQ;
					end
				end
			end

			// ---- 4. Repeat test: word 0 against the written value
			T_REP_REQ: begin
				solo    <= 1'b1;
				rd_addr <= 25'd0;
				rd_req  <= 1'b1;
				state   <= T_REP_WAIT;
			end

			T_REP_WAIT: begin
				if (rd_ack) begin
					rd_req <= 1'b0;
					if (rd_dout != truth_q) rep_err <= rep_err + 1'b1;
					if (rep_n == (FULL ? 17'h0FFFF : 17'h000FF)) begin
						solo  <= 1'b0;
						done  <= 1'b1;
						pass  <= (sum_read == sum_written) && (mix_read == mix_written)
						      && (rep_err == 32'd0) && (rd_dout == truth_q);
						state <= T_IDLE;
					end else begin
						rep_n <= rep_n + 1'b1;
						state <= T_REP_REQ;
					end
				end
			end

			default: state <= T_IDLE;
			endcase
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	logic done_d;
	always_ff @(posedge clk) begin
		done_d <= done;
		if (done && !done_d)
			$display("g42_sdram_selftest: %s  sum %08X/%08X  mix %08X/%08X  repeat errors %0d  capture S%0d",
			         pass ? "PASS" : "FAIL", sum_read, sum_written, mix_read, mix_written,
			         rep_err, best_sel);
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
