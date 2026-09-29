//============================================================================
//  Atari G42 for MiSTer
//  g42_sloop.sv -- SLOOP program ROM bank controller
//
//  The last 32 KB of the 68000's program ROM space, $078000-$07FFFF, is a
//  window onto one of four 8 KB banks (ROM $078000 + bank x $2000, mirrored
//  four times through the window). The SLOOP chip watches the address bus and
//  changes the bank when the program touches particular addresses in a
//  particular order, which a copied program without the chip cannot do.
//  Follows MAME atarig42.cpp roadriot_sloop_tweak / guardians_sloop_tweak.
//  Danger Express has no SLOOP: its window reads the ROM unbanked.
//
//  Road Riot 4WD (byte addresses; each step is one bus cycle, read or write):
//    sequence 1   $068000                      state 1
//                 $068EEE          if state 1  state 2
//                 $000124/$000678/$000ABC/$001024
//                                  if state 2  next = 0/1/2/3, state 3
//                 $069158/$06A690/$06E708/$071166
//                                  if state 3  bank = next; always state 0
//    sequence 2   $05EDB4 / $05DB0A            offset += 2 / += 1; from
//                                  state 0 this first clears the offset and
//                                  enters state 10
//                 $05F042          if state 10 bank = (bank + offset) & 3,
//                                  offset = 0, state 0
//
//  Guardians of the 'Hood: the last eight bus cycles at or above $07F7C0
//  (cycles below it are not recorded) are compared with four patterns. The
//  words of block n ($07F7C0 + n x $10) touched in the order 0,7,1,6,2,5,3,4
//  select bank n.
//
//  MAME applies the change before it returns the data of the cycle that made
//  it, so the ROM fetch must use the bank one clock after acc_strobe (see
//  g42_top). The state survives a watchdog reset, as in MAME (only its
//  machine_start clears it): it is cleared by rst_n alone.
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_sloop
	import g42_pkg::*;
(
	input  wire        clk,
	input  wire        rst_n,

	input  wire [7:0]  sloop_type,       // SLOOP_NONE / _ROADRIOT / _GUARDIAN (MRA)
	input  wire [18:1] cpu_addr,         // 68000 word address within $000000-$07FFFF
	input  wire        acc_strobe,       // one clock per bus cycle in $000000-$07FFFF

	output logic [1:0] bank              // current bank, valid the clock after acc_strobe
);

	wire [18:0] a = {cpu_addr, 1'b0};    // byte address, for comparison with MAME

	//========================================================================
	//  Road Riot 4WD
	//========================================================================
	localparam [3:0] RR_IDLE = 4'd0, RR_S1 = 4'd1, RR_S2 = 4'd2, RR_S3 = 4'd3,
	                 RR_OFS  = 4'd10;

	logic [3:0] rr_state;
	logic [1:0] rr_next;                 // bank chosen in state 2, applied in state 3
	logic [1:0] rr_offset;               // only (bank + offset) & 3 is ever used

	//========================================================================
	//  Guardians of the 'Hood
	//========================================================================
	// One history entry per bus cycle at or above $07F7C0: bit 5 set means the
	// cycle was outside the pattern area $07F7C0-$07F7FF (MAME keeps the whole
	// offset; any value outside the area only has to fail the compare), bits
	// 4:0 are the word within it. Entry k is at bits [6k +: 6], entry 0 the
	// oldest. Packed rather than an array: Quartus 17 is unreliable with
	// unpacked arrays in function arguments.
	logic [47:0] g_hist;

	wire        g_record = (a >= 19'h7F7C0);
	wire [5:0]  g_entry  = (a[18:6] == 13'h1FDF) ? {1'b0, a[5:1]}  // $07F7C0-$07F7FF
	                                             : 6'h20;
	wire [47:0] g_new    = {g_entry, g_hist[47:6]};                 // history after this cycle

	// Pattern for block n: word n*8 + {0,7,1,6,2,5,3,4}[k] in entry k
	function automatic logic [47:0] g_pattern(input logic [1:0] n);
		g_pattern = {1'b0, n, 3'd4,  1'b0, n, 3'd3,  1'b0, n, 3'd5,  1'b0, n, 3'd2,
		             1'b0, n, 3'd6,  1'b0, n, 3'd1,  1'b0, n, 3'd7,  1'b0, n, 3'd0};
	endfunction

	//========================================================================
	//  State update, one bus cycle at a time
	//========================================================================
	always_ff @(posedge clk) begin
		if (!rst_n) begin
			bank      <= 2'd0;
			rr_state  <= RR_IDLE;
			rr_next   <= 2'd0;
			rr_offset <= 2'd0;
			g_hist    <= {8{6'h20}};
		end
		else if (acc_strobe) begin
			case (sloop_type)
			SLOOP_ROADRIOT: begin
				case (a)
				19'h68000: rr_state <= RR_S1;
				19'h68EEE: if (rr_state == RR_S1) rr_state <= RR_S2;
				19'h00124: if (rr_state == RR_S2) begin rr_next <= 2'd0; rr_state <= RR_S3; end
				19'h00678: if (rr_state == RR_S2) begin rr_next <= 2'd1; rr_state <= RR_S3; end
				19'h00ABC: if (rr_state == RR_S2) begin rr_next <= 2'd2; rr_state <= RR_S3; end
				19'h01024: if (rr_state == RR_S2) begin rr_next <= 2'd3; rr_state <= RR_S3; end

				// Lock in the bank chosen by sequence 1. The game writes one of
				// these depending on $FF8007; any of them ends the sequence.
				19'h69158, 19'h6A690, 19'h6E708, 19'h71166: begin
					if (rr_state == RR_S3) bank <= rr_next;
					rr_state <= RR_IDLE;
				end

				// Sequence 2: relative bank change
				19'h5EDB4: begin
					if (rr_state == RR_IDLE) begin
						rr_state  <= RR_OFS;
						rr_offset <= 2'd2;
					end else begin
						rr_offset <= rr_offset + 2'd2;
					end
				end
				19'h5DB0A: begin
					if (rr_state == RR_IDLE) begin
						rr_state  <= RR_OFS;
						rr_offset <= 2'd1;
					end else begin
						rr_offset <= rr_offset + 2'd1;
					end
				end
				19'h5F042: begin
					if (rr_state == RR_OFS) begin
						bank      <= bank + rr_offset;
						rr_offset <= 2'd0;
						rr_state  <= RR_IDLE;
					end
				end

				default: ;
				endcase
			end

			SLOOP_GUARDIAN: begin
				if (g_record) begin
					g_hist <= g_new;
					if      (g_new == g_pattern(2'd0)) bank <= 2'd0;
					else if (g_new == g_pattern(2'd1)) bank <= 2'd1;
					else if (g_new == g_pattern(2'd2)) bank <= 2'd2;
					else if (g_new == g_pattern(2'd3)) bank <= 2'd3;
				end
			end

			default: ;
			endcase
		end
	end

endmodule

`default_nettype wire
