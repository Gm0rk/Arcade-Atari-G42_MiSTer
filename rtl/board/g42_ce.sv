//============================================================================
//  Atari G42 for MiSTer
//  g42_ce.sv -- clock enable generator
//
//  The whole core runs on clk_sys with clock enables (one clock domain, no
//  CDC). clk_sys = 57.272727 MHz = 4 x the 14.318181 MHz master oscillator
//  = 8 x the 7.159090 MHz dot clock, so every main-board enable is an exact
//  integer divide. Same scheme as the Atari G1 core (g1_ce.sv).
//
//  fx68k takes two phase enables, enPhi1 and enPhi2, pulsed alternately half
//  a CPU clock apart; the CPU frequency is the enPhi1 rate.
//
//  The JSA III has its own 3.579545 MHz crystal. Deriving its enables from
//  clk_sys ignores the drift between the two crystals, which is inaudible,
//  and keeps a single clock domain.
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_ce (
	input  wire  clk,        // clk_sys, 57.272727 MHz
	input  wire  rst_n,
	input  wire  cpu_div2,   // 1 = run the 68000 at 7.159 MHz (diagnostic)

	output logic ce_pix,     //  7.159090 MHz dot clock
	output logic ce_cpu_p1,  // fx68k phase 1; its rate is the 68000 clock
	output logic ce_cpu_p2,  // fx68k phase 2
	output logic ce_ym,      //  3.579545 MHz YM2151
	output logic ce_6502,    //  1.789773 MHz JSA III 6502
	output logic ce_oki      //  1.193182 MHz OKI6295 (JSA crystal / 3)
);

	//------------------------------------------------------------------------
	// Main divider: div[1:0] == 0 -> 14.318 MHz, div[2:0] == 0 -> 7.159 MHz
	//------------------------------------------------------------------------
	logic [2:0] div;

	always_ff @(posedge clk) begin
		if (!rst_n) div <= '0;
		else        div <= div + 1'b1;
	end

	always_comb begin
		ce_pix = (div[2:0] == 3'd0);

		if (cpu_div2) begin
			ce_cpu_p1 = (div[2:0] == 3'd0);
			ce_cpu_p2 = (div[2:0] == 3'd4);
		end else begin
			ce_cpu_p1 = (div[1:0] == 2'd0);
			ce_cpu_p2 = (div[1:0] == 2'd2);
		end
	end

	//------------------------------------------------------------------------
	// Sound rates:  clk_sys / 16 = YM2151, / 32 = 6502, YM rate / 3 = OKI
	//------------------------------------------------------------------------
	logic [4:0] snd_div;
	logic [1:0] oki_div;

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			snd_div <= '0;
			oki_div <= '0;
			ce_ym   <= 1'b0;
			ce_6502 <= 1'b0;
			ce_oki  <= 1'b0;
		end else begin
			snd_div <= snd_div + 1'b1;
			ce_ym   <= (snd_div[3:0] == 4'd0);
			ce_6502 <= (snd_div[4:0] == 5'd0);

			ce_oki <= 1'b0;
			if (snd_div[3:0] == 4'd0) begin
				if (oki_div == 2'd2) begin
					oki_div <= 2'd0;
					ce_oki  <= 1'b1;
				end else begin
					oki_div <= oki_div + 1'b1;
				end
			end
		end
	end

endmodule

`default_nettype wire
