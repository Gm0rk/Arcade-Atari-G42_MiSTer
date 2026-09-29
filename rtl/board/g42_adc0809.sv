//============================================================================
//  Atari G42 for MiSTer
//  g42_adc0809.sv -- ADC0809 converter for Road Riot's wheel and pedal
//
//  68000 $E00020-$E0002F, one byte per word on D15:8 (MAME umask16 $FF00).
//  MAME a2d_select_w / a2d_data_r: a write starts a conversion of channel
//  (word offset & 7); a read returns the result register and also starts a
//  new conversion of the channel it addresses. IN2 bit 3 is EOC.
//      channel 0   steering wheel (A2D0, centre $80)
//      channel 1   gas pedal      (A2D1, rest $00)
//      channels 2-7 not connected: MAME's default input reads $FF
//
//  Timing follows MAME adc0808.cpp with the ADC clock at 14.318181 MHz / 16
//  (894.9 kHz = 64 clk_sys): from the start, +2 ADC clocks the input is
//  sampled into the result register and EOC drops; +66 it is sampled again
//  and EOC rises. The result powers up as $FF and EOC as 1, as in MAME.
//  Danger Express and Guardians have no ADC: the core does not start
//  conversions there and the 68000 reads $FF (MAME a2d_data_r).
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_adc0809 #(
	parameter int SAMPLE_CYCLES = 128,   // 2 ADC clocks
	parameter int CONV_CYCLES   = 4224   // 66 ADC clocks
)(
	input  wire        clk,
	input  wire        rst_n,

	input  wire [7:0]  wheel,            // channel 0, offset binary, $80 = centre
	input  wire [7:0]  pedal,            // channel 1, $00 = released

	input  wire [2:0]  chan_sel,         // 68000 word offset from $E00020
	input  wire        start,            // one clock per read or write of $E00020-$E0002F
	output logic [7:0] data,             // result register
	output logic       eoc               // end of conversion, IN2 bit 3
);

	localparam int CNT_W = $clog2(CONV_CYCLES + 1);

	logic [CNT_W-1:0] cnt;
	logic             converting;
	logic [2:0]       cur_chan;

	logic [7:0] chan_value;
	always_comb begin
		case (cur_chan)
			3'd0:    chan_value = wheel;
			3'd1:    chan_value = pedal;
			default: chan_value = 8'hFF;
		endcase
	end

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			converting <= 1'b0;
			cnt        <= '0;
			cur_chan   <= 3'd0;
			data       <= 8'hFF;
			eoc        <= 1'b1;
		end
		else if (start) begin
			// EOC keeps its state until the sample point, as in MAME
			cur_chan   <= chan_sel;
			converting <= 1'b1;
			cnt        <= '0;
		end
		else if (converting) begin
			if (cnt == CNT_W'(SAMPLE_CYCLES)) begin
				data <= chan_value;
				eoc  <= 1'b0;
			end
			if (cnt >= CNT_W'(CONV_CYCLES)) begin
				converting <= 1'b0;
				eoc        <= 1'b1;
				data       <= chan_value;
			end else begin
				cnt <= cnt + 1'b1;
			end
		end
	end

endmodule

`default_nettype wire
