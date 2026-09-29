//============================================================================
//  Atari G42 for MiSTer
//  g42_controls.sv -- Road Riot 4WD steering wheel and gas pedal from a pad
//
//  The cabinet's wheel and pedal are potentiometers read through the ADC0809
//  (g42_adc0809): channel 0 wheel (MAME A2D0, AD_STICK_X, centre $80), channel
//  1 pedal (A2D1, PEDAL, rest $00). The game calibrates both in its service
//  menu and keeps the result in the EEPROM; the factory image expects the
//  full $00-$FF range.
//
//  Wheel, in order of precedence:
//    - the left analog stick X, outside its dead zone, scaled by the OSD
//      sensitivity;
//    - the d-pad, ramping out 10 per frame and back to centre 20 per frame
//      (MAME KEYDELTA 10), the way a keyboard steers in MAME;
//    - a paddle or spinner-as-paddle (absolute position), once it has moved:
//      paddle_0 reads 0 with no paddle attached, so it is ignored until then;
//    - otherwise the small analog value near centre, for fine control.
//  Pedal: the largest of the Pedal button (ramps up 16 per frame while held,
//  MAME KEYDELTA 16, down 32 after release) and the right stick pushed up.
//
//  Clock domain: clk_sys; frame_tick is one clock per frame.
//============================================================================

`default_nettype none

module g42_controls #(
	parameter int DEADZONE   = 16,       // |analog| below this counts as centred
	parameter int WHEEL_OUT  = 10,       // d-pad ramp out, per frame
	parameter int WHEEL_BACK = 20,       // d-pad ramp back to centre, per frame
	parameter int PEDAL_UP   = 16,       // Pedal button ramp while held
	parameter int PEDAL_DOWN = 32,       // ... and after release
	parameter int RS_DEAD    = 8         // right-stick pedal dead zone
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        frame_tick,

	input  wire [1:0]  dpad,             // [0] right, [1] left
	input  wire        pedal_btn,
	input  wire [15:0] l_analog,         // {Y, X}, signed -127..+127
	input  wire [15:0] r_analog,         // {Y, X}, signed -127..+127
	input  wire [7:0]  paddle,           // 0..255
	input  wire [1:0]  sensitivity,      // OSD: 0 medium (100%), 1 high (150%), 2/3 low (50%)

	output logic [7:0] wheel,            // offset binary, $80 = centre
	output logic [7:0] pedal             // $00 = released
);

	//------------------------------------------------------------------------
	// Analog steering, scaled before the conversion to offset binary so the
	// centre stays exactly $80. The axis must be in a signed variable first:
	// a concatenation is unsigned, which breaks >>> and * for leftward values.
	//------------------------------------------------------------------------
	wire signed [7:0] ax = l_analog[7:0];

	localparam logic signed [7:0] DZ   = 8'(DEADZONE);
	localparam logic signed [9:0] OUT  = 10'(WHEEL_OUT);
	localparam logic signed [9:0] BACK = 10'(WHEEL_BACK);

	wire analog_live = (ax >= DZ) || (ax <= -DZ);

	function automatic logic signed [7:0] scale(input logic signed [7:0] v, input logic [1:0] sens);
		logic signed [9:0] a, s;
		begin
			a = v;
			case (sens)
				2'd0:    s = a;
				2'd1:    s = (a * 10'sd3) >>> 1;
				default: s = a >>> 1;
			endcase
			if      (s >  10'sd127) scale =  8'sd127;
			else if (s < -10'sd127) scale = -8'sd127;
			else                    scale = s[7:0];
		end
	endfunction

	//------------------------------------------------------------------------
	// D-pad ramp
	//------------------------------------------------------------------------
	logic signed [7:0] ramp;

	always_ff @(posedge clk) begin
		logic signed [9:0] n;
		if (!rst_n) begin
			ramp <= 8'sd0;
		end
		else if (frame_tick) begin
			n = ramp;
			if (analog_live)             n = '0;
			else if (dpad[0] && !dpad[1]) n = n + OUT;
			else if (dpad[1] && !dpad[0]) n = n - OUT;
			else if (n >  BACK)           n = n - BACK;
			else if (n < -BACK)           n = n + BACK;
			else                          n = '0;
			if      (n >  10'sd127) ramp <=  8'sd127;
			else if (n < -10'sd127) ramp <= -8'sd127;
			else                    ramp <= n[7:0];
		end
	end

	//------------------------------------------------------------------------
	// Paddle: used only once it has moved away from its power-up value
	//------------------------------------------------------------------------
	logic [7:0] paddle_first;
	logic       paddle_seen, paddle_live;

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			paddle_seen <= 1'b0;
			paddle_live <= 1'b0;
		end else begin
			if (!paddle_seen) begin
				paddle_first <= paddle;
				paddle_seen  <= 1'b1;
			end else if (paddle != paddle_first) begin
				paddle_live <= 1'b1;
			end
		end
	end

	wire signed [7:0] sx = scale(ax, sensitivity);

	always_ff @(posedge clk) begin
		if (analog_live)           wheel <= {~sx[7], sx[6:0]};
		else if (ramp != 8'sd0)    wheel <= {~ramp[7], ramp[6:0]};
		else if (paddle_live)      wheel <= paddle;
		else                       wheel <= {~sx[7], sx[6:0]};
	end

	//------------------------------------------------------------------------
	// Pedal
	//------------------------------------------------------------------------
	logic [7:0] pbtn;

	always_ff @(posedge clk) begin
		if (!rst_n)
			pbtn <= 8'd0;
		else if (frame_tick) begin
			if (pedal_btn)
				pbtn <= (pbtn > 8'(255 - PEDAL_UP)) ? 8'd255 : pbtn + 8'(PEDAL_UP);
			else
				pbtn <= (pbtn < 8'(PEDAL_DOWN))     ? 8'd0   : pbtn - 8'(PEDAL_DOWN);
		end
	end

	// Right stick up is negative Y. Nine bits so -(-128) does not overflow;
	// x9/4 stretches the travel past the dead zone to the full 0..255.
	localparam logic signed [8:0] RSD = 9'(RS_DEAD);
	wire signed [8:0] rs_up  = -$signed({r_analog[15], r_analog[15:8]});
	wire signed [8:0] rs_net = rs_up - RSD;
	wire       [11:0] rs_mag = (rs_net > 9'sd0) ? ({3'd0, rs_net} * 12'd9) >> 2 : 12'd0;
	wire        [7:0] prs    = (rs_mag > 12'd255) ? 8'd255 : rs_mag[7:0];

	always_ff @(posedge clk) pedal <= (pbtn > prs) ? pbtn : prs;

endmodule

`default_nettype wire
