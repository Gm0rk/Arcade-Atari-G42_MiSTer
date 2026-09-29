//============================================================================
//  Atari G42 for MiSTer
//  g42_watchdog.sv -- watchdog timer at $E03800
//
//  MAME's WATCHDOG_TIMER with the default period: the game is reset if no
//  write reaches $E03800 for 3 seconds. Counted in frames (180 at
//  59.923 Hz), like the board's watchdog, which is clocked from the video
//  timing. The OSD can disable it. Same as the Atari G1 core.
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_watchdog #(
	parameter int TIMEOUT_FRAMES = 180,  // frames without a kick before reset
	parameter int HOLD_FRAMES    = 4     // frames the reset is held once fired
)(
	input  wire  clk,
	input  wire  rst_n,
	input  wire  disable_wd,             // OSD: hold the watchdog off
	input  wire  kick,                   // one clock per write to $E03800
	input  wire  frame_tick,             // one clock per frame (VBLANK start)
	output logic wd_reset                // level: hold the game in reset
);

	localparam int CNT_W = $clog2(TIMEOUT_FRAMES + 1);

	logic [CNT_W-1:0] count;
	logic [2:0]       hold;

	always_ff @(posedge clk) begin
		if (!rst_n || disable_wd) begin
			// Fully cleared, so re-enabling mid-game cannot fire a stale timeout
			count    <= '0;
			hold     <= '0;
			wd_reset <= 1'b0;
		end
		else begin
			if (kick) count <= '0;

			if (hold != 3'd0) begin
				if (frame_tick) begin
					hold <= hold - 1'b1;
					if (hold == 3'd1) wd_reset <= 1'b0;
				end
			end
			else if (frame_tick && !kick) begin
				if (count >= CNT_W'(TIMEOUT_FRAMES)) begin
					wd_reset <= 1'b1;
					hold     <= 3'(HOLD_FRAMES);
					count    <= '0;
				end else begin
					count <= count + 1'b1;
				end
			end
		end
	end

endmodule

`default_nettype wire
