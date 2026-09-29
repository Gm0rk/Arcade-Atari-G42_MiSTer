//============================================================================
//  Atari G42 for MiSTer
//  g42_rle_scaler.sv -- per-object placement, scaling and clipping arithmetic
//
//  The setup half of MAME draw_rle() and draw_rle_zoom[_hflip](), once per
//  object:
//
//      scaled_xoffs = (scale * xoffs) >> 12          (arithmetic shift)
//      scaled_yoffs = (scale * yoffs) >> 12
//      if hflip: scaled_xoffs = ((scale * width) >> 12) - scaled_xoffs
//      sx = x - scaled_xoffs,  sy = y - scaled_yoffs
//      sw = max(1, ((scale << 4) * width  + $7FFF) >> 16)
//      sh = max(1, ((scale << 4) * height + $7FFF) >> 16)
//      dx = (width  << 16) / sw,  dy = (height << 16) / sh
//      ex = sx + sw - 1,  ey = sy + sh - 1
//      nothing drawn if sx > 335, ex < 0, sy > 239 or ey < 0
//
//  One scale field drives both axes ($1000 = 1:1). The offsets can exceed
//  16 bits at large scales, so positions are 22 bits here (G1 truncated
//  them to 16).
//
//  Clipping is turned into what g42_rle_render needs per row:
//    rows    y0 = max(sy, 0), nrows = min(ey, 239) - y0 + 1,
//            sourcey0 = dy/2 + max(0, -sy) * dy
//    columns a row's pixel k lands at sx + k (ex - k when flipped). MAME's
//            clipped loops skip the pixels left of the screen (right of it
//            when flipped) and stop at the other clip edge; its unclipped
//            loops stop only when the row's runs end, which can be one or
//            two pixels past the scaled width. Both reduce to "write pixel
//            k if it lies in [L, R]" with
//              not flipped: L = 0,  R = sx < 0   ? min(ex, 335)  : 335
//              flipped:     R = 335, L = ex > 335 ? max(sx, 0)   : 0
//            The renderer starts at the first pixel inside (k0, at dest0,
//            sourcex0 = dx/2 + k0 * dx) and stops at the far edge (bound).
//    fetch   src_limit = (dx/2 + k_last * dx) >> 16 is the last source
//            pixel the far edge can reach; the renderer does not fetch the
//            rest of a row (MAME's clipped loop stops there too).
//
//  One multiplier and one divider, sequenced: about 75 clocks per object.
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_rle_scaler (
	input  wire                clk,
	input  wire                rst_n,

	// ---- Request (held from start until done) --------------------------------
	input  wire                start,
	input  wire [15:0]         scale,      // 4.12 fixed point, non-zero
	input  wire [9:0]          width,      // g42_rle_prescan, non-zero
	input  wire [8:0]          height,
	input  wire signed [15:0]  xoffs,      // ROM header word 0
	input  wire signed [15:0]  yoffs,      // ROM header word 1
	input  wire signed [9:0]   xpos,       // descriptor word 2 [15:6]
	input  wire signed [9:0]   ypos,       // descriptor word 3 [15:6]
	input  wire                hflip,

	// ---- Result (valid from done until the next start) ------------------------
	output logic               skip,       // nothing on screen
	output logic [7:0]         y0,         // first destination row
	output logic [8:0]         nrows,      // destination rows, 1..240
	output logic [31:0]        sourcey0,   // 16.16 source row of y0
	output logic [25:0]        dy,         // 16.16 source step per row
	output logic [25:0]        dx,         // 16.16 source step per pixel
	output logic [27:0]        sourcex0,   // 16.16 source column of dest0
	output logic [8:0]         dest0,      // first destination column
	output logic [8:0]         bound,      // last column: R, or L when flipped
	output logic [11:0]        src_limit,  // last source column needed
	output logic               done        // pulse
);

	localparam logic signed [21:0] SCR_R = 22'sd335;
	localparam logic signed [21:0] SCR_B = 22'sd239;

	//========================================================================
	//  Shared multiplier: 27 x 18 signed, registered in and out (one DSP)
	//========================================================================
	logic signed [26:0] mul_a;
	logic signed [17:0] mul_b;
	logic signed [44:0] mul_p;

	always_ff @(posedge clk) mul_p <= mul_a * mul_b;

	//========================================================================
	//  Shared restoring divider: 26-bit dividend / 15-bit divisor
	//========================================================================
	// This block alone drives div_rem, div_quo, div_bit, div_run, div_done.
	logic [25:0] div_num;
	logic [14:0] div_den;
	logic [14:0] div_rem;          // < div_den
	logic [25:0] div_quo;
	logic [4:0]  div_bit;
	logic        div_run, div_done, div_start;

	wire [15:0] div_shift = {div_rem, div_num[div_bit]};
	wire        div_fits  = (div_shift >= {1'b0, div_den});

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			div_run  <= 1'b0;
			div_done <= 1'b0;
		end
		else begin
			div_done <= 1'b0;
			if (div_start) begin
				div_rem <= '0;
				div_quo <= '0;
				div_bit <= 5'd25;
				div_run <= 1'b1;
			end
			else if (div_run) begin
				div_rem          <= div_fits ? 15'(div_shift - {1'b0, div_den}) : div_shift[14:0];
				div_quo[div_bit] <= div_fits;
				if (div_bit == 5'd0) begin
					div_run  <= 1'b0;
					div_done <= 1'b1;
				end else begin
					div_bit <= div_bit - 5'd1;
				end
			end
		end
	end

	//========================================================================
	//  Sequencer
	//========================================================================
	typedef enum logic [4:0] {
		S_IDLE,
		S_XO, S_XO_R, S_YO, S_YO_R, S_W, S_W_R, S_H, S_H_R,
		S_PLACE, S_REJECT,
		S_DX, S_DX_W, S_DY, S_DY_W,
		S_SY, S_SY_R, S_K0, S_K0_R, S_KL, S_KL_R,
		S_DONE
	} state_t;

	state_t state;

	logic signed [21:0] sxo, syo;         // scaled offsets
	logic [14:0]        sw, sh;           // scaled size, >= 1
	logic signed [21:0] sx, sy, ex, ey;
	logic [16:0]        k0, k_last;       // pixel indices of the column range

	// Product fields
	wire signed [21:0] p_shr12   = mul_p[33:12];                   // (a*b) >> 12
	wire [25:0]        p_u26     = mul_p[25:0];
	// ((p << 4) + $7FFF) >> 16 == (p + $7FF) >> 12 (the low 4 bits of the
	// $7FFF can never carry), clamped to at least 1
	wire [14:0]        p_round   = 15'(({1'b0, p_u26} + 27'h7FF) >> 12);
	wire [14:0]        p_scaled  = (p_round == 15'd0) ? 15'd1 : p_round;
	// (dx/2 + k_last * dx) >> 16: the source column of the far edge's pixel
	wire [28:0]        sx_last   = 29'(({20'd0, dx[25:1]} + $unsigned(mul_p)) >> 16);

	always_ff @(posedge clk) begin
		done      <= 1'b0;
		div_start <= 1'b0;

		if (!rst_n) begin
			state <= S_IDLE;
			skip  <= 1'b0;
		end
		else begin
			case (state)

			S_IDLE:
				if (start) begin
					skip  <= 1'b0;
					mul_a <= 27'(xoffs);
					mul_b <= {2'b00, scale};
					state <= S_XO;
				end

			//---- scaled offsets ------------------------------------------------
			S_XO: state <= S_XO_R;
			S_XO_R: begin
				sxo   <= p_shr12;
				mul_a <= 27'(yoffs);
				state <= S_YO;
			end
			S_YO: state <= S_YO_R;
			S_YO_R: begin
				syo   <= p_shr12;
				mul_a <= {17'd0, width};
				state <= S_W;
			end

			//---- scaled width; the hotspot mirrors about it when flipped -------
			S_W: state <= S_W_R;
			S_W_R: begin
				sw    <= p_scaled;
				if (hflip) sxo <= $signed({8'd0, p_u26[25:12]}) - sxo;
				mul_a <= {18'd0, height};
				state <= S_H;
			end
			S_H: state <= S_H_R;
			S_H_R: begin
				sh    <= p_scaled;
				state <= S_PLACE;
			end

			//---- placement and trivial reject ------------------------------------
			S_PLACE: begin
				sx    <= 22'(xpos) - sxo;
				sy    <= 22'(ypos) - syo;
				ex    <= 22'(xpos) - sxo + $signed({7'd0, sw}) - 22'sd1;
				ey    <= 22'(ypos) - syo + $signed({7'd0, sh}) - 22'sd1;
				state <= S_REJECT;
			end

			S_REJECT:
				if (sx > SCR_R || ex < 0 || sy > SCR_B || ey < 0) begin
					skip  <= 1'b1;
					state <= S_DONE;
				end else begin
					// Rows
					y0    <= (sy < 0) ? 8'd0 : sy[7:0];
					nrows <= 9'(((ey > SCR_B) ? SCR_B : ey) - ((sy < 0) ? 22'sd0 : sy) + 22'sd1);
					// Columns
					if (!hflip) begin
						dest0  <= (sx < 0) ? 9'd0 : sx[8:0];
						bound  <= (sx < 0 && ex < SCR_R) ? ex[8:0] : 9'(SCR_R);
						k0     <= (sx < 0) ? 17'(-sx) : 17'd0;
						k_last <= 17'(((sx < 0 && ex < SCR_R) ? ex : SCR_R) - sx);
					end else begin
						dest0  <= (ex > SCR_R) ? 9'(SCR_R) : ex[8:0];
						bound  <= (ex > SCR_R && sx > 0) ? sx[8:0] : 9'd0;
						k0     <= (ex > SCR_R) ? 17'(ex - SCR_R) : 17'd0;
						k_last <= 17'(ex - ((ex > SCR_R && sx > 0) ? sx : 22'sd0));
					end
					// dx = (width << 16) / sw
					div_num   <= {width, 16'd0};
					div_den   <= sw;
					div_start <= 1'b1;
					state     <= S_DX_W;
				end

			S_DX_W:
				if (div_done) begin
					dx        <= div_quo;
					div_num   <= {1'b0, height, 16'd0};
					div_den   <= sh;
					div_start <= 1'b1;
					state     <= S_DY_W;
				end

			S_DY_W:
				if (div_done) begin
					dy    <= div_quo;
					mul_a <= {1'b0, div_quo};
					mul_b <= (sy < 0) ? 18'(-sy) : 18'd0;
					state <= S_SY;
				end

			//---- start positions -------------------------------------------------
			S_SY: state <= S_SY_R;
			S_SY_R: begin
				sourcey0 <= {6'd0, dy[25:1]} + mul_p[31:0];
				mul_a    <= {1'b0, dx};
				mul_b    <= {1'b0, k0};
				state    <= S_K0;
			end
			S_K0: state <= S_K0_R;
			S_K0_R: begin
				sourcex0 <= {3'd0, dx[25:1]} + mul_p[27:0];
				mul_b    <= {1'b0, k_last};
				state    <= S_KL;
			end
			S_KL: state <= S_KL_R;
			S_KL_R: begin
				src_limit <= (sx_last[28:12] != 17'd0) ? 12'hFFF : sx_last[11:0];
				state     <= S_DONE;
			end

			S_DONE: begin
				done  <= 1'b1;
				state <= S_IDLE;
			end

			default: state <= S_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
