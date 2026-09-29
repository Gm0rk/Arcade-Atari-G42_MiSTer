//============================================================================
//  Atari G42 for MiSTer
//  g42_scroll.sv -- per-line playfield scroll, colour bank and tile bank
//
//  G42 has no scroll registers: every scanline has two control words in
//  columns 48-63 of the alpha tilemap. MAME atarig42_state::scanline_update:
//
//      offset = (scanline / 8) * 64 + 48, lines 0..255 (offset < $800)
//      X word (latched only if bit 15):
//          xscroll    = (word >> 5) & $3FF      10 bits: the playfield is 1024 wide
//          color_bank = word & $1F
//      Y word (latched only if bit 15):
//          yscroll    = ((word >> 6) - scanline) & $1FF
//          tile_bank  = word & 7
//
//  so line S uses alpha words (S >> 3) * 64 + 48 + 2 * (S & 7) and +1. A
//  value persists until a later line latches a new one, across frames too:
//  lines 240-255 are never displayed but their words still latch, and the
//  result carries into the next frame's line 0. g42_video therefore asks for
//  every line 0..255 exactly once per frame, in order.
//
//  Y scroll is relative to the line whose word set it, so one stored value
//  scrolls the following lines normally; a word on every line gives a
//  per-line raster effect (Road Riot's road).
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_scroll (
	input  wire         clk,
	input  wire         rst_n,

	input  wire         start,              // process the words of 'line'
	input  wire [7:0]   line,               // 0..255

	// ---- Work RAM read port (arbitrated in g42_video, highest priority) ----
	output logic [14:0] vram_addr,          // word offset from $FF0000
	output logic        vram_req,
	input  wire  [15:0] vram_dout,
	input  wire         vram_ack,           // one clock after the grant, with data

	// ---- Latched state ----------------------------------------------------
	output logic [9:0]  xscroll,
	output logic [8:0]  yscroll,
	output logic [4:0]  color_bank,         // [4:2] playfield priority, [1:0] palette bits 9:8
	output logic [2:0]  tile_bank,          // tile code bits 14:12
	output logic        done                // one clock: this line's words are in
);

	// Alpha tilemap at CPU $FF6000 = work RAM word offset $3000.
	localparam [14:0] ALPHA_WORD_BASE = 15'h3000;

	// The work RAM ack is the grant delayed by one clock, not tied to an
	// address: a request still high in its ack clock is granted again. So
	// each read drops the request in its ack clock and the stray ack that
	// follows lands in a state that ignores acks (S_YREQ, S_IDLE), as in G1.
	typedef enum logic [1:0] { S_IDLE, S_X, S_YREQ, S_Y } state_t;

	state_t     state;
	logic [7:0] cur_line;

	// $3000 + (line >> 3) * 64 + 48 + 2 * (line & 7): row in [10:6], column
	// 48 + 2i = {2'b11, i, 1'b0} in [5:0]. Line 255 is word $37FE, in range.
	function automatic [14:0] x_word_addr(input [7:0] l);
		x_word_addr = ALPHA_WORD_BASE | {4'd0, l[7:3], 2'b11, l[2:0], 1'b0};
	endfunction

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			state      <= S_IDLE;
			cur_line   <= 8'd0;
			vram_req   <= 1'b0;
			vram_addr  <= '0;
			xscroll    <= '0;
			yscroll    <= '0;
			color_bank <= '0;
			tile_bank  <= '0;
			done       <= 1'b0;
		end
		else begin
			done <= 1'b0;

			case (state)
			S_IDLE:
				if (start) begin
					cur_line  <= line;
					vram_addr <= x_word_addr(line);
					vram_req  <= 1'b1;
					state     <= S_X;
				end

			S_X:
				if (vram_ack) begin
					if (vram_dout[15]) begin
						xscroll    <= vram_dout[14:5];
						color_bank <= vram_dout[4:0];
					end
					vram_req <= 1'b0;
					state    <= S_YREQ;
				end

			S_YREQ: begin
				vram_addr <= x_word_addr(cur_line) | 15'd1;
				vram_req  <= 1'b1;
				state     <= S_Y;
			end

			S_Y:
				if (vram_ack) begin
					if (vram_dout[15]) begin
						// Relative to this line; the 9-bit wrap is intended.
						yscroll   <= 9'(vram_dout[14:6] - {1'b0, cur_line});
						tile_bank <= vram_dout[2:0];
					end
					vram_req <= 1'b0;
					done     <= 1'b1;
					state    <= S_IDLE;
				end

			default: state <= S_IDLE;
			endcase
		end
	end

endmodule

`default_nettype wire
