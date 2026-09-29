//============================================================================
//  Atari G42 for MiSTer
//  g42_alpha.sv -- alphanumerics layer, 64 x 32 tiles of 8 x 8 at 4bpp
//
//  Follows MAME atarig42_v.cpp get_alpha_tile_info; line buffers as the G1
//  core's g1_alpha.sv. Tile map at work RAM words $3000-$37FF (CPU
//  $FF6000-$FF6FFF), 64 words per row, no scroll.
//  Tile word:
//    bits 15:12  colour, including bit 15
//    bit    15   opaque: pen 0 is drawn (TILE_FORCE_LAYER0)
//    bits 11:0   character code
//  Palette index = (colour << 4) | pen, base $000.
//
//  Character ROM is gfx_8x8x4_packed_msb at SDR_ALPHA: 32 bytes per
//  character, row r at byte 4r, high nibble leftmost. Columns 48-63 of every
//  row also hold the playfield control words (g42_scroll); like MAME, this
//  layer does not special-case them (only 42 columns are ever on screen).
//
//  Fetch pipeline as g42_playfield: the map word of character t+1 is read
//  while t's two ROM words are fetched, t is stored while t+1 is fetched, and
//  the next ROM request is presented while the current one is in flight.
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_alpha
	import g42_pkg::*;
(
	input  wire         clk,
	input  wire         rst_n,

	input  wire         start,              // fetch 'line' into the back buffer
	input  wire [7:0]   line,               // 0..239
	input  wire         swap,               // displayed line start: back <-> front

	// ---- Work RAM read port (tile map) ------------------------------------
	output logic [14:0] vram_addr,
	output logic        vram_req,
	input  wire  [15:0] vram_dout,
	input  wire         vram_ack,           // grant delayed by one clock

	// ---- SDRAM read port (character ROM) ----------------------------------
	output logic [24:0] rom_addr,           // byte address of the pending request
	output logic        rom_req,            // a request is pending (not yet granted)
	input  wire         rom_gnt,            // the pending request was taken this clock
	input  wire  [15:0] rom_dout,
	input  wire         rom_ack,            // data of the oldest granted request

	// ---- Display side -----------------------------------------------------
	input  wire [8:0]   disp_x,             // 0..335
	output logic [3:0]  disp_pen,           // one clock after disp_x
	output logic [3:0]  disp_color,
	output logic        disp_opaque,        // pen 0 is drawn
	output logic        busy
);

	// Alpha tilemap at CPU $FF6000 = work RAM word offset $3000.
	localparam [14:0] ALPHA_WORD_BASE = 15'h3000;

	// 336 visible pixels = 42 characters.
	localparam int TILES = 42;

	//========================================================================
	//  Ping-pong line buffers, {colour[3:0], pen[3:0]}
	//========================================================================
	// One pixel written per clock, flat array with the buffer select as the
	// top address bit (see g42_playfield). 1024 deep: {sel, x} reaches 847.
	// The opaque flag is colour bit 3, so it needs no storage of its own.
	logic [7:0] lbuf [1024];
	logic       wr_sel;
	wire        rd_sel = ~wr_sel;

	logic [7:0] rd_data;
	always_ff @(posedge clk)
		rd_data <= lbuf[{rd_sel, disp_x}];

	assign disp_pen    = rd_data[3:0];
	assign disp_color  = rd_data[7:4];
	assign disp_opaque = rd_data[7];

	//========================================================================
	//  Stages (see g42_playfield)
	//========================================================================
	logic       active;
	logic [7:0] cur_line;

	wire [2:0] row_in_tile = cur_line[2:0];
	wire [4:0] tile_row    = cur_line[7:3];

	// Stage 1: map, one character ahead; stray acks land in M_IDLE.
	typedef enum logic [0:0] { M_IDLE, M_WAIT } mstate_t;

	mstate_t     mstate;
	logic [5:0]  m_tile;        // next character to map, 0..42
	logic        m_have;
	logic [11:0] m_code;
	logic [3:0]  m_color;

	// Stage 2: requests, word 0 (pixels 0-3) then word 1 (pixels 4-7)
	logic        r_have;
	logic        r_word;
	logic [11:0] r_code;
	logic [3:0]  r_color;
	logic [5:0]  r_tile;

	// SDR_ALPHA + code * 32 + row * 4 (+ 2)
	function automatic [24:0] char_addr(input [11:0] code, input [2:0] row, input half);
		char_addr = SDR_ALPHA + {8'd0, code, row, half, 1'b0};
	endfunction

	// Stage 3: returned words and the store
	logic        d_word;
	logic [3:0]  d_color;
	logic [5:0]  d_tile;
	logic [15:0] w0;

	logic        s_busy;
	logic [2:0]  s_px;
	logic [5:0]  s_tile;
	logic [3:0]  s_color;
	logic [15:0] s_w0, s_w1;

	// The second word may only return once the store registers are free.
	assign rom_req = r_have && !(r_word && s_busy);

	assign busy = active;

	// Pixel nibble, high nibble leftmost.
	logic [3:0] nib;
	always_comb begin
		case (s_px)
			3'd0: nib = s_w0[15:12];
			3'd1: nib = s_w0[11:8];
			3'd2: nib = s_w0[7:4];
			3'd3: nib = s_w0[3:0];
			3'd4: nib = s_w1[15:12];
			3'd5: nib = s_w1[11:8];
			3'd6: nib = s_w1[7:4];
			3'd7: nib = s_w1[3:0];
		endcase
	end

	always_ff @(posedge clk) begin
		if (s_busy)
			lbuf[{wr_sel, {s_tile, 3'd0} + {6'd0, s_px}}] <= {s_color, nib};
	end

	//========================================================================
	//  Control
	//========================================================================
	wire r_load = m_have && (!r_have || (rom_gnt && r_word));

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			active   <= 1'b0;
			wr_sel   <= 1'b0;
			mstate   <= M_IDLE;
			vram_req <= 1'b0;
			m_have   <= 1'b0;
			r_have   <= 1'b0;
			s_busy   <= 1'b0;
		end
		else begin
			if (swap)
				wr_sel <= ~wr_sel;

			if (start && !active) begin
				active   <= 1'b1;
				cur_line <= line;
				m_tile   <= 6'd0;
				d_word   <= 1'b0;
			end

			// ---- stage 1: map --------------------------------------------
			case (mstate)
			M_IDLE:
				if (active && !m_have && m_tile != 6'(TILES)) begin
					vram_addr <= ALPHA_WORD_BASE | {4'd0, tile_row, m_tile};
					vram_req  <= 1'b1;
					mstate    <= M_WAIT;
				end

			M_WAIT:
				if (vram_ack) begin
					vram_req <= 1'b0;
					m_code   <= vram_dout[11:0];
					m_color  <= vram_dout[15:12];
					m_have   <= 1'b1;
					m_tile   <= m_tile + 1'b1;
					mstate   <= M_IDLE;
				end

			default: mstate <= M_IDLE;
			endcase

			// ---- stage 2: requests ---------------------------------------
			if (rom_gnt) begin
				r_word <= 1'b1;
				if (!r_word) begin
					rom_addr <= char_addr(r_code, row_in_tile, 1'b1);
					d_color  <= r_color;
					d_tile   <= r_tile;
				end
				else begin
					r_have <= 1'b0;
				end
			end
			if (r_load) begin
				r_have   <= 1'b1;
				r_word   <= 1'b0;
				r_code   <= m_code;
				r_color  <= m_color;
				r_tile   <= m_tile - 1'b1;
				rom_addr <= char_addr(m_code, row_in_tile, 1'b0);
				m_have   <= 1'b0;
			end

			// ---- stage 3: returned words and the store --------------------
			if (s_busy) begin
				s_px <= s_px + 1'b1;
				if (s_px == 3'd7) begin
					s_busy <= 1'b0;
					if (s_tile == 6'(TILES - 1))
						active <= 1'b0;
				end
			end
			if (rom_ack) begin
				d_word <= ~d_word;
				if (!d_word) begin
					w0 <= rom_dout;
				end
				else begin
					s_w0    <= w0;
					s_w1    <= rom_dout;
					s_color <= d_color;
					s_tile  <= d_tile;
					s_px    <= 3'd0;
					s_busy  <= 1'b1;
				end
			end
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	always_ff @(posedge clk) begin
		if (rst_n && swap && active)
			$display("%t g42_alpha: swap during a fetch", $time);
		if (rst_n && start && active)
			$display("%t g42_alpha: start while busy ignored", $time);
		if (rst_n && rom_ack && d_word && s_busy)
			$display("%t g42_alpha: second word returned while storing", $time);
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
