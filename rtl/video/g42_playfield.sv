//============================================================================
//  Atari G42 for MiSTer
//  g42_playfield.sv -- playfield layer, 128 x 64 tiles of 8 x 8 at 6bpp
//
//  Follows MAME atarig42_v.cpp (get_playfield_tile_info, the playfield scan
//  mapper) and the gfx layouts of atarig42.cpp; line buffers and fetch order
//  as the G1 core's g1_playfield.sv.
//
//  Tile map: work RAM words $1000-$2FFF (CPU $FF2000-$FF5FFF). Tilemap column
//  c, row r is at bank * 4096 + r * 64 + (c % 64) with bank = 1 - c / 64, so
//  columns 64-127 are the first 4096 words: word = {~c[6], c[6], r, c[5:0]}.
//  Tile word:
//    bit    15   X flip (TILE_FLIPX)
//    bits 14:12  tile colour
//    bits 11:0   code, low 12 bits; code = (tile_bank << 12) | word[11:0],
//                modulo the ROM's tile count (16384, or 32768 for Guardians)
//
//  Pixels: 6 bits from the three thirds of MAME's "tiles" region, A (first),
//  B, C, one 16-bit big-endian word per tile row each at SDRAM
//  SDR_PFx + code * 16 + row * 2. pflayout + pftoplayout + blend_gfx(0, 2,
//  $0F, $30) give, for pixel x (derived from the layouts' MSB-first bit
//  offsets and checked against a generic MAME decode of every tile of every
//  game, sim/video/py/check_pf_permutation.py):
//      x    pen[5]  pen[4]  pen[3:0]        x    pen[5]  pen[4]  pen[3:0]
//      0    C[12]   C[8]    B[15:12]        4    C[4]    C[0]    B[7:4]
//      1    C[13]   C[9]    B[11:8]         5    C[5]    C[1]    B[3:0]
//      2    C[14]   C[10]   A[15:12]        6    C[6]    C[2]    A[7:4]
//      3    C[15]   C[11]   A[11:8]         7    C[7]    C[3]    A[3:0]
//  (pen[3:0] is the G1 permutation with hi = B, lo = A.) Only Road Riot's
//  tiles use pen bit 5.
//
//  Scroll (from g42_scroll, latched at the fetch start): source pixel of
//  screen (x, line) is ((x + xscroll) & 1023, (line + yscroll) & 511).
//
//  Fetch pipeline. G1 fetched one tile at a time (map word, ROM words with a
//  gap after each, eight store clocks), leaving the SDRAM idle between words;
//  each gap hands the bus to the RLE engine for a whole access. Here three
//  stages overlap so the next ROM request is already presented when the
//  current one is granted: the map stage reads tile t+1's map word while tile
//  t's ROM words are fetched, and tile t's eight pixels are stored while tile
//  t+1's words are fetched. The SDRAM side is still one word in flight at a
//  time, granted by g42_video's G1 handshake; words return in request order.
//  A tile's C word is not requested while the previous tile is still being
//  stored, so the store registers are free whatever the SDRAM latency.
//
//  The line buffers hold {tile colour, pen} per pixel; the line's colour bank
//  and fine X scroll are kept per buffer and swap with it, so they belong to
//  the displayed line (G1 kept one fine-X register, which the next line's
//  fetch overwrote while the current line was on screen).
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_playfield
	import g42_pkg::*;
(
	input  wire         clk,
	input  wire         rst_n,

	input  wire         tiles32k,           // cfg.flags[3]: 32768 tiles, else 16384

	// Fetch 'line' into the back buffer; pulse after g42_scroll has latched
	// this line's words, never while busy or with swap.
	input  wire         start,
	input  wire [7:0]   line,               // 0..239
	input  wire         swap,               // displayed line start: back <-> front

	// ---- Line state (from g42_scroll) -------------------------------------
	input  wire [9:0]   xscroll,
	input  wire [8:0]   yscroll,
	input  wire [2:0]   tile_bank,
	input  wire [4:0]   color_bank,

	// ---- Work RAM read port (tile map) ------------------------------------
	output logic [14:0] vram_addr,
	output logic        vram_req,
	input  wire  [15:0] vram_dout,
	input  wire         vram_ack,           // grant delayed by one clock

	// ---- SDRAM read port (tile ROM) ---------------------------------------
	output logic [24:0] rom_addr,           // byte address of the pending request
	output logic        rom_req,            // a request is pending (not yet granted)
	input  wire         rom_gnt,            // the pending request was taken this clock
	input  wire  [15:0] rom_dout,
	input  wire         rom_ack,            // data of the oldest granted request

	// ---- Display side -----------------------------------------------------
	input  wire [8:0]   disp_x,             // 0..335
	output logic [5:0]  disp_pen,           // one clock after disp_x
	output logic [2:0]  disp_color,         // tile colour, one clock after disp_x
	output logic [4:0]  disp_bank,          // colour bank of the displayed line
	output logic        busy
);

	// 336 visible pixels plus one tile for the fine scroll offset.
	localparam int TILES = 43;

	//========================================================================
	//  Ping-pong line buffers, {tile colour[2:0], pen[5:0]}
	//========================================================================
	// One pixel written per clock: a whole tile row at once needs eight write
	// ports and will not infer as block RAM. Flat array, buffer select as the
	// top address bit, sized to the addressing ({sel, x}), as in G1.
	logic [8:0] lbuf [1024];
	logic       wr_sel;
	wire        rd_sel = ~wr_sel;

	// Per-buffer line state, written at the fetch start for the back buffer.
	logic [2:0] fine_x [2];
	logic [4:0] bank   [2];

	logic [8:0] rd_data;
	always_ff @(posedge clk)
		rd_data <= lbuf[{rd_sel, disp_x + {6'd0, fine_x[rd_sel]}}];

	assign disp_pen   = rd_data[5:0];
	assign disp_color = rd_data[8:6];
	assign disp_bank  = bank[rd_sel];

	//========================================================================
	//  Line state
	//========================================================================
	logic       active;         // a line fetch is in progress
	logic [8:0] src_y;          // scrolled source row
	logic [6:0] first_col;      // tilemap column of tile 0
	logic [2:0] cur_bank;       // tile bank for this line

	wire [2:0] row_in_tile = src_y[2:0];
	wire [5:0] tile_row    = src_y[8:3];

	//========================================================================
	//  Stage 1: tile map, one tile ahead of the ROM stage
	//========================================================================
	// The work RAM ack is the grant delayed by one clock: the request is
	// dropped in the ack clock and the stray ack that follows lands in
	// M_IDLE, which does not look at acks (as g42_scroll).
	//------------------------------------------------------------------------
	typedef enum logic [0:0] { M_IDLE, M_WAIT } mstate_t;

	mstate_t     mstate;
	logic [5:0]  m_tile;        // next tile to map, 0..43
	logic        m_have;        // m_* hold a mapped tile not yet taken
	logic [14:0] m_code;
	logic [2:0]  m_color;
	logic        m_flip;

	wire [6:0] m_col = first_col + {1'b0, m_tile};           // wraps at 128

	//========================================================================
	//  Stage 2: ROM requests A, B, C of the current tile
	//========================================================================
	logic        r_have;        // r_* hold a tile with words left to request
	logic [1:0]  r_word;        // next word: 0 A, 1 B, 2 C
	logic [14:0] r_code;
	logic [2:0]  r_color;
	logic        r_flip;
	logic [5:0]  r_tile;

	// Code * 16 + row * 2 inside a 512 KB third.
	function automatic [24:0] word_addr(input [24:0] base, input [14:0] code, input [2:0] row);
		word_addr = base + {6'd0, code, row, 1'b0};
	endfunction

	//========================================================================
	//  Stage 3: returned words, then eight store clocks
	//========================================================================
	logic [1:0]  d_word;        // word the next ack carries: 0 A, 1 B, 2 C
	logic [2:0]  d_color;       // attributes of the tile whose words return
	logic        d_flip;
	logic [5:0]  d_tile;
	logic [15:0] w_a, w_b;

	logic        s_busy;        // storing a tile
	logic [2:0]  s_px;
	logic [5:0]  s_tile;
	logic [2:0]  s_color;
	logic        s_flip;
	logic [15:0] s_a, s_b, s_c;

	// The C word may only return once the store registers are free.
	assign rom_req = r_have && !(r_word == 2'd2 && s_busy);

	assign busy = active;

	//------------------------------------------------------------------------
	// Pixel select: the layout permutation above; a flip reverses the pixel
	// index (tile_draw walks x backwards).
	//------------------------------------------------------------------------
	wire [2:0] px_f = s_flip ? ~s_px : s_px;

	logic [5:0] pen;
	always_comb begin
		case (px_f)
			3'd0: pen = {s_c[12], s_c[8],  s_b[15:12]};
			3'd1: pen = {s_c[13], s_c[9],  s_b[11:8]};
			3'd2: pen = {s_c[14], s_c[10], s_a[15:12]};
			3'd3: pen = {s_c[15], s_c[11], s_a[11:8]};
			3'd4: pen = {s_c[4],  s_c[0],  s_b[7:4]};
			3'd5: pen = {s_c[5],  s_c[1],  s_b[3:0]};
			3'd6: pen = {s_c[6],  s_c[2],  s_a[7:4]};
			3'd7: pen = {s_c[7],  s_c[3],  s_a[3:0]};
		endcase
	end

	always_ff @(posedge clk) begin
		if (s_busy)
			lbuf[{wr_sel, {s_tile, 3'd0} + {6'd0, s_px}}] <= {s_color, pen};
	end

	//========================================================================
	//  Control
	//========================================================================
	wire r_load = m_have && (!r_have || (rom_gnt && r_word == 2'd2));

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

			// ---- line start ----------------------------------------------
			if (start && !active) begin
				active         <= 1'b1;
				src_y          <= 9'({1'b0, line} + yscroll);
				first_col      <= xscroll[9:3];
				fine_x[wr_sel] <= xscroll[2:0];
				bank[wr_sel]   <= color_bank;
				cur_bank       <= tile_bank;
				m_tile         <= 6'd0;
				d_word         <= 2'd0;
			end

			// ---- stage 1: map --------------------------------------------
			case (mstate)
			M_IDLE:
				if (active && !m_have && m_tile != 6'(TILES)) begin
					vram_addr <= {1'b0, ~m_col[6], m_col[6], tile_row, m_col[5:0]};
					vram_req  <= 1'b1;
					mstate    <= M_WAIT;
				end

			M_WAIT:
				if (vram_ack) begin
					vram_req <= 1'b0;
					// MAME code % elements: 14 or 15 bits
					m_code   <= {tiles32k & cur_bank[2], cur_bank[1:0], vram_dout[11:0]};
					m_color  <= vram_dout[14:12];
					m_flip   <= vram_dout[15];
					m_have   <= 1'b1;
					m_tile   <= m_tile + 1'b1;
					mstate   <= M_IDLE;
				end

			default: mstate <= M_IDLE;
			endcase

			// ---- stage 2: requests ---------------------------------------
			if (rom_gnt) begin
				r_word <= r_word + 1'b1;
				case (r_word)
					2'd0:    rom_addr <= word_addr(SDR_PFB, r_code, row_in_tile);
					2'd1:    rom_addr <= word_addr(SDR_PFC, r_code, row_in_tile);
					default: r_have   <= 1'b0;
				endcase
				// the attributes travel with the first word to stage 3
				if (r_word == 2'd0) begin
					d_color <= r_color;
					d_flip  <= r_flip;
					d_tile  <= r_tile;
				end
			end
			if (r_load) begin
				r_have   <= 1'b1;
				r_word   <= 2'd0;
				r_code   <= m_code;
				r_color  <= m_color;
				r_flip   <= m_flip;
				r_tile   <= m_tile - 1'b1;
				rom_addr <= word_addr(SDR_PFA, m_code, row_in_tile);
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
				d_word <= (d_word == 2'd2) ? 2'd0 : d_word + 1'b1;
				case (d_word)
					2'd0: w_a <= rom_dout;
					2'd1: w_b <= rom_dout;
					default: begin
						s_a     <= w_a;
						s_b     <= w_b;
						s_c     <= rom_dout;
						s_color <= d_color;
						s_flip  <= d_flip;
						s_tile  <= d_tile;
						s_px    <= 3'd0;
						s_busy  <= 1'b1;
					end
				endcase
			end
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	always_ff @(posedge clk) begin
		if (rst_n && swap && active)
			$display("%t g42_playfield: swap during a fetch", $time);
		if (rst_n && start && active)
			$display("%t g42_playfield: start while busy ignored", $time);
		if (rst_n && rom_ack && d_word == 2'd2 && s_busy)
			$display("%t g42_playfield: C word returned while storing", $time);
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
