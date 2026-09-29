//============================================================================
//  Atari G42 for MiSTer
//  g42_dbg_text.sv -- diagnostic overlay: labelled hex readout in a panel
//
//  Draws N_ROWS labelled 32-bit values as 5x7 glyphs in 8x8 cells, in a panel
//  at (BOX_X, BOX_Y). Each row is 13 cells: a 4-character label, a gap and
//  eight hex digits (104 x 160 pixels for 20 rows). The caller draws the
//  glyphs white on a black panel and leaves the picture around it untouched.
//  From the Atari G1 core (g1_dbg_text.sv), with this core's rows.
//
//  Row labels, in order (LABELS_PK; the values are described where the top
//  level builds them):
//    BLD CPU BOOT WDOG IRQ ROMW CACH SND JSA ASIC MOGO RLE VFET PALW SUMW SUMR REPE PHAS SW03 SW47 LOAD
//
//  Clock domain: clk_sys; outputs are two clocks behind vis_x/vis_y.
//============================================================================

`default_nettype none

module g42_dbg_text #(
	parameter int N_ROWS = 21,
	parameter int BOX_X  = 8,            // top-left corner of the panel, in pixels
	parameter int BOX_Y  = 8
)(
	input  wire                  clk,
	input  wire  [8:0]           vis_x,
	input  wire  [7:0]           vis_y,
	// Values, packed MSB first: row 0 occupies bits [32*N_ROWS-1 -: 32].
	input  wire  [32*N_ROWS-1:0] vals,
	// Ninth digit for row 0 (BLD: YYMMDD and a three-digit build), drawn in
	// the gap cell.
	input  wire  [3:0]           row0_msn,

	output logic                 pix,    // glyph pixel (draw white)
	output logic                 in_box  // inside the panel (draw black)
);

	localparam int COLS  = 13;           // 4 label + 1 gap + 8 hex
	localparam int CELL  = 8;
	localparam int BOX_W = COLS   * CELL;
	localparam int BOX_H = N_ROWS * CELL;

	// Font and labels are packed vectors: Quartus 17.0 Lite crashes on
	// unpacked array ports and handles unpacked constant arrays poorly.
	// Character codes: 0-9, A-Z = 10-35, 36 = space. Glyph g occupies bits
	// [35*(36-g) +: 35], row-major, MSB = top-left. Generated from the G1 font.
	localparam logic [37*35-1:0] FONT_PK = 1295'b01110100011001110101110011000101110001000110000100001000010000100011100111010001000010001000100010001111111111000100010000010000011000101110000100011001010100101111100010000101111110000111100000100001100010111000110010001000011110100011000101110111110000100010001000100001000010000111010001100010111010001100010111001110100011000101111000010001001100011101000110001111111000110001100011111010001100011111010001100011111001110100011000010000100001000101110111001001010001100011000110010111001111110000100001111010000100001111111111100001000011110100001000010000011101000110000101111000110001011111000110001100011111110001100011000101110001000010000100001000010001110001110001000010000100001010010011001000110010101001100010100100101000110000100001000010000100001000011111100011101110101101011000110001100011000111001101011001110001100011000101110100011000110001100011000101110111101000110001111101000010000100000111010001100011000110101100100110111110100011000111110101001001010001011111000010000011100000100001111101111100100001000010000100001000010010001100011000110001100011000101110100011000110001100011000101010001001000110001100011010110101110111000110001100010101000100010101000110001100011000101010001000010000100001001111100001000100010001000100001111100000000000000000000000000000000000;

	// 6-bit character codes; row r, column c at [6*(4*(N_ROWS-1-r) + (3-c)) +: 6].
	localparam logic [21*4*6-1:0] LABELS_PK = 504'b001011010101001101100100001100011001011110100100001011011000011000011101100000001101011000010000010010011011011010100100011011011000010110100000001100001010001100010001011100010111001101100100010011011100001010100100001010011100010010001100010110011000010000011000011011010101001110100100011111001111001110011101011001001010010101100000011100011110010110100000011100011110010110011011011011001110011001001110011001010001001010011100011100100000000000000011011100100000000100000111010101011000001010001101;

	wire in_panel = (vis_x >= BOX_X) && (vis_x < BOX_X + BOX_W)
	             && (vis_y >= BOX_Y) && (vis_y < BOX_Y + BOX_H);

	wire [8:0] px = vis_x - BOX_X[8:0];
	wire [7:0] py = vis_y - BOX_Y[7:0];

	wire [4:0] cell_x = px[7:3];         // character column, 0..12
	wire [4:0] cell_y = py[7:3];         // character row
	wire [2:0] ox     = px[2:0];         // pixel within the cell
	wire [2:0] oy     = py[2:0];

	// 5x7 glyph in an 8x8 cell: columns 1-5, rows 0-6
	wire glyph_area = (ox >= 3'd1) && (ox < 3'd6) && (oy < 3'd7);
	wire [2:0] gx   = ox - 3'd1;
	wire [2:0] gy   = oy;

	wire       row0     = (cell_y == 5'd0);
	wire       is_label = (cell_x < 5'd4);
	wire       is_msn   = row0 && (cell_x == 5'd4);
	wire       is_hex   = (cell_x >= 5'd5) && (cell_x < 5'd13);
	wire [2:0] hex_i    = 3'(cell_x - 5'd5);  // 0 = most significant nibble

	logic [5:0]  ch;
	logic [31:0] row_val;
	logic [3:0]  nib;

	always_comb begin
		ch      = 6'd36;
		row_val = 32'd0;
		nib     = 4'd0;
		if (in_panel && (cell_y < N_ROWS)) begin
			row_val = vals[32*(N_ROWS-1-cell_y) +: 32];
			nib     = row_val[4*(7-hex_i) +: 4];
			if (is_label)
				ch = LABELS_PK[6*(4*(N_ROWS-1-cell_y) + (3-cell_x[1:0])) +: 6];
			else if (is_msn)
				ch = {2'd0, row0_msn};
			else if (is_hex)
				ch = {2'd0, nib};
		end
	end

	//------------------------------------------------------------------------
	// Two register stages: the character selection and the font lookup (a x35
	// multiply and a 1295-to-1 select) do not fit in one clock together.
	//------------------------------------------------------------------------
	logic [5:0] ch_r;
	logic [2:0] gx_r, gy_r;
	logic       glyph_r, in_box_r;

	always_ff @(posedge clk) begin
		ch_r     <= ch;
		gx_r     <= gx;
		gy_r     <= gy;
		glyph_r  <= in_panel && glyph_area;
		in_box_r <= in_panel;
	end

	always_ff @(posedge clk) begin
		pix    <= glyph_r && FONT_PK[35*(36-ch_r) + (34 - (gy_r*5 + gx_r))];
		in_box <= in_box_r;
	end

endmodule

`default_nettype wire
