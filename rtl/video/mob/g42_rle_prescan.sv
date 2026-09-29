//============================================================================
//  Atari G42 for MiSTer
//  g42_rle_prescan.sv -- load-time object size table
//
//  The RLE ROM stores neither width nor height. After the ROM download this
//  walks every object once (MAME prescan_rle()) and fills a table the render
//  engine reads in one clock per object:
//
//    height = number of rows before the terminator (a count of 0)
//    width  = maximum over the rows of the sum of the row's run lengths
//
//  Object header, 4 words per object at the start of the ROM:
//    word 0  X hotspot offset, signed       (read per object at render time)
//    word 1  Y hotspot offset, signed       (read per object at render time)
//    word 2  [10:8] encoding mode, [7:0] data offset bits 23:16
//    word 3  data offset bits 15:0 (word offset into the ROM)
//  A row is a count word and that many packet words (two RLE bytes each).
//
//  Unlike Atari G1, whose table held all 78 bits of an object, only what
//  has to be precomputed is stored: {width[9:0], height[8:0]}. The render
//  sequencer reads the four header words from SDRAM when it sets an object
//  up (four accesses per object, a few hundred per frame). An object MAME
//  treats as invalid (data offset below its own header entry or past the
//  ROM) is stored as width 0, as is a valid object with no rows: MAME draws
//  nothing for either, so width 0 simply means "skip".
//
//  Table: 5120 x 19 bits, enough for Guardians' 4750 objects (Road Riot
//  832, Danger Express 4096): 10 M10K as five 1024 x 10 pairs. Codes at or
//  above obj_count are never looked up (MAME: code < m_objectcount).
//
//  The walk streams SDRAM reads back to back, one request always pending.
//  Expected results (valid objects, max width, max height):
//    Road Riot 771 / 513 / 199, Guardians 3444 / 305 / 231,
//    Danger Express 3719 / 513 / 302.
//  It reads every word of every object once: about 1, 3 and 4 M words, so
//  roughly 0.1, 0.35 and 0.5 s at ~7 clocks per access. A DRAW before it has
//  finished is ignored (g42_rle); CHECKSUM works meanwhile.
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_rle_prescan
	import g42_pkg::*;
(
	input  wire         clk,
	input  wire         rst_n,

	// ---- Control ----------------------------------------------------------
	input  wire         start,          // level: start when idle (held until busy)
	input  wire [12:0]  obj_count,      // objects in the ROM (MRA config)
	input  wire [22:0]  rom_words,      // ROM length in words (MAME length())
	output logic        busy,

	// ---- SDRAM read port (RLE object ROM) ---------------------------------
	output logic [24:0] rom_addr,
	output logic        rom_req,
	input  wire [15:0]  rom_dout,
	input  wire         rom_ack,

	// ---- Size table read port (1 clock) -----------------------------------
	input  wire [12:0]  q_code,
	output logic [9:0]  q_width,        // 0 = do not draw
	output logic [8:0]  q_height,

	// ---- Statistics --------------------------------------------------------
	output logic [12:0] stat_valid,     // objects with width > 0
	output logic [9:0]  stat_max_w,
	output logic [8:0]  stat_max_h
);

	// The RLE decode tables' package is imported here rather than in the
	// header: Quartus 17.0 accepts only one import in a module header.
	import g42_rle_pkg::*;

	localparam int ENTRIES = 5120;

	//========================================================================
	//  Table
	//========================================================================
	// {width[9:0], height[8:0]} per object code: 5120 x 19, twelve M10K as
	// 2048 x 5 (g42_dpram_d2k, Intel's RAM coding template; see
	// g42_dpram.sv).
	wire  [18:0] q_entry;
	logic [12:0] wr_code;
	logic [18:0] wr_entry;
	logic        wr_en;

	g42_dpram_d2k #(.DW(19), .AW(13), .DEPTH(ENTRIES)) u_dims
	(
		.clk   (clk),
		.we    (wr_en),
		.waddr (wr_code),
		.wdata (wr_entry),
		.raddr (q_code),
		.rdata (q_entry)
	);

	assign q_width  = q_entry[18:9];
	assign q_height = q_entry[8:0];

	//========================================================================
	//  Walk
	//========================================================================
	typedef enum logic [2:0] {
		P_IDLE,
		P_HDR,          // header words 2 and 3
		P_ROW,          // a row's count word
		P_PKT,          // packet words
		P_STORE
	} pstate_t;

	pstate_t     pstate;
	logic [12:0] code;
	logic        hdr_second;     // reading header word 3
	logic [7:0]  off_hi;
	logic [2:0]  mode;
	logic [22:0] ptr;            // word offset of the next read
	logic [15:0] entries_left;
	logic [12:0] row_width;      // up to 255 entries x 32 pixels in principle
	logic [12:0] width;
	logic [10:0] height;         // MAME stops a walk at 1024 rows

	assign busy = (pstate != P_IDLE);

	// Header address: code * 4 + word, in words.
	function automatic [24:0] hdr_addr(input [12:0] c, input w3);
		hdr_addr = SDR_RLE + {9'd0, c, 1'b1, w3, 1'b0};
	endfunction

	function automatic [24:0] word_addr(input [22:0] p);
		word_addr = SDR_RLE + {1'b0, p, 1'b0};
	endfunction

	// The 24-bit data offset of the header being read, and MAME's validity
	// test: offset >= which * 4 && offset < length.
	wire [23:0] hdr_offset = {off_hi, rom_dout};
	wire        hdr_valid  = (hdr_offset >= {9'd0, code, 2'd0})
	                      && (hdr_offset <  {1'b0, rom_words});

	wire [15:0] row_count  = rle_count(rom_dout);
	wire [12:0] word_run   = {8'd0, rle_run(mode, rom_dout[7:0])}
	                       + {8'd0, rle_run(mode, rom_dout[15:8])};
	wire [12:0] row_width_n = row_width + word_run;

	always_ff @(posedge clk) begin
		wr_en <= 1'b0;

		if (!rst_n) begin
			pstate     <= P_IDLE;
			rom_req    <= 1'b0;
			stat_valid <= '0;
			stat_max_w <= '0;
			stat_max_h <= '0;
		end
		else begin
			case (pstate)

			P_IDLE:
				if (start) begin
					code       <= 13'd0;
					hdr_second <= 1'b0;
					stat_valid <= '0;
					stat_max_w <= '0;
					stat_max_h <= '0;
					rom_addr   <= hdr_addr(13'd0, 1'b0);
					rom_req    <= 1'b1;
					pstate     <= P_HDR;
				end

			//---- Header words 2 and 3 ----------------------------------------
			// The requester keeps rom_req high through the ack clock, where
			// the arbiter ignores it, and presents the next address after it.
			P_HDR:
				if (rom_ack) begin
					if (!hdr_second) begin
						mode       <= rom_dout[10:8];
						off_hi     <= rom_dout[7:0];
						hdr_second <= 1'b1;
						rom_addr   <= hdr_addr(code, 1'b1);
					end else begin
						hdr_second <= 1'b0;
						width      <= '0;
						height     <= '0;
						ptr        <= hdr_offset[22:0];
						if (hdr_valid) begin
							rom_addr <= word_addr(hdr_offset[22:0]);
							pstate   <= P_ROW;
						end else begin
							rom_req <= 1'b0;
							pstate  <= P_STORE;
						end
					end
				end

			//---- Row count -------------------------------------------------
			P_ROW:
				if (rom_ack) begin
					ptr          <= ptr + 23'd1;
					entries_left <= row_count;
					row_width    <= '0;
					if (row_count == 16'd0) begin
						rom_req <= 1'b0;
						pstate  <= P_STORE;
					end else if (ptr + 23'd1 >= rom_words) begin
						// MAME: "base < end" ends the object at the ROM end
						// after counting this row; its packets are cut off.
						height  <= height + 11'd1;
						rom_req <= 1'b0;
						pstate  <= P_STORE;
					end else begin
						rom_addr <= word_addr(ptr + 23'd1);
						pstate   <= P_PKT;
					end
				end

			//---- Packets: both bytes' runs added in one clock ------------------
			P_PKT:
				if (rom_ack) begin
					ptr          <= ptr + 23'd1;
					entries_left <= entries_left - 16'd1;
					row_width    <= row_width_n;
					if (entries_left == 16'd1 || ptr + 23'd1 >= rom_words) begin
						// Row done: fold its width into the maximum.
						if (row_width_n > width) width <= row_width_n;
						height <= height + 11'd1;
						if (height == 11'd1023 || ptr + 23'd1 >= rom_words) begin
							rom_req <= 1'b0;                // MAME's 1024-row cap
							pstate  <= P_STORE;
						end else begin
							rom_addr <= word_addr(ptr + 23'd1);
							pstate   <= P_ROW;
						end
					end else begin
						rom_addr <= word_addr(ptr + 23'd1);
					end
				end

			//---- Store and advance -----------------------------------------------
			P_STORE: begin
				wr_code  <= code;
				wr_entry <= (width == '0) ? 19'd0
				          : {(width  > 13'd1023) ? 10'd1023 : width[9:0],
				             (height > 11'd511)  ? 9'd511   : height[8:0]};
				wr_en    <= 1'b1;
				if (width != '0) begin
					stat_valid <= stat_valid + 13'd1;
					if (width[9:0]  > stat_max_w) stat_max_w <= width[9:0];
					if (height[8:0] > stat_max_h) stat_max_h <= height[8:0];
				end
				if (code + 13'd1 >= obj_count || code == 13'(ENTRIES - 1)) begin
					pstate <= P_IDLE;
				end else begin
					code     <= code + 13'd1;
					rom_addr <= hdr_addr(code + 13'd1, 1'b0);
					rom_req  <= 1'b1;
					pstate   <= P_HDR;
				end
			end

			default: pstate <= P_IDLE;
			endcase
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	always_ff @(posedge clk) begin
		if (rst_n && pstate == P_STORE && width != '0 && (width > 13'd1023 || height > 11'd511))
			$error("g42_rle_prescan: object %0d is %0d x %0d, beyond the table's 10/9 bits",
			       code, width, height);
		if (rst_n && start && pstate == P_IDLE && obj_count > 13'(ENTRIES))
			$error("g42_rle_prescan: obj_count %0d exceeds the %0d-entry table", obj_count, ENTRIES);
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
