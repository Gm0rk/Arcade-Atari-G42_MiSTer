//============================================================================
//  Atari G42 for MiSTer
//  g42_rle.sv -- motion object engine ("growth renderer") top level
//
//  Port of MAME atarirle.cpp as configured by atarig42.cpp (modesc_0x200 for
//  Road Riot, modesc_0x400 for Guardians and Danger Express), derived from
//  the Atari G1 engine:
//
//      g42_rle_prescan    load-time width/height table, indexed by code
//      g42_rle_objram     descriptor snapshot, six words per object
//      g42_rle_sort       256-bucket draw order
//      g42_rle_scaler     per-object placement, scaling and clipping
//      g42_rle_render     row fetcher and zoom renderer
//      g42_rle_fb         double framebuffer, control register, erase engine
//      g42_rle_checksum   ROM checksums (self-tests) and ROM length
//
//  Control: io_latch ($E00050) bits 13:11 {FRAME, ERASE, MOGO}, see
//  g42_rle_fb. Command: every write to $FF7000 latches CHECKSUM if the value
//  is 0, else DRAW (MAME mo_command_w; a byte write puts the byte on both
//  halves, so "0" means the written byte is 0); NOP until the first write.
//  A MOGO rising edge executes the command.
//
//  DRAW: snapshot object RAM (bucketing the orders on the way), then for
//  each object in draw order: read its descriptor, skip it if scale is 0,
//  its code is not below the ROM's object count or its size table entry is
//  empty; read its four ROM header words from SDRAM; scale; wait for any
//  erase of the target buffer; render. The render has a whole frame
//  (Guardians renders every frame, the others every other frame). A DRAW
//  that arrives while one is still running, or before the prescan is done,
//  is ignored and counted (mogo_drops).
//
//  Pixel colour: MAME master aligns the pen base to the object's depth,
//      pen = (palette base + (color << 4)) & ~((1 << bpp) - 1), pixel = pen + value
//  (MAME 0.264 does not; see sim/rle/README.md). Bits 8:0 of the pen are
//  {color[4:0], 0000} with the low bpp bits cleared; bits 10:9 come back in
//  g42_rle_fb and the mixer.
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_rle
	import g42_pkg::*;
(
	input  wire         clk,
	input  wire         rst_n,

	// ---- Configuration (MRA) -------------------------------------------------
	input  wire [12:0]  obj_count,      // objects in the ROM: 832, 4750 or 4096
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [7:0]   mo_base,        // MO palette base >> 8: $02 or $04 (bit 1 used)
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [5:0]   color_mask,     // colour field mask: $1F or $3F

	// ---- Registers written by the 68000 ----------------------------------------
	input  wire [2:0]   ctrl,           // io_latch bits 13:11 {FRAME, ERASE, MOGO}
	input  wire         ctrl_wr,        // one clock per write of io_latch's upper byte
	input  wire [15:0]  cmd,            // value written to $FF7000
	input  wire         cmd_wr,         // one clock per write to $FF7000
	input  wire [15:0]  objram_word0,   // work RAM word 0 (CHECKSUM count - 1), snooped

	// ---- Raster ------------------------------------------------------------------
	input  wire [8:0]   vpos,           // raster line; any value >= 240 in VBLANK
	input  wire         vblank_rise,    // one clock at the start of VBLANK

	// ---- Work RAM port (snapshot reads, CHECKSUM writes) -------------------------
	output logic [14:0] vram_addr,      // word address; combinational from vram_ack
	output logic        vram_req,       // read request, lowest priority
	input  wire [15:0]  vram_dout,      // read data, with vram_ack
	input  wire         vram_ack,       // the previous clock's read was taken
	output logic [15:0] vram_din,       // CHECKSUM write data
	output logic        vram_we,        // CHECKSUM write, bypasses the arbiter

	// ---- SDRAM read port (RLE object ROM at SDR_RLE) -------------------------------
	output logic [24:0] rom_addr,       // byte address, held with rom_req until rom_ack
	output logic        rom_req,
	input  wire [15:0]  rom_dout,       // big-endian word, valid with rom_ack
	input  wire         rom_ack,

	// ---- ROM loader snoop ----------------------------------------------------------
	input  wire         load_active,    // ROM (index 0) download in progress
	input  wire         load_rst_n,     // checksum accumulator reset (PLL lock)
	input  wire [24:0]  load_addr,
	input  wire [15:0]  load_data,
	input  wire         load_we,
	input  wire         load_complete,  // start the prescan (level, held until busy)

	// ---- Framebuffer display read ------------------------------------------------
	input  wire [8:0]   disp_x,         // visible column 0..335
	input  wire [7:0]   disp_y,         // visible line 0..239
	output logic [12:0] disp_pixel,     // {priority, index[9:0]} next clock; 0 = none

	// ---- Status ------------------------------------------------------------------
	output logic        prescan_busy,
	output logic        render_busy,
	output logic        erase_busy,
	output logic        cksum_busy,     // CHECKSUM write-back running
	output logic [12:0] stat_valid,     // prescan: objects with a non-empty entry
	output logic [9:0]  stat_max_w,     // prescan: widest object
	output logic [8:0]  stat_max_h,     // prescan: tallest object
	output logic [7:0]  mogo_drops,     // DRAWs ignored (busy or prescanning), saturating
	output logic [23:0] render_clocks,  // duration of the last DRAW, saturating
	output logic [22:0] rom_words       // RLE ROM length in words, from the loader
);

	//========================================================================
	//  Command decode
	//========================================================================
	localparam [1:0] CMD_NOP = 2'd0, CMD_DRAW = 2'd1, CMD_CKSUM = 2'd2;

	logic [1:0] command;
	always_ff @(posedge clk) begin
		if (!rst_n)      command <= CMD_NOP;
		else if (cmd_wr) command <= (cmd == 16'd0) ? CMD_CKSUM : CMD_DRAW;
	end

	//========================================================================
	//  Framebuffer and control register
	//========================================================================
	wire        mogo_rise, rnd_hold;
	logic       draw_accept;
	wire [8:0]  fb_x;
	wire [7:0]  fb_y;
	wire [11:0] fb_pixel;
	wire        fb_we;

	g42_rle_fb u_fb (
		.clk         (clk),
		.rst_n       (rst_n),
		.ctrl        (ctrl),
		.ctrl_wr     (ctrl_wr),
		.vpos        (vpos),
		.vblank_rise (vblank_rise),
		.base_bit9   (mo_base[1]),
		.color_bit5  (color_mask[5]),
		.rnd_x       (fb_x),
		.rnd_y       (fb_y),
		.rnd_pixel   (fb_pixel),
		.rnd_we      (fb_we),
		.disp_x      (disp_x),
		.disp_y      (disp_y),
		.disp_pixel  (disp_pixel),
		.mogo_rise   (mogo_rise),
		.rnd_accept  (draw_accept),
		.rnd_hold    (rnd_hold),
		.erase_busy  (erase_busy)
	);

	//========================================================================
	//  Checksum and ROM length
	//========================================================================
	wire [14:0] ck_addr;
	wire [15:0] ck_data;
	wire        ck_we, ck_busy;

	assign cksum_busy = ck_busy || ck_we;

	g42_rle_checksum u_cksum (
		.clk          (clk),
		.rst_n        (rst_n),
		.acc_rst_n    (load_rst_n),
		.load_active  (load_active),
		.load_addr    (load_addr),
		.load_data    (load_data),
		.load_we      (load_we),
		.rom_words    (rom_words),
		.cksum_start  (mogo_rise && command == CMD_CKSUM && !ck_busy),
		.objram_word0 (objram_word0),
		.wr_addr      (ck_addr),
		.wr_data      (ck_data),
		.wr_en        (ck_we),
		.busy         (ck_busy)
	);

	//========================================================================
	//  Prescan
	//========================================================================
	wire [24:0] ps_rom_addr;
	wire        ps_rom_req;
	wire [9:0]  ps_width;
	wire [8:0]  ps_height;
	wire [14:0] obj_code;

	g42_rle_prescan u_prescan (
		.clk        (clk),
		.rst_n      (rst_n),
		.start      (load_complete),
		.obj_count  (obj_count),
		.rom_words  (rom_words),
		.busy       (prescan_busy),
		.rom_addr   (ps_rom_addr),
		.rom_req    (ps_rom_req),
		.rom_dout   (rom_dout),
		.rom_ack    (rom_ack && prescan_busy),
		.q_code     (obj_code[12:0]),
		.q_width    (ps_width),
		.q_height   (ps_height),
		.stat_valid (stat_valid),
		.stat_max_w (stat_max_w),
		.stat_max_h (stat_max_h)
	);

	//========================================================================
	//  Descriptor snapshot and sort
	//========================================================================
	logic        snap_start, clear_start, walk_start, walk_next;
	logic [7:0]  desc_index;
	wire         snap_busy, clear_busy;
	wire [14:0]  or_vaddr;
	wire         or_vreq;
	wire         push;
	wire [7:0]   push_obj, push_order;
	wire         obj_hflip;
	wire [4:0]   obj_color;
	wire [2:0]   obj_priority;
	wire signed [9:0] obj_xpos, obj_ypos;
	wire [15:0]  obj_scale;
	wire [7:0]   walk_obj;
	wire         walk_valid, walk_done;

	g42_rle_objram u_objram (
		.clk          (clk),
		.rst_n        (rst_n),
		.snap_start   (snap_start),
		.snap_busy    (snap_busy),
		.snap_hold    (ck_we),
		.vram_addr    (or_vaddr),
		.vram_req     (or_vreq),
		.vram_dout    (vram_dout),
		.vram_ack     (vram_ack),
		.push         (push),
		.push_obj     (push_obj),
		.push_order   (push_order),
		.obj_index    (desc_index),
		.color_mask   (color_mask[4:0]),
		.obj_code     (obj_code),
		.obj_hflip    (obj_hflip),
		.obj_color    (obj_color),
		.obj_priority (obj_priority),
		.obj_xpos     (obj_xpos),
		.obj_ypos     (obj_ypos),
		.obj_scale    (obj_scale)
	);

	g42_rle_sort u_sort (
		.clk         (clk),
		.rst_n       (rst_n),
		.clear_start (clear_start),
		.clear_busy  (clear_busy),
		.push        (push),
		.push_obj    (push_obj),
		.push_order  (push_order),
		.walk_start  (walk_start),
		.walk_next   (walk_next),
		.walk_obj    (walk_obj),
		.walk_valid  (walk_valid),
		.walk_done   (walk_done)
	);

	// The checksum write takes the address lines for its clock; the
	// snapshot holds its read off then (snap_hold).
	always_comb begin
		vram_addr = ck_we ? ck_addr : or_vaddr;
		vram_req  = or_vreq;
		vram_din  = ck_data;
		vram_we   = ck_we;
	end

	//========================================================================
	//  Header fetch: four words at code * 8
	//========================================================================
	logic        hdr_active;
	logic [1:0]  hdr_word;
	logic [24:0] hdr_addr;
	logic signed [15:0] hdr_xoffs, hdr_yoffs;
	logic [2:0]  hdr_mode;
	logic [6:0]  hdr_off_hi;     // offset bits 22:16 (bit 23: past 8 MB, never valid)
	logic [15:0] hdr_off_lo;

	//========================================================================
	//  Scaler and renderer
	//========================================================================
	logic        scale_start, rnd_start;
	wire         sc_skip, sc_done;
	wire [7:0]   sc_y0;
	wire [8:0]   sc_nrows, sc_dest0, sc_bound;
	wire [31:0]  sc_sourcey0;
	wire [25:0]  sc_dy, sc_dx;
	wire [27:0]  sc_sourcex0;
	wire [11:0]  sc_src_limit;

	g42_rle_scaler u_scaler (
		.clk       (clk),
		.rst_n     (rst_n),
		.start     (scale_start),
		.scale     (obj_scale),
		.width     (ps_width),
		.height    (ps_height),
		.xoffs     (hdr_xoffs),
		.yoffs     (hdr_yoffs),
		.xpos      (obj_xpos),
		.ypos      (obj_ypos),
		.hflip     (obj_hflip),
		.skip      (sc_skip),
		.y0        (sc_y0),
		.nrows     (sc_nrows),
		.sourcey0  (sc_sourcey0),
		.dy        (sc_dy),
		.dx        (sc_dx),
		.sourcex0  (sc_sourcex0),
		.dest0     (sc_dest0),
		.bound     (sc_bound),
		.src_limit (sc_src_limit),
		.done      (sc_done)
	);

	// Pen base bits 8:0: {color[4:0], 0000} with the object's low bpp bits
	// cleared (MAME master's alignment); the packet value fills them.
	logic [8:0] pen;
	always_comb begin
		pen = {obj_color[4:0], 4'd0};
		case (rle_mode_bpp(hdr_mode))
			3'd5:    pen[4]   = 1'b0;
			3'd6:    pen[5:4] = 2'b00;
			default: ;
		endcase
	end

	wire [24:0] rnd_rom_addr;
	wire        rnd_rom_req, rnd_done;

	g42_rle_render u_render (
		.clk       (clk),
		.rst_n     (rst_n),
		.start     (rnd_start),
		.data_ptr  ({hdr_off_hi, hdr_off_lo}),
		.mode      (hdr_mode),
		.pix_base  ({obj_priority, pen}),
		.hflip     (obj_hflip),
		.y0        (sc_y0),
		.nrows     (sc_nrows),
		.sourcey0  (sc_sourcey0),
		.dy        (sc_dy),
		.dx        (sc_dx),
		.sourcex0  (sc_sourcex0),
		.dest0     (sc_dest0),
		.bound     (sc_bound),
		.src_limit (sc_src_limit),
		.rom_addr  (rnd_rom_addr),
		.rom_req   (rnd_rom_req),
		.rom_dout  (rom_dout),
		.rom_ack   (rom_ack && !prescan_busy && !hdr_active),
		.fb_x      (fb_x),
		.fb_y      (fb_y),
		.fb_pixel  (fb_pixel),
		.fb_we     (fb_we),
		.done      (rnd_done)
	);

	//========================================================================
	//  SDRAM port: prescan (after a ROM load), header fetch, renderer. Only
	//  one is ever active.
	//========================================================================
	always_comb begin
		if (prescan_busy) begin
			rom_addr = ps_rom_addr;
			rom_req  = ps_rom_req;
		end else if (hdr_active) begin
			rom_addr = hdr_addr;
			rom_req  = 1'b1;
		end else begin
			rom_addr = rnd_rom_addr;
			rom_req  = rnd_rom_req;
		end
	end

	//========================================================================
	//  Render sequencer
	//========================================================================
	typedef enum logic [3:0] {
		R_IDLE, R_CLEAR, R_SNAP, R_SNAP_END, R_WALK, R_DESC, R_DIMS,
		R_HDR, R_SCALE, R_HOLD, R_RENDER, R_NEXT
	} rstate_t;

	rstate_t    rstate;
	logic [1:0] lat;

	assign render_busy = (rstate != R_IDLE);
	assign walk_next   = (rstate == R_NEXT);

	// High in the clock R_IDLE accepts a MOGO as a DRAW; g42_rle_fb latches
	// the render target on it.
	assign draw_accept = (rstate == R_IDLE) && mogo_rise && (command == CMD_DRAW)
	                  && !prescan_busy;

	always_ff @(posedge clk) begin
		snap_start  <= 1'b0;
		clear_start <= 1'b0;
		walk_start  <= 1'b0;
		scale_start <= 1'b0;
		rnd_start   <= 1'b0;

		if (!rst_n) begin
			rstate     <= R_IDLE;
			hdr_active <= 1'b0;
			mogo_drops <= 8'd0;
		end
		else begin
			// A DRAW that cannot be taken is only counted.
			if (mogo_rise && command == CMD_DRAW && !draw_accept && mogo_drops != 8'hFF)
				mogo_drops <= mogo_drops + 8'd1;

			case (rstate)

			R_IDLE:
				if (draw_accept) begin
					clear_start <= 1'b1;
					lat         <= 2'd0;
					rstate      <= R_CLEAR;
				end

			// clear_busy rises the clock after clear_start.
			R_CLEAR:
				if (lat != 2'd2) lat <= lat + 2'd1;
				else if (!clear_busy) begin
					snap_start <= 1'b1;
					lat        <= 2'd0;
					rstate     <= R_SNAP;
				end

			R_SNAP:
				if (lat != 2'd2) lat <= lat + 2'd1;
				else if (!snap_busy) begin
					lat    <= 2'd0;
					rstate <= R_SNAP_END;
				end

			// The last push reaches head[] two clocks after the copy ends.
			R_SNAP_END:
				if (lat != 2'd2) lat <= lat + 2'd1;
				else begin
					walk_start <= 1'b1;
					rstate     <= R_WALK;
				end

			R_WALK:
				if (walk_valid) begin
					desc_index <= walk_obj;
					lat        <= 2'd0;
					rstate     <= R_DESC;
				end
				else if (walk_done && !walk_start) begin
					rstate <= R_IDLE;
				end

			// Fields are valid three clocks after desc_index is set; the size
			// table is read from obj_code meanwhile and valid a clock later.
			R_DESC:
				if (lat != 2'd2) lat <= lat + 2'd1;
				else if (obj_scale == 16'd0 || obj_code >= {2'd0, obj_count})
					rstate <= R_NEXT;                         // MAME: scale > 0 && code < count
				else
					rstate <= R_DIMS;

			R_DIMS:
				if (ps_width == 10'd0) begin
					rstate <= R_NEXT;                         // invalid or empty object
				end else begin
					hdr_addr   <= SDR_RLE + {9'd0, obj_code[12:0], 3'd0};
					hdr_word   <= 2'd0;
					hdr_active <= 1'b1;
					rstate     <= R_HDR;
				end

			// Four header words, one request always pending.
			R_HDR:
				if (rom_ack) begin
					case (hdr_word)
						2'd0: hdr_xoffs <= rom_dout;
						2'd1: hdr_yoffs <= rom_dout;
						2'd2: begin
							hdr_mode   <= rom_dout[10:8];
							hdr_off_hi <= rom_dout[6:0];
						end
						default: hdr_off_lo <= rom_dout;
					endcase
					hdr_word <= hdr_word + 2'd1;
					hdr_addr <= hdr_addr + 25'd2;
					if (hdr_word == 2'd3) begin
						hdr_active  <= 1'b0;
						scale_start <= 1'b1;
						rstate      <= R_SCALE;
					end
				end

			R_SCALE:
				if (sc_done) rstate <= sc_skip ? R_NEXT : R_HOLD;

			// Wait out any queued or running erase of the render target.
			R_HOLD:
				if (!rnd_hold) begin
					rnd_start <= 1'b1;
					rstate    <= R_RENDER;
				end

			R_RENDER:
				if (rnd_done) rstate <= R_NEXT;

			R_NEXT: rstate <= R_WALK;

			default: rstate <= R_IDLE;
			endcase
		end
	end

	// Duration of the last DRAW, for the diagnostic overlay and the tests.
	logic [23:0] rclk;
	always_ff @(posedge clk) begin
		if (!rst_n) begin
			rclk          <= '0;
			render_clocks <= '0;
		end else if (draw_accept) begin
			rclk <= 24'd1;
		end else if (rstate != R_IDLE) begin
			if (rclk != 24'hFFFFFF) rclk <= rclk + 24'd1;
		end else if (rclk != 24'd0) begin
			render_clocks <= rclk;
			rclk          <= 24'd0;
		end
	end

endmodule

`default_nettype wire
