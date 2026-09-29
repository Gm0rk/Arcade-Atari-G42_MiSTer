//============================================================================
//  Atari G42 for MiSTer
//  g42_rle_objram.sv -- motion object descriptor snapshot and field extraction
//
//  The 256 descriptors live at CPU $FF0000-$FF0FFF, inside the 64 KB work
//  RAM. MAME reads them in sort_and_render(), instantly at the MOGO; here a
//  DRAW copies them into BRAM and releases the work RAM video port, which
//  would otherwise be contended with the tile fetchers for a whole frame.
//
//  The games start rewriting object RAM soon after the MOGO (earliest seen
//  in MAME: Danger Express 3.8 lines, Guardians 7.5, Road Riot 13.2), so the
//  copy has to be quick. It reads only the six words the engine uses and
//  keeps one read in flight every clock: 1536 reads, about 1600 clocks
//  (0.4 lines) while the port is free, which in VBLANK it is.
//
//  Descriptor (modesc_0x200 / modesc_0x400 in atarig42.cpp), 8 words:
//    word 0  [14:0] code, [15] hflip
//    word 1  [9:4] color ($1F mask on Road Riot, $3F on the others; bit 9
//            is also priority bit 0), [11:9] priority
//    word 2  [15:6] X position, 10-bit signed
//    word 3  [15:6] Y position, 10-bit signed
//    word 4  scale, 4.12 fixed point ($1000 = 1:1, 0 = skip)
//    word 6  [7:0] order
//    words 5 and 7 are not used
//  Each object's order goes to g42_rle_sort as it arrives (push), in object
//  order as MAME's bucket loop; the rest is stored, 60 bits per object.
//
//  Work RAM port (arbitrated in the video module, lowest priority): the
//  word presented with vram_req is read if the port is free that clock, and
//  vram_ack comes one clock later with vram_dout. An ack therefore says the
//  previous clock's address was taken, so the address to present is chosen
//  combinationally from vram_ack: the next word after an ack, else the same
//  word again. vram_addr/vram_req are combinational from vram_ack.
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_rle_objram (
	input  wire         clk,
	input  wire         rst_n,

	input  wire         snap_start,    // pulse: copy object RAM (DRAW)
	output logic        snap_busy,
	input  wire         snap_hold,     // the checksum owns vram_addr this clock

	// ---- Work RAM read port -------------------------------------------------
	output logic [14:0] vram_addr,
	output logic        vram_req,
	input  wire [15:0]  vram_dout,
	input  wire         vram_ack,

	// ---- Order of each object, as it arrives (to g42_rle_sort) --------------
	output logic        push,          // one clock per object, objects 0..255
	output logic [7:0]  push_obj,
	output logic [7:0]  push_order,

	// ---- Descriptor read: fields valid two clocks after obj_index -----------
	input  wire [7:0]   obj_index,
	input  wire [4:0]   color_mask,    // colour mask bits 4:0 (MRA config)
	output logic [14:0] obj_code,
	output logic        obj_hflip,
	output logic [4:0]  obj_color,     // masked; bit 5 is priority bit 0
	output logic [2:0]  obj_priority,
	output logic signed [9:0] obj_xpos,
	output logic signed [9:0] obj_ypos,
	output logic [15:0] obj_scale
);

	//========================================================================
	//  Snapshot storage
	//========================================================================
	// {w0[15:0], w1[11:4], w2[15:6], w3[15:6], w4[15:0]} = 60 bits x 256:
	// two M10K (256 x 40 each). Written once per object when its last word
	// arrives, from the words collected below.
	//========================================================================
	logic [59:0] snap [256];

	//========================================================================
	//  Copy sequencer
	//========================================================================
	// idx counts the words still to be taken: obj = object, wsel = 0..5 for
	// descriptor words 0, 1, 2, 3, 4, 6.
	//========================================================================
	logic        active;
	logic [7:0]  cur_obj;       // oldest word not yet taken
	logic [2:0]  cur_w;
	logic [15:0] w0, w4;
	logic [7:0]  w1;             // [11:4]: priority and colour
	logic [9:0]  w2, w3;         // [15:6]: position

	function automatic [2:0] word_of(input [2:0] wsel);
		word_of = (wsel == 3'd5) ? 3'd6 : wsel;
	endfunction

	// On an ack the oldest word has been read: present the one after it.
	wire       last_word = (cur_obj == 8'hFF) && (cur_w == 3'd5);
	wire [7:0] pres_obj  = !vram_ack      ? cur_obj
	                     : (cur_w == 3'd5) ? cur_obj + 8'd1 : cur_obj;
	wire [2:0] pres_w    = !vram_ack      ? cur_w
	                     : (cur_w == 3'd5) ? 3'd0 : cur_w + 3'd1;

	assign vram_addr = {4'd0, pres_obj, word_of(pres_w)};
	assign vram_req  = active && !(vram_ack && last_word) && !snap_hold;
	assign snap_busy = active;

	always_ff @(posedge clk) begin
		push <= 1'b0;
		if (!rst_n) begin
			active  <= 1'b0;
			cur_obj <= 8'd0;
			cur_w   <= 3'd0;
		end
		else if (snap_start) begin
			active  <= 1'b1;
			cur_obj <= 8'd0;
			cur_w   <= 3'd0;
		end
		else if (active && vram_ack) begin
			case (cur_w)
				3'd0: w0 <= vram_dout;
				3'd1: w1 <= vram_dout[11:4];
				3'd2: w2 <= vram_dout[15:6];
				3'd3: w3 <= vram_dout[15:6];
				3'd4: w4 <= vram_dout;
				default: begin
					// Word 6: the object is complete.
					snap[cur_obj] <= {w0, w1, w2, w3, w4};
					push       <= 1'b1;
					push_obj   <= cur_obj;
					push_order <= vram_dout[7:0];
				end
			endcase
			cur_obj <= pres_obj;
			cur_w   <= pres_w;
			if (last_word) active <= 1'b0;
		end
	end

	//========================================================================
	//  Descriptor read and field extraction (two clocks)
	//========================================================================
	logic [59:0] q;

	always_ff @(posedge clk) q <= snap[obj_index];

	wire [15:0] q_w0   = q[59:44];
	wire [7:0]  q_w1   = q[43:36];      // w1[11:4]
	wire [9:0]  q_x    = q[35:26];      // w2[15:6]
	wire [9:0]  q_y    = q[25:16];      // w3[15:6]
	wire [15:0] q_w4   = q[15:0];

	always_ff @(posedge clk) begin
		obj_code     <= q_w0[14:0];
		obj_hflip    <= q_w0[15];
		obj_color    <= q_w1[4:0] & color_mask;
		obj_priority <= q_w1[7:5];
		obj_xpos     <= q_x;
		obj_ypos     <= q_y;
		obj_scale    <= q_w4;
	end

endmodule

`default_nettype wire
