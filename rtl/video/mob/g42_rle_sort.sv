//============================================================================
//  Atari G42 for MiSTer
//  g42_rle_sort.sv -- motion object draw order
//
//  Draw order is the 8-bit order field of each descriptor. Port of MAME
//  sort_and_render():
//
//      for objnum in 0 .. 255:                // build 256 linked lists
//          next[objnum] = head[order[objnum]] // push onto the front
//          head[order[objnum]] = objnum
//      for order in 1 .. 255:                 // render; order 0 is skipped
//          walk head[order] via next[]
//
//  Order 0 is never drawn. Each list is pushed at the front, so objects of
//  equal order draw in descending index order.
//
//  Build: clear the 256 heads (256 clocks, before the snapshot starts), then
//  one push per object as g42_rle_objram delivers the orders; a push needs
//  two clocks and they come at least six apart. Walk: two clocks per empty
//  bucket, one per object. walk_done is a level from the end of the walk
//  until the next clear.
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_rle_sort (
	input  wire         clk,
	input  wire         rst_n,

	// ---- Build --------------------------------------------------------------
	input  wire         clear_start,   // pulse: empty all buckets
	output logic        clear_busy,
	input  wire         push,          // from g42_rle_objram, objects 0..255
	input  wire [7:0]   push_obj,
	input  wire [7:0]   push_order,
	input  wire         walk_start,    // pulse: the build is complete

	// ---- Walk ---------------------------------------------------------------
	input  wire         walk_next,     // pulse: done with walk_obj
	output logic [7:0]  walk_obj,      // object to render
	output logic        walk_valid,    // walk_obj is meaningful
	output logic        walk_done      // every bucket has been walked
);

	//========================================================================
	//  Storage: {valid, object} per entry, one M10K each
	//========================================================================
	// head[] keeps one read and one write address so it stays a RAM.
	//========================================================================
	logic [8:0] head [256];
	logic [8:0] nxt  [256];

	logic [7:0] head_raddr, head_waddr;
	logic [8:0] head_wdata, head_q;
	logic       head_we;

	logic [7:0] nxt_raddr;
	logic [8:0] nxt_q;
	logic       nxt_we;
	logic [7:0] nxt_waddr;
	logic [8:0] nxt_wdata;

	always_ff @(posedge clk) begin
		if (head_we) head[head_waddr] <= head_wdata;
		head_q <= head[head_raddr];
	end

	always_ff @(posedge clk) begin
		if (nxt_we) nxt[nxt_waddr] <= nxt_wdata;
		nxt_q <= nxt[nxt_raddr];
	end

	//========================================================================
	//  Control
	//========================================================================
	typedef enum logic [2:0] {
		S_IDLE, S_CLEAR, S_BUILD, S_SCAN, S_SCAN_W, S_OUT, S_NEXT_W
	} sstate_t;

	sstate_t    state;
	logic [7:0] clear_i;
	logic       push_d;        // second clock of a push: head_q is ready
	logic [7:0] push_obj_d, push_order_d;
	logic [7:0] order;         // bucket being walked

	assign clear_busy = (state == S_CLEAR);

	always_comb begin
		// head[] ports: the clear and the push share the write port; the
		// push reads its bucket, the walk reads the bucket it is on.
		head_raddr = (state == S_BUILD) ? push_order : order;
		if (state == S_CLEAR) begin
			head_waddr = clear_i;
			head_wdata = 9'd0;
			head_we    = 1'b1;
		end else begin
			head_waddr = push_order_d;
			head_wdata = {1'b1, push_obj_d};
			head_we    = push_d;
		end
		nxt_waddr = push_obj_d;
		nxt_wdata = head_q;               // old head (or empty) of the bucket
		nxt_we    = push_d;
		nxt_raddr = walk_obj;
	end

	always_ff @(posedge clk) begin
		if (!rst_n) begin
			state      <= S_IDLE;
			push_d     <= 1'b0;
			walk_valid <= 1'b0;
			walk_done  <= 1'b0;
		end
		else begin
			push_d       <= push && (state == S_BUILD);
			push_obj_d   <= push_obj;
			push_order_d <= push_order;

			if (clear_start) begin
				clear_i    <= 8'd0;
				walk_valid <= 1'b0;
				walk_done  <= 1'b0;
				state      <= S_CLEAR;
			end
			else case (state)

			S_IDLE: ;

			S_CLEAR:
				if (clear_i == 8'hFF) state <= S_BUILD;
				else                  clear_i <= clear_i + 8'd1;

			S_BUILD:
				if (walk_start) begin
					order <= 8'd1;          // order 0 is never drawn
					state <= S_SCAN_W;
				end

			// head_q follows order with one clock of latency.
			S_SCAN_W: state <= S_SCAN;

			S_SCAN:
				if (head_q[8]) begin
					walk_obj   <= head_q[7:0];
					walk_valid <= 1'b1;
					state      <= S_OUT;
				end
				else if (order == 8'hFF) begin
					walk_done <= 1'b1;
					state     <= S_IDLE;
				end
				else begin
					order <= order + 8'd1;
					state <= S_SCAN_W;
				end

			// nxt_q (next[walk_obj]) is ready one clock after walk_obj.
			S_OUT:
				if (walk_next) begin
					walk_valid <= 1'b0;
					state      <= S_NEXT_W;
				end

			S_NEXT_W:
				if (nxt_q[8]) begin
					walk_obj   <= nxt_q[7:0];
					walk_valid <= 1'b1;
					state      <= S_OUT;
				end
				else if (order == 8'hFF) begin
					walk_done <= 1'b1;
					state     <= S_IDLE;
				end
				else begin
					order <= order + 8'd1;
					state <= S_SCAN_W;
				end

			default: state <= S_IDLE;
			endcase
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	always_ff @(posedge clk)
		if (rst_n && push && state != S_BUILD)
			$error("g42_rle_sort: push outside the build phase");
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
