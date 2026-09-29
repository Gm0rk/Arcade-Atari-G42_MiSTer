//============================================================================
//  Atari G42 for MiSTer
//  g42_tdpram.sv -- true dual-port block RAM, one clock
//
//  Two ports, each with its own address, both able to read and write, on
//  one clock. For synthesis it is an explicit altsyncram in
//  BIDIR_DUAL_PORT mode (the form sys/sd_card.sv uses); for simulation, a
//  behavioural model of the same modes.
//
//  Why explicit: Quartus 17.0 inferred the 68000 work RAM (port A read and
//  write at one address, port B read only) as two simple dual-port copies
//  per byte lane, 96 M10K instead of 64. With the chip's block RAM that
//  close to full, its resource-aware RAM inference then built the design's
//  smaller memories (and some of the framework's) from registers, and the
//  design no longer fitted. A true dual-port M10K holds both ports in one
//  copy.
//
//  Read-during-write (M10K, one clock):
//    same port   the port's q returns the data being written (NEW_DATA)
//    other port  undefined (DONT_CARE); the model returns the old data.
//  Both ports' q come one clock after their address, unregistered outputs.
//
//  Clock domain: clk
//============================================================================

`default_nettype none

module g42_tdpram #(
	parameter int DW = 8,                 // data width
	parameter int AW = 10                 // address width; depth is 2^AW
)(
	input  wire          clk,

	// ---- Port A ----------------------------------------------------------
	input  wire [AW-1:0] addr_a,
	input  wire [DW-1:0] din_a,
	input  wire          we_a,
	output logic [DW-1:0] q_a,            // mem[addr_a], one clock later

	// ---- Port B ----------------------------------------------------------
	input  wire [AW-1:0] addr_b,
	input  wire [DW-1:0] din_b,
	input  wire          we_b,
	output logic [DW-1:0] q_b             // mem[addr_b], one clock later
);

`ifdef SIMULATION

	//------------------------------------------------------------------------
	//  Behavioural model (Verilator)
	//------------------------------------------------------------------------
	logic [DW-1:0] mem [0:(1<<AW)-1];

	always_ff @(posedge clk) begin
		if (we_a) begin
			mem[addr_a] <= din_a;
			q_a         <= din_a;             // NEW_DATA on the writing port
		end
		else begin
			q_a <= mem[addr_a];
		end
	end

	always_ff @(posedge clk) begin
		if (we_b) begin
			mem[addr_b] <= din_b;
			q_b         <= din_b;
		end
		else begin
			q_b <= mem[addr_b];
		end
	end

`else

	//------------------------------------------------------------------------
	//  Quartus: altsyncram, BIDIR_DUAL_PORT, both ports on clock0
	//------------------------------------------------------------------------
	altsyncram #(
		.operation_mode                     ("BIDIR_DUAL_PORT"),
		.width_a                            (DW),
		.widthad_a                          (AW),
		.numwords_a                         (1 << AW),
		.width_b                            (DW),
		.widthad_b                          (AW),
		.numwords_b                         (1 << AW),
		.width_byteena_a                    (1),
		.width_byteena_b                    (1),
		.address_reg_b                      ("CLOCK0"),
		.indata_reg_b                       ("CLOCK0"),
		.wrcontrol_wraddress_reg_b          ("CLOCK0"),
		.outdata_reg_a                      ("UNREGISTERED"),
		.outdata_reg_b                      ("UNREGISTERED"),
		.outdata_aclr_a                     ("NONE"),
		.outdata_aclr_b                     ("NONE"),
		.clock_enable_input_a               ("BYPASS"),
		.clock_enable_input_b               ("BYPASS"),
		.clock_enable_output_a              ("BYPASS"),
		.clock_enable_output_b              ("BYPASS"),
		.read_during_write_mode_port_a      ("NEW_DATA_NO_NBE_READ"),
		.read_during_write_mode_port_b      ("NEW_DATA_NO_NBE_READ"),
		.read_during_write_mode_mixed_ports ("DONT_CARE"),
		.power_up_uninitialized             ("FALSE"),
		.ram_block_type                     ("M10K"),
		.intended_device_family             ("Cyclone V"),
		.lpm_type                           ("altsyncram")
	) ram (
		.clock0         (clk),
		.address_a      (addr_a),
		.data_a         (din_a),
		.wren_a         (we_a),
		.q_a            (q_a),
		.address_b      (addr_b),
		.data_b         (din_b),
		.wren_b         (we_b),
		.q_b            (q_b),

		.clock1         (1'b1),
		.aclr0          (1'b0),
		.aclr1          (1'b0),
		.addressstall_a (1'b0),
		.addressstall_b (1'b0),
		.byteena_a      (1'b1),
		.byteena_b      (1'b1),
		.clocken0       (1'b1),
		.clocken1       (1'b1),
		.clocken2       (1'b1),
		.clocken3       (1'b1),
		.rden_a         (1'b1),
		.rden_b         (1'b1),
		.eccstatus      ()
	);

`endif

endmodule

`default_nettype wire
