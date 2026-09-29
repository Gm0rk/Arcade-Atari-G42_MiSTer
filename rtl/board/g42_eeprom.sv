//============================================================================
//  Atari G42 for MiSTer
//  g42_eeprom.sv -- 2816 parallel EEPROM (2 K x 8) with unlock protection
//
//  68000 $FA0000-$FA0FFF, low byte only (MAME umask16 $00FF). Holds the
//  operator settings, statistics and, for Road Riot, the control
//  calibration. MAME: EEPROM_2816 with lock_after_write(true): a write is
//  ignored unless the 68000 has written to $E00060 since the last accepted
//  one, which guards the settings against a runaway program.
//
//  Contents come from three places, in the order MiSTer sends them:
//    1. the factory image in the ROM set (*-eeprom.5c), snooped from the ROM
//       download (def_*), as MAME loads it when there is no .nv file;
//    2. the saved NVRAM file, if there is one (nv_*, ioctl index 2);
//    3. the game, at run time.
//  All loads happen with the game held in reset. The array starts as $FF,
//  like a blank part, in case a set has no factory image.
//
//  Clock domain: clk_sys.
//============================================================================

`default_nettype none

module g42_eeprom (
	input  wire         clk,
	input  wire         rst_n,

	// ---- 68000 side ----------------------------------------------------------
	input  wire [10:0]  cpu_addr,     // byte address 0..2047 (68000 A11:A1)
	input  wire [7:0]   cpu_din,      // D[7:0]
	input  wire         cpu_wr,       // one clock per write cycle with /LDS
	input  wire         cpu_unlock,   // one clock per write to $E00060
	output logic [7:0]  cpu_dout,     // one clock after cpu_addr

	// ---- Factory image, from the ROM download ----------------------------------
	input  wire         def_we,
	input  wire [10:0]  def_addr,
	input  wire [7:0]   def_din,

	// ---- MiSTer NVRAM save / load (hps_io ioctl, index 2) ------------------------
	input  wire         nv_wr,        // load: one clock per byte
	input  wire [10:0]  nv_addr,
	input  wire [7:0]   nv_din,
	output logic [7:0]  nv_dout,      // save: one clock after nv_addr

	output logic        nv_dirty      // one clock per accepted 68000 write
);

	logic [7:0] mem [2048];

	initial begin
		for (int i = 0; i < 2048; i++) mem[i] = 8'hFF;
	end

	//------------------------------------------------------------------------
	// Unlock: set by a write to $E00060, cleared by the next accepted write
	//------------------------------------------------------------------------
	logic unlocked;
	wire  write_accepted = cpu_wr && unlocked;

	always_ff @(posedge clk) begin
		nv_dirty <= 1'b0;
		if (!rst_n) begin
			unlocked <= 1'b0;
		end else begin
			if (cpu_unlock)     unlocked <= 1'b1;
			if (write_accepted) begin
				unlocked <= 1'b0;
				nv_dirty <= 1'b1;
			end
		end
	end

	//------------------------------------------------------------------------
	// One write port for all three writers; they never overlap in time.
	//------------------------------------------------------------------------
	logic [10:0] wa;
	logic [7:0]  wd;
	logic        we;

	always_comb begin
		if (nv_wr) begin
			wa = nv_addr;  wd = nv_din;  we = 1'b1;
		end else if (def_we) begin
			wa = def_addr; wd = def_din; we = 1'b1;
		end else begin
			wa = cpu_addr; wd = cpu_din; we = write_accepted;
		end
	end

	always_ff @(posedge clk) begin
		if (we) mem[wa] <= wd;
		cpu_dout <= mem[cpu_addr];
	end

	always_ff @(posedge clk) nv_dout <= mem[nv_addr];

endmodule

`default_nettype wire
