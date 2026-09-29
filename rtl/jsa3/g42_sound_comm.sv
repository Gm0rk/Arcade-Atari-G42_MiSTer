//============================================================================
//  Atari G42 for MiSTer
//  g42_sound_comm.sv -- 68000 <-> 6502 communication latches (JSA III)
//
//  Port of MAME's atari_sound_comm_device (atariscom.cpp): two single-byte
//  latches, each with a full flag that drives an interrupt line as a level.
//
//      68000 writes $E00041 -> main_to_sound latch, flag set, 6502 NMI asserted
//      6502  reads  $2802   -> byte taken, flag cleared
//      6502  writes $2A02   -> sound_to_main latch, flag set, 68000 IRQ5 asserted
//      68000 reads  $E00031 -> byte taken, flag cleared
//
//  The flags are output raw (1 = full); polarity is applied where they are
//  read: 68000 IN2 bits 5/4 (active low), 6502 RDIO bits 6 (active low) and
//  5 (active high).
//
//  sound_reset is a level (io_latch bit 4 low): both latches are held empty
//  and zero while it is high. MAME instead clears them once per io_latch
//  write with bit 4 low (m_jsa->reset()), so a command written while the
//  board is held in reset survives there and is lost here. All three games
//  flush the command latch ($280A) at 6502 start-up, so nothing depends on it.
//
//  main_dout is captured when the 68000 read starts: a response written by
//  the 6502 later in the same 68000 bus cycle then stays in the latch for the
//  next read instead of being returned with its flag already cleared (MAME's
//  read handler takes the byte and clears the flag atomically).
//
//  Clock domain: clk_sys
//============================================================================

`default_nettype none

module g42_sound_comm (
	input  wire        clk,                 // clk_sys
	input  wire        rst_n,               // global reset
	input  wire        sound_reset,         // level: board held in reset, latches kept empty

	//------------------------------------------------------------------------
	// 68000 side
	//------------------------------------------------------------------------
	input  wire [7:0]  main_din,            // command byte
	input  wire        main_wr,             // one clock per write of $E00041
	input  wire        main_rd,             // one clock at the start of a read of $E00031
	output logic [7:0] main_dout,           // response byte, held for the rest of the read
	output logic       main_irq,            // -> 68000 IRQ5 (level)

	//------------------------------------------------------------------------
	// 6502 side
	//------------------------------------------------------------------------
	input  wire [7:0]  snd_din,             // response byte
	input  wire        snd_wr,              // one clock per write of $2A02
	input  wire        snd_rd,              // one clock per read of $2802
	output logic [7:0] snd_dout,            // command byte
	output logic       snd_nmi,             // -> 6502 NMI (level; the 6502 takes the edge)

	//------------------------------------------------------------------------
	// Latch-full flags, raw (1 = full)
	//------------------------------------------------------------------------
	output logic       main_to_sound_ready, // command latch full
	output logic       sound_to_main_ready  // response latch full
);

	logic [7:0] m2s_data, s2m_data, s2m_held;

	//========================================================================
	//  Main -> sound
	//========================================================================
	always_ff @(posedge clk) begin
		if (!rst_n || sound_reset) begin
			m2s_data            <= 8'h00;
			main_to_sound_ready <= 1'b0;
		end
		else begin
			if (main_wr) begin
				m2s_data            <= main_din;
				main_to_sound_ready <= 1'b1;
			end
			// Write and read in the same clock: the write wins and the flag
			// stays set, as on a real latch.
			else if (snd_rd) begin
				main_to_sound_ready <= 1'b0;
			end
		end
	end

	assign snd_dout = m2s_data;
	assign snd_nmi  = main_to_sound_ready;

	//========================================================================
	//  Sound -> main
	//========================================================================
	always_ff @(posedge clk) begin
		if (!rst_n || sound_reset) begin
			s2m_data            <= 8'h00;
			s2m_held            <= 8'h00;
			sound_to_main_ready <= 1'b0;
		end
		else begin
			if (main_rd) s2m_held <= s2m_data;

			if (snd_wr) begin
				s2m_data            <= snd_din;
				sound_to_main_ready <= 1'b1;
			end
			else if (main_rd) begin
				sound_to_main_ready <= 1'b0;
			end
		end
	end

	// In the main_rd clock the live latch, afterwards the byte that read took.
	assign main_dout = main_rd ? s2m_data : s2m_held;
	assign main_irq  = sound_to_main_ready;

endmodule

`default_nettype wire
