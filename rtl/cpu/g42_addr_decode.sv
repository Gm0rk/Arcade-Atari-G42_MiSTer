//============================================================================
//  Atari G42 for MiSTer
//  g42_addr_decode.sv -- 68000 address decode
//
//  Combinational decode of the 68000 bus into one-hot device selects and
//  local addresses. This is the only module that knows the memory map, which
//  is MAME atarig42.cpp main_map, address for address (no mirrors: the board's
//  decode PALs are read-protected and MAME is the only reference).
//
//    $000000-$07FFFF  R   program ROM; $078000-$07FFFF is the SLOOP window
//                     W   (no memory: the SLOOP sees the cycle)
//    $080000-$080001  R   ROM word past the program: $4E75 (RTS) on Guardians,
//                         where the game calls it, $0000 otherwise (MAME)
//    $E00000-$E00001  R   IN0
//    $E00002-$E00003  R   IN1
//    $E00010-$E00011  R   IN2
//    $E00012-$E00013  R   JSA III port (coins, self test, latch flags)
//    $E00020-$E0002F  RW  ADC0809, D15:8 (Road Riot); read and write both start
//                         a conversion of channel (word offset & 7)
//    $E00031          R   sound response from the JSA III (byte, D7:0)
//    $E00041          W   sound command to the JSA III (byte, D7:0)
//    $E00050-$E00051  W   I/O latch: ASIC65 reset, MO control, sound reset
//    $E00060-$E00061  W   EEPROM unlock
//    $E03000-$E03001  W   VBLANK interrupt acknowledge
//    $E03800-$E03801  W   watchdog
//    $E80000-$E80FFF  RW  4 KB RAM
//    $F40000-$F40001  R   ASIC65 status
//    $F60000-$F60001  R   ASIC65 data
//    $F80000-$F80003  W   ASIC65 parameter ($F80000) and command ($F80002)
//    $FA0000-$FA0FFF  RW  EEPROM, 2 KB, low byte only (umask16 $00FF)
//    $FC0000-$FC0FFF  RW  palette RAM, 2048 entries
//    $FF0000-$FFFFFF  RW  64 KB work RAM (object RAM, tile maps, MO command)
//  Everything else is unmapped: reads return $0000 and writes are ignored,
//  both with an immediate DTACK, as in MAME.
//============================================================================

`default_nettype none

module g42_addr_decode (
	// ---- 68000 bus ---------------------------------------------------------
	input  wire [23:1]  addr,
	input  wire         as_n,            // address strobe (gated off during IACK)
	input  wire         rw_n,            // 1 = read, 0 = write
	input  wire         uds_n,           // upper data strobe (D15:8)
	input  wire         lds_n,           // lower data strobe (D7:0)

	// ---- Selects, qualified by a valid bus cycle ----------------------------
	output logic        sel_rom,         // program ROM read
	output logic        sel_rom_wr,      // write to program ROM space (SLOOP only)
	output logic        sel_rom_x,       // $080000 word
	output logic        sel_in0,
	output logic        sel_in1,
	output logic        sel_in2,
	output logic        sel_jsaiii,
	output logic        sel_adc,
	output logic        sel_snd_resp,
	output logic        sel_snd_cmd,
	output logic        sel_io_latch,
	output logic        sel_eeprom_unlock,
	output logic        sel_irq_ack,
	output logic        sel_watchdog,
	output logic        sel_ram2,        // $E80000 4 KB RAM
	output logic        sel_asic_stat,
	output logic        sel_asic_read,
	output logic        sel_asic_write,
	output logic        sel_eeprom,
	output logic        sel_palette,
	output logic        sel_ram,         // $FF0000 work RAM
	output logic        sel_unmapped,

	// ---- Local addresses ----------------------------------------------------
	output logic [14:0] ram_addr,        // work RAM word offset
	output logic [10:0] ram2_addr,       // 4 KB RAM word offset
	output logic [10:0] pal_addr,        // palette entry
	output logic [10:0] eeprom_addr,     // EEPROM byte
	output logic [2:0]  adc_chan,        // ADC channel = word offset from $E00020
	output logic        asic_cmd         // ASIC65 write: 0 parameter, 1 command
);

	// Valid cycle: AS low and at least one data strobe. A write's strobes come
	// a clock after AS; nothing is selected until they do.
	wire cyc = !as_n && (!uds_n || !lds_n);

	wire [23:0] ba = {addr, 1'b0};       // byte address, for readability

	wire in_rom   = (ba[23:19] == 5'b00000);                   // $000000-$07FFFF
	wire in_e00   = (ba[23:16] == 8'hE0) && (ba[15:12] == 4'h0); // $E00000-$E00FFF
	wire in_e03   = (ba[23:16] == 8'hE0) && (ba[15:12] == 4'h3); // $E03000-$E03FFF

	always_comb begin
		sel_rom           = 1'b0;
		sel_rom_wr        = 1'b0;
		sel_rom_x         = 1'b0;
		sel_in0           = 1'b0;
		sel_in1           = 1'b0;
		sel_in2           = 1'b0;
		sel_jsaiii        = 1'b0;
		sel_adc           = 1'b0;
		sel_snd_resp      = 1'b0;
		sel_snd_cmd       = 1'b0;
		sel_io_latch      = 1'b0;
		sel_eeprom_unlock = 1'b0;
		sel_irq_ack       = 1'b0;
		sel_watchdog      = 1'b0;
		sel_ram2          = 1'b0;
		sel_asic_stat     = 1'b0;
		sel_asic_read     = 1'b0;
		sel_asic_write    = 1'b0;
		sel_eeprom        = 1'b0;
		sel_palette       = 1'b0;
		sel_ram           = 1'b0;
		sel_unmapped      = 1'b0;

		if (cyc) begin
			if (in_rom) begin
				sel_rom    =  rw_n;
				sel_rom_wr = !rw_n;
			end
			else if (ba[23:1] == 23'h040000) begin      // $080000
				sel_rom_x    =  rw_n;
				sel_unmapped = !rw_n;
			end
			else if (in_e00) begin
				case (ba[11:4])
				8'h00: begin                              // $E00000-$E0000F
					if      (ba[3:1] == 3'd0) sel_in0 = rw_n;
					else if (ba[3:1] == 3'd1) sel_in1 = rw_n;
					else                      sel_unmapped = 1'b1;
				end
				8'h01: begin                              // $E00010-$E0001F
					if      (ba[3:1] == 3'd0) sel_in2    = rw_n;
					else if (ba[3:1] == 3'd1) sel_jsaiii = rw_n;
					else                      sel_unmapped = 1'b1;
				end
				8'h02: sel_adc = 1'b1;                    // $E00020-$E0002F
				8'h03: begin                              // $E00030
					if (ba[3:1] == 3'd0) sel_snd_resp = rw_n;
					else                 sel_unmapped = 1'b1;
				end
				8'h04: begin                              // $E00040
					if (ba[3:1] == 3'd0) sel_snd_cmd = !rw_n;
					else                 sel_unmapped = 1'b1;
				end
				8'h05: begin                              // $E00050
					if (ba[3:1] == 3'd0) sel_io_latch = !rw_n;
					else                 sel_unmapped = 1'b1;
				end
				8'h06: begin                              // $E00060
					if (ba[3:1] == 3'd0) sel_eeprom_unlock = !rw_n;
					else                 sel_unmapped = 1'b1;
				end
				default: sel_unmapped = 1'b1;
				endcase
			end
			else if (in_e03) begin
				if      (ba[11:1] == 11'h000) sel_irq_ack  = !rw_n;      // $E03000
				else if (ba[11:1] == 11'h400) sel_watchdog = !rw_n;      // $E03800
				else                          sel_unmapped = 1'b1;
			end
			else if (ba[23:12] == 12'hE80) sel_ram2      = 1'b1;          // $E80000
			else if (ba[23:1]  == 23'h7A0000) sel_asic_stat = rw_n;      // $F40000
			else if (ba[23:1]  == 23'h7B0000) sel_asic_read = rw_n;      // $F60000
			else if (ba[23:2]  == 22'h3E0000) sel_asic_write = !rw_n;    // $F80000-$F80003
			else if (ba[23:12] == 12'hFA0) sel_eeprom    = 1'b1;          // $FA0000
			else if (ba[23:12] == 12'hFC0) sel_palette   = 1'b1;          // $FC0000
			else if (ba[23:16] == 8'hFF)   sel_ram       = 1'b1;          // $FF0000
			else                           sel_unmapped  = 1'b1;
		end
	end

	assign ram_addr    = addr[15:1];
	assign ram2_addr   = addr[11:1];
	assign pal_addr    = addr[11:1];
	assign eeprom_addr = addr[11:1];
	assign adc_chan    = addr[3:1];
	assign asic_cmd    = addr[1];

endmodule

`default_nettype wire
