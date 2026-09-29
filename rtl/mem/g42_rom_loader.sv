//============================================================================
//  Atari G42 for MiSTer
//  g42_rom_loader.sv -- MRA download to SDRAM, configuration capture and the
//                       on-chip copies of the EEPROM image and DSP program
//
//    ioctl_index 0 : the ROM image, written to SDRAM unchanged (the MRA does
//                    all the arranging; see the <romstruct> in each MRA)
//    ioctl_index 1 : the configuration bytes (g42_pkg::g42_cfg_t)
//
//  While the image goes by, two regions are also copied on chip:
//    SDR_EEP  2 KB  the factory EEPROM image -> g42_eeprom (eep_*), one byte
//                   at a time, so a game with no saved NVRAM starts from it as
//                   in MAME
//    SDR_DSP  8 KB  the ASIC65 TMS32010 program -> g42_asic65 (dsp_*), one
//                   big-endian word at a time; the DSP needs single-clock
//                   program reads, which SDRAM cannot give
//  Both stay in SDRAM too, where they are never read.
//
//  Clock domain: clk_sys (ioctl is synchronous to it through hps_io).
//============================================================================

`default_nettype none

module g42_rom_loader
	import g42_pkg::*;
(
	input  wire         clk,
	input  wire         rst_n,

	// ---- ioctl download bus (hps_io) --------------------------------------------
	input  wire         ioctl_download,
	input  wire [15:0]  ioctl_index,
	input  wire         ioctl_wr,
	input  wire [26:0]  ioctl_addr,
	input  wire [7:0]   ioctl_dout,
	output logic        ioctl_wait,

	// ---- SDRAM write port: held until sdr_ack (one clock) ------------------------
	output logic [24:0] sdr_addr,        // byte address, bit 0 = 0
	output logic [15:0] sdr_din,
	output logic        sdr_we,
	input  wire         sdr_ack,

	// ---- On-chip copies ----------------------------------------------------------
	output logic        eep_we,          // factory EEPROM byte
	output logic [10:0] eep_addr,
	output logic [7:0]  eep_din,
	output logic        dsp_we,          // ASIC65 program word
	output logic [11:0] dsp_addr,
	output logic [15:0] dsp_din,

	// ---- Status and configuration --------------------------------------------------
	output g42_cfg_t    cfg,
	output logic        cfg_valid,       // configuration bytes received
	output logic        rom_loading,     // ROM download in progress
	output logic        rom_loaded       // a ROM download has completed
);

	//------------------------------------------------------------------------
	// Download type. ioctl_index[5:0] is the MRA <rom index>; the upper bits
	// carry a file-extension index, so compare only the low six.
	//------------------------------------------------------------------------
	wire is_rom = ioctl_download && (ioctl_index[5:0] == 6'd0);
	wire is_cfg = ioctl_download && (ioctl_index[5:0] == 6'd1);

	assign rom_loading = is_rom;

	logic is_rom_d, is_cfg_d;
	always_ff @(posedge clk) begin
		is_rom_d <= is_rom;
		is_cfg_d <= is_cfg;
	end

	//------------------------------------------------------------------------
	// Configuration. Stored by ioctl_addr, so a short or padded blob cannot
	// shift the fields. cfg_valid and rom_loaded are not cleared by rst_n: the
	// framework pulses reset as a download ends, which would wipe them.
	//------------------------------------------------------------------------
	logic [7:0] cfg_bytes [CFG_BYTES];

	always_ff @(posedge clk) begin
		if (is_cfg && !is_cfg_d) begin
			cfg_valid <= 1'b0;
			for (int i = 0; i < CFG_BYTES; i++) cfg_bytes[i] <= 8'h00;
		end
		else if (is_cfg && ioctl_wr && (ioctl_addr < CFG_BYTES)) begin
			cfg_bytes[ioctl_addr[2:0]] <= ioctl_dout;
			cfg_valid <= 1'b1;
		end
	end

	always_comb begin
		cfg.game_id       = cfg_bytes[0];
		cfg.sloop         = cfg_bytes[1];
		cfg.flags         = cfg_bytes[2];
		cfg.mo_base       = cfg_bytes[3];
		cfg.pf_base       = cfg_bytes[4];
		cfg.mo_color_mask = cfg_bytes[5];
		cfg.obj_count_h   = cfg_bytes[6];
		cfg.obj_count_l   = cfg_bytes[7];
	end

	always_ff @(posedge clk) begin
		if (is_rom && !is_rom_d) rom_loaded <= 1'b0;
		if (is_rom_d && !is_rom) rom_loaded <= 1'b1;
	end

	//------------------------------------------------------------------------
	// Byte pairing: the even byte is D15:8 (68000 order). ioctl is held off
	// while a word waits for the arbiter.
	//------------------------------------------------------------------------
	logic [7:0] byte_hi;
	logic       wr_pending;

	assign ioctl_wait = wr_pending;

	wire [24:0] a       = ioctl_addr[24:0];
	wire [24:0] dsp_off = a - SDR_DSP;   // SDR_DSP is not 8 KB aligned

	always_ff @(posedge clk) begin
		eep_we <= 1'b0;
		dsp_we <= 1'b0;

		if (!rst_n) begin
			byte_hi    <= 8'h00;
			wr_pending <= 1'b0;
			sdr_we     <= 1'b0;
			sdr_addr   <= '0;
			sdr_din    <= '0;
		end
		else begin
			if (wr_pending && sdr_ack) begin
				wr_pending <= 1'b0;
				sdr_we     <= 1'b0;
			end

			if (is_rom && ioctl_wr) begin
				if (!a[0]) begin
					byte_hi <= ioctl_dout;
				end else begin
					sdr_addr   <= {a[24:1], 1'b0};
					sdr_din    <= {byte_hi, ioctl_dout};
					sdr_we     <= 1'b1;
					wr_pending <= 1'b1;

					if (a >= SDR_DSP && a < SDR_DSP_END) begin
						dsp_we   <= 1'b1;
						dsp_addr <= dsp_off[12:1];
						dsp_din  <= {byte_hi, ioctl_dout};
					end
				end

				if (a >= SDR_EEP && a < SDR_EEP_END) begin
					eep_we   <= 1'b1;
					eep_addr <= a[10:0];
					eep_din  <= ioctl_dout;
				end
			end
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	int stall;
	always_ff @(posedge clk) begin
		if (wr_pending && !sdr_ack) begin
			stall <= stall + 1;
			if (stall > 10000) $fatal(1, "g42_rom_loader: SDRAM write not acknowledged");
		end else begin
			stall <= 0;
		end
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
