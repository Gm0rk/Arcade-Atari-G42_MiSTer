//============================================================================
//  Atari G42 for MiSTer
//  g42_sdram_arb.sv -- SDRAM port arbiter
//
//  Multiplexes the ROM loader and the core's readers onto one single-port
//  SDRAM controller (req/ready handshake), so the controller can be swapped
//  without touching the requesters.
//
//  Fixed priority, highest first. Not round-robin: the requesters' latency
//  tolerances differ too much for fairness to make sense.
//    0  Loader      only active before the core runs; the top slot means the
//                   download cannot be starved.
//    1  CPU         a waiting 68000 stalls on DTACK, which stalls the game.
//    2  OKI sample  at most one access per ~200 clocks, but hard real-time:
//                   JT6295 samples its ROM data a fixed ~384 clk_sys after
//                   changing the address and does not wait.
//    3  Tile fetch  hard real-time: both line fetchers must finish within the
//                   scanline, or the line is skipped.
//    4  RLE fetch   by far the largest consumer, but soft real-time: with a
//                   double-buffered framebuffer the object renderer has a full
//                   frame and absorbs long stalls.
//
//  Bandwidth at 114.5 MHz and ~4 clocks per access: ~28 M accesses/s. CPU
//  ~1.8 M/s worst case (14.3 MHz / 8 clocks per bus cycle), tiles ~1.8 M/s
//  (two reads per tile row, 0.9 M rows/s), OKI ~0.03 M/s, RLE the rest.
//
//  Clock domain: clk_sys. The controller runs at clk_ram = 2 x clk_sys, an
//  exact multiple, so the crossing is synchronous.
//============================================================================

`default_nettype none

module g42_sdram_arb (
	input  wire         clk,
	input  wire         rst_n,

	// Requester ports: req and addr are held until ack. ack lasts one clock
	// and dout is valid in that clock.

	// 0: ROM loader (write)
	input  wire [24:0]  ld_addr,
	input  wire [15:0]  ld_din,
	input  wire         ld_we,
	output logic        ld_ack,

	// 1: CPU program fetch (read), shared by the 68000 and the 6502
	input  wire [24:0]  cpu_addr,
	input  wire         cpu_req,
	output logic [15:0] cpu_dout,
	output logic        cpu_ack,

	// 2: playfield / alpha tile fetch (read). Only this channel has a grant:
	// tile_gnt is high in the clock the request is taken (tile_addr latched).
	// The requester (g42_video) never re-presents a granted request, which lets
	// this channel be re-granted in its own ack clock.
	input  wire [24:0]  tile_addr,
	input  wire         tile_req,
	output logic        tile_gnt,
	output logic [15:0] tile_dout,
	output logic        tile_ack,

	// 3: RLE object fetch (read)
	input  wire [24:0]  rle_addr,
	input  wire         rle_req,
	output logic [15:0] rle_dout,
	output logic        rle_ack,

	// 4: OKI6295 ADPCM sample fetch (read)
	input  wire [24:0]  oki_addr,
	input  wire         oki_req,
	output logic [15:0] oki_dout,
	output logic        oki_ack,

	//------------------------------------------------------------------------
	// Single-port SDRAM controller
	//------------------------------------------------------------------------
	output logic [24:0] sdr_addr,

	// Debug: address of the last completed CPU-channel transaction, recorded
	// here where address and completion pair up unambiguously (sampling
	// sdr_addr outside can catch a later grant).
	output logic [24:0] dbg_cpu_done_addr,

	// Self-test: hold the tile, RLE and OKI channels idle (exclusive reads).
	input  wire         solo_cpu,
	output logic [15:0] sdr_din,
	output logic        sdr_we,
	output logic        sdr_rd,
	input  wire  [15:0] sdr_dout,
	input  wire         sdr_ready    // one clock: the request has completed
);

	typedef enum logic [2:0] {
		CH_NONE, CH_LOAD, CH_CPU, CH_TILE, CH_RLE, CH_OKI
	} chan_t;

	chan_t active;
	logic  busy;

	//------------------------------------------------------------------------
	// Ack masking. Acks are registered, so in the ack clock the requester's
	// req is still high while busy has already cleared. Without the mask the
	// grant logic starts the same transfer again, and the duplicate returns
	// the wrong word. A requester that wants another transfer asks again on
	// the next clock.
	//------------------------------------------------------------------------
	wire ld_we_g    = ld_we    && !ld_ack;
	wire cpu_req_g  = cpu_req  && !cpu_ack;
	// No mask on the tile channel: g42_video never re-presents a granted
	// request (it marks it taken on tile_gnt), so a request in the ack clock
	// is always the other fetcher's new one. Masking it would hand that slot
	// to the RLE engine, which then holds the bus for a whole access and makes
	// the line fetch overrun.
	wire tile_req_g = tile_req && !solo_cpu;
	wire rle_req_g  = rle_req  && !rle_ack  && !solo_cpu;
	wire oki_req_g  = oki_req  && !oki_ack  && !solo_cpu;

	// Exactly the condition under which the grant logic takes the tile
	// request. Combinational, so the requester records the owner at the same
	// edge the address is latched.
	assign tile_gnt = rst_n && !busy && !ld_we_g && !cpu_req_g && !oki_req_g && tile_req_g;

	//------------------------------------------------------------------------
	// Grant: strict priority, evaluated only when idle
	//------------------------------------------------------------------------
	always_ff @(posedge clk) begin
		if (!rst_n) begin
			active   <= CH_NONE;
			busy     <= 1'b0;
			sdr_we   <= 1'b0;
			sdr_rd   <= 1'b0;
			sdr_addr <= '0;
			sdr_din  <= '0;
		end
		else begin
			// Acks are single-clock.
			ld_ack   <= 1'b0;
			cpu_ack  <= 1'b0;
			tile_ack <= 1'b0;
			rle_ack  <= 1'b0;
			oki_ack  <= 1'b0;

			if (!busy) begin
				sdr_we <= 1'b0;
				sdr_rd <= 1'b0;

				if (ld_we_g) begin
					active   <= CH_LOAD;
					sdr_addr <= ld_addr;
					sdr_din  <= ld_din;
					sdr_we   <= 1'b1;
					busy     <= 1'b1;
				end
				else if (cpu_req_g) begin
					active   <= CH_CPU;
					sdr_addr <= cpu_addr;
					sdr_rd   <= 1'b1;
					busy     <= 1'b1;
				end
				else if (oki_req_g) begin
					active   <= CH_OKI;
					sdr_addr <= oki_addr;
					sdr_rd   <= 1'b1;
					busy     <= 1'b1;
				end
				else if (tile_req_g) begin
					active   <= CH_TILE;
					sdr_addr <= tile_addr;
					sdr_rd   <= 1'b1;
					busy     <= 1'b1;
				end
				else if (rle_req_g) begin
					active   <= CH_RLE;
					sdr_addr <= rle_addr;
					sdr_rd   <= 1'b1;
					busy     <= 1'b1;
				end
			end
			else if (sdr_ready) begin
				// Completion: return the result to the requester, free the bus.
				sdr_we <= 1'b0;
				sdr_rd <= 1'b0;
				busy   <= 1'b0;
				active <= CH_NONE;

				case (active)
					CH_LOAD: ld_ack   <= 1'b1;
					CH_CPU:  begin
						cpu_dout          <= sdr_dout;
						cpu_ack           <= 1'b1;
						dbg_cpu_done_addr <= sdr_addr;
					end
					CH_TILE: begin tile_dout <= sdr_dout; tile_ack <= 1'b1; end
					CH_RLE:  begin rle_dout  <= sdr_dout; rle_ack  <= 1'b1; end
					CH_OKI:  begin oki_dout  <= sdr_dout; oki_ack  <= 1'b1; end
					default: ;
				endcase
			end
		end
	end

`ifdef SIMULATION
	// synthesis translate_off
	// A grant that never completes freezes the core with no other symptom.
	int watchdog;
	always_ff @(posedge clk) begin
		if (busy && !sdr_ready) begin
			watchdog <= watchdog + 1;
			if (watchdog > 1000)
				$fatal(1, "g42_sdram_arb: channel %0d granted but sdr_ready never came",
					   active);
		end else watchdog <= 0;
	end
	// synthesis translate_on
`endif

endmodule

`default_nettype wire
