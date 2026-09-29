# ===========================================================================
#  Atari G42 for MiSTer
#  Arcade-Atari-G42.sdc -- timing constraints specific to this core
#
#  sys/sys_top.sdc supplies the framework constraints (CLK_50M, HPS, video
#  output, PLL clock groups). This file adds the SDRAM interface, multicycle
#  paths for clock-enabled logic, and false paths for static configuration.
#
#  After a compile, check hold slack on every SDRAM read-return path, not
#  just the summary: a small negative hold there passes most reads and
#  corrupts a few (report commands at the end of this file).
# ===========================================================================

# ---------------------------------------------------------------------------
#  Derived clocks
# ---------------------------------------------------------------------------
#  clk_sys =  57.272724 MHz = 4 x the 14.318181 MHz master oscillator
#  clk_ram = 114.545448 MHz = 2 x clk_sys
#
#  The exact 2:1 ratio from one PLL makes the SDRAM crossing synchronous.
#  Moving clk_ram to an unrelated frequency needs real CDC synchronisers.
#
#  Do not call derive_pll_clocks or derive_clock_uncertainty here:
#  sys/sys_top.sdc already does both. A second call creates duplicate clock
#  objects and the constraints below apply to only one copy. sys_top.sdc also
#  puts all outputs of this PLL in one clock group, so clk_sys and clk_ram are
#  analysed against each other, as they must be.
# ---------------------------------------------------------------------------

set clk_sys {*|pll|pll_inst|altera_pll_i|*[0].*|divclk}
set clk_ram {*|pll|pll_inst|altera_pll_i|*[1].*|divclk}

# ---------------------------------------------------------------------------
#  SDRAM clock
# ---------------------------------------------------------------------------
#  SDRAM_CLK comes from PLL output 2, phase-shifted -90 degrees. The delay
#  figures below are the standard MiSTer values for a -7 grade 32 MB module.
#
#  -source names an internal PLL node, so get_pins needs -compatibility_mode
#  (as in sys_top.sdc). Without it the collection is empty, the clock is not
#  created, and every SDRAM delay constraint that references it is dropped.
#  The guard reports that case in the compilation messages.
# ---------------------------------------------------------------------------
if {[llength [get_clocks -nowarn SDRAM_CLK_pin]] == 0} {
	create_generated_clock -name SDRAM_CLK_pin \
		-source [get_pins -compatibility_mode {*|pll|pll_inst|altera_pll_i|*[2].*|divclk}] \
		[get_ports SDRAM_CLK]

	if {[llength [get_clocks -nowarn SDRAM_CLK_pin]] == 0} {
		puts "ERROR (Arcade-Atari-G42.sdc): SDRAM_CLK_pin was NOT created --\
the -source pattern matched no pin. Every SDRAM I/O delay below this point\
will silently fail too. Check the pattern against the actual PLL instance\
name in the CLOCKS section of a timing report."
	}
}

# ---------------------------------------------------------------------------
#  SDRAM read data
# ---------------------------------------------------------------------------
#  Brace every get_ports/get_pins pattern that contains '[', even a single
#  port: an unbraced SDRAM_DQ[*] is a Tcl command substitution, and the error
#  aborts every constraint after it.
#
#  Read-return margins with SDRAM_CLK at -90 degrees (2.18 ns late at
#  114.545 MHz) and these delays, per capture point (sdram.sv rd_phase and
#  rd_half):
#
#      t8   rising    setup  +0.15 ns   hold  +5.38 ns   tight
#      t8.5 falling   setup  +4.51 ns   hold  +1.02 ns   balanced
#      t9   rising    setup  +8.88 ns   hold  -3.35 ns   fails hold
#      t10  rising    setup +17.61 ns   hold -12.08 ns   fails hold
#
#  On paper the falling edge before t9 is the balanced point. Fitter
#  placement moves the real window between compiles, so by default the core's
#  boot-time self-test measures all eight settings and uses the best (OSD:
#  Debug -> SDRAM capture).
# ---------------------------------------------------------------------------
set_input_delay  -clock SDRAM_CLK_pin -max 6.4 [get_ports {SDRAM_DQ[*]}]
set_input_delay  -clock SDRAM_CLK_pin -min 3.2 [get_ports {SDRAM_DQ[*]}]

# ---------------------------------------------------------------------------
#  SDRAM outputs
# ---------------------------------------------------------------------------
#  Applied per port group. If get_ports is given a list in which any name
#  matches nothing (for example a pin Quartus removed because the RTL drives
#  it with a constant), the whole call fails and every port in the list loses
#  its constraint. Per group, a missing port costs only that group, and it is
#  reported.
# ---------------------------------------------------------------------------
foreach sdram_grp {
	{SDRAM_A[*]}
	{SDRAM_BA[*]}
	{SDRAM_DQ[*]}
	{SDRAM_DQML}
	{SDRAM_DQMH}
	{SDRAM_nCS}
	{SDRAM_nRAS}
	{SDRAM_nCAS}
	{SDRAM_nWE}
	{SDRAM_CKE}
} {
	set grp_ports [get_ports -nowarn $sdram_grp]
	if {[llength $grp_ports] == 0} {
		puts "WARNING (Arcade-Atari-G42.sdc): no port matches $sdram_grp --\
skipping its output delay. If this is nCS or CKE, the signal has probably\
been optimised away as a constant in sdram.sv."
	} else {
		set_output_delay -clock SDRAM_CLK_pin -max  1.5 $grp_ports
		set_output_delay -clock SDRAM_CLK_pin -min -0.8 $grp_ports
	}
}

# ---------------------------------------------------------------------------
#  Multicycle paths
# ---------------------------------------------------------------------------
#  Everything runs on clk_sys with clock enables, so logic that advances only
#  on an enable has several clocks to settle. Claim a multicycle only where
#  the enable rate is guaranteed: a wrong one passes timing analysis and
#  fails on hardware.
# ---------------------------------------------------------------------------

#  68000: ce_cpu_p1/p2 are 2 clk_sys cycles apart at 14.318 MHz (the OSD's
#  7.159 MHz setting only relaxes this). Same claim as the Atari G1 core.
set_multicycle_path -from [get_registers {*u_board|u_cpu|*}] -to [get_registers {*u_board|u_cpu|*}] -setup -end 2
set_multicycle_path -from [get_registers {*u_board|u_cpu|*}] -to [get_registers {*u_board|u_cpu|*}] -hold  -end 1

#  JSA III 6502 at 1.79 MHz: 32 clk_sys cycles per enable. Only 8 are
#  claimed, leaving margin if the enable generator changes.
set_multicycle_path -from [get_registers {*u_jsa3|u_cpu|*}] -to [get_registers {*u_jsa3|u_cpu|*}] -setup -end 8
set_multicycle_path -from [get_registers {*u_jsa3|u_cpu|*}] -to [get_registers {*u_jsa3|u_cpu|*}] -hold  -end 7

#  No multicycle on the RLE scaler: unlike G1's, its multiplier and divider
#  are sequenced one step per clock.

# ---------------------------------------------------------------------------
#  False paths
# ---------------------------------------------------------------------------
#  SDRAM capture setting: rd_phase/rd_half come from clk_sys (the self-test
#  sweep or the OSD) and change only between accesses. sdram.sv registers
#  them on clk_ram as rd_phase_q/rd_half_q.
set_false_path -to [get_registers {*u_sdram|rd_phase_q*}]
set_false_path -to [get_registers {*u_sdram|rd_half_q*}]
#  MRA configuration: latched once during the ROM download, static after.
#  It fans out widely (SLOOP type, palette bases, object count).
set_false_path -from [get_registers {*u_loader|cfg_bytes*}]

#  OSD status bits change only when the user opens the menu.
set_false_path -from [get_registers {*hps_io*|status*}]

# ---------------------------------------------------------------------------
#  Checking the SDRAM paths
# ---------------------------------------------------------------------------
#  After a fit, run these in the TimeQuest console:
#
#     report_timing -hold -npaths 50 -detail full_path \
#         -to [get_ports {SDRAM_DQ[*]}]
#     report_timing -hold -npaths 50 -detail full_path \
#         -from [get_ports {SDRAM_DQ[*]}]
# ---------------------------------------------------------------------------
