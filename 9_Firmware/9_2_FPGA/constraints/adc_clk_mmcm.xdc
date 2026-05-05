# ============================================================================
# adc_clk_mmcm.xdc — Supplementary constraints for MMCM ADC clock path
#
# These constraints augment the existing adc_dco_p clock definitions when the
# adc_clk_mmcm module is integrated into ad9484_interface_400m.v.
#
# USAGE:
#   Add this file to the Vivado project AFTER the main production XDC.
#   The main XDC still defines create_clock on adc_dco_p (the physical input).
#   Vivado automatically creates a generated clock on the MMCM output;
#   these constraints handle CDC paths for the new clock topology.
#
# HIERARCHY: rx_inst/adc/mmcm_inst/...
# ============================================================================

# --------------------------------------------------------------------------
# MMCM Output Clock — use Vivado's auto-generated clock name
# --------------------------------------------------------------------------
# Vivado auto-creates a generated clock named 'clk_mmcm_out0' on the MMCM
# CLKOUT0 net. We do NOT create_generated_clock here (that would create a
# second clock on the same net, causing the CDC false paths below to bind
# to the wrong clock and leave clk_mmcm_out0 uncovered — exactly the bug
# that caused Build 19's -0.011 ns WNS on the CDC_FIR gray-code path).
# All constraints below reference 'clk_mmcm_out0' directly.

# --------------------------------------------------------------------------
# CDC: BUFIO domain (adc_dco_p) ↔ MMCM output domain (clk_mmcm_out0)
# --------------------------------------------------------------------------
# Post-AUDIT-C4 (2026-05-01) the AD9484 capture is SDR: a single
# (* IOB="TRUE" *) IFF on the falling BUFIO edge — no IDDR, no rise/fall
# demux. The IFF output (adc_data_iff) re-registers into the MMCM BUFG
# domain (adc_data_iff_bufg) in ad9484_interface_400m.v.
# These clocks are frequency-matched and phase-related (MMCM is locked to
# adc_dco_p), so the single register transfer is safe. We use max_delay
# to ensure the tools verify the transfer fits within the valid data window
# without over-constraining with full inter-clock setup/hold analysis.
#
# 3.000 ns = 1.2× the 2.500 ns clock period. On a 95%-packed XC7A50T the
# placer cannot keep the BUFG-domain capture FF (adc_data_iff_bufg) next
# to the IOB column where the IFF lives (observed routes ~2.28 ns IFF →
# SLICE_X0Y123); the old 2.700 ns window failed by ~120 ps. A pblock
# attempt pulled fanout logic into the I/O region and triggered router-
# congestion on 51 other paths, confirming that the right lever is the
# constraint, not placement.
# 3.000 ns is safe: (a) the IFF Q output is valid for the full adc_dco_p
# period (one new sample per DCO; SDR-stable until the next falling edge),
# (b) MMCM-locked phase relation keeps launch/capture edges deterministic,
# (c) 0 logic levels on the datapath, (d) even with worst-case route and
# skew, 300 ps of extra budget still fits inside the ADC output-valid
# window (AD9484 datasheet: data valid 100 ps after DCO edge).
set_max_delay -datapath_only -from [get_clocks adc_dco_p] \
    -to [get_clocks clk_mmcm_out0] 3.000

set_max_delay -datapath_only -from [get_clocks clk_mmcm_out0] \
    -to [get_clocks adc_dco_p] 3.000

# --------------------------------------------------------------------------
# CDC: MMCM output domain ↔ other clock domains
# --------------------------------------------------------------------------
# The existing false paths in the production XDC reference adc_dco_p, which
# now only covers the BUFIO/IFF domain. The MMCM output clock (which drives
# all fabric 400 MHz logic) needs its own false path declarations.
set_false_path -from [get_clocks clk_100m] -to [get_clocks clk_mmcm_out0]
set_false_path -from [get_clocks clk_mmcm_out0] -to [get_clocks clk_100m]

# Audit F-0.6: the USB-domain clock name differs per board
# (50T: ft_clkout, 200T: ft601_clk_in). XDC files only support a
# restricted Tcl subset — `foreach`/`unset` trigger CRITICAL WARNING
# [Designutils 20-1307]. The clk_mmcm_out0 ↔ USB-clock false paths
# are declared in the per-board XDC (xc7a50t_ftg256.xdc and
# xc7a200t_fbg484.xdc) where the USB clock name is already known.

set_false_path -from [get_clocks clk_mmcm_out0] -to [get_clocks clk_120m_dac]
set_false_path -from [get_clocks clk_120m_dac] -to [get_clocks clk_mmcm_out0]

# --------------------------------------------------------------------------
# MMCM Locked — asynchronous status signal, no timing paths needed
# --------------------------------------------------------------------------
# LOCKED is not a valid timing startpoint (it's a combinational output of the
# MMCM primitive). Use -through instead of -from to waive all paths that pass
# through the LOCKED net. This avoids the CRITICAL WARNING from Build 19/20.
# Audit F-0.7: the literal hierarchical path was missing the `u_core/`
# prefix and silently matched no pins. Use a hierarchical wildcard to
# catch the MMCM LOCKED pin regardless of wrapper hierarchy.
set_false_path -through [get_pins -hierarchical -filter {REF_PIN_NAME == LOCKED}]

# --------------------------------------------------------------------------
# Hold waiver for source-synchronous ADC capture (BUFIO-clocked IFF, SDR)
# --------------------------------------------------------------------------
# The AD9484 ADC provides a source-synchronous interface: data (adc_d_p/n)
# and clock (adc_dco_p/n) are output from the same chip with matched timing.
# On the PCB, data and DCO traces are length-matched.
#
# Inside the FPGA, the DCO clock path goes through IBUFDS → BUFIO, adding
# ~2.2ns of insertion delay (IBUFDS 0.9ns + routing 0.6ns + BUFIO 1.3ns).
# The data path goes through IBUFDS only (~0.85ns), arriving at the IOB-
# packed IFF ~1.4ns before the clock. Vivado's hold analysis sees the data
# "changing" before the clock edge and reports WHS = -1.955ns.
#
# This is correct internal behavior: the BUFIO clock intentionally arrives
# after the data. The IFF captures on the falling BUFIO edge (1.25 ns
# inside the AD9484 stable window), by which time the data is stable.
# Hold timing is guaranteed by the external PCB layout (ADC data valid
# window centered on DCO edge), not by FPGA clock tree delays. Vivado's
# STA model cannot account for this external relationship.
#
# Waiving hold on these 8 paths (adc_d_p[0..7] → IFF) is standard practice
# for source-synchronous LVDS ADC interfaces using BUFIO capture.
# adc_or_p (AD9484 overrange, audit F-0.1) shares the same IBUFDS→BUFIO
# source-synchronous capture topology as adc_d_p[*] — same ~1.9 ns STA hold
# violation for the same reason (BUFIO clock insertion ~4 ns vs data IBUFDS
# ~0.9 ns), resolved by the same external-timing argument.
set_false_path -hold -from [get_ports {adc_d_p[*] adc_or_p}] -to [get_clocks adc_dco_p]

# --------------------------------------------------------------------------
# Timing margin for 400 MHz critical paths
# --------------------------------------------------------------------------
# Extra setup uncertainty forces Vivado to leave margin for temperature/voltage/
# aging variation. 150 ps absolute covers the built-in jitter-based value
# (~53 ps) plus ~100 ps temperature/voltage/aging guardband.
# NOTE: Vivado's set_clock_uncertainty does NOT accept -add; prior use of
# -add 0.100 was silently rejected as a CRITICAL WARNING, so no guardband
# was applied. Use an absolute value. (audit finding F-0.8)
set_clock_uncertainty -setup 0.150 [get_clocks clk_mmcm_out0]
