#!/usr/bin/env bash
# ============================================================================
# run_xfft_xsim.sh — Compile + run xfft_2048 wrapper testbench in Vivado XSim
#
# Verifies the wrapper with the real LogiCORE FFT v9.1 (xfft_2048_ip).
# Cannot run in iverilog because the IP uses Xilinx primitives.
#
# Usage (on remote Vivado box):
#   cd ~/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA
#   bash scripts/50t/run_xfft_xsim.sh
#
# Output: /tmp/xfft_xsim.log  (look for "ALL TESTS PASSED")
# ============================================================================
set -e

PROJ_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
IP_NETLIST="$PROJ_ROOT/ip/xfft_2048_ip/xfft_2048_ip_sim_netlist.v"
WRAPPER="$PROJ_ROOT/xfft_2048.v"
TB="$PROJ_ROOT/tb/tb_xfft_2048_xsim.v"

WORK_DIR="$PROJ_ROOT/build_xsim_xfft"
mkdir -p "$WORK_DIR"
cd "$WORK_DIR"

echo "===== Compiling Verilog sources ====="
# Wrapper + testbench with the IP-on define. -i adds the FPGA root so
# `include "radar_params.vh"` resolves from inside tb/.
xvlog -d FFT_USE_XILINX_IP -i "$PROJ_ROOT" "$WRAPPER" "$TB"
# IP simulation netlist — references unisim primitives
xvlog "$IP_NETLIST"
# fft_engine etc. NOT needed because FFT_USE_XILINX_IP routes around it,
# but the wrapper still must compile cleanly under both branches; if xvlog
# complains about an unresolved fft_engine reference (it shouldn't because
# the `else` branch is hidden by the define), include it here:
# xvlog "$PROJ_ROOT/fft_engine.v"

echo "===== Elaborating ====="
# `glbl` is a Vivado-supplied module that Xilinx primitives (FDRE etc.)
# reference for the global GSR/GTS signals. Elaborating it as a second top
# satisfies the unresolved-reference error xelab raises for the IP netlist.
xelab -L unisims_ver -L secureip --debug typical \
      tb_xfft_2048_xsim glbl -snapshot tb_xfft_2048_snap

echo "===== Running simulation ====="
xsim tb_xfft_2048_snap --runall --log /tmp/xfft_xsim.log

echo "===== Done. Tail of log: ====="
tail -40 /tmp/xfft_xsim.log
