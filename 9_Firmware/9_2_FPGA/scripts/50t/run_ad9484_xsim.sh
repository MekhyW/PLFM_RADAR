#!/usr/bin/env bash
# ============================================================================
# run_ad9484_xsim.sh — Compile + run tb_ad9484_xsim in Vivado XSim
#
# Verifies ad9484_interface_400m.v with REAL Xilinx UNISIM primitives
# (IBUFDS, IBUFGDS, BUFIO, BUFG, IDDR, MMCME2_ADV) — cannot run in iverilog.
#
# Closes PR-X.1 F-7.4: TB now waits on the MMCM lock indicator instead of
# guessing at a fixed 5-cycle delay (the MMCM SIM model takes ~4096 DCO
# cycles to lock).
#
# Usage (on remote Vivado box):
#   cd ~/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA
#   bash scripts/50t/run_ad9484_xsim.sh
#
# Output: /tmp/ad9484_xsim.log  (look for "ALL TESTS PASSED")
# ============================================================================
set -e

PROJ_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
DUT="$PROJ_ROOT/ad9484_interface_400m.v"
MMCM_WRAPPER="$PROJ_ROOT/adc_clk_mmcm.v"
TB="$PROJ_ROOT/tb/tb_ad9484_xsim.v"

WORK_DIR="$PROJ_ROOT/build_xsim_ad9484"
mkdir -p "$WORK_DIR"
cd "$WORK_DIR"

echo "===== Compiling Verilog sources ====="
xvlog -i "$PROJ_ROOT" "$DUT" "$MMCM_WRAPPER" "$TB"
# glbl.v supplies GSR/GTS for Xilinx primitives; xelab needs it as a second top.
xvlog "${XILINX_VIVADO}/data/verilog/src/glbl.v"

echo "===== Elaborating ====="
# `glbl` provides GSR/GTS for Xilinx primitives (IBUFDS, IDDR, MMCME2, etc.)
xelab -L unisims_ver -L secureip --debug typical \
      tb_ad9484_xsim glbl -snapshot tb_ad9484_xsim_snap

echo "===== Running simulation ====="
xsim tb_ad9484_xsim_snap --runall --log /tmp/ad9484_xsim.log

echo "===== Done. Tail of log: ====="
tail -60 /tmp/ad9484_xsim.log
