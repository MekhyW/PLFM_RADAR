#!/usr/bin/env bash
# ============================================================================
# run_mf_chain_xsim.sh — Run tb_matched_filter_processing_chain in Vivado XSim
#
# Drives the full MF chain (matched_filter_processing_chain →
# fft_engine_axi_bridge → xfft_2048 → xfft_2048_ip = real LogiCORE FFT v9.1)
# under XSim with `FFT_USE_XILINX_IP` defined. The hand-written RTL above
# the IP (chain FSM, BRAMs, conj-mult, sat-truncate boundaries) is the same
# code that runs in iverilog regression — this run validates it works
# correctly against the actual IP timing and scaling, not just the
# fft_engine.v fallback.
#
# Usage (on remote Vivado box):
#   cd ~/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA
#   bash scripts/50t/run_mf_chain_xsim.sh
#
# Output: /tmp/mf_chain_xsim.log
# ============================================================================
set -e

PROJ_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
IP_NETLIST="$PROJ_ROOT/ip/xfft_2048_ip/xfft_2048_ip_sim_netlist.v"

WORK_DIR="$PROJ_ROOT/build_xsim_mf_chain"
mkdir -p "$WORK_DIR"
cd "$WORK_DIR"

# Symlink tb/ into the work dir so $readmemh("tb/mf_golden_*.hex") resolves
# from xsim's CWD (build_xsim_mf_chain), not the project root.
ln -sfn "$PROJ_ROOT/tb" "$WORK_DIR/tb"

echo "===== Compiling Verilog sources ====="
# RTL chain (MF chain + bridge + wrapper). FFT_USE_XILINX_IP routes the
# wrapper to xfft_2048_ip; the iverilog `else` branch (fft_engine fallback)
# is hidden by the preprocessor so fft_engine.v is not needed here.
xvlog -d FFT_USE_XILINX_IP -i "$PROJ_ROOT" \
    "$PROJ_ROOT/tb/tb_matched_filter_processing_chain.v" \
    "$PROJ_ROOT/matched_filter_processing_chain.v" \
    "$PROJ_ROOT/fft_engine_axi_bridge.v" \
    "$PROJ_ROOT/xfft_2048.v" \
    "$PROJ_ROOT/chirp_reference_rom.v" \
    "$PROJ_ROOT/frequency_matched_filter.v"
# IP simulation netlist — references unisim primitives
xvlog "$IP_NETLIST"

echo "===== Elaborating ====="
xelab -L unisims_ver -L secureip --debug typical \
      tb_matched_filter_processing_chain glbl -snapshot tb_mf_chain_snap

echo "===== Running simulation ====="
xsim tb_mf_chain_snap --runall --log /tmp/mf_chain_xsim.log

echo "===== Done. Tail of log: ====="
tail -50 /tmp/mf_chain_xsim.log
