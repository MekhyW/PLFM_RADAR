################################################################################
# gen_xfft_2048_ip.tcl — Generate Xilinx LogiCORE FFT (xfft_v9_1) for AERIS-10
#
# Produces ip/xfft_2048/xfft_2048.xci configured for the matched-filter chain:
#   - Transform Length: 2048
#   - Architecture:     Pipelined Streaming I/O (Radix-2, 11 stages)
#   - Data Format:      Fixed Point
#   - Scaling:          Scaled (fixed schedule via cfg_tdata SCALE_SCH bits)
#                       Schedule [1,1,1,1,1,1,1,1,1,1,1] = /N (unitary FFT).
#                       AUDIT-C10/C-8 resolution: BFP previously hid a per-frame
#                       block exponent the bridge dropped, making sim/silicon
#                       absolute magnitudes incomparable. Scaled mode locks a
#                       deterministic /N scaling matched in fft_engine.v fallback.
#   - Rounding:         Convergent (round-to-even)
#   - Input Width:      32-bit per real/imag (PR-O.7 widening — chain feeds
#                       Q30 conjugate-mult product into IFFT without
#                       Q30→Q15 truncation; FWD passes sign-extend their
#                       16-bit ADC/ref samples to 32-bit. AXIS data tdata
#                       is 64-bit packed {Q[31:0], I[31:0]}.)
#   - Phase Width:      16-bit
#   - Output Ordering:  Natural Order
#   - Throttle Scheme:  Non Real Time (allows downstream backpressure)
#   - Memory:           Block RAM for data, reorder, phase factors
#
# Usage (run on remote Vivado box):
#   cd ~/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA
#   vivado -mode batch -source scripts/50t/gen_xfft_2048_ip.tcl
#
# Output: ip/xfft_2048_ip/xfft_2048_ip.xci  (committed; build_50t.tcl reads this)
# Note: IP module is named xfft_2048_ip to avoid collision with the wrapper
# module xfft_2048 in xfft_2048.v.
################################################################################

set script_dir   [file dirname [file normalize [info script]]]
set project_root [file normalize [file join $script_dir "../.."]]
set ip_dir       [file join $project_root "ip"]
set fpga_part    "xc7a50tftg256-2"

file mkdir $ip_dir

# Spin up a throwaway in-memory project just for IP generation.
create_project -in_memory -part $fpga_part
set_property ip_repo_paths $ip_dir [current_project]

# Create the IP. Any prior version is overwritten via -force.
create_ip -name xfft -vendor xilinx.com -library ip \
    -version 9.1 -module_name xfft_2048_ip -dir $ip_dir -force

set ip [get_ips xfft_2048_ip]

set_property -dict [list \
    CONFIG.transform_length          {2048}                       \
    CONFIG.implementation_options    {pipelined_streaming_io}     \
    CONFIG.channels                  {1}                          \
    CONFIG.data_format               {fixed_point}                \
    CONFIG.scaling_options           {scaled}                     \
    CONFIG.rounding_modes            {convergent_rounding}        \
    CONFIG.input_width               {32}                         \
    CONFIG.phase_factor_width        {16}                         \
    CONFIG.output_ordering           {natural_order}              \
    CONFIG.cyclic_prefix_insertion   {false}                      \
    CONFIG.throttle_scheme           {nonrealtime}                \
    CONFIG.target_clock_frequency    {100}                        \
    CONFIG.target_data_throughput    {50}                         \
    CONFIG.complex_mult_type         {use_mults_resources}        \
    CONFIG.butterfly_type            {use_xtremedsp_slices}       \
    CONFIG.memory_options_data       {block_ram}                  \
    CONFIG.memory_options_reorder    {block_ram}                  \
    CONFIG.memory_options_phase_factors {block_ram}               \
    CONFIG.memory_options_hybrid     {false}                      \
] $ip

# Generate synthesis + simulation targets so XSim and Vivado synth both work.
generate_target {synthesis simulation instantiation_template} $ip
synth_ip $ip

puts "================================================================"
puts "  xfft_2048_ip IP generation complete"
puts "  XCI:  $ip_dir/xfft_2048_ip/xfft_2048_ip.xci"
puts "  DCP:  [get_property IP_OUTPUT_DIR $ip]/xfft_2048_ip.dcp"
puts "================================================================"

close_project
exit 0
