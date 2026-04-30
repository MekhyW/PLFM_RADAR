`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// radar_transmitter — DAC-side wrapper around plfm_chirp_controller_v2.
//
// chirp-v2 PR-E reorganization:
//   chirp_scheduler (clk_100m, in receiver_final) is now the master timekeeper.
//   It emits {wave_sel[1:0], chirp_pulse, frame_pulse} on clk_100m. We bridge
//   them to clk_120m_dac here:
//      - wave_sel + chirp_pulse → cdc_async_fifo (Cummings style #2). Each
//        chirp_pulse pushes wave_sel into the FIFO; the dst-side dst_valid
//        pulse drives plfm_chirp_controller_v2.dst_chirp_valid.
//      - frame_pulse → toggle CDC → 1-cycle pulse on clk_120m_dac for
//        chirp_counter clear and the new_chirp_frame status output.
//
// Beam steering:
//   stm32_new_elevation / stm32_new_azimuth still run through edge detectors
//   on clk_100m and feed the controller's internal beam counters. These are
//   independent of the chirp FSM and unchanged from chirp-v1.
//
// stm32_new_chirp:
//   Removed from this module — the scheduler in receiver_final now owns chirp
//   timing. The top-level GPIO is still wired to receiver_final via
//   stm32_new_chirp_rx; the transmitter has no separate path.
//////////////////////////////////////////////////////////////////////////////////
module radar_transmitter(
    // System Clocks
    input wire clk_100m,           // System clock
    input wire clk_120m_dac,       // 120MHz DAC clock
    input wire reset_n,            // Reset synchronized to clk_120m_dac
    input wire reset_100m_n,       // Reset synchronized to clk_100m (for edge detectors/CDC)

    // DAC Interface
    output wire [7:0] dac_data,
    output wire dac_clk,
    output wire dac_sleep,
    output wire rx_mixer_en,
    output wire tx_mixer_en,

    // Scheduler outputs from receiver_final (clk_100m domain) — PR-E
    input wire [1:0]  sched_wave_sel,
    input wire        sched_chirp_pulse,
    input wire        sched_frame_pulse,

    // STM32 Control Interface (chirp moved to scheduler; beam-step still here)
    input wire stm32_new_elevation,
    input wire stm32_new_azimuth,
    input wire stm32_mixers_enable,

    output wire fpga_rf_switch,

    // ADAR1000 Control Interface
    output wire adar_tx_load_1,
    output wire adar_rx_load_1,
    output wire adar_tx_load_2,
    output wire adar_rx_load_2,
    output wire adar_tx_load_3,
    output wire adar_rx_load_3,
    output wire adar_tx_load_4,
    output wire adar_rx_load_4,
    output wire adar_tr_1,
    output wire adar_tr_2,
    output wire adar_tr_3,
    output wire adar_tr_4,

    // Level Shifter SPI Interface (STM32F7 to ADAR1000)
    input wire stm32_sclk_3v3,
    input wire stm32_mosi_3v3,
    output wire stm32_miso_3v3,
    input wire stm32_cs_adar1_3v3,
    input wire stm32_cs_adar2_3v3,
    input wire stm32_cs_adar3_3v3,
    input wire stm32_cs_adar4_3v3,

    output wire stm32_sclk_1v8,
    output wire stm32_mosi_1v8,
    input wire stm32_miso_1v8,
    output wire stm32_cs_adar1_1v8,
    output wire stm32_cs_adar2_1v8,
    output wire stm32_cs_adar3_1v8,
    output wire stm32_cs_adar4_1v8,

    // Beam Position Tracking
    output wire [5:0] current_elevation,
    output wire [5:0] current_azimuth,
    output wire [5:0] current_chirp,
    output wire new_chirp_frame
);

// ========== SPI LEVEL SHIFTER PASSTHROUGH ==========
// FPGA bridges 3.3V STM32 SPI bus (Bank 15) to 1.8V ADAR1000 SPI bus (Bank 34).
// The FPGA I/O banks handle the actual voltage translation; these assigns
// route the signals through the fabric.
assign stm32_sclk_1v8      = stm32_sclk_3v3;
assign stm32_mosi_1v8       = stm32_mosi_3v3;
assign stm32_miso_3v3       = stm32_miso_1v8;
assign stm32_cs_adar1_1v8   = stm32_cs_adar1_3v3;
assign stm32_cs_adar2_1v8   = stm32_cs_adar2_3v3;
assign stm32_cs_adar3_1v8   = stm32_cs_adar3_3v3;
assign stm32_cs_adar4_1v8   = stm32_cs_adar4_3v3;

// Beam-step edge detection (STM32 GPIO -> clk_100m pulses)
wire new_elevation_pulse;
wire new_azimuth_pulse;

// CDC: Synchronized versions of async STM32 GPIO inputs to clk_100m
wire stm32_new_elevation_sync;
wire stm32_new_azimuth_sync;

// CDC: stm32_mixers_enable into clk_120m_dac domain
wire mixers_enable_120m;

// PR-E: scheduler bridge outputs in clk_120m_dac domain
wire        dst_chirp_valid;
wire [1:0]  dst_wave_sel;
wire        sched_overrun_unused;
wire        frame_pulse_120m;

// Chirp Control Signals
wire [7:0] chirp_data;
wire chirp_valid;
wire chirp_sequence_done;

// ============================================================================
// PR-E: chirp_pulse + wave_sel CDC (clk_100m → clk_120m_dac)
//
// Each scheduler chirp_pulse on clk_100m pushes wave_sel into a Gray-coded
// async FIFO. The dst side auto-drains so dst_chirp_valid is a 1-cycle pulse
// on clk_120m_dac, and dst_wave_sel carries the matching waveform identity.
// ============================================================================
cdc_async_fifo #(
    .WIDTH(2),
    .DEPTH(4)
) cdc_chirp_fifo (
    .src_clk     (clk_100m),
    .dst_clk     (clk_120m_dac),
    .src_reset_n (reset_100m_n),
    .dst_reset_n (reset_n),
    .src_data    (sched_wave_sel),
    .src_valid   (sched_chirp_pulse),
    .dst_data    (dst_wave_sel),
    .dst_valid   (dst_chirp_valid),
    .overrun     (sched_overrun_unused)
);

// ============================================================================
// frame_pulse toggle CDC (clk_100m → clk_120m_dac)
// ============================================================================
reg frame_toggle_100m;
always @(posedge clk_100m or negedge reset_100m_n) begin
    if (!reset_100m_n)
        frame_toggle_100m <= 1'b0;
    else if (sched_frame_pulse)
        frame_toggle_100m <= ~frame_toggle_100m;
end

wire frame_toggle_120m;
cdc_single_bit #(.STAGES(3)) cdc_frame_toggle (
    .src_clk(clk_100m),
    .dst_clk(clk_120m_dac),
    .reset_n(reset_n),
    .src_signal(frame_toggle_100m),
    .dst_signal(frame_toggle_120m)
);

reg frame_toggle_120m_prev;
always @(posedge clk_120m_dac or negedge reset_n) begin
    if (!reset_n)
        frame_toggle_120m_prev <= 1'b0;
    else
        frame_toggle_120m_prev <= frame_toggle_120m;
end
assign frame_pulse_120m = frame_toggle_120m ^ frame_toggle_120m_prev;

// ============================================================================
// stm32_mixers_enable level CDC into clk_120m_dac
// ============================================================================
cdc_single_bit #(.STAGES(3)) cdc_mixers_en_120m (
    .src_clk(clk_100m),         // Treat as pseudo-source (GPIO is async)
    .dst_clk(clk_120m_dac),
    .reset_n(reset_n),
    .src_signal(stm32_mixers_enable),
    .dst_signal(mixers_enable_120m)
);

// ============================================================================
// Beam-step CDC + edge detection (clk_100m, unchanged from v1)
// ============================================================================
cdc_single_bit #(.STAGES(2)) cdc_stm32_elevation (
    .src_clk(clk_100m),
    .dst_clk(clk_100m),
    .reset_n(reset_100m_n),
    .src_signal(stm32_new_elevation),
    .dst_signal(stm32_new_elevation_sync)
);

cdc_single_bit #(.STAGES(2)) cdc_stm32_azimuth (
    .src_clk(clk_100m),
    .dst_clk(clk_100m),
    .reset_n(reset_100m_n),
    .src_signal(stm32_new_azimuth),
    .dst_signal(stm32_new_azimuth_sync)
);

edge_detector_enhanced elevation_edge (
    .clk(clk_100m),
    .reset_n(reset_100m_n),
    .signal_in(stm32_new_elevation_sync),
    .rising_falling_edge(new_elevation_pulse)
);

edge_detector_enhanced azimuth_edge (
    .clk(clk_100m),
    .reset_n(reset_100m_n),
    .signal_in(stm32_new_azimuth_sync),
    .rising_falling_edge(new_azimuth_pulse)
);

// ============================================================================
// PLFM Chirp Generator (chirp-v2)
// ============================================================================
plfm_chirp_controller_v2 plfm_chirp_inst (
    .clk_120m       (clk_120m_dac),
    .clk_100m       (clk_100m),
    .reset_n        (reset_n),
    .reset_100m_n   (reset_100m_n),
    .mixers_enable  (mixers_enable_120m),

    // Scheduler bridge (clk_120m_dac, post-CDC)
    .dst_chirp_valid (dst_chirp_valid),
    .dst_wave_sel    (dst_wave_sel),
    .frame_pulse_120m(frame_pulse_120m),

    // Beam-step pulses (clk_100m)
    .new_elevation  (new_elevation_pulse),
    .new_azimuth    (new_azimuth_pulse),

    // DAC outputs
    .chirp_data     (chirp_data),
    .chirp_valid    (chirp_valid),
    .new_chirp_frame(new_chirp_frame),
    .chirp_done     (chirp_sequence_done),
    .rf_switch_ctrl (fpga_rf_switch),
    .rx_mixer_en    (rx_mixer_en),
    .tx_mixer_en    (tx_mixer_en),

    // ADAR
    .adar_tx_load_1 (adar_tx_load_1),
    .adar_rx_load_1 (adar_rx_load_1),
    .adar_tx_load_2 (adar_tx_load_2),
    .adar_rx_load_2 (adar_rx_load_2),
    .adar_tx_load_3 (adar_tx_load_3),
    .adar_rx_load_3 (adar_rx_load_3),
    .adar_tx_load_4 (adar_tx_load_4),
    .adar_rx_load_4 (adar_rx_load_4),
    .adar_tr_1      (adar_tr_1),
    .adar_tr_2      (adar_tr_2),
    .adar_tr_3      (adar_tr_3),
    .adar_tr_4      (adar_tr_4),

    // Status counters
    .chirp_counter    (current_chirp),
    .elevation_counter(current_elevation),
    .azimuth_counter  (current_azimuth)
);

// ============================================================================
// DAC Output Interface
// ============================================================================
dac_interface_enhanced dac_interface_inst (
    .clk_120m   (clk_120m_dac),
    .reset_n    (reset_n),
    .chirp_data (chirp_data),
    .chirp_valid(chirp_valid),
    .dac_data   (dac_data),
    .dac_clk    (dac_clk),
    .dac_sleep  (dac_sleep)
);

endmodule
