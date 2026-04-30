`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Testbench: plfm_chirp_controller_v2 (chirp-v2 PR-E)
//
// The v2 module is a pure DAC playback driver — it no longer owns its own
// LISTEN/GUARD/DONE FSM (that moved into chirp_scheduler on the RX side).
// Tests here verify:
//   - Reset behavior (IDLE, idle-code 128, all flags low)
//   - IDLE hold while mixers_enable=0
//   - SHORT/MEDIUM/LONG chirp playback durations match LUT lengths
//   - chirp_data exits idle code and rf_switch / adar_tr / chirp_valid go
//     active during CHIRP, deassert after
//   - chirp_counter increments per chirp and clears on frame_pulse_120m
//   - mixer enables: tx_mixer_en active during CHIRP, rx_mixer_en otherwise
//   - elevation_counter / azimuth_counter still bump on STM32 toggles
//
// Sample counts (must mirror plfm_chirp_controller_v2.v localparams):
//   SHORT  = 120, MEDIUM = 600, LONG = 3600
//////////////////////////////////////////////////////////////////////////////
`include "radar_params.vh"

module tb_chirp_controller;

// ---- Sample-count constants (match the RTL) ----
localparam integer SHORT_SAMPLES  = 120;
localparam integer MEDIUM_SAMPLES = 600;
localparam integer LONG_SAMPLES   = 3600;

// =========================================================================
// CLOCK GENERATION
// =========================================================================
reg clk_120m, clk_100m;
reg reset_n, reset_100m_n;

// 120 MHz: period = 8.333 ns
initial clk_120m = 0;
always #4.166 clk_120m = ~clk_120m;

// 100 MHz: period = 10 ns
initial clk_100m = 0;
always #5 clk_100m = ~clk_100m;

// =========================================================================
// DUT SIGNALS
// =========================================================================
reg        mixers_enable;
reg        dst_chirp_valid;
reg [1:0]  dst_wave_sel;
reg        frame_pulse_120m;
reg        new_elevation;
reg        new_azimuth;

wire [7:0] chirp_data;
wire chirp_valid;
wire new_chirp_frame;
wire chirp_done;
wire rf_switch_ctrl;
wire rx_mixer_en, tx_mixer_en;
wire adar_tx_load_1, adar_rx_load_1;
wire adar_tx_load_2, adar_rx_load_2;
wire adar_tx_load_3, adar_rx_load_3;
wire adar_tx_load_4, adar_rx_load_4;
wire adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4;
wire [5:0] chirp_counter;
wire [5:0] elevation_counter;
wire [5:0] azimuth_counter;

// =========================================================================
// DUT
// =========================================================================
plfm_chirp_controller_v2 dut (
    .clk_120m(clk_120m),
    .clk_100m(clk_100m),
    .reset_n(reset_n),
    .reset_100m_n(reset_100m_n),
    .mixers_enable(mixers_enable),
    .dst_chirp_valid(dst_chirp_valid),
    .dst_wave_sel(dst_wave_sel),
    .frame_pulse_120m(frame_pulse_120m),
    .new_elevation(new_elevation),
    .new_azimuth(new_azimuth),
    .chirp_data(chirp_data),
    .chirp_valid(chirp_valid),
    .new_chirp_frame(new_chirp_frame),
    .chirp_done(chirp_done),
    .rf_switch_ctrl(rf_switch_ctrl),
    .rx_mixer_en(rx_mixer_en),
    .tx_mixer_en(tx_mixer_en),
    .adar_tx_load_1(adar_tx_load_1),
    .adar_rx_load_1(adar_rx_load_1),
    .adar_tx_load_2(adar_tx_load_2),
    .adar_rx_load_2(adar_rx_load_2),
    .adar_tx_load_3(adar_tx_load_3),
    .adar_rx_load_3(adar_rx_load_3),
    .adar_tx_load_4(adar_tx_load_4),
    .adar_rx_load_4(adar_rx_load_4),
    .adar_tr_1(adar_tr_1),
    .adar_tr_2(adar_tr_2),
    .adar_tr_3(adar_tr_3),
    .adar_tr_4(adar_tr_4),
    .chirp_counter(chirp_counter),
    .elevation_counter(elevation_counter),
    .azimuth_counter(azimuth_counter)
);

// =========================================================================
// TEST INFRASTRUCTURE
// =========================================================================
integer test_num;
integer pass_count;
integer fail_count;
integer total_tests;

task check;
    input [255:0] test_name;
    input condition;
    begin
        test_num = test_num + 1;
        if (condition) begin
            $display("  [PASS] Test %0d: %0s", test_num, test_name);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] Test %0d: %0s", test_num, test_name);
            fail_count = fail_count + 1;
        end
    end
endtask

// Pulse dst_chirp_valid for 1 cycle on clk_120m with the requested wave_sel
task issue_chirp;
    input [1:0] wsel;
    begin
        @(posedge clk_120m);
        dst_wave_sel    <= wsel;
        dst_chirp_valid <= 1'b1;
        @(posedge clk_120m);
        dst_chirp_valid <= 1'b0;
    end
endtask

// Wait until DUT enters ST_IDLE again (chirp finished), with timeout
task wait_for_idle;
    input integer timeout_cycles;
    integer i;
    begin
        for (i = 0; i < timeout_cycles; i = i + 1) begin
            @(posedge clk_120m);
            if (dut.state == 1'b0) begin
                i = timeout_cycles;
            end
        end
    end
endtask

// Pulse frame_pulse_120m for 1 cycle on clk_120m
task pulse_frame;
    begin
        @(posedge clk_120m);
        frame_pulse_120m <= 1'b1;
        @(posedge clk_120m);
        frame_pulse_120m <= 1'b0;
    end
endtask

// =========================================================================
// MAIN
// =========================================================================
initial begin
    $dumpfile("tb_chirp_controller.vcd");
    $dumpvars(0, tb_chirp_controller);

    test_num = 0;
    pass_count = 0;
    fail_count = 0;

    reset_n          = 0;
    reset_100m_n     = 0;
    mixers_enable    = 0;
    dst_chirp_valid  = 0;
    dst_wave_sel     = `RP_WAVE_SHORT;
    frame_pulse_120m = 0;
    new_elevation    = 0;
    new_azimuth      = 0;

    $display("");
    $display("============================================================");
    $display("  PLFM CHIRP CONTROLLER V2 TESTBENCH (chirp-v2 PR-E)");
    $display("  SHORT=%0d, MEDIUM=%0d, LONG=%0d samples",
             SHORT_SAMPLES, MEDIUM_SAMPLES, LONG_SAMPLES);
    $display("============================================================");
    $display("");

    // ---------- Reset ----------
    $display("--- Group 1: Reset ---");
    #100;
    check("Reset: state == IDLE",        dut.state == 1'b0);
    check("Reset: chirp_data == 128",    chirp_data == 8'd128);
    check("Reset: chirp_valid low",      chirp_valid == 1'b0);
    check("Reset: rf_switch_ctrl low",   rf_switch_ctrl == 1'b0);
    check("Reset: chirp_done low",       chirp_done == 1'b0);
    check("Reset: chirp_counter == 0",   chirp_counter == 6'd0);
    check("Reset: elevation_counter==1", elevation_counter == 6'd1);
    check("Reset: azimuth_counter==1",   azimuth_counter == 6'd1);

    @(posedge clk_120m);
    reset_n <= 1;
    @(posedge clk_100m);
    reset_100m_n <= 1;
    @(posedge clk_120m);

    // ---------- IDLE hold without mixers_enable ----------
    $display("--- Group 2: IDLE Hold (mixers_enable=0) ---");
    issue_chirp(`RP_WAVE_SHORT);
    repeat (4) @(posedge clk_120m);
    check("Without mixers_enable, no transition into CHIRP", dut.state == 1'b0);
    check("Without mixers_enable, chirp_data stays 128",     chirp_data == 8'd128);
    check("Without mixers_enable, chirp_valid stays 0",      chirp_valid == 1'b0);

    // ---------- SHORT chirp playback ----------
    $display("--- Group 3: SHORT chirp playback (120 samples) ---");
    mixers_enable = 1;
    @(posedge clk_120m);

    issue_chirp(`RP_WAVE_SHORT);

    // 1 dst_clk for IDLE→CHIRP transition, 1 more for CHIRP-branch output
    // registers (rf_switch / adar_tr / chirp_valid) to assert.
    @(posedge clk_120m);
    @(posedge clk_120m); #1;
    check("SHORT: enters CHIRP",                 dut.state == 1'b1);
    check("SHORT: rf_switch_ctrl asserted",      rf_switch_ctrl == 1'b1);
    check("SHORT: adar_tr_1 asserted",           adar_tr_1 == 1'b1);
    check("SHORT: chirp_valid asserted",         chirp_valid == 1'b1);
    check("SHORT: tx_mixer_en asserted",         tx_mixer_en == 1'b1);
    check("SHORT: rx_mixer_en deasserted",       rx_mixer_en == 1'b0);

    // Drain the chirp window and confirm we land back in IDLE within bound.
    wait_for_idle(SHORT_SAMPLES + 20);
    check("SHORT: returns to IDLE within 120+20 cycles", dut.state == 1'b0);
    check("SHORT: rf_switch_ctrl deasserted in IDLE",    rf_switch_ctrl == 1'b0);
    check("SHORT: chirp_data idle code 128 in IDLE",     chirp_data == 8'd128);
    check("SHORT: chirp_counter incremented to 1",       chirp_counter == 6'd1);

    // ---------- MEDIUM chirp playback ----------
    $display("--- Group 4: MEDIUM chirp playback (600 samples) ---");
    issue_chirp(`RP_WAVE_MEDIUM);
    @(posedge clk_120m);
    check("MEDIUM: enters CHIRP",            dut.state == 1'b1);
    check("MEDIUM: active_max_samples==600", dut.active_max_samples == 12'd600);
    wait_for_idle(MEDIUM_SAMPLES + 20);
    check("MEDIUM: returns to IDLE",         dut.state == 1'b0);
    check("MEDIUM: chirp_counter == 2",      chirp_counter == 6'd2);

    // ---------- LONG chirp playback ----------
    $display("--- Group 5: LONG chirp playback (3600 samples) ---");
    issue_chirp(`RP_WAVE_LONG);
    @(posedge clk_120m);
    check("LONG: enters CHIRP",               dut.state == 1'b1);
    check("LONG: active_max_samples==3600",   dut.active_max_samples == 12'd3600);
    wait_for_idle(LONG_SAMPLES + 20);
    check("LONG: returns to IDLE",            dut.state == 1'b0);
    check("LONG: chirp_counter == 3",         chirp_counter == 6'd3);

    // ---------- frame_pulse clears chirp_counter ----------
    $display("--- Group 6: frame_pulse clears chirp_counter ---");
    pulse_frame();
    @(posedge clk_120m);
    check("frame_pulse: chirp_counter back to 0", chirp_counter == 6'd0);

    // ---------- LUT data (chirp_data leaves idle during CHIRP) ----------
    $display("--- Group 7: LUT-driven chirp_data ---");
    issue_chirp(`RP_WAVE_SHORT);
    repeat (4) @(posedge clk_120m);
    check("SHORT mid-chirp: chirp_data != 128 (LUT-driven)", chirp_data != 8'd128);
    wait_for_idle(SHORT_SAMPLES + 20);

    // ---------- Mixer disable resets state ----------
    $display("--- Group 8: Mixer disable ---");
    issue_chirp(`RP_WAVE_MEDIUM);
    repeat (10) @(posedge clk_120m);
    mixers_enable = 0;
    repeat (3) @(posedge clk_120m);
    check("Mixer disable: chirp_data idle 128",    chirp_data == 8'd128);
    check("Mixer disable: chirp_valid 0",          chirp_valid == 1'b0);
    check("Mixer disable: rf_switch_ctrl 0",       rf_switch_ctrl == 1'b0);
    check("Mixer disable: tx_mixer_en 0",          tx_mixer_en == 1'b0);
    check("Mixer disable: rx_mixer_en 0",          rx_mixer_en == 1'b0);
    check("Mixer disable: state forced IDLE",      dut.state == 1'b0);

    // ---------- Beam-step counters ----------
    $display("--- Group 9: Beam steering counters ---");
    new_elevation = 1;
    @(posedge clk_100m);
    @(posedge clk_100m);
    check("Elevation: increments on toggle",
          elevation_counter == 6'd2 || elevation_counter == 6'd3);
    new_elevation = 0;

    new_azimuth = 1;
    @(posedge clk_100m);
    @(posedge clk_100m);
    check("Azimuth: increments on toggle",
          azimuth_counter == 6'd2 || azimuth_counter == 6'd3);
    new_azimuth = 0;

    // ---------- ADAR load pins tied low ----------
    $display("--- Group 10: ADAR load pins ---");
    check("adar_tx_load_1 tied low", adar_tx_load_1 == 1'b0);
    check("adar_rx_load_1 tied low", adar_rx_load_1 == 1'b0);
    check("adar_tx_load_4 tied low", adar_tx_load_4 == 1'b0);
    check("adar_rx_load_4 tied low", adar_rx_load_4 == 1'b0);

    // =====================================================================
    // SUMMARY
    // =====================================================================
    $display("");
    $display("============================================================");
    total_tests = pass_count + fail_count;
    $display("  RESULTS: %0d/%0d tests passed", pass_count, total_tests);
    if (fail_count == 0)
        $display("  STATUS: ALL TESTS PASSED");
    else
        $display("  STATUS: %0d TESTS FAILED", fail_count);
    $display("============================================================");
    $display("");

    #100;
    $finish;
end

// Timeout watchdog
initial begin
    #500000;  // 500 µs — covers LONG playback (~30 µs) + headroom
    $display("TIMEOUT: Simulation took too long!");
    $finish;
end

endmodule
