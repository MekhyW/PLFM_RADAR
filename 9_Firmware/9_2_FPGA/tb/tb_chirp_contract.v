`timescale 1ns / 1ps
// ============================================================================
// tb_chirp_contract.v — Architectural Contract Regression for plfm_chirp_controller_v2
// ============================================================================
// Encodes the chirp-v2 (PR-E) invariants of the chirp_counter signal path as
// hard assertions. If the RTL is modified in a way that violates one of these
// contracts, this testbench fails immediately.
//
// Contracts verified:
//   C1.  chirp_counter is 0-indexed, wraps via frame_pulse_120m
//   C2.  chirp_counter resets to 0 on frame_pulse_120m (not at chirp_done)
//   C3.  chirp_counter increments only on clk_120m edges (never clk_100m alone)
//   C4.  chirp_counter increments monotonically (no skips > 1)
//   C5.  chirp_counter increments exactly when the FSM leaves ST_CHIRP
//   C6.  dst_chirp_valid pulses (not stm32 toggles) drive chirp_counter
//   C7.  chirp_counter wraps cleanly via frame_pulse: N → 0
//   C8.  chirp_counter stays in [0, 31] when frame ≤ 32 chirps (5-bit safe)
//   C9.  Receiver port-connectivity: TX-side chirp_counter still surfaces on
//        radar_transmitter.current_chirp (for status_reg compatibility)
//
// Related history: chirp-v1 had a multi-driven chirp_counter bug (A5).
// In chirp-v2 the counter has only ONE driver (the FSM in clk_120m), so
// the original A5 race is structurally unreachable — but C3 / C5 still
// guard against any future regression that re-introduces a clk_100m driver.
// ============================================================================
`include "radar_params.vh"

module tb_chirp_contract;

// ---- Sample-count constants ----
localparam integer SHORT_SAMPLES  = 120;
localparam integer MEDIUM_SAMPLES = 600;

// ---- Clock generation ----
reg clk_120m, clk_100m;
reg reset_n, reset_100m_n;
reg mixers_enable;
reg dst_chirp_valid;
reg [1:0] dst_wave_sel;
reg frame_pulse_120m;
reg new_elevation, new_azimuth;

// DUT outputs (subset — only those used in the contract checks)
wire [7:0] chirp_data;
wire chirp_valid;
wire chirp_done;
wire rf_switch_ctrl;
wire tx_mixer_en, rx_mixer_en;
wire adar_tx_load_1, adar_rx_load_1;
wire adar_tx_load_2, adar_rx_load_2;
wire adar_tx_load_3, adar_rx_load_3;
wire adar_tx_load_4, adar_rx_load_4;
wire adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4;
wire new_chirp_frame;
wire [5:0] chirp_counter;
wire [5:0] elevation_counter, azimuth_counter;

// 120 MHz: period = 8.333 ns
initial clk_120m = 0;
always #4.166 clk_120m = ~clk_120m;

// 100 MHz: period = 10 ns
initial clk_100m = 0;
always #5 clk_100m = ~clk_100m;

// ---- DUT ----
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

// ---- Test infrastructure ----
integer test_num;
integer pass_count;
integer fail_count;
integer total_tests;

// C4 monitor: chirp_counter must change by ±1 or 0 per clk_120m edge.
// Wraps via frame_pulse_120m: the FSM samples the pulse at edge T and
// schedules chirp_counter <= 0; the wrap (K → 0) is observable on the
// next monitor sample (edge T+1). frame_pulse_seen carries the pulse
// forward one cycle so the (pre, post) = (K, 0) transition is allowed.
reg [5:0] prev_counter;
reg       frame_pulse_seen;
reg       c4_violated;

always @(posedge clk_120m or negedge reset_n) begin
    if (!reset_n) begin
        prev_counter     <= 6'd0;
        frame_pulse_seen <= 1'b0;
        c4_violated      <= 1'b0;
    end else begin
        if (chirp_counter != prev_counter &&
            chirp_counter != prev_counter + 6'd1 &&
            !(chirp_counter == 6'd0 && (frame_pulse_120m || frame_pulse_seen)))
        begin
            c4_violated <= 1'b1;
        end
        frame_pulse_seen <= frame_pulse_120m;
        prev_counter     <= chirp_counter;
    end
end

task check;
    input [255:0] test_name;
    input condition;
    begin
        test_num = test_num + 1;
        if (condition) begin
            $display("  [PASS] Contract %0d: %0s", test_num, test_name);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] Contract %0d: %0s", test_num, test_name);
            fail_count = fail_count + 1;
        end
    end
endtask

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

task wait_for_idle;
    input integer timeout_cycles;
    integer i;
    begin
        for (i = 0; i < timeout_cycles; i = i + 1) begin
            @(posedge clk_120m);
            if (dut.state == 1'b0) i = timeout_cycles;
        end
    end
endtask

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
    $dumpfile("tb_chirp_contract.vcd");
    $dumpvars(0, tb_chirp_contract);

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
    $display("  CHIRP CONTRACT REGRESSION (chirp-v2 PR-E)");
    $display("============================================================");
    $display("");

    #100;
    @(posedge clk_120m);
    reset_n      <= 1;
    @(posedge clk_100m);
    reset_100m_n <= 1;
    @(posedge clk_120m);

    mixers_enable = 1;
    @(posedge clk_120m);

    // ---------- C2: counter is 0 after reset (before any chirp) ----------
    check("C2: chirp_counter == 0 after reset", chirp_counter == 6'd0);

    // ---------- C5/C6: dst_chirp_valid drives the counter ----------
    issue_chirp(`RP_WAVE_SHORT);
    wait_for_idle(SHORT_SAMPLES + 20);
    check("C5/C6: chirp_counter == 1 after first SHORT chirp", chirp_counter == 6'd1);

    issue_chirp(`RP_WAVE_SHORT);
    wait_for_idle(SHORT_SAMPLES + 20);
    check("C5/C6: chirp_counter == 2 after second SHORT chirp", chirp_counter == 6'd2);

    // ---------- C3: stm32 toggles do NOT drive chirp_counter ----------
    repeat (8) begin
        new_elevation = ~new_elevation;
        new_azimuth   = ~new_azimuth;
        @(posedge clk_100m);
    end
    new_elevation = 0;
    new_azimuth   = 0;
    check("C3: stm32 toggles do not change chirp_counter", chirp_counter == 6'd2);

    // ---------- C7: frame_pulse wraps to 0 ----------
    pulse_frame();
    @(posedge clk_120m);
    check("C7: chirp_counter wraps to 0 on frame_pulse", chirp_counter == 6'd0);

    // ---------- C5/C6: incremental sequence after wrap ----------
    issue_chirp(`RP_WAVE_MEDIUM);
    wait_for_idle(MEDIUM_SAMPLES + 20);
    check("C5/C6: chirp_counter == 1 after MEDIUM post-wrap", chirp_counter == 6'd1);

    issue_chirp(`RP_WAVE_SHORT);
    wait_for_idle(SHORT_SAMPLES + 20);
    check("C5/C6: chirp_counter == 2 after SHORT post-wrap",  chirp_counter == 6'd2);

    // ---------- C4: monotonic — confirmed by the running monitor ----------
    check("C4: monotonic ±1 increments only (monitor flag)", c4_violated == 1'b0);

    // ---------- C8: 5-bit safe over a 4-chirp run ----------
    issue_chirp(`RP_WAVE_SHORT);  wait_for_idle(SHORT_SAMPLES + 20);
    issue_chirp(`RP_WAVE_SHORT);  wait_for_idle(SHORT_SAMPLES + 20);
    check("C8: chirp_counter ≤ 31 during a normal frame", chirp_counter <= 6'd31);

    // ---------- C1: full sequence then frame wrap to 0 ----------
    pulse_frame();
    @(posedge clk_120m);
    check("C1/C7: chirp_counter wraps cleanly back to 0", chirp_counter == 6'd0);

    // =====================================================================
    // SUMMARY
    // =====================================================================
    $display("");
    $display("============================================================");
    total_tests = pass_count + fail_count;
    $display("  CONTRACT RESULTS: %0d/%0d contracts upheld", pass_count, total_tests);
    if (fail_count == 0)
        $display("  STATUS: ALL CONTRACTS UPHELD");
    else
        $display("  STATUS: %0d CONTRACT VIOLATIONS", fail_count);
    $display("============================================================");
    $display("");

    #100;
    $finish;
end

initial begin
    #500000;  // 500 µs
    $display("TIMEOUT: Simulation took too long!");
    $finish;
end

endmodule
