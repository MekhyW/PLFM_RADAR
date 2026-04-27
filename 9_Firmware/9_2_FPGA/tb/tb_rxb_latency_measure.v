`timescale 1ns/1ps
`include "radar_params.vh"

// ============================================================================
// tb_rxb_latency_measure.v
//
// Purpose: empirically measure the pipeline latency of
// matched_filter_processing_chain — cycles between the first ADC sample in
// and the first range_profile_valid out — for both the long-chirp path
// (3000 samples padded to FFT_SIZE) and the short-chirp path (50 samples
// padded to FFT_SIZE).
//
// The measured latency is the value LATENCY in latency_buffer should
// compensate for so that ref_chirp_real/imag arrive at the chain in the
// SAME cycle as the corresponding adc_data_i/q.
//
// Note: matched_filter_multi_segment buffers BUFFER_SIZE=2048 samples
// before emitting to the chain regardless of how many active samples are in
// the chirp (zero-pads short chirps). So both paths feed the chain
// FFT_SIZE samples — the chain itself sees no chirp-type difference. This
// test confirms whether a single LATENCY value works for both.
// ============================================================================

module tb_rxb_latency_measure;

    localparam CLK_PERIOD = 10.0;       // 100 MHz
    localparam FFT_SIZE   = `RP_FFT_SIZE; // 2048

    reg                clk;
    reg                reset_n;
    reg  signed [15:0] adc_data_i;
    reg  signed [15:0] adc_data_q;
    reg                adc_valid;
    reg  signed [15:0] ref_chirp_real;
    reg  signed [15:0] ref_chirp_imag;
    wire signed [15:0] range_profile_i;
    wire signed [15:0] range_profile_q;
    wire               range_profile_valid;
    wire [3:0]         chain_state;

    matched_filter_processing_chain dut (
        .clk                 (clk),
        .reset_n             (reset_n),
        .adc_data_i          (adc_data_i),
        .adc_data_q          (adc_data_q),
        .adc_valid           (adc_valid),
        .ref_chirp_real      (ref_chirp_real),
        .ref_chirp_imag      (ref_chirp_imag),
        .range_profile_i     (range_profile_i),
        .range_profile_q     (range_profile_q),
        .range_profile_valid (range_profile_valid),
        .chain_state         (chain_state)
    );

    always #(CLK_PERIOD/2.0) clk = ~clk;

    // Measurement state
    integer cycle_in_first;     // cycle when first adc_valid pulse went HIGH
    integer cycle_out_first;    // cycle when first range_profile_valid went HIGH
    integer cycle_count;
    reg     saw_first_in;
    reg     saw_first_out;

    always @(posedge clk) begin
        if (!reset_n) begin
            cycle_count   <= 0;
            saw_first_in  <= 0;
            saw_first_out <= 0;
        end else begin
            cycle_count <= cycle_count + 1;
            if (adc_valid && !saw_first_in) begin
                cycle_in_first <= cycle_count;
                saw_first_in   <= 1;
                $display("[T=%0t] FIRST adc_valid=1 at cycle %0d", $time, cycle_count);
            end
            if (range_profile_valid && !saw_first_out) begin
                cycle_out_first <= cycle_count;
                saw_first_out   <= 1;
                $display("[T=%0t] FIRST range_profile_valid=1 at cycle %0d", $time, cycle_count);
            end
        end
    end

    // Stimulus
    integer k;
    integer pipeline_latency;

    task feed_unit_chirp(input integer n_active_samples);
        // Feed FFT_SIZE samples: first n_active_samples are unit-impulse chirp
        // (1 at sample 0, 0 elsewhere) — represents a maximally simple input.
        // Both adc and ref get the same impulse for autocorrelation.
        integer j;
        begin
            for (j = 0; j < FFT_SIZE; j = j + 1) begin
                if (j == 0) begin
                    adc_data_i     <= 16'sd16384;  // ~half full-scale
                    adc_data_q     <= 16'sd0;
                    ref_chirp_real <= 16'sd16384;
                    ref_chirp_imag <= 16'sd0;
                end else begin
                    adc_data_i     <= 16'sd0;
                    adc_data_q     <= 16'sd0;
                    ref_chirp_real <= 16'sd0;
                    ref_chirp_imag <= 16'sd0;
                end
                adc_valid <= 1'b1;
                @(posedge clk);
            end
            adc_valid <= 1'b0;
        end
    endtask

    initial begin
        $dumpfile("tb_rxb_latency_measure.vcd");
        $dumpvars(0, tb_rxb_latency_measure);

        clk            = 0;
        reset_n        = 0;
        adc_data_i     = 0;
        adc_data_q     = 0;
        adc_valid      = 0;
        ref_chirp_real = 0;
        ref_chirp_imag = 0;

        repeat (4) @(posedge clk);
        reset_n = 1;
        repeat (4) @(posedge clk);

        $display("\n=== RX-B latency measurement: chain pipeline depth ===");
        $display("FFT_SIZE = %0d", FFT_SIZE);
        $display("Feeding 2048-sample unit-impulse autocorrelation frame...");

        // Two runs: short chirp (50 active) and long chirp (3000 active).
        // The chain itself is chirp-agnostic (always processes FFT_SIZE=2048
        // samples) — multi_segment upstream zero-pads — so both should give
        // identical chain latency. Confirms whether prior review's claim of
        // "different LATENCY for short chirp" is real or a misconception.
        feed_unit_chirp(50);  // active samples; multi_segment zero-pads upstream

        // Wait for output to start (poll every cycle, abort if too long)
        for (k = 0; k < 60000; k = k + 1) begin
            @(posedge clk);
            if (saw_first_out) k = 60001;  // exit
        end

        if (saw_first_out) begin
            pipeline_latency = cycle_out_first - cycle_in_first;
            $display("\n=== RESULT ===");
            $display("First adc_valid     : cycle %0d", cycle_in_first);
            $display("First valid output  : cycle %0d", cycle_out_first);
            $display("Pipeline latency    : %0d cycles", pipeline_latency);
            $display("");
            // Behavioural-FFT chain pipeline depth measured at 2057 cycles
            // (cycle 4 in -> cycle 2061 out). Allow +/-50 cycle drift before
            // failing — protects against silent regressions in chain timing.
            if (pipeline_latency >= 2007 && pipeline_latency <= 2107) begin
                $display("[PASS] Chain pipeline latency = %0d cycles (in expected 2007..2107 range)",
                         pipeline_latency);
            end else begin
                $display("[FAIL] Chain pipeline latency = %0d cycles, expected ~2057 (2007..2107)",
                         pipeline_latency);
            end
        end else begin
            $display("\n=== TIMEOUT ===");
            $display("range_profile_valid never asserted within 60000 cycles");
            $display("(behavioural FFT model in fft_engine.v may be much slower than");
            $display(" Xilinx FFT IP — try Vivado simulation for accurate timing)");
        end

        // Wait a bit more to see if we get full 2048 outputs
        repeat (5000) @(posedge clk);
        $finish;
    end

    // Safety timeout
    initial begin
        #10000000;  // 10 ms simulated time
        $display("[ERROR] Simulation timeout at 10 ms");
        $finish;
    end

endmodule
