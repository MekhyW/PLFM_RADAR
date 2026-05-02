`timescale 1ns / 1ps
// ============================================================================
// tb_xfft_2048_xsim.v — XSim verification of xfft_2048 wrapper with real IP
// ============================================================================
// Compiled with `+define+FFT_USE_XILINX_IP` so the wrapper instantiates the
// LogiCORE FFT v9.1 (xfft_2048_ip). Cannot run in iverilog because that path
// uses Xilinx primitives (DSP48E1, BRAM18). For iverilog, leave the define
// off and the wrapper falls back to the fft_engine batched implementation.
//
// Three minimal stimuli:
//   1. DC      (re=10000, im=0)        → peak bin = 0 with large magnitude;
//                                         all other bins near zero.
//   2. Impulse (single sample (10000,0)) → output magnitude flat across all bins
//                                         (DFT of a delta = constant).
//   3. Tone    (cos+jsin at bin K=128) → peak bin = K with large magnitude;
//                                         all other bins near zero.
//
// PASS criteria:
//   - peak bin matches expected
//   - peak magnitude > 8× mean of non-peak bins (analogous to receiver-chain
//     SNR check that's been used elsewhere in this codebase)
// ============================================================================

`include "radar_params.vh"

module tb_xfft_2048_xsim;

    localparam CLK_PERIOD = 10.0;       // 100 MHz
    localparam N          = 2048;
    localparam LOG2N      = 11;

    reg         aclk      = 0;
    reg         aresetn   = 0;

    // AUDIT-C10/C-8: cfg_tdata widened to 24 bits (scaled mode SCALE_SCH+FWD/INV).
    // PR-O.7: data AXIS widened to 64-bit packed {Q[31:0], I[31:0]} —
    // matches the regenerated xfft_2048_ip with input_width=32.
    reg  [23:0] cfg_tdata;
    reg         cfg_tvalid;
    wire        cfg_tready;

    reg  [63:0] din_tdata;
    reg         din_tvalid;
    reg         din_tlast;
    wire        din_tready;

    wire [63:0] dout_tdata;
    wire        dout_tvalid;
    wire        dout_tlast;
    reg         dout_tready;

    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num   = 0;

    integer k;
    integer out_idx;
    integer peak_bin;
    integer peak_mag;
    integer mean_others;
    integer mag_sum_others;
    integer this_mag;
    integer cur_re, cur_im;

    // Capture the entire output frame (32-bit per channel, PR-O.7)
    reg signed [31:0] out_re [0:N-1];
    reg signed [31:0] out_im [0:N-1];
    integer           out_collected;

    always #(CLK_PERIOD/2) aclk = ~aclk;

    xfft_2048 dut (
        .aclk                 (aclk),
        .aresetn              (aresetn),
        .s_axis_config_tdata  (cfg_tdata),
        .s_axis_config_tvalid (cfg_tvalid),
        .s_axis_config_tready (cfg_tready),
        .s_axis_data_tdata    (din_tdata),
        .s_axis_data_tvalid   (din_tvalid),
        .s_axis_data_tlast    (din_tlast),
        .s_axis_data_tready   (din_tready),
        .m_axis_data_tdata    (dout_tdata),
        .m_axis_data_tvalid   (dout_tvalid),
        .m_axis_data_tlast    (dout_tlast),
        .m_axis_data_tready   (dout_tready)
    );

    // Continuously capture output frame
    always @(posedge aclk) begin
        if (aresetn && dout_tvalid && dout_tready && out_collected < N) begin
            out_re[out_collected] <= $signed(dout_tdata[31:0]);
            out_im[out_collected] <= $signed(dout_tdata[63:32]);
            out_collected         <= out_collected + 1;
        end
    end

    // ----------------------------------------------------------------
    // Send config (FWD = bit 0 = 1)
    // ----------------------------------------------------------------
    task send_config;
        input fwd;
        begin
            @(posedge aclk);
            // {pad[0], SCALE_SCH[21:0], FWD/INV[0]} — see radar_params.vh
            cfg_tdata  <= {1'b0, `RP_FFT_SCALE_SCH, fwd};
            cfg_tvalid <= 1'b1;
            @(posedge aclk);
            while (!cfg_tready) @(posedge aclk);
            @(posedge aclk);
            cfg_tvalid <= 1'b0;
        end
    endtask

    // ----------------------------------------------------------------
    // Stream N samples; src=0 DC, 1 impulse, 2 tone (bin K=128)
    // ----------------------------------------------------------------
    task stream_frame;
        input integer src;
        integer i;
        real    arg;
        integer re16, im16;
        begin
            out_collected = 0;
            @(posedge aclk);
            din_tvalid <= 1'b1;
            for (i = 0; i < N; i = i + 1) begin
                case (src)
                0: begin re16 = 10000; im16 = 0; end
                1: begin re16 = (i == 0) ? 10000 : 0; im16 = 0; end
                2: begin
                    arg  = 6.2831853 * 128.0 * i / N;
                    re16 = $rtoi(10000.0 * $cos(arg));
                    im16 = $rtoi(10000.0 * $sin(arg));
                   end
                default: begin re16 = 0; im16 = 0; end
                endcase
                // PR-O.7: AXIS data is now 64-bit packed {Q[31:0], I[31:0]}.
                // Sign-extend the 16-bit stim to 32-bit for the wider input.
                din_tdata <= {{16{im16[15]}}, im16[15:0], {16{re16[15]}}, re16[15:0]};
                din_tlast <= (i == N-1);
                @(posedge aclk);
                while (!din_tready) @(posedge aclk);
            end
            din_tvalid <= 1'b0;
            din_tlast  <= 1'b0;
        end
    endtask

    // ----------------------------------------------------------------
    // Wait until the full output frame has been captured (out_collected == N)
    // or a deadline elapses.
    // ----------------------------------------------------------------
    task wait_frame;
        input integer max_cycles;
        integer t;
        begin
            t = 0;
            while (out_collected < N && t < max_cycles) begin
                @(posedge aclk);
                t = t + 1;
            end
            if (out_collected < N) begin
                $display("[FAIL] Timed out collecting frame: got %0d / %0d after %0d cycles",
                         out_collected, N, t);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Locate peak |Re|+|Im| bin in captured frame
    // ----------------------------------------------------------------
    task analyze_frame;
        output integer pk_bin;
        output integer pk_mag;
        output integer mean_other;
        integer i, mag, sum;
        begin
            pk_bin = 0;
            pk_mag = 0;
            sum    = 0;
            for (i = 0; i < N; i = i + 1) begin
                mag = (out_re[i] < 0 ? -out_re[i] : out_re[i])
                    + (out_im[i] < 0 ? -out_im[i] : out_im[i]);
                if (mag > pk_mag) begin
                    pk_mag = mag;
                    pk_bin = i;
                end
                sum = sum + mag;
            end
            mean_other = (sum - pk_mag) / (N - 1);
        end
    endtask

    task check;
        input cond;
        input [511:0] label;
        begin
            test_num = test_num + 1;
            if (cond) begin
                $display("[PASS] T%0d: %0s", test_num, label);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] T%0d: %0s", test_num, label);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_xfft_2048_xsim.vcd");
        $dumpvars(0, tb_xfft_2048_xsim);

        cfg_tdata   = 0;
        cfg_tvalid  = 0;
        din_tdata   = 0;
        din_tvalid  = 0;
        din_tlast   = 0;
        dout_tready = 1;          // Always accept output
        out_collected = 0;

        repeat (10) @(posedge aclk);
        aresetn = 1'b1;
        repeat (10) @(posedge aclk);

        // ============================================================
        // T1: DC stimulus → expect peak at bin 0
        // ============================================================
        $display("\n--- DC stimulus ---");
        send_config(1'b1);
        stream_frame(0);
        wait_frame(20000);
        analyze_frame(peak_bin, peak_mag, mean_others);
        $display("  peak_bin=%0d peak_mag=%0d mean_others=%0d",
                 peak_bin, peak_mag, mean_others);
        check(peak_bin == 0,                  "DC -> peak at bin 0");
        check(peak_mag > 8 * mean_others + 1, "DC -> peak/mean > 8x");

        // ============================================================
        // T2: Impulse → expect roughly flat magnitude
        // ============================================================
        $display("\n--- Impulse stimulus ---");
        send_config(1'b1);
        stream_frame(1);
        wait_frame(20000);
        analyze_frame(peak_bin, peak_mag, mean_others);
        $display("  peak_bin=%0d peak_mag=%0d mean_others=%0d",
                 peak_bin, peak_mag, mean_others);
        // For an impulse at sample 0, |X[k]| is constant; peak/mean ratio
        // close to 1. Allow up to 3x to account for bit-width quantization.
        check(peak_mag < 3 * mean_others + 100,
              "Impulse -> flat spectrum (peak < 3x mean)");

        // ============================================================
        // T3: Complex tone at bin 128 → expect peak at bin 128
        // ============================================================
        $display("\n--- Tone (bin 128) stimulus ---");
        send_config(1'b1);
        stream_frame(2);
        wait_frame(20000);
        analyze_frame(peak_bin, peak_mag, mean_others);
        $display("  peak_bin=%0d peak_mag=%0d mean_others=%0d",
                 peak_bin, peak_mag, mean_others);
        check(peak_bin == 128,                "Tone -> peak at bin 128");
        check(peak_mag > 8 * mean_others + 1, "Tone -> peak/mean > 8x");

        $display("");
        $display("============================================");
        $display("  XFFT_2048 (Xilinx LogiCORE) XSim RESULTS");
        $display("  PASSED: %0d / %0d", pass_count, test_num);
        $display("  FAILED: %0d / %0d", fail_count, test_num);
        if (fail_count == 0)
            $display("  ** ALL TESTS PASSED **");
        else
            $display("  ** %0d TEST(S) FAILED **", fail_count);
        $display("============================================");

        #100;
        $finish;
    end

    // Global timeout — never let the sim run forever
    initial begin
        #2000000;  // 2 ms
        $display("[FAIL] Global timeout @ 2 ms");
        $finish;
    end

endmodule
