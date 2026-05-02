`timescale 1ns / 1ps
// ============================================================================
// xfft_2048.v — 2048-point FFT wrapper (Xilinx LogiCORE for synth/XSim,
// in-house fft_engine fallback for iverilog)
// ============================================================================
// AXI-Stream port list mirrors Xilinx LogiCORE Fast Fourier Transform v9.1
// (PG109). Two implementation branches selected by `FFT_USE_XILINX_IP`:
//
//   `define FFT_USE_XILINX_IP  → instantiates xfft_2048_ip (LogiCORE FFT v9.1)
//                                 Pipelined Streaming I/O, scaled mode, 32-bit
//                                 input/output (PR-O.7 widening).
//                                 Use for: Vivado synth, remote XSim sim.
//
//   `undef  FFT_USE_XILINX_IP  → instantiates fft_engine batched one-shot
//                                 (collect N → compute → drain N).
//                                 Use for: iverilog local sim only.
//
// Throughput on production silicon (Xilinx IP path): ~N + ~150 cycles per
// transform with full overlap → ~6600 cycles for 3 sequential transforms in
// the matched-filter chain, vs the 16700-cycle PRI budget. Closes RX-NEW-3.
//
// Data format: {Q[31:0], I[31:0]} packed 64-bit on s_axis/m_axis_data_tdata.
// PR-O.7 widened the path from 16- to 32-bit so the IFFT can consume the
// frequency_matched_filter Q30 product directly without the BFP-era
// >>15+saturate that crushed chirp/DC/impulse autocorrelations to zero under
// deterministic /N scaling — see project_mf_chain_dynrange_defect_2026-05-02.
//
// Config tdata layout (24-bit, scaled mode — see AUDIT-C10/C-8 in
// radar_params.vh `RP_FFT_SCALE_SCH):
//   bit  0     = FWD/INV   (1 = forward, 0 = inverse)
//   bits[22:1] = SCALE_SCH (22 bits, fixed schedule from RP_FFT_SCALE_SCH)
//   bit  23    = byte-align padding
//
// Scaled mode replaces the previous Block-Floating-Point setting. BFP returned
// a per-frame BLK_EXP on m_axis_data_tuser that the bridge dropped — sim and
// silicon disagreed on absolute magnitude per frame, breaking CFAR alpha
// portability. Scaled with schedule `RP_FFT_SCALE_SCH = [1,1,…,1] gives
// deterministic /N output, mirrored in fft_engine.v fallback.
// ============================================================================

module xfft_2048 (
    input  wire        aclk,
    input  wire        aresetn,

    // Configuration channel (AXI-Stream slave). 24-bit tdata carries
    // {pad, SCALE_SCH[21:0], FWD/INV}.
    input  wire [23:0] s_axis_config_tdata,
    input  wire        s_axis_config_tvalid,
    output wire        s_axis_config_tready,

    // Data input channel (AXI-Stream slave). 64-bit packed {Q[31:0], I[31:0]}.
    input  wire [63:0] s_axis_data_tdata,
    input  wire        s_axis_data_tvalid,
    input  wire        s_axis_data_tlast,
    output wire        s_axis_data_tready,

    // Data output channel (AXI-Stream master). 64-bit packed {Q[31:0], I[31:0]}.
    // No tuser — scaled mode does not emit BLK_EXP, and the design has no
    // XK_INDEX / OVFLO consumers.
    output wire [63:0] m_axis_data_tdata,
    output wire        m_axis_data_tvalid,
    output wire        m_axis_data_tlast,
    input  wire        m_axis_data_tready
);

`ifdef FFT_USE_XILINX_IP
// ============================================================================
// XILINX LOGICORE FFT v9.1 — production / XSim path
// ============================================================================
// Side-channels (status/event) are tied off here; if downstream needs them
// (e.g. for pipeline-stall debug), surface them through this wrapper.

wire [7:0] xfft_status_tdata;
wire       xfft_status_tvalid;
// tuser still exists on the IP port surface (Vivado emits a 1-bit dummy in
// scaled mode with no XK_INDEX/OVFLO). Wired to a local sink so the placer
// elides it.
wire [7:0] xfft_dout_tuser_unused;

xfft_2048_ip u_xfft (
    .aclk                        (aclk),
    .s_axis_config_tdata         (s_axis_config_tdata),
    .s_axis_config_tvalid        (s_axis_config_tvalid),
    .s_axis_config_tready        (s_axis_config_tready),
    .s_axis_data_tdata           (s_axis_data_tdata),
    .s_axis_data_tvalid          (s_axis_data_tvalid),
    .s_axis_data_tready          (s_axis_data_tready),
    .s_axis_data_tlast           (s_axis_data_tlast),
    .m_axis_data_tdata           (m_axis_data_tdata),
    .m_axis_data_tuser           (xfft_dout_tuser_unused),
    .m_axis_data_tvalid          (m_axis_data_tvalid),
    .m_axis_data_tready          (m_axis_data_tready),
    .m_axis_data_tlast           (m_axis_data_tlast),
    .m_axis_status_tdata         (xfft_status_tdata),
    .m_axis_status_tvalid        (xfft_status_tvalid),
    .m_axis_status_tready        (1'b1),
    .event_frame_started         (),
    .event_tlast_unexpected      (),
    .event_tlast_missing         (),
    .event_status_channel_halt   (),
    .event_data_in_channel_halt  (),
    .event_data_out_channel_halt ()
);

`else
// ============================================================================
// FALLBACK — fft_engine batched one-shot (iverilog path only)
// ============================================================================
// Collect N samples → kick fft_engine → drain N samples. Throughput is
// ~N (collect) + ~160 K (compute) + ~N (drain). NOT representative of the
// real LogiCORE — used only for unit-level iverilog regression coverage.
// ============================================================================

localparam N         = 2048;
localparam LOG2N     = 11;
localparam CNT_W     = LOG2N + 1;

localparam [2:0] S_IDLE   = 3'd0,
                 S_FEED   = 3'd1,
                 S_RUN    = 3'd2,
                 S_OUTPUT = 3'd3;

reg [2:0] state;
reg       inverse_reg;

(* ram_style = "block" *) reg signed [31:0] in_buf_re  [0:N-1];
(* ram_style = "block" *) reg signed [31:0] in_buf_im  [0:N-1];
(* ram_style = "block" *) reg signed [31:0] out_buf_re [0:N-1];
(* ram_style = "block" *) reg signed [31:0] out_buf_im [0:N-1];

reg [CNT_W-1:0] in_count;
reg [CNT_W-1:0] feed_count;
reg [CNT_W-1:0] out_total;
reg [CNT_W-1:0] out_count;

reg                fft_start;
reg                fft_inverse;
reg signed [31:0]  fft_din_re, fft_din_im;
reg                fft_din_valid;
wire signed [31:0] fft_dout_re, fft_dout_im;
wire               fft_dout_valid;
wire               fft_busy;
wire               fft_done;

reg                in_buf_we;
reg [LOG2N-1:0]    in_buf_waddr;
reg signed [31:0]  in_buf_wdata_re, in_buf_wdata_im;
reg                out_buf_we;
reg [LOG2N-1:0]    out_buf_waddr;
reg signed [31:0]  out_buf_wdata_re, out_buf_wdata_im;

reg signed [31:0]  out_rd_re, out_rd_im;
reg                out_rd_valid;

fft_engine #(
    .N(N), .LOG2N(LOG2N), .DATA_W(32), .INTERNAL_W(32),
    .TWIDDLE_W(16), .TWIDDLE_FILE("fft_twiddle_2048.mem")
) fft_core (
    .clk(aclk), .reset_n(aresetn),
    .start(fft_start), .inverse(fft_inverse),
    .din_re(fft_din_re), .din_im(fft_din_im), .din_valid(fft_din_valid),
    .dout_re(fft_dout_re), .dout_im(fft_dout_im), .dout_valid(fft_dout_valid),
    .busy(fft_busy), .done(fft_done)
);

assign s_axis_config_tready = (state == S_IDLE);
assign s_axis_data_tready   = (state == S_FEED) && (in_count < N);
assign m_axis_data_tdata    = {out_rd_im, out_rd_re};
assign m_axis_data_tvalid   = out_rd_valid;
assign m_axis_data_tlast    = out_rd_valid && (out_count == N);

always @(posedge aclk) begin
    if (in_buf_we) begin
        in_buf_re[in_buf_waddr] <= in_buf_wdata_re;
        in_buf_im[in_buf_waddr] <= in_buf_wdata_im;
    end
    if (out_buf_we) begin
        out_buf_re[out_buf_waddr] <= out_buf_wdata_re;
        out_buf_im[out_buf_waddr] <= out_buf_wdata_im;
    end
end

always @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        state            <= S_IDLE;
        inverse_reg      <= 1'b0;
        in_count         <= 0;
        feed_count       <= 0;
        out_total        <= 0;
        out_count        <= 0;
        fft_start        <= 1'b0;
        fft_inverse      <= 1'b0;
        fft_din_re       <= 0;
        fft_din_im       <= 0;
        fft_din_valid    <= 1'b0;
        in_buf_we        <= 1'b0;
        in_buf_waddr     <= 0;
        in_buf_wdata_re  <= 0;
        in_buf_wdata_im  <= 0;
        out_buf_we       <= 1'b0;
        out_buf_waddr    <= 0;
        out_buf_wdata_re <= 0;
        out_buf_wdata_im <= 0;
        out_rd_re        <= 0;
        out_rd_im        <= 0;
        out_rd_valid     <= 1'b0;
    end else begin
        fft_start     <= 1'b0;
        fft_din_valid <= 1'b0;
        in_buf_we     <= 1'b0;
        out_buf_we    <= 1'b0;

        case (state)
        S_IDLE: begin
            in_count   <= 0;
            feed_count <= 0;
            out_total  <= 0;
            out_count  <= 0;
            out_rd_valid <= 1'b0;
            if (s_axis_config_tvalid) begin
                inverse_reg <= ~s_axis_config_tdata[0];
                state       <= S_FEED;
            end
        end

        S_FEED: begin
            if (in_count < N) begin
                if (s_axis_data_tvalid) begin
                    in_buf_we       <= 1'b1;
                    in_buf_waddr    <= in_count[LOG2N-1:0];
                    in_buf_wdata_re <= s_axis_data_tdata[31:0];
                    in_buf_wdata_im <= s_axis_data_tdata[63:32];
                    in_count        <= in_count + 1;
                end
            end else begin
                fft_start   <= 1'b1;
                fft_inverse <= inverse_reg;
                feed_count  <= 0;
                out_total   <= 0;
                state       <= S_RUN;
            end
        end

        S_RUN: begin
            if (feed_count < N) begin
                fft_din_re    <= in_buf_re[feed_count[LOG2N-1:0]];
                fft_din_im    <= in_buf_im[feed_count[LOG2N-1:0]];
                fft_din_valid <= 1'b1;
                feed_count    <= feed_count + 1;
            end
            if (fft_dout_valid && out_total < N) begin
                out_buf_we       <= 1'b1;
                out_buf_waddr    <= out_total[LOG2N-1:0];
                out_buf_wdata_re <= fft_dout_re;
                out_buf_wdata_im <= fft_dout_im;
                out_total        <= out_total + 1;
            end
            if (fft_done && out_total >= N) begin
                state     <= S_OUTPUT;
                out_count <= 0;
                out_rd_valid <= 1'b0;
            end
        end

        S_OUTPUT: begin
            if (m_axis_data_tready || !out_rd_valid) begin
                if (out_count < N) begin
                    out_rd_re    <= out_buf_re[out_count[LOG2N-1:0]];
                    out_rd_im    <= out_buf_im[out_count[LOG2N-1:0]];
                    out_rd_valid <= 1'b1;
                    out_count    <= out_count + 1;
                end else begin
                    out_rd_valid <= 1'b0;
                    state        <= S_IDLE;
                end
            end
        end

        default: state <= S_IDLE;
        endcase
    end
end

`ifdef SIMULATION
integer init_k;
initial begin
    for (init_k = 0; init_k < N; init_k = init_k + 1) begin
        in_buf_re[init_k]  = 0;
        in_buf_im[init_k]  = 0;
        out_buf_re[init_k] = 0;
        out_buf_im[init_k] = 0;
    end
end
`endif

`endif  // FFT_USE_XILINX_IP

endmodule
