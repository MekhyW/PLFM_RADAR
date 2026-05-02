`timescale 1ns / 1ps

// frequency_matched_filter.v
//
// Conjugate complex multiply for the matched-filter chain:
//   out = (a + jb) * conj(c + jd) = (ac + bd) + j(bc - ad)
//
// Inputs are 16-bit Q15 (post-FWD-FFT). Output is the full 32-bit Q30 product
// — no trailing >>15 + saturate. The matched-filter chain widens the path to
// the IFFT to 32-bit (AUDIT-MF-DYNRANGE / PR-O.7), so the IFFT consumes the
// raw Q30 product. Truncating here threw away the bottom 15 bits of every bin
// and crushed chirp / DC / impulse autocorrelations to zero once PR-O switched
// the FFT from BFP to deterministic /N scaling — see project_mf_chain_dynrange
// _defect_2026-05-02 in memory.
module frequency_matched_filter (
    input wire clk,
    input wire reset_n,

    // Input from Forward FFT (16-bit Q15)
    input wire signed [15:0] fft_real_in,
    input wire signed [15:0] fft_imag_in,
    input wire fft_valid_in,

    // Reference Chirp (16-bit Q15) — FFT(transmitted chirp)
    input wire signed [15:0] ref_chirp_real,
    input wire signed [15:0] ref_chirp_imag,

    // Output (32-bit Q30) — FFT(input) * conj(FFT(reference))
    output wire signed [31:0] filtered_real,
    output wire signed [31:0] filtered_imag,
    output wire filtered_valid,

    output wire [1:0] state
);

// Pipeline registers
reg signed [15:0] a_reg, b_reg, c_reg, d_reg;
reg valid_p1;
reg signed [31:0] ac_reg, bd_reg, bc_reg, ad_reg;
reg valid_p2;
reg signed [31:0] real_sum, imag_sum;
reg valid_p3;
reg signed [31:0] real_out, imag_out;
reg valid_out;

// ========== PIPELINE STAGE 1: REGISTER INPUTS ==========
// Sync reset: enables DSP48E1 absorption (fixes DPOR-1/DPIP-1 DRC)
always @(posedge clk) begin
    if (!reset_n) begin
        a_reg <= 16'd0; b_reg <= 16'd0;
        c_reg <= 16'd0; d_reg <= 16'd0;
        valid_p1 <= 1'b0;
    end else begin
        if (fft_valid_in) begin
            a_reg <= fft_real_in;      // a
            b_reg <= fft_imag_in;      // b
            c_reg <= ref_chirp_real;   // c
            d_reg <= ref_chirp_imag;   // d
        end
        valid_p1 <= fft_valid_in;
    end
end

// ========== PIPELINE STAGE 2: MULTIPLICATIONS ==========
// Q15 * Q15 = Q30
always @(posedge clk) begin
    if (!reset_n) begin
        ac_reg <= 32'd0; bd_reg <= 32'd0;
        bc_reg <= 32'd0; ad_reg <= 32'd0;
        valid_p2 <= 1'b0;
    end else begin
        ac_reg <= a_reg * c_reg;
        bd_reg <= b_reg * d_reg;
        bc_reg <= b_reg * c_reg;
        ad_reg <= a_reg * d_reg;

        valid_p2 <= valid_p1;
    end
end

// ========== PIPELINE STAGE 3: ADDITIONS ==========
// Conjugate multiply: (ac + bd) + j(bc - ad). Q30 sum, 32-bit container.
always @(posedge clk) begin
    if (!reset_n) begin
        real_sum <= 32'd0;
        imag_sum <= 32'd0;
        valid_p3 <= 1'b0;
    end else begin
        real_sum <= ac_reg + bd_reg;
        imag_sum <= bc_reg - ad_reg;

        valid_p3 <= valid_p2;
    end
end

// ========== PIPELINE STAGE 4: REGISTER OUT ==========
// Pass Q30 product through. The IFFT downstream consumes the full 32-bit
// width (PR-O.7); no truncation here.
always @(posedge clk) begin
    if (!reset_n) begin
        real_out  <= 32'd0;
        imag_out  <= 32'd0;
        valid_out <= 1'b0;
    end else begin
        if (valid_p3) begin
            real_out <= real_sum;
            imag_out <= imag_sum;
        end
        valid_out <= valid_p3;
    end
end

assign filtered_real  = real_out;
assign filtered_imag  = imag_out;
assign filtered_valid = valid_out;

assign state = {valid_out, valid_p3};

endmodule
