`timescale 1ns / 1ps
// matched_filter_multi_segment.v

`include "radar_params.vh"

module matched_filter_multi_segment (
    input wire clk,           // 100MHz
    input wire reset_n,
    
    // Input from DDC (100 MSPS)
    input wire signed [17:0] ddc_i,
    input wire signed [17:0] ddc_q,
    input wire ddc_valid,
    
    // Chirp control (from chirp_scheduler — chirp-v2 wave_sel rail)
    input wire [1:0] wave_sel,        // 00=SHORT, 01=MEDIUM, 10=LONG
    input wire [5:0] chirp_counter,

    // Chirp boundary — 1-cycle pulse from chirp_scheduler. Replaces the old
    // mc_new_chirp toggle + XOR edge detector; mc_new_elevation/azimuth are
    // gone (they were dead — no consumer in this module).
    input wire chirp_pulse,

    // Reference chirp (chirp_reference_rom selects waveform via wave_sel)
    input wire [15:0] ref_chirp_real,
    input wire [15:0] ref_chirp_imag,
    
    // Memory system interface
    output reg [1:0] segment_request,
    output wire [10:0] sample_addr_out,  // Tell memory which sample we need (11-bit for 2048)
    output reg mem_request,
    input wire mem_ready,
    
    // Output: Pulse compressed
    output wire signed [15:0] pc_i_w,
    output wire signed [15:0] pc_q_w,
    output wire pc_valid_w,
    
    // Status
    output reg [3:0] status
);

// ========== FIXED PARAMETERS ==========
parameter BUFFER_SIZE = `RP_FFT_SIZE;              // 2048
parameter LONG_CHIRP_SAMPLES   = 3000;             // 30 us @ 100 MHz
parameter MEDIUM_CHIRP_SAMPLES = 500;              // 5 us @ 100 MHz (chirp-v2)
parameter SHORT_CHIRP_SAMPLES  = 100;              // 1 us @ 100 MHz (chirp-v2; was 50)
parameter OVERLAP_SAMPLES = `RP_OVERLAP_SAMPLES;   // 128
parameter SEGMENT_ADVANCE = `RP_SEGMENT_ADVANCE;   // 2048 - 128 = 1920 samples
parameter DEBUG = 1;                               // Debug output control

// Segment counts (overlap-save). LONG spans 2 segments; SHORT and MEDIUM
// both fit in a single 2048 buffer with zero-pad.
parameter LONG_SEGMENTS  = `RP_LONG_SEGMENTS_3KM;  // 2 segments (30 us / 2048-128 overlap)
parameter SHORT_SEGMENTS = 1;                      // SHORT or MEDIUM, single segment

// Convenience nets so the FSM body reads cleanly.
wire is_long   = (wave_sel == `RP_WAVE_LONG);
wire is_medium = (wave_sel == `RP_WAVE_MEDIUM);

// ========== FIXED INTERNAL SIGNALS ==========
reg signed [31:0] pc_i, pc_q;
reg pc_valid;

// Dual buffer for overlap-save — BRAM inferred for synthesis
(* ram_style = "block" *) reg signed [15:0] input_buffer_i [0:BUFFER_SIZE-1];
(* ram_style = "block" *) reg signed [15:0] input_buffer_q [0:BUFFER_SIZE-1];
reg [11:0] buffer_write_ptr;    // 12-bit for 0..2048
reg [11:0] buffer_read_ptr;     // 12-bit for 0..2048
reg buffer_has_data;
reg buffer_processing;
reg [15:0] chirp_samples_collected;

// BRAM write port signals
reg        buf_we;
reg [10:0] buf_waddr;           // 11-bit for 0..2047
reg signed [15:0] buf_wdata_i, buf_wdata_q;

// BRAM read port signals
reg [10:0] buf_raddr;           // 11-bit for 0..2047
reg signed [15:0] buf_rdata_i, buf_rdata_q;

// State machine
reg [3:0] state;
localparam ST_IDLE = 0;
localparam ST_COLLECT_DATA = 1;
localparam ST_ZERO_PAD = 2;
localparam ST_WAIT_REF = 3;
localparam ST_PROCESSING = 4;
localparam ST_WAIT_FFT = 5;
localparam ST_OUTPUT = 6;
localparam ST_NEXT_SEGMENT = 7;
localparam ST_OVERLAP_COPY = 8;

// Segment tracking
reg [2:0] current_segment;        // 0-3
reg [2:0] total_segments;
reg segment_done;
reg chirp_complete;
reg saw_chain_output;             // Flag: chain started producing output

// Overlap cache — captured during ST_PROCESSING, written back in ST_OVERLAP_COPY
// Uses sync-only write block to allow distributed RAM inference (not FFs).
// 128 entries = distributed RAM (LUTRAM), NOT BRAM (too shallow).
reg signed [15:0] overlap_cache_i [0:OVERLAP_SAMPLES-1];
reg signed [15:0] overlap_cache_q [0:OVERLAP_SAMPLES-1];
reg [7:0] overlap_copy_count;

// Overlap cache write port signals (driven from FSM, used in sync-only block)
reg        ov_we;
reg [6:0]  ov_waddr;
reg signed [15:0] ov_wdata_i, ov_wdata_q;

// Processing chain signals
wire [15:0] fft_pc_i, fft_pc_q;
wire fft_pc_valid;
wire [3:0] fft_chain_state;

// Buffer for FFT input
reg [15:0] fft_input_i, fft_input_q;
reg fft_input_valid;
reg fft_start;

// ========== SAMPLE ADDRESS OUTPUT ==========
assign sample_addr_out = buffer_read_ptr[10:0];

// ========== BUFFER INITIALIZATION ==========
integer buf_init;
integer ov_init;
initial begin
    for (buf_init = 0; buf_init < BUFFER_SIZE; buf_init = buf_init + 1) begin
        input_buffer_i[buf_init] = 16'd0;
        input_buffer_q[buf_init] = 16'd0;
    end
    for (ov_init = 0; ov_init < OVERLAP_SAMPLES; ov_init = ov_init + 1) begin
        overlap_cache_i[ov_init] = 16'd0;
        overlap_cache_q[ov_init] = 16'd0;
    end
end

// ========== BRAM WRITE PORT (synchronous, no async reset) ==========
always @(posedge clk) begin
    if (buf_we) begin
        input_buffer_i[buf_waddr] <= buf_wdata_i;
        input_buffer_q[buf_waddr] <= buf_wdata_q;
    end
end

// ========== OVERLAP CACHE WRITE PORT (sync only — distributed RAM inference) ==========
// Removing async reset from memory write path prevents Vivado from
// synthesizing the 128x16 arrays as FFs + mux trees.
always @(posedge clk) begin
    if (ov_we) begin
        overlap_cache_i[ov_waddr] <= ov_wdata_i;
        overlap_cache_q[ov_waddr] <= ov_wdata_q;
    end
end

// ========== BRAM READ PORT (synchronous, no async reset) ==========
always @(posedge clk) begin
    buf_rdata_i <= input_buffer_i[buf_raddr];
    buf_rdata_q <= input_buffer_q[buf_raddr];
end

// ========== FIXED STATE MACHINE WITH OVERLAP-SAVE ==========
integer i;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= ST_IDLE;
        buffer_write_ptr <= 0;
        buffer_read_ptr <= 0;
        buffer_has_data <= 0;
        buffer_processing <= 0;
        current_segment <= 0;
        segment_done <= 0;
        segment_request <= 0;
        mem_request <= 0;
        pc_valid <= 0;
        status <= 0;
        chirp_samples_collected <= 0;
        chirp_complete <= 0;
        saw_chain_output <= 0;
        fft_input_valid <= 0;
        fft_start <= 0;
        buf_we <= 0;
        buf_waddr <= 0;
        buf_wdata_i <= 0;
        buf_wdata_q <= 0;
        buf_raddr <= 0;
        ov_we <= 0;
        ov_waddr <= 0;
        ov_wdata_i <= 0;
        ov_wdata_q <= 0;
        overlap_copy_count <= 0;
    end else begin
        pc_valid <= 0;
        mem_request <= 0;
        fft_input_valid <= 0;
        buf_we <= 0;  // Default: no write
        ov_we <= 0;   // Default: no overlap write
        
        case (state)
            ST_IDLE: begin
                // Reset for new chirp
                buffer_write_ptr <= 0;
                buffer_read_ptr <= 0;
                buffer_has_data <= 0;
                buffer_processing <= 0;
                current_segment <= 0;
                segment_done <= 0;
                chirp_samples_collected <= 0;
                chirp_complete <= 0;
                saw_chain_output <= 0;
                
                // Wait for chirp start (1-cycle pulse from chirp_scheduler)
                if (chirp_pulse) begin
                    state <= ST_COLLECT_DATA;
                    total_segments <= is_long ? LONG_SEGMENTS[2:0] : SHORT_SEGMENTS[2:0];

                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Starting %s chirp, segments: %d",
                             is_long ? "LONG" : (is_medium ? "MEDIUM" : "SHORT"),
                             is_long ? LONG_SEGMENTS : SHORT_SEGMENTS);
                    $display("[MULTI_SEG_FIXED] Overlap: %d samples, Advance: %d samples",
                             OVERLAP_SAMPLES, SEGMENT_ADVANCE);
                    `endif
                end
            end
            
            ST_COLLECT_DATA: begin
                // Collect samples for current segment with overlap-save
                if (ddc_valid && buffer_write_ptr < BUFFER_SIZE) begin
                    // Store in buffer via BRAM write port
                    buf_we <= 1;
                    buf_waddr <= buffer_write_ptr[10:0];
                    // [RX-A FIX] ddc_i = {{2{gc_i[15]}}, gc_i} — top 2 bits are
                    // sign-extension. The previous `ddc_i[17:2] + ddc_i[1]`
                    // was a gratuitous /4 scaling (~12 dB dynamic-range loss).
                    // fft_engine has INTERNAL_W=32 with saturating 16-bit output,
                    // so full 16-bit input is safe (no bit-growth overflow risk).
                    buf_wdata_i <= ddc_i[15:0];
                    buf_wdata_q <= ddc_q[15:0];
                    
                    buffer_write_ptr <= buffer_write_ptr + 1;
                    chirp_samples_collected <= chirp_samples_collected + 1;
                    
                    // Debug: Show first few samples
                    if (chirp_samples_collected < 10 && buffer_write_ptr < 10) begin
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] Store[%0d]: I=%h Q=%h", 
                                 buffer_write_ptr, 
                                 ddc_i[17:2] + ddc_i[1], 
                                 ddc_q[17:2] + ddc_q[1]);
                        `endif
                    end
                    
                    // SHORT/MEDIUM single-segment path: collect waveform-specific
                    // sample count then zero-pad to 2048. SHORT=100 (1 us),
                    // MEDIUM=500 (5 us). LONG path falls through to the
                    // multi-segment overlap-save block below.
                    if (!is_long) begin
                        if (( is_medium && chirp_samples_collected >= MEDIUM_CHIRP_SAMPLES - 1) ||
                            (!is_medium && chirp_samples_collected >= SHORT_CHIRP_SAMPLES  - 1)) begin
                            state <= ST_ZERO_PAD;
                            chirp_complete <= 1;  // Bug A fix: mark chirp done so ST_OUTPUT exits to IDLE
                            `ifdef SIMULATION
                            $display("[MULTI_SEG_FIXED] %s chirp: collected %d samples, starting zero-pad",
                                     is_medium ? "Medium" : "Short",
                                     chirp_samples_collected + 1);
                            `endif
                        end
                    end
                end
                
                // LONG CHIRP: segment-ready and chirp-complete checks
                // evaluated every clock (not gated by ddc_valid) to avoid
                // missing the transition when buffer_write_ptr updates via
                // non-blocking assignment one cycle after the last write.
                //
                // Overlap-save fix: fill the FULL FFT_SIZE-sample buffer before
                // processing.  For segment 0 this means FFT_SIZE fresh samples.
                // For segments 1+, write_ptr starts at OVERLAP_SAMPLES (128)
                // so we collect 896 new samples to fill the buffer.
                if (is_long) begin
                    if (buffer_write_ptr >= BUFFER_SIZE) begin
                        buffer_has_data <= 1;
                        state <= ST_WAIT_REF;
                        segment_request <= current_segment[1:0];
                        mem_request <= 1;

                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] Segment %d ready: %d samples collected",
                                 current_segment, chirp_samples_collected);
                        `endif
                    end

                    if (chirp_samples_collected >= LONG_CHIRP_SAMPLES && !chirp_complete) begin
                        chirp_complete <= 1;
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] End of long chirp reached");
                        `endif
                        // If buffer isn't full yet, zero-pad the remainder
                        // (last segment with fewer than 896 new samples)
                        if (buffer_write_ptr < BUFFER_SIZE) begin
                            state <= ST_ZERO_PAD;
                            `ifdef SIMULATION
                            $display("[MULTI_SEG_FIXED] Last segment partial: zero-padding from %0d to %0d",
                                     buffer_write_ptr, BUFFER_SIZE - 1);
                            `endif
                        end
                    end
                end
            end
            
            ST_ZERO_PAD: begin
                // Zero-pad remaining buffer via BRAM write port
                buf_we <= 1;
                buf_waddr <= buffer_write_ptr[10:0];
                buf_wdata_i <= 16'd0;
                buf_wdata_q <= 16'd0;
                buffer_write_ptr <= buffer_write_ptr + 1;
                
                if (buffer_write_ptr >= BUFFER_SIZE - 1) begin
                    // Done zero-padding
                    buffer_has_data <= 1;
                    buffer_write_ptr <= 0;
                    state <= ST_WAIT_REF;
                    segment_request <= is_long ? current_segment[1:0] : 2'd0;
                    mem_request <= 1;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Zero-pad complete, buffer full");
                    `endif
                end
            end
            
            ST_WAIT_REF: begin
                // Wait for memory to provide reference coefficients
                buf_raddr <= 11'd0;  // Pre-present addr 0 so buf_rdata is ready next cycle
                if (mem_ready) begin
                    // Start processing — buf_rdata[0] will be valid on FIRST clock of ST_PROCESSING
                    buffer_processing <= 1;
                    buffer_read_ptr <= 0;
                    fft_start <= 1;
                    state <= ST_PROCESSING;
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Reference ready, starting processing segment %d",
                             current_segment);
                    `endif
                end
            end
            
            ST_PROCESSING: begin
                // Feed data to FFT chain from BRAM.
                // buf_raddr was pre-presented in ST_WAIT_REF (=0), so
                // buf_rdata already contains data[0] on the first clock here.
                // Each cycle: feed buf_rdata, present NEXT address.
                if ((buffer_processing) && (buffer_read_ptr < BUFFER_SIZE)) begin
                    // 1. Feed BRAM read data to FFT (valid for current buffer_read_ptr)
                    fft_input_i <= buf_rdata_i;
                    fft_input_q <= buf_rdata_q;
                    fft_input_valid <= 1;
                    
                    // 2. Request corresponding reference sample
                    mem_request <= 1'b1;
                    
                    // 3. Cache tail samples for overlap-save (via sync-only write port)
                    if (buffer_read_ptr >= SEGMENT_ADVANCE) begin
                        ov_we <= 1;
                        ov_waddr <= buffer_read_ptr - SEGMENT_ADVANCE;  // 0..OVERLAP-1
                        ov_wdata_i <= buf_rdata_i;
                        ov_wdata_q <= buf_rdata_q;
                    end
                    
                    // Debug every 100 samples
                    if (buffer_read_ptr % 100 == 0) begin
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] Processing[%0d]: ADC I=%h Q=%h",
                                buffer_read_ptr,
                                buf_rdata_i,
                                buf_rdata_q);
                        `endif
                    end
                    
                    // Present NEXT read address (for next cycle)
                    buf_raddr <= buffer_read_ptr[10:0] + 11'd1;
                    buffer_read_ptr <= buffer_read_ptr + 1;
                    
                end else if (buffer_read_ptr >= BUFFER_SIZE) begin
                    // Done feeding buffer
                    fft_input_valid <= 0;
                    mem_request <= 0;
                    buffer_processing <= 0;
                    buffer_has_data <= 0;
                    saw_chain_output <= 0;
                    state <= ST_WAIT_FFT;  // CRITICAL: Wait for FFT completion
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Finished feeding %d samples to FFT, waiting...",
                             BUFFER_SIZE);
                    `endif
                end
            end
            
            ST_WAIT_FFT: begin
                // Wait for the processing chain to complete ALL outputs.
                // The chain streams FFT_SIZE samples (fft_pc_valid=1 for FFT_SIZE clocks),
                // then transitions to ST_DONE (9) -> ST_IDLE (0).
                // We track when output starts (saw_chain_output) and only
                // proceed once the chain returns to idle after outputting.
                if (fft_pc_valid) begin
                    saw_chain_output <= 1;
                end
                
                if (saw_chain_output && fft_chain_state == 4'd0) begin
                    // Chain has returned to idle after completing all output
                    saw_chain_output <= 0;
                    state <= ST_OUTPUT;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Chain complete for segment %d, entering ST_OUTPUT",
                             current_segment);
                    `endif
                end
            end
            
            ST_OUTPUT: begin
                // Store FFT output
                pc_i <= fft_pc_i;
                pc_q <= fft_pc_q;
                pc_valid <= 1;
                segment_done <= 1;
                
                `ifdef SIMULATION
                $display("[MULTI_SEG_FIXED] Output segment %d: I=%h Q=%h",
                         current_segment, fft_pc_i, fft_pc_q);
                `endif
                
                // Check if we need more segments
                if (current_segment < total_segments - 1 || !chirp_complete) begin
                    state <= ST_NEXT_SEGMENT;
                end else begin
                    // All segments complete
                    state <= ST_IDLE;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] All %d segments complete",
                             total_segments);
                    `endif
                end
            end
            
            ST_NEXT_SEGMENT: begin
                // Prepare for next segment with OVERLAP-SAVE
                current_segment <= current_segment + 1;
                segment_done <= 0;
                
                if (is_long) begin
                    // OVERLAP-SAVE: Write cached tail samples back to BRAM [0..127]
                    overlap_copy_count <= 0;
                    state <= ST_OVERLAP_COPY;
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Overlap-save: writing %d cached samples",
                             OVERLAP_SAMPLES);
                    `endif
                end else begin
                    // SHORT or MEDIUM: only one segment, no overlap-save.
                    buffer_write_ptr <= 0;
                    if (!chirp_complete) begin
                        state <= ST_COLLECT_DATA;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
            end
            
            ST_OVERLAP_COPY: begin
                // Write one cached overlap sample per cycle to BRAM
                buf_we <= 1;
                buf_waddr <= {{3{1'b0}}, overlap_copy_count};
                buf_wdata_i <= overlap_cache_i[overlap_copy_count];
                buf_wdata_q <= overlap_cache_q[overlap_copy_count];
                
                if (overlap_copy_count < OVERLAP_SAMPLES - 1) begin
                    overlap_copy_count <= overlap_copy_count + 1;
                end else begin
                    // All 128 samples written back
                    buffer_write_ptr <= OVERLAP_SAMPLES;
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Overlap-save: copied %d samples, write_ptr=%d",
                             OVERLAP_SAMPLES, OVERLAP_SAMPLES);
                    `endif
                    
                    if (!chirp_complete) begin
                        state <= ST_COLLECT_DATA;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
            end
            
            default: begin
                state <= ST_IDLE;
            end
        endcase
        
        // Update status — bit 0 echoes is_long for legacy probes; full
        // wave_sel is consumed at the module boundary.
        status <= {state[2:0], is_long};
    end
end

// ========== PROCESSING CHAIN INSTANTIATION ==========
matched_filter_processing_chain m_f_p_c(
    .clk(clk),
    .reset_n(reset_n),
    
    // Input ADC Data
    .adc_data_i(fft_input_i),
    .adc_data_q(fft_input_q),
    .adc_valid(fft_input_valid),// && buffer_processing),

    // RX-A1: chain.chirp_counter removed (was unused inside the chain).
    // multi_segment.chirp_counter input is now formally unused but kept
    // on the port list for potential future per-chirp sequencing.

    // Reference Chirp Memory Interface (single pair — upstream selects long/short)
    .ref_chirp_real(ref_chirp_real),
    .ref_chirp_imag(ref_chirp_imag),
    
    // Output
    .range_profile_i(fft_pc_i),
    .range_profile_q(fft_pc_q),
    .range_profile_valid(fft_pc_valid),
    
    // Status
    .chain_state(fft_chain_state)
);

// ========== DEBUG MONITOR ==========
`ifdef SIMULATION
reg [31:0] dbg_cycles;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        dbg_cycles <= 0;
    end else begin
        dbg_cycles <= dbg_cycles + 1;
        
        // Monitor state transitions
        if (dbg_cycles % 1000 == 0 && state != ST_IDLE) begin
            $display("[MULTI_SEG_MONITOR @%0d] state=%0d, segment=%0d/%0d, samples=%0d",
                     dbg_cycles, state, current_segment, total_segments,
                     chirp_samples_collected);
        end
    end
end
`endif

// ========== OUTPUT CONNECTIONS ==========
assign pc_i_w = fft_pc_i;
assign pc_q_w = fft_pc_q;
assign pc_valid_w = fft_pc_valid;

endmodule