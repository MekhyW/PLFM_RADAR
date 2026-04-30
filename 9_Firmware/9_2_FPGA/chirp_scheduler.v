`timescale 1ns / 1ps

`include "radar_params.vh"

/**
 * chirp_scheduler.v  (chirp-v2 PR-D, replaces radar_mode_controller.v)
 *
 * Single source of truth for waveform identity and inter-chirp timing on the
 * RX side. Drives `wave_sel[1:0]` and `chirp_pulse` natively; downstream
 * modules (chirp_reference_rom, matched_filter_multi_segment, mti_canceller)
 * consume those without 1-bit shims.
 *
 * Operating modes (host_radar_mode, opcode 0x01):
 *   2'b00  STM32 pass-through  STM32 owns chirp timing; we follow stm32_*
 *                              toggles and announce the wave_sel that matches
 *                              the current sub-frame index.
 *   2'b01  Auto-scan           Internal FSM cycles SHORT, MEDIUM, LONG sub-
 *                              frames in order (host_subframe_enable masks
 *                              individual waveforms out without recompiling).
 *                              Each sub-frame fires `host_chirps_per_subframe`
 *                              chirps at the per-waveform timing.
 *   2'b10  Single-chirp debug  One chirp per host_trigger pulse, waveform
 *                              from host_debug_wave_sel.
 *   2'b11  Track               Host-cued dwell on one beam + one waveform
 *                              for host_track_chirp_count chirps. A watchdog
 *                              falls back to mode 01 after
 *                              RP_DEF_TRACK_WATCHDOG_FRAMES idle frames so a
 *                              USB-yank does not silently drop coverage.
 *
 * Pulse outputs (chirp_pulse, subframe_pulse, frame_pulse) are 1-cycle
 * positive pulses, not toggles. The legacy mc_new_*-style toggles are gone.
 *
 * Clock domain: clk (100 MHz), async-low reset.
 */

module chirp_scheduler (
    input  wire clk,
    input  wire reset_n,

    // Top-level mode and 3-bit sub-frame enable mask (LONG|MEDIUM|SHORT)
    input  wire [1:0] host_mode,
    input  wire [2:0] host_subframe_enable,

    // 3-ladder timing (100 MHz cycles). host_*_listen sums with host_guard
    // to define the inter-chirp PRI. Each waveform has independent chirp/
    // listen so SHORT can run faster while LONG covers full eclipse.
    input  wire [15:0] host_short_chirp_cycles,
    input  wire [15:0] host_short_listen_cycles,
    input  wire [15:0] host_medium_chirp_cycles,
    input  wire [15:0] host_medium_listen_cycles,
    input  wire [15:0] host_long_chirp_cycles,
    input  wire [15:0] host_long_listen_cycles,
    input  wire [15:0] host_guard_cycles,

    // Frame structure (chirps per Doppler sub-frame, default 16)
    input  wire [5:0]  host_chirps_per_subframe,

    // Single-chirp debug (mode 10)
    input  wire        host_trigger,
    input  wire [1:0]  host_debug_wave_sel,

    // Track mode (mode 11)
    input  wire        host_track_request,
    input  wire [1:0]  host_track_wave_sel,
    input  wire [8:0]  host_track_chirp_count,
    input  wire [5:0]  host_track_beam_az,
    input  wire [5:0]  host_track_beam_el,

    // STM32 pass-through (mode 00) toggle inputs (CDC-synced upstream)
    input  wire stm32_new_chirp,
    input  wire stm32_new_subframe,
    input  wire stm32_new_frame,

    // ====== Outputs ======
    output reg  [1:0]  wave_sel,         // canonical waveform identity
    output reg         chirp_pulse,      // 1-cycle pulse: chirp begins this clk
    output reg         subframe_pulse,   // 1-cycle pulse: sub-frame complete
    output reg         frame_pulse,      // 1-cycle pulse: frame complete
    output reg  [5:0]  chirp_counter,    // chirp index inside current frame
    output reg  [1:0]  subframe_id,      // 0=SHORT, 1=MEDIUM, 2=LONG

    // Currently selected timing for the in-flight chirp (PR-E TX async FIFO)
    output wire [15:0] cfg_chirp_cycles,
    output wire [15:0] cfg_listen_cycles,
    output wire [15:0] cfg_guard_cycles,

    // Track-mode beam pointer (latched on host_track_request rising edge)
    output reg         track_mode_active,
    output reg  [5:0]  track_beam_az,
    output reg  [5:0]  track_beam_el
);

// ============================================================================
// Edge / pulse detection on async inputs
// ============================================================================
reg trigger_prev;
reg track_request_prev;
reg stm32_new_chirp_prev;
reg stm32_new_subframe_prev;
reg stm32_new_frame_prev;

wire trigger_pulse        = host_trigger        & ~trigger_prev;
wire track_request_pulse  = host_track_request  & ~track_request_prev;
wire stm32_chirp_edge     = stm32_new_chirp     ^ stm32_new_chirp_prev;
wire stm32_subframe_edge  = stm32_new_subframe  ^ stm32_new_subframe_prev;
wire stm32_frame_edge     = stm32_new_frame     ^ stm32_new_frame_prev;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        trigger_prev            <= 1'b0;
        track_request_prev      <= 1'b0;
        stm32_new_chirp_prev    <= 1'b0;
        stm32_new_subframe_prev <= 1'b0;
        stm32_new_frame_prev    <= 1'b0;
    end else begin
        trigger_prev            <= host_trigger;
        track_request_prev      <= host_track_request;
        stm32_new_chirp_prev    <= stm32_new_chirp;
        stm32_new_subframe_prev <= stm32_new_subframe;
        stm32_new_frame_prev    <= stm32_new_frame;
    end
end

// ============================================================================
// Sub-frame helpers — pure functions of (subframe, mask)
// ============================================================================
function [1:0] first_enabled_subframe;
    input [2:0] mask;
    begin
        if      (mask[0]) first_enabled_subframe = 2'd0;  // SHORT
        else if (mask[1]) first_enabled_subframe = 2'd1;  // MEDIUM
        else if (mask[2]) first_enabled_subframe = 2'd2;  // LONG
        else              first_enabled_subframe = 2'd0;  // mask=000 fallback
    end
endfunction

function [1:0] next_enabled_subframe;
    input [1:0] cur;
    input [2:0] mask;
    reg   [1:0] try0, try1, try2;
    begin
        // Walk forward from cur+1, wrapping at 3, find first enabled bit.
        try0 = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
        try1 = (try0 == 2'd2) ? 2'd0 : (try0 + 2'd1);
        try2 = (try1 == 2'd2) ? 2'd0 : (try1 + 2'd1);
        if      (mask[try0]) next_enabled_subframe = try0;
        else if (mask[try1]) next_enabled_subframe = try1;
        else if (mask[try2]) next_enabled_subframe = try2;
        else                 next_enabled_subframe = cur;  // mask=000 fallback
    end
endfunction

function [1:0] subframe_to_wave;
    input [1:0] sf;
    begin
        case (sf)
            2'd0:    subframe_to_wave = `RP_WAVE_SHORT;
            2'd1:    subframe_to_wave = `RP_WAVE_MEDIUM;
            2'd2:    subframe_to_wave = `RP_WAVE_LONG;
            default: subframe_to_wave = `RP_WAVE_SHORT;
        endcase
    end
endfunction

// ============================================================================
// Track watchdog — count frames since last host_track_request rising edge.
// effective_mode collapses to scan once the watchdog expires so a USB stall
// does not silently freeze coverage on one beam.
// ============================================================================
reg [7:0] track_idle_frames;
wire watchdog_expired = (track_idle_frames >= `RP_DEF_TRACK_WATCHDOG_FRAMES);

wire [1:0] effective_mode = (host_mode == `RP_MODE_TRACK && watchdog_expired)
                          ? `RP_MODE_AUTO_3KM
                          : host_mode;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        track_idle_frames <= 8'd0;
    end else if (track_request_pulse) begin
        track_idle_frames <= 8'd0;
    end else if (frame_pulse && track_idle_frames != 8'hFF) begin
        track_idle_frames <= track_idle_frames + 8'd1;
    end
end

// Latch beam pointer at the start of every track dwell.
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        track_beam_az <= 6'd0;
        track_beam_el <= 6'd0;
    end else if (track_request_pulse) begin
        track_beam_az <= host_track_beam_az;
        track_beam_el <= host_track_beam_el;
    end
end

// ============================================================================
// Output mux for selected timing — wave_sel drives chirp/listen window length.
// guard is shared across waveforms.
// ============================================================================
reg [15:0] sel_chirp_cycles;
reg [15:0] sel_listen_cycles;
always @(*) begin
    case (wave_sel)
        `RP_WAVE_SHORT:  begin
            sel_chirp_cycles  = host_short_chirp_cycles;
            sel_listen_cycles = host_short_listen_cycles;
        end
        `RP_WAVE_MEDIUM: begin
            sel_chirp_cycles  = host_medium_chirp_cycles;
            sel_listen_cycles = host_medium_listen_cycles;
        end
        `RP_WAVE_LONG:   begin
            sel_chirp_cycles  = host_long_chirp_cycles;
            sel_listen_cycles = host_long_listen_cycles;
        end
        default: begin
            sel_chirp_cycles  = host_short_chirp_cycles;
            sel_listen_cycles = host_short_listen_cycles;
        end
    endcase
end
assign cfg_chirp_cycles  = sel_chirp_cycles;
assign cfg_listen_cycles = sel_listen_cycles;
assign cfg_guard_cycles  = host_guard_cycles;

// ============================================================================
// Main FSM
// ============================================================================
localparam S_IDLE    = 3'd0;
localparam S_CHIRP   = 3'd1;
localparam S_LISTEN  = 3'd2;
localparam S_GUARD   = 3'd3;
localparam S_ADVANCE = 3'd4;

reg [2:0]  state;
reg [16:0] timer;             // 17 bits cover LONG+listen+guard worst case
reg [5:0]  track_remaining;   // saturated copy of host_track_chirp_count

// Pre-computed wires used inside the FSM advance logic so non-blocking
// updates to subframe_id / wave_sel see the correct next value in the same
// clock edge as the bookkeeping update.
wire [1:0] first_sf = first_enabled_subframe(host_subframe_enable);
wire [1:0] next_sf  = next_enabled_subframe(subframe_id, host_subframe_enable);

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state             <= S_IDLE;
        timer             <= 17'd0;
        wave_sel          <= `RP_WAVE_SHORT;
        chirp_pulse       <= 1'b0;
        subframe_pulse    <= 1'b0;
        frame_pulse       <= 1'b0;
        chirp_counter     <= 6'd0;
        subframe_id       <= 2'd0;
        track_mode_active <= 1'b0;
        track_remaining   <= 6'd0;
    end else begin
        // Pulses default low — set high for one cycle on relevant transitions.
        chirp_pulse    <= 1'b0;
        subframe_pulse <= 1'b0;
        frame_pulse    <= 1'b0;

        case (effective_mode)

        // --------------------------------------------------------------------
        // MODE 00 — STM32 pass-through. STM32 owns chirp timing; we walk
        // sub-frames in step with stm32_chirp_edge so wave_sel always matches
        // the chirp the firmware just fired.
        // --------------------------------------------------------------------
        `RP_MODE_STM32_PASSTHROUGH: begin
            state             <= S_IDLE;
            timer             <= 17'd0;
            track_mode_active <= 1'b0;

            if (stm32_chirp_edge) begin
                chirp_pulse <= 1'b1;
                if (chirp_counter < host_chirps_per_subframe - 6'd1) begin
                    chirp_counter <= chirp_counter + 6'd1;
                end else begin
                    chirp_counter  <= 6'd0;
                    subframe_pulse <= 1'b1;
                    subframe_id    <= next_sf;
                    wave_sel       <= subframe_to_wave(next_sf);
                    if (next_sf == first_sf)
                        frame_pulse <= 1'b1;
                end
            end

            // STM32 firmware can pulse subframe/frame toggles directly when it
            // wants to force-advance (e.g. abort current sub-frame). These
            // override the chirp-driven walk above.
            if (stm32_subframe_edge) subframe_pulse <= 1'b1;
            if (stm32_frame_edge)    frame_pulse    <= 1'b1;
        end

        // --------------------------------------------------------------------
        // MODE 01 — Auto-scan over enabled sub-frames.
        // --------------------------------------------------------------------
        `RP_MODE_AUTO_3KM: begin
            track_mode_active <= 1'b0;
            case (state)
                S_IDLE: begin
                    timer         <= 17'd0;
                    chirp_counter <= 6'd0;
                    subframe_id   <= first_sf;
                    wave_sel      <= subframe_to_wave(first_sf);
                    chirp_pulse   <= 1'b1;
                    state         <= S_CHIRP;
                end
                S_CHIRP: begin
                    if (timer + 17'd1 >= {1'b0, sel_chirp_cycles}) begin
                        timer <= 17'd0;
                        state <= S_LISTEN;
                    end else timer <= timer + 17'd1;
                end
                S_LISTEN: begin
                    if (timer + 17'd1 >= {1'b0, sel_listen_cycles}) begin
                        timer <= 17'd0;
                        state <= S_GUARD;
                    end else timer <= timer + 17'd1;
                end
                S_GUARD: begin
                    if (timer + 17'd1 >= {1'b0, host_guard_cycles}) begin
                        timer <= 17'd0;
                        state <= S_ADVANCE;
                    end else timer <= timer + 17'd1;
                end
                S_ADVANCE: begin
                    if (chirp_counter < host_chirps_per_subframe - 6'd1) begin
                        chirp_counter <= chirp_counter + 6'd1;
                        chirp_pulse   <= 1'b1;
                        state         <= S_CHIRP;
                    end else begin
                        chirp_counter  <= 6'd0;
                        subframe_pulse <= 1'b1;
                        subframe_id    <= next_sf;
                        wave_sel       <= subframe_to_wave(next_sf);
                        if (next_sf == first_sf)
                            frame_pulse <= 1'b1;
                        chirp_pulse <= 1'b1;
                        state       <= S_CHIRP;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end

        // --------------------------------------------------------------------
        // MODE 10 — Single-chirp debug. One chirp per host_trigger.
        // --------------------------------------------------------------------
        `RP_MODE_SINGLE_DEBUG: begin
            track_mode_active <= 1'b0;
            case (state)
                S_IDLE: begin
                    timer <= 17'd0;
                    if (trigger_pulse) begin
                        wave_sel    <= host_debug_wave_sel;
                        chirp_pulse <= 1'b1;
                        state       <= S_CHIRP;
                    end
                end
                S_CHIRP: begin
                    if (timer + 17'd1 >= {1'b0, sel_chirp_cycles}) begin
                        timer <= 17'd0;
                        state <= S_LISTEN;
                    end else timer <= timer + 17'd1;
                end
                S_LISTEN: begin
                    if (timer + 17'd1 >= {1'b0, sel_listen_cycles}) begin
                        timer <= 17'd0;
                        state <= S_IDLE;
                    end else timer <= timer + 17'd1;
                end
                default: state <= S_IDLE;
            endcase
        end

        // --------------------------------------------------------------------
        // MODE 11 — Track dwell. Watchdog fallback handled by effective_mode.
        // --------------------------------------------------------------------
        `RP_MODE_TRACK: begin
            track_mode_active <= 1'b1;
            case (state)
                S_IDLE: begin
                    timer <= 17'd0;
                    if (track_request_pulse) begin
                        wave_sel        <= host_track_wave_sel;
                        // chirp_counter is 6 bits; clip the dwell length to
                        // avoid wrapping inside a single dwell.
                        track_remaining <= (host_track_chirp_count > 9'd63)
                                         ? 6'd63
                                         : host_track_chirp_count[5:0];
                        chirp_counter   <= 6'd0;
                        chirp_pulse     <= 1'b1;
                        state           <= S_CHIRP;
                    end
                end
                S_CHIRP: begin
                    if (timer + 17'd1 >= {1'b0, sel_chirp_cycles}) begin
                        timer <= 17'd0;
                        state <= S_LISTEN;
                    end else timer <= timer + 17'd1;
                end
                S_LISTEN: begin
                    if (timer + 17'd1 >= {1'b0, sel_listen_cycles}) begin
                        timer <= 17'd0;
                        state <= S_GUARD;
                    end else timer <= timer + 17'd1;
                end
                S_GUARD: begin
                    if (timer + 17'd1 >= {1'b0, host_guard_cycles}) begin
                        timer <= 17'd0;
                        state <= S_ADVANCE;
                    end else timer <= timer + 17'd1;
                end
                S_ADVANCE: begin
                    if (chirp_counter < track_remaining) begin
                        chirp_counter <= chirp_counter + 6'd1;
                        chirp_pulse   <= 1'b1;
                        state         <= S_CHIRP;
                    end else begin
                        // Dwell complete = one track frame. Watchdog ticks
                        // here on every dwell; host re-pulsing track_request
                        // resets it.
                        frame_pulse   <= 1'b1;
                        chirp_counter <= 6'd0;
                        state         <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end

        endcase
    end
end

endmodule
