`timescale 1ns / 1ps

`include "radar_params.vh"

/**
 * usb_data_interface.v
 *
 * FT601 USB 3.0 SuperSpeed FIFO Interface (32-bit data + 4-bit BE, 100 MHz
 * ft601_clk_in). 200T premium dev board variant. 50T production board uses
 * usb_data_interface_ft2232h.v (FT2232H, 8-bit, 60 MHz) instead.
 *
 * PR-AD: brought to v2 bulk parity with the FT2232H driver (was 5+ PRs behind).
 *   - 2-bit cfar_detect_class (PR-F) replaces obsolete 1-bit cfar_detection
 *   - v2 bulk frame protocol (PR-G): 9-B header + range + doppler-mag + 2-bit
 *     detect + 1-B footer per frame
 *   - 34-byte status packet (M-5): 8 x 32-bit words + header + footer
 *   - subframe_enable snapshot (PR-U / M-8) echoed in frame byte 2 bits[5:3]
 *   - status_words[6] CFAR telemetry (PR-G)
 *   - status_words[7] MEDIUM PRI readback (M-5)
 *
 * INTERNAL ARCHITECTURE = MIRROR OF FT2232H driver. Same 3 BRAMs (doppler_mag,
 * range, detect), same RMW pipeline, same CDC chain, same WR FSM states. The
 * only divergence is the output stage:
 *
 *   FT2232H: 1 byte per cycle on `ft_data[7:0]`, 60 MHz ft_clk.
 *   FT601:   bytes accumulated into 32-bit `ft601_data[31:0]` and emitted
 *            every 4 bytes (or partial at section end) at 100 MHz
 *            ft601_clk_in.
 *
 * BYTE-ORDER CONVENTION (FT601 lane mapping):
 *   byte N of the FT2232H wire stream -> ft601_data[8*(N%4) + 7 : 8*(N%4)]
 *   The host's USB endpoint reads BE-lane order: BE[0] (data[7:0]) first,
 *   BE[1] (data[15:8]) next, BE[2] (data[23:16]) third, BE[3] (data[31:24])
 *   last. So byte 0 of the stream lands in ft601_data[7:0] with BE[0]=1.
 *   AD.2 cross-comparison TB (tb_usb_drivers_parity.v) asserts byte-equality
 *   between FT2232H ft_data and FT601 ft601_data lane reconstruction on the
 *   same stimulus.
 *
 * THROUGHPUT (PR-AD pack-and-emit at 1 byte / ft601_clk cycle):
 *   100 MHz x 1 B/cycle = 100 MB/s sustained. 200T+SUPPORT_LONG_RANGE worst-
 *   case frame ~ 458 KB at 178 fps = ~ 81 MB/s -> 23% slack. Sufficient for
 *   production; a future PR can lift to 400 MB/s with 4-byte-per-cycle BRAM
 *   restructuring if a higher frame-rate variant lands.
 *
 * USB DISCONNECT RECOVERY:
 *   Clock-activity watchdog in clk domain detects ft601_clk_in stalls (USB
 *   cable unplugged). After ~0.65 ms of silence (65536 system clocks) it
 *   asserts ft601_clk_lost, OR'd into the FT-domain reset so FSMs and FIFOs
 *   return to a clean state. 2-stage reset synchronizer deasserts cleanly
 *   when ft601_clk_in resumes.
 *
 * Clock domains:
 *   clk           = 100 MHz system clock (radar data domain)
 *   ft601_clk_in  = 100 MHz from FT601 CLKOUT (USB FIFO domain;
 *                   asynchronous to clk despite same nominal frequency)
 */

module usb_data_interface (
    input wire clk,              // Main clock (100 MHz)
    input wire reset_n,          // System reset (clk domain)
    input wire ft601_reset_n,    // FT601-domain synchronized reset

    // Radar data inputs (clk domain)
    input wire [31:0] range_profile,           // {range_q[15:0], range_i[15:0]}
    input wire range_valid,
    input wire [15:0] doppler_real,
    input wire [15:0] doppler_imag,
    input wire doppler_valid,
    // PR-G: 2-bit class replaces obsolete 1-bit cfar_detection.
    input wire [`RP_DETECT_CLASS_WIDTH-1:0] cfar_detect_class,
    input wire cfar_valid,

    // Bulk frame protocol inputs (clk domain)
    // [RX-D] Widened to RP_RANGE_BIN_WIDTH_MAX (9-bit on 50T, 12-bit on 200T)
    // to match upstream pipeline. In 3 km mode only bins 0..511 are exercised
    // and the frame wire protocol still emits 512x32=16384 cells. 20 km mode
    // (4096 bins, 131072 cells) requires a wire-protocol extension before
    // bins 512..4095 can be transported to the host.
    input wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_in,
    input wire [`RP_DOPPLER_BIN_WIDTH-1:0]   doppler_bin_in,  // 6-bit (PR-F): {sub_frame[1:0], bin[3:0]}
    input wire                               frame_complete,  // 1-cycle pulse from radar_receiver_final edge detector

    // FT601 Interface (245 Synchronous FIFO mode, 32-bit)
    inout wire [31:0] ft601_data,    // Bidirectional data bus
    output reg [3:0]  ft601_be,      // Byte-enable (active-high, lane mask)

    // VESTIGIAL OUTPUTS - kept for 200T board port compatibility. The 200T
    // XDC routes these to physical pins G21 (TXE) and G22 (RXF). Removing
    // them from the RTL would break the 200T build. Reset to 1 and never
    // driven; the actual FT601 flow-control inputs are ft601_txe / ft601_rxf
    // below.
    output reg ft601_txe_n,
    output reg ft601_rxf_n,

    input  wire ft601_txe,           // TXE# from FT601: 0 = FIFO has space
    input  wire ft601_rxf,           // RXF# from FT601: 0 = data available
    output reg  ft601_wr_n,          // Write strobe (active low)
    output reg  ft601_rd_n,          // Read strobe (active low)
    output reg  ft601_oe_n,          // Output enable (active low)
    output reg  ft601_siwu_n,        // Send Immediate / WakeUp (active low)

    // FT601 buffer-select indicators (unused, retained for port compatibility)
    input wire [1:0] ft601_srb,
    input wire [1:0] ft601_swb,

    // Clock forwarding
    output wire ft601_clk_out,       // ODDR-forwarded copy of ft601_clk_in
    input  wire ft601_clk_in,        // 100 MHz from FT601 CLKOUT

    // Host command outputs (ft601_clk_in domain - CDC'd by consumer)
    output reg [31:0] cmd_data,
    output reg cmd_valid,
    output reg [7:0]  cmd_opcode,
    output reg [7:0]  cmd_addr,
    output reg [15:0] cmd_value,

    // Stream control input (clk domain, CDC'd internally)
    input wire [5:0] stream_control,

    // PR-U / M-8: per-frame sub-frame enable mask (clk domain, CDC'd
    // internally, snapshotted at frame_complete). {LONG, MEDIUM, SHORT}.
    // Echoed in v2 frame byte 2 bits[5:3] so the host CRT can detect
    // when an operator disables a sub-frame and downgrade confidence.
    input wire [2:0] subframe_enable,

    // Status readback inputs (clk domain, CDC'd internally)
    input wire status_request,
    input wire [15:0] status_cfar_threshold,
    input wire [5:0]  status_stream_ctrl,
    // status_radar_mode + status_range_mode retired in PR-AB.b expanded.
    // Bits in status_words[0][23:22] and status_words[4][1:0] are reserved 0.
    input wire [15:0] status_long_chirp,
    input wire [15:0] status_long_listen,
    input wire [15:0] status_guard,
    input wire [15:0] status_short_chirp,
    input wire [15:0] status_short_listen,
    input wire [15:0] status_medium_chirp,     // M-5: status_words[7][31:16]
    input wire [15:0] status_medium_listen,    // M-5: status_words[7][15:0]
    input wire [5:0]  status_chirps_per_elev,
    input wire        status_chirps_mismatch,  // TX-G: host requested chirps != Doppler FFT size

    // Self-test status readback
    input wire [4:0]  status_self_test_flags,
    input wire [7:0]  status_self_test_detail,
    input wire        status_self_test_busy,

    // AGC status readback
    input wire [3:0]  status_agc_current_gain,
    input wire [7:0]  status_agc_peak_magnitude,
    input wire [7:0]  status_agc_saturation_count,
    input wire        status_agc_enable,

    // AUDIT-S10: control-fault flags (clk domain). Exposed in status_words[5]
    // [6:5] for host-side observability. 2-stage level CDC into ft601_clk_in.
    input wire        status_range_decim_watchdog,  // audit F-6.4
    input wire        status_ddc_cic_fir_overrun,   // audit F-1.2

    // PR-AB.b expanded commit 5: beam-ready handshake watchdog (clk domain).
    // Sticky in chirp_scheduler; 2-FF level CDC into ft601_clk_in. Packed
    // into status_words[4][1] - same word-4 layout as FT2232H driver.
    input wire        status_beam_handshake_watchdog,

    // PR-G: 2-tier CFAR telemetry (clk domain -> status_words[6]).
    // Slow-changing per-frame values; 2-stage level CDC into ft601_clk_in.
    input wire [7:0]  status_cfar_alpha_soft,
    input wire [16:0] status_detect_threshold_soft,
    input wire [15:0] status_detect_count_cand
);

// ============================================================================
// CONSTANTS
// ============================================================================
localparam HEADER        = 8'hAA;
localparam FOOTER        = 8'h55;
localparam STATUS_HEADER = 8'hBB;

localparam NUM_RANGE_BINS   = `RP_NUM_RANGE_BINS;    // 512
localparam NUM_DOPPLER_BINS = `RP_NUM_DOPPLER_BINS;  // 48 (PR-F)
localparam RANGE_BIN_BITS   = `RP_RANGE_BIN_BITS;    // 9
localparam DOPPLER_BIN_BITS = `RP_DOPPLER_BIN_WIDTH; // 6 (PR-F)
localparam FRAME_CELLS      = NUM_RANGE_BINS * (1 << DOPPLER_BIN_BITS);  // 32768
localparam FRAME_ADDR_W     = RANGE_BIN_BITS + DOPPLER_BIN_BITS;         // 15

localparam DETECT_BITS_PER_CELL = `RP_DETECT_BITS_PER_CELL;              // 2
localparam DETECT_BYTE_ADDR_W   = FRAME_ADDR_W + 1 - 3;                  // 13
localparam DETECT_BYTE_LAST     = ((FRAME_CELLS * DETECT_BITS_PER_CELL) / 8) - 1;  // 8191
localparam DETECT_BIT_ADDR_W    = FRAME_ADDR_W + 1;                      // 16

localparam FRAME_HDR_BYTES           = `RP_FRAME_HDR_BYTES;              // 9 (PR-G)
localparam RANGE_SECTION_BYTES       = NUM_RANGE_BINS * 2;
localparam DOPPLER_MAG_SECTION_BYTES = NUM_RANGE_BINS * NUM_DOPPLER_BINS * 2;
localparam [DOPPLER_BIN_BITS-1:0] DOP_BIN_LAST = NUM_DOPPLER_BINS[DOPPLER_BIN_BITS-1:0] - 1'b1;
localparam VALID_DET_BYTES_PER_RANGE = (NUM_DOPPLER_BINS * DETECT_BITS_PER_CELL + 7) / 8;  // 12
localparam DETECT_SECTION_BYTES      = NUM_RANGE_BINS * VALID_DET_BYTES_PER_RANGE;          // 6144
localparam [3:0] DET_BYTE_LAST_PER_RANGE = VALID_DET_BYTES_PER_RANGE[3:0] - 4'd1;           // 11

localparam STATUS_PKT_LEN = 6'd34;  // M-5

// ============================================================================
// WRITE FSM STATES (FPGA -> Host, ft601_clk_in domain)
// ============================================================================
localparam [3:0] WR_IDLE         = 4'd0,
                 WR_FRAME_HDR    = 4'd1,
                 WR_RANGE_DATA   = 4'd2,
                 WR_DOPPLER_DATA = 4'd3,
                 WR_DETECT_DATA  = 4'd4,
                 WR_FRAME_FOOTER = 4'd5,
                 WR_STATUS_SEND  = 4'd6,
                 WR_DONE         = 4'd7;

reg [3:0] wr_state;

// AUDIT-C12 instrumentation: ft601_clk_in -> clk handshake. Toggles when
// WR_FSM completes a successful frame transfer (WR_DONE -> WR_IDLE).
reg wr_done_toggle;

// ============================================================================
// READ FSM STATES (Host -> FPGA, ft601_clk_in domain)
// ============================================================================
// FT601 32-bit reads land all 4 bytes in a single bus cycle, so the RD path
// is simpler than FT2232H's 4-byte shift register: assert OE, assert RD,
// sample one 32-bit word, deassert.
localparam [2:0] RD_IDLE      = 3'd0,
                 RD_OE_ASSERT = 3'd1,
                 RD_READING   = 3'd2,
                 RD_DEASSERT  = 3'd3,
                 RD_PROCESS   = 3'd4;

reg [2:0]  rd_state;
reg [31:0] rd_captured;

// ============================================================================
// DATA BUS DIRECTION CONTROL
// ============================================================================
reg [31:0] ft601_data_out;
reg        ft601_data_oe;

assign ft601_data = ft601_data_oe ? ft601_data_out : 32'hzzzz_zzzz;

// ============================================================================
// FRAME BRAM - Doppler Magnitude (clk write, ft601_clk_in read)
// ============================================================================
// Simple dual-port BRAM: port A = write (100 MHz), port B = read.
// Address = {range_bin[8:0], doppler_bin[5:0]} = 15 bits, 32768 entries.
// Data = 16-bit Manhattan magnitude |I| + |Q|.

(* ram_style = "block" *) reg [15:0] doppler_mag_bram [0:FRAME_CELLS-1];

reg [FRAME_ADDR_W-1:0] mag_wr_addr;
reg [15:0]             mag_wr_data;
reg                    mag_wr_en;

always @(posedge clk) begin
    if (mag_wr_en)
        doppler_mag_bram[mag_wr_addr] <= mag_wr_data;
end

reg [FRAME_ADDR_W-1:0] mag_rd_addr;
reg [15:0]             mag_rd_data;

always @(posedge ft601_clk_in) begin
    mag_rd_data <= doppler_mag_bram[mag_rd_addr];
end

// ============================================================================
// RANGE PROFILE BRAM (clk write, ft601_clk_in read)
// ============================================================================
// 512 entries x 16-bit magnitude. Stores Manhattan magnitude |I|+|Q|.

(* ram_style = "block" *) reg [15:0] range_bram [0:NUM_RANGE_BINS-1];

reg [RANGE_BIN_BITS-1:0] range_wr_addr;
reg [15:0]               range_wr_data;
reg                      range_wr_en;

always @(posedge clk) begin
    if (range_wr_en)
        range_bram[range_wr_addr] <= range_wr_data;
end

reg [RANGE_BIN_BITS-1:0] range_rd_addr;
reg [15:0]               range_rd_data;

always @(posedge ft601_clk_in) begin
    range_rd_data <= range_bram[range_rd_addr];
end

// ============================================================================
// DETECT-CLASS BRAM (clk write, ft601_clk_in read) - PR-G: 2 bits per cell
// ============================================================================
// FRAME_CELLS cells x 2 bits = 65536 bits stored as 8192 x 8-bit bytes.
// Each byte packs 4 consecutive cells (MSB-first):
//   byte[N] bits[7:6] = cell[4*N + 0]   (doppler_bin[1:0] = 00)
//   byte[N] bits[5:4] = cell[4*N + 1]   (doppler_bin[1:0] = 01)
//   byte[N] bits[3:2] = cell[4*N + 2]   (doppler_bin[1:0] = 10)
//   byte[N] bits[1:0] = cell[4*N + 3]   (doppler_bin[1:0] = 11)

(* ram_style = "block" *) reg [7:0] detect_bram [0:DETECT_BYTE_LAST];

reg [DETECT_BYTE_ADDR_W-1:0] detect_wr_addr;
reg [7:0]                    detect_wr_data;
reg                          detect_wr_en;

always @(posedge clk) begin
    if (detect_wr_en)
        detect_bram[detect_wr_addr] <= detect_wr_data;
end

reg [DETECT_BYTE_ADDR_W-1:0] detect_rd_addr;
reg [7:0]                    detect_rd_data;

always @(posedge ft601_clk_in) begin
    detect_rd_data <= detect_bram[detect_rd_addr];
end

// Detection BRAM read-modify-write pipeline (clk domain)
reg [DETECT_BYTE_ADDR_W-1:0]      detect_rmw_addr;
reg [1:0]                          detect_rmw_cell_idx;
reg [`RP_DETECT_CLASS_WIDTH-1:0]   detect_rmw_value;
reg [1:0]                          detect_rmw_state;  // 0=idle, 1=read, 2=write

// Port-A read for RMW (clk domain, separate from ft601_clk read port)
reg [7:0] detect_rmw_rddata;
always @(posedge clk) begin
    detect_rmw_rddata <= detect_bram[detect_rmw_addr];
end

// ============================================================================
// MANHATTAN MAGNITUDE COMPUTATION (combinational)
// ============================================================================
wire [15:0] abs_doppler_i = doppler_real[15] ? (~doppler_real + 16'd1) : doppler_real;
wire [15:0] abs_doppler_q = doppler_imag[15] ? (~doppler_imag + 16'd1) : doppler_imag;
wire [16:0] manhattan_sum = {1'b0, abs_doppler_i} + {1'b0, abs_doppler_q};
wire [15:0] manhattan_mag = manhattan_sum[16] ? 16'hFFFF : manhattan_sum[15:0];

wire [15:0] range_i = range_profile[15:0];
wire [15:0] range_q = range_profile[31:16];
wire [15:0] abs_range_i = range_i[15] ? (~range_i + 16'd1) : range_i;
wire [15:0] abs_range_q = range_q[15] ? (~range_q + 16'd1) : range_q;
wire [16:0] range_manhattan = {1'b0, abs_range_i} + {1'b0, abs_range_q};
wire [15:0] range_mag = range_manhattan[16] ? 16'hFFFF : range_manhattan[15:0];

// ============================================================================
// FRAME WRITE LOGIC (clk domain, 100 MHz)
// ============================================================================
// Accumulates one full frame of data into BRAMs. On frame_complete: toggles
// frame_ready signal for CDC into ft601_clk_in domain.

reg [15:0]                    frame_number;
reg                           frame_ready_toggle;
reg                           frame_filling;
reg [DETECT_BYTE_ADDR_W-1:0]  detect_clear_addr;
reg                           detect_clearing;
reg [RANGE_BIN_BITS-1:0]      range_write_counter;

// Forward declaration of wr_done_pulse (driven by AUDIT-C12 block below) -
// used by the main writer always block to retrigger detect_clearing after
// each USB transfer (PR-Z A6 Bug C fix).
(* ASYNC_REG = "TRUE" *) reg [2:0] wr_done_sync;
reg                                wr_done_prev;
wire                               wr_done_pulse = wr_done_sync[2] ^ wr_done_prev;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        frame_number        <= 16'd0;
        frame_ready_toggle  <= 1'b0;
        frame_filling       <= 1'b1;
        mag_wr_en           <= 1'b0;
        mag_wr_addr         <= {FRAME_ADDR_W{1'b0}};
        mag_wr_data         <= 16'd0;
        range_wr_en         <= 1'b0;
        range_wr_addr       <= {RANGE_BIN_BITS{1'b0}};
        range_wr_data       <= 16'd0;
        detect_wr_en        <= 1'b0;
        detect_wr_addr      <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_wr_data      <= 8'd0;
        detect_clearing     <= 1'b0;
        detect_clear_addr   <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_rmw_state    <= 2'd0;
        detect_rmw_addr     <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_rmw_cell_idx <= 2'd0;
        detect_rmw_value    <= `RP_DETECT_NONE;
        range_write_counter <= {RANGE_BIN_BITS{1'b0}};
    end else begin
        mag_wr_en    <= 1'b0;
        range_wr_en  <= 1'b0;
        detect_wr_en <= 1'b0;

        // === Detect-class BRAM bulk clear (runs after frame_complete) ===
        if (detect_clearing) begin
            detect_wr_en   <= 1'b1;
            detect_wr_addr <= detect_clear_addr;
            detect_wr_data <= 8'd0;
            if (detect_clear_addr == DETECT_BYTE_LAST[DETECT_BYTE_ADDR_W-1:0]) begin
                detect_clearing   <= 1'b0;
                detect_clear_addr <= {DETECT_BYTE_ADDR_W{1'b0}};
            end else begin
                detect_clear_addr <= detect_clear_addr + {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
            end
        end

        // === Detect-class RMW state machine (PR-G: 2-bit pack) ===
        case (detect_rmw_state)
            2'd0: begin /* idle */ end
            2'd1: begin
                detect_rmw_state <= 2'd2;
            end
            2'd2: begin
                detect_wr_en   <= 1'b1;
                detect_wr_addr <= detect_rmw_addr;
                detect_wr_data <= (detect_rmw_rddata & ~(8'b11000000 >> ({1'b0, detect_rmw_cell_idx} << 1)))
                                | (({6'b0, detect_rmw_value} << ((3 - {1'b0, detect_rmw_cell_idx}) << 1)));
                detect_rmw_state <= 2'd0;
            end
            default: detect_rmw_state <= 2'd0;
        endcase

        // === Doppler magnitude write ===
        if (doppler_valid && frame_filling) begin
            mag_wr_en   <= 1'b1;
            mag_wr_addr <= {range_bin_in, doppler_bin_in};
            mag_wr_data <= manhattan_mag;
        end

        // === Range profile write ===
        if (range_valid && frame_filling) begin
            range_wr_en         <= 1'b1;
            range_wr_addr       <= range_write_counter;
            range_wr_data       <= range_mag;
            range_write_counter <= range_write_counter + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
        end

        // === CFAR detect-class write (read-modify-write) ===
        if (cfar_valid && frame_filling && detect_rmw_state == 2'd0 && !detect_clearing) begin
            detect_rmw_addr     <= {range_bin_in, doppler_bin_in[DOPPLER_BIN_BITS-1:2]};
            detect_rmw_cell_idx <= doppler_bin_in[1:0];
            detect_rmw_value    <= cfar_detect_class;
            detect_rmw_state    <= 2'd1;
        end

        // === Frame complete: latch frame, signal ft601 domain ===
        if (frame_complete) begin
            frame_ready_toggle  <= ~frame_ready_toggle;
            frame_number        <= frame_number + 16'd1;
            frame_filling       <= 1'b0;
            range_write_counter <= {RANGE_BIN_BITS{1'b0}};
        end

        if (!frame_filling && !frame_complete) begin
            frame_filling <= 1'b1;
        end

        // PR-Z A6 (Bug C) fix: trigger detect_clearing on wr_done_pulse so
        // the clear runs in the dead zone between frames and finishes long
        // before the next frame's cfar CMP starts.
        if (!detect_clearing && wr_done_pulse) begin
            detect_clearing   <= 1'b1;
            detect_clear_addr <= {DETECT_BYTE_ADDR_W{1'b0}};
        end
    end
end

// ============================================================================
// AUDIT-C12: frame_pending + frame_drop_count (clk domain)
// ============================================================================
reg        frame_pending;
reg [6:0]  frame_drop_count;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        frame_pending    <= 1'b0;
        frame_drop_count <= 7'd0;
        wr_done_sync     <= 3'b000;
        wr_done_prev     <= 1'b0;
    end else begin
        wr_done_sync <= {wr_done_sync[1:0], wr_done_toggle};
        wr_done_prev <= wr_done_sync[2];

        if (frame_complete) begin
            if (frame_pending && frame_drop_count != 7'd127)
                frame_drop_count <= frame_drop_count + 7'd1;
            frame_pending <= 1'b1;
        end else if (wr_done_pulse) begin
            frame_pending <= 1'b0;
        end
    end
end

// ============================================================================
// TOGGLE CDC: clk (100 MHz) -> ft601_clk_in (100 MHz, async)
// ============================================================================

// --- Toggle registers (clk domain) ---
reg status_req_toggle;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        status_req_toggle <= 1'b0;
    end else begin
        if (status_request)
            status_req_toggle <= ~status_req_toggle;
    end
end

// --- 3-stage synchronizers (ft601_clk_in domain) ---
(* ASYNC_REG = "TRUE" *) reg [2:0] frame_ready_sync;
(* ASYNC_REG = "TRUE" *) reg [2:0] status_toggle_sync;

reg frame_ready_prev;
reg status_toggle_prev;

wire frame_ready_ft = frame_ready_sync[2] ^ frame_ready_prev;
wire status_req_ft  = status_toggle_sync[2] ^ status_toggle_prev;

// --- Stream control CDC (6-bit, only [2:0] used in PR-G v2). ---
(* ASYNC_REG = "TRUE" *) reg [5:0] stream_ctrl_sync_0;
(* ASYNC_REG = "TRUE" *) reg [5:0] stream_ctrl_sync_1;

// --- PR-G: 2-tier CFAR telemetry CDC (clk -> ft601_clk_in, 2-stage). ---
(* ASYNC_REG = "TRUE" *) reg [7:0]  alpha_soft_sync_0;
reg [7:0]                           alpha_soft_sync_1;
(* ASYNC_REG = "TRUE" *) reg [16:0] det_thr_soft_sync_0;
reg [16:0]                          det_thr_soft_sync_1;
(* ASYNC_REG = "TRUE" *) reg [15:0] det_count_cand_sync_0;
reg [15:0]                          det_count_cand_sync_1;

// --- AUDIT-C12: frame_drop_count CDC (2-stage) ---
(* ASYNC_REG = "TRUE" *) reg [6:0] frame_drop_sync_0;
reg [6:0]                          frame_drop_sync_1;

// --- AUDIT-S10: control-fault flag CDC (clk -> ft601_clk_in, 2-stage) ---
(* ASYNC_REG = "TRUE" *) reg range_decim_watchdog_sync_0;
reg                          range_decim_watchdog_sync_1;
(* ASYNC_REG = "TRUE" *) reg ddc_cic_fir_overrun_sync_0;
reg                          ddc_cic_fir_overrun_sync_1;
// PR-AB.b expanded commit 5: beam-handshake watchdog sticky CDC.
(* ASYNC_REG = "TRUE" *) reg beam_handshake_wd_sync_0;
reg                          beam_handshake_wd_sync_1;

wire stream_range_en   = stream_ctrl_sync_1[0];
wire stream_doppler_en = stream_ctrl_sync_1[1];
wire stream_cfar_en    = stream_ctrl_sync_1[2];

// --- Frame metadata snapshot (latched in clk domain) ---
reg [15:0] frame_number_snapshot;
reg [2:0]  stream_flags_snapshot;
// PR-U / M-8: snapshot of host_subframe_enable at frame_complete edge.
reg [2:0]  subframe_enable_snapshot;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        frame_number_snapshot    <= 16'd0;
        stream_flags_snapshot    <= 3'b111;
        subframe_enable_snapshot <= 3'b111;
    end else if (frame_complete) begin
        frame_number_snapshot    <= frame_number;
        stream_flags_snapshot    <= stream_control[2:0];
        subframe_enable_snapshot <= subframe_enable;
    end
end

// --- Status snapshot (ft601_clk_in domain) - M-5: 8 words ---
reg [31:0] status_words [0:7];

// Byte counter for write FSM (max section size dictates 16-bit width)
reg [15:0] wr_byte_idx;

// BRAM read address counters for frame transfer
reg [RANGE_BIN_BITS-1:0]    range_rd_idx;
reg [RANGE_BIN_BITS-1:0]    dop_range_idx;
reg [DOPPLER_BIN_BITS-1:0]  dop_doppler_idx;
reg                         wr_byte_phase;  // 0=MSB, 1=LSB for 16-bit values
reg [RANGE_BIN_BITS-1:0]    det_range_idx;
reg [3:0]                   det_doppler_byte_idx;

// ============================================================================
// CLOCK-ACTIVITY WATCHDOG (clk domain)
// ============================================================================
// Detects when ft601_clk_in stops (USB cable unplugged). Toggle in
// ft601_clk_in domain, sync into clk domain, watch for stalls. After
// 2^16 = 65536 clk cycles (~0.65 ms) without a transition: ft601_clk_lost.
reg ft601_heartbeat;
always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n)
        ft601_heartbeat <= 1'b0;
    else
        ft601_heartbeat <= ~ft601_heartbeat;
end

(* ASYNC_REG = "TRUE" *) reg [1:0] ft601_hb_sync;
reg ft601_hb_prev;
reg [15:0] ft601_clk_timeout;
reg ft601_clk_lost;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        ft601_hb_sync     <= 2'b00;
        ft601_hb_prev     <= 1'b0;
        ft601_clk_timeout <= 16'd0;
        ft601_clk_lost    <= 1'b0;
    end else begin
        ft601_hb_sync <= {ft601_hb_sync[0], ft601_heartbeat};
        ft601_hb_prev <= ft601_hb_sync[1];

        if (ft601_hb_sync[1] != ft601_hb_prev) begin
            ft601_clk_timeout <= 16'd0;
            ft601_clk_lost    <= 1'b0;
        end else if (!ft601_clk_lost) begin
            if (ft601_clk_timeout == 16'hFFFF)
                ft601_clk_lost <= 1'b1;
            else
                ft601_clk_timeout <= ft601_clk_timeout + 16'd1;
        end
    end
end

wire ft601_reset_raw_n = ft601_reset_n & ~ft601_clk_lost;
(* ASYNC_REG = "TRUE" *) reg [1:0] ft601_reset_sync;
always @(posedge ft601_clk_in or negedge ft601_reset_raw_n) begin
    if (!ft601_reset_raw_n)
        ft601_reset_sync <= 2'b00;
    else
        ft601_reset_sync <= {ft601_reset_sync[0], 1'b1};
end
wire ft601_effective_reset_n = ft601_reset_sync[1];

// ============================================================================
// BYTE PRODUCTION (combinational, ft601_clk_in domain)
// ============================================================================
// Mirrors the byte each FT2232H WR state would drive onto ft_data_out this
// cycle. Same dispatch on (wr_state, wr_byte_idx, wr_byte_phase, BRAM read
// outputs). The pack-and-emit stage downstream coalesces 4 bytes into a
// 32-bit word + BE mask.
reg [7:0] byte_now;
reg       is_section_end;
always @(*) begin
    byte_now       = 8'h00;
    is_section_end = 1'b0;
    case (wr_state)
        WR_FRAME_HDR: begin
            case (wr_byte_idx[3:0])
                4'd0: byte_now = HEADER;
                4'd1: byte_now = `RP_USB_PROTOCOL_VERSION;
                // PR-U / M-8: byte 2 = {2'b00, subframe_enable[2:0], stream_flags[2:0]}.
                4'd2: byte_now = {2'b00, subframe_enable_snapshot, stream_flags_snapshot};
                4'd3: byte_now = frame_number_snapshot[15:8];
                4'd4: byte_now = frame_number_snapshot[7:0];
                4'd5: byte_now = NUM_RANGE_BINS[15:8];
                4'd6: byte_now = NUM_RANGE_BINS[7:0];
                4'd7: byte_now = NUM_DOPPLER_BINS[15:8];
                4'd8: byte_now = NUM_DOPPLER_BINS[7:0];
                default: byte_now = 8'h00;
            endcase
            is_section_end = (wr_byte_idx[3:0] == 4'd8);
        end

        WR_RANGE_DATA: begin
            byte_now = (!wr_byte_phase) ? range_rd_data[15:8] : range_rd_data[7:0];
            is_section_end = (wr_byte_idx == RANGE_SECTION_BYTES[15:0] - 16'd1);
        end

        WR_DOPPLER_DATA: begin
            byte_now = (!wr_byte_phase) ? mag_rd_data[15:8] : mag_rd_data[7:0];
            is_section_end = (wr_byte_idx == DOPPLER_MAG_SECTION_BYTES[15:0] - 16'd1);
        end

        WR_DETECT_DATA: begin
            byte_now = detect_rd_data;
            is_section_end = (wr_byte_idx == DETECT_SECTION_BYTES[15:0] - 16'd1);
        end

        WR_FRAME_FOOTER: begin
            byte_now = FOOTER;
            is_section_end = 1'b1;
        end

        WR_STATUS_SEND: begin
            case (wr_byte_idx[5:0])
                6'd0:  byte_now = STATUS_HEADER;
                6'd1:  byte_now = status_words[0][31:24];
                6'd2:  byte_now = status_words[0][23:16];
                6'd3:  byte_now = status_words[0][15:8];
                6'd4:  byte_now = status_words[0][7:0];
                6'd5:  byte_now = status_words[1][31:24];
                6'd6:  byte_now = status_words[1][23:16];
                6'd7:  byte_now = status_words[1][15:8];
                6'd8:  byte_now = status_words[1][7:0];
                6'd9:  byte_now = status_words[2][31:24];
                6'd10: byte_now = status_words[2][23:16];
                6'd11: byte_now = status_words[2][15:8];
                6'd12: byte_now = status_words[2][7:0];
                6'd13: byte_now = status_words[3][31:24];
                6'd14: byte_now = status_words[3][23:16];
                6'd15: byte_now = status_words[3][15:8];
                6'd16: byte_now = status_words[3][7:0];
                6'd17: byte_now = status_words[4][31:24];
                6'd18: byte_now = status_words[4][23:16];
                6'd19: byte_now = status_words[4][15:8];
                6'd20: byte_now = status_words[4][7:0];
                6'd21: byte_now = status_words[5][31:24];
                6'd22: byte_now = status_words[5][23:16];
                6'd23: byte_now = status_words[5][15:8];
                6'd24: byte_now = status_words[5][7:0];
                6'd25: byte_now = status_words[6][31:24];
                6'd26: byte_now = status_words[6][23:16];
                6'd27: byte_now = status_words[6][15:8];
                6'd28: byte_now = status_words[6][7:0];
                6'd29: byte_now = status_words[7][31:24];
                6'd30: byte_now = status_words[7][23:16];
                6'd31: byte_now = status_words[7][15:8];
                6'd32: byte_now = status_words[7][7:0];
                6'd33: byte_now = FOOTER;
                default: byte_now = 8'h00;
            endcase
            is_section_end = (wr_byte_idx[5:0] == STATUS_PKT_LEN - 6'd1);
        end

        default: begin
            byte_now       = 8'h00;
            is_section_end = 1'b0;
        end
    endcase
end

// ============================================================================
// PACK-AND-EMIT STAGE (ft601_clk_in domain)
// ============================================================================
// Accumulate up to 4 bytes into pending_word[31:0]. Emit when lane 3 fills,
// or when is_section_end fires on a non-multiple-of-4 boundary (frame
// header byte 8, footer, status byte 33). Lane mapping: byte 0 -> [7:0],
// byte 1 -> [15:8], byte 2 -> [23:16], byte 3 -> [31:24].
reg [1:0]  pack_lane;
reg [23:0] pending_word_lo;  // bytes 0..2 captured before emit (byte 3 driven combinationally)

// State helpers (set inside WR FSM)
reg byte_grant;     // 1 = this cycle's byte_now is "accepted" by the FSM
reg pack_emit_now;  // 1 = drive ft601_data_out + ft601_be + ft601_wr_n=0 this cycle

integer si;

always @(posedge ft601_clk_in or negedge ft601_effective_reset_n) begin
    if (!ft601_effective_reset_n) begin
        frame_ready_sync       <= 3'b000;
        status_toggle_sync     <= 3'b000;
        frame_ready_prev       <= 1'b0;
        status_toggle_prev     <= 1'b0;
        stream_ctrl_sync_0     <= `RP_STREAM_CTRL_DEFAULT;
        stream_ctrl_sync_1     <= `RP_STREAM_CTRL_DEFAULT;
        frame_drop_sync_0      <= 7'd0;
        frame_drop_sync_1      <= 7'd0;
        range_decim_watchdog_sync_0 <= 1'b0;
        range_decim_watchdog_sync_1 <= 1'b0;
        ddc_cic_fir_overrun_sync_0  <= 1'b0;
        ddc_cic_fir_overrun_sync_1  <= 1'b0;
        beam_handshake_wd_sync_0    <= 1'b0;
        beam_handshake_wd_sync_1    <= 1'b0;
        alpha_soft_sync_0     <= 8'd0;
        alpha_soft_sync_1     <= 8'd0;
        det_thr_soft_sync_0   <= 17'd0;
        det_thr_soft_sync_1   <= 17'd0;
        det_count_cand_sync_0 <= 16'd0;
        det_count_cand_sync_1 <= 16'd0;
        for (si = 0; si < 8; si = si + 1)
            status_words[si] <= 32'd0;
        wr_state              <= WR_IDLE;
        wr_byte_idx           <= 16'd0;
        wr_byte_phase         <= 1'b0;
        dop_range_idx         <= {RANGE_BIN_BITS{1'b0}};
        dop_doppler_idx       <= {DOPPLER_BIN_BITS{1'b0}};
        range_rd_idx          <= {RANGE_BIN_BITS{1'b0}};
        range_rd_addr         <= {RANGE_BIN_BITS{1'b0}};
        det_range_idx         <= {RANGE_BIN_BITS{1'b0}};
        det_doppler_byte_idx  <= 4'd0;
        detect_rd_addr        <= {DETECT_BYTE_ADDR_W{1'b0}};
        mag_rd_addr           <= {FRAME_ADDR_W{1'b0}};
        rd_state              <= RD_IDLE;
        rd_captured           <= 32'd0;
        ft601_data_out        <= 32'd0;
        ft601_data_oe         <= 1'b0;
        ft601_be              <= 4'b1111;
        ft601_rd_n            <= 1'b1;
        ft601_wr_n            <= 1'b1;
        ft601_oe_n            <= 1'b1;
        ft601_siwu_n          <= 1'b1;
        cmd_data              <= 32'd0;
        cmd_valid             <= 1'b0;
        cmd_opcode            <= 8'd0;
        cmd_addr              <= 8'd0;
        cmd_value             <= 16'd0;
        wr_done_toggle        <= 1'b0;
        pack_lane             <= 2'd0;
        pending_word_lo       <= 24'd0;
    end else begin
        cmd_valid <= 1'b0;

        // 3-stage toggle synchronizers
        frame_ready_sync   <= {frame_ready_sync[1:0], frame_ready_toggle};
        status_toggle_sync <= {status_toggle_sync[1:0], status_req_toggle};
        frame_ready_prev   <= frame_ready_sync[2];
        status_toggle_prev <= status_toggle_sync[2];

        // Stream control CDC
        stream_ctrl_sync_0 <= stream_control;
        stream_ctrl_sync_1 <= stream_ctrl_sync_0;

        // AUDIT-C12: frame_drop_count CDC
        frame_drop_sync_0 <= frame_drop_count;
        frame_drop_sync_1 <= frame_drop_sync_0;

        // AUDIT-S10: control-fault flag CDC
        range_decim_watchdog_sync_0 <= status_range_decim_watchdog;
        range_decim_watchdog_sync_1 <= range_decim_watchdog_sync_0;
        ddc_cic_fir_overrun_sync_0  <= status_ddc_cic_fir_overrun;
        ddc_cic_fir_overrun_sync_1  <= ddc_cic_fir_overrun_sync_0;
        beam_handshake_wd_sync_0    <= status_beam_handshake_watchdog;
        beam_handshake_wd_sync_1    <= beam_handshake_wd_sync_0;

        // PR-G: 2-tier CFAR telemetry CDC
        alpha_soft_sync_0     <= status_cfar_alpha_soft;
        alpha_soft_sync_1     <= alpha_soft_sync_0;
        det_thr_soft_sync_0   <= status_detect_threshold_soft;
        det_thr_soft_sync_1   <= det_thr_soft_sync_0;
        det_count_cand_sync_0 <= status_detect_count_cand;
        det_count_cand_sync_1 <= det_count_cand_sync_0;

        // Status snapshot on request
        if (status_req_ft) begin
            // Word 0: {0xFF[31:24], reserved[23:22]=0, stream[21:16], threshold[15:0]}
            status_words[0] <= {8'hFF, 2'd0, status_stream_ctrl, status_cfar_threshold};
            status_words[1] <= {status_long_chirp, status_long_listen};
            status_words[2] <= {status_guard, status_short_chirp};
            status_words[3] <= {status_short_listen, 10'd0, status_chirps_per_elev};
            // Word 4 layout (PR-AB.b expanded commit 5):
            //   [31:28] agc_current_gain
            //   [27:20] agc_peak_magnitude
            //   [19:12] agc_saturation_count
            //   [11]    agc_enable
            //   [10]    chirps_mismatch (TX-G)
            //   [9:2]   alpha_soft echo (Q4.4)
            //   [1]     beam_handshake_watchdog_fired (sticky)
            //   [0]     reserved 0
            status_words[4] <= {status_agc_current_gain,
                                status_agc_peak_magnitude,
                                status_agc_saturation_count,
                                status_agc_enable,
                                status_chirps_mismatch,
                                alpha_soft_sync_1,
                                beam_handshake_wd_sync_1,
                                1'd0};
            // Word 5: {frame_drop_count[31:25], self_test_busy[24], 8'd0,
            //          self_test_detail[15:8], reserved[7], cic_fir_overrun[6],
            //          range_decim_watchdog[5], self_test_flags[4:0]}
            status_words[5] <= {frame_drop_sync_1, status_self_test_busy,
                                8'd0, status_self_test_detail,
                                1'd0,
                                ddc_cic_fir_overrun_sync_1,
                                range_decim_watchdog_sync_1,
                                status_self_test_flags};
            // PR-G word 6: {detect_count_cand[15:0], detect_threshold_soft[15:0]}
            status_words[6] <= {det_count_cand_sync_1,
                                (det_thr_soft_sync_1[16] ? 16'hFFFF
                                                         : det_thr_soft_sync_1[15:0])};
            // M-5 word 7: {medium_chirp[15:0], medium_listen[15:0]}
            status_words[7] <= {status_medium_chirp, status_medium_listen};
        end

        // ================================================================
        // READ FSM - Host -> FPGA command path (32-bit single-word read)
        // ================================================================
        case (rd_state)
            RD_IDLE: begin
                if (wr_state == WR_IDLE && !ft601_rxf) begin
                    ft601_oe_n    <= 1'b0;
                    ft601_data_oe <= 1'b0;
                    rd_state      <= RD_OE_ASSERT;
                end
            end

            RD_OE_ASSERT: begin
                if (!ft601_rxf) begin
                    ft601_rd_n <= 1'b0;
                    rd_state   <= RD_READING;
                end else begin
                    ft601_oe_n <= 1'b1;
                    rd_state   <= RD_IDLE;
                end
            end

            RD_READING: begin
                rd_captured <= ft601_data;
                ft601_rd_n  <= 1'b1;
                rd_state    <= RD_DEASSERT;
            end

            RD_DEASSERT: begin
                ft601_oe_n <= 1'b1;
                rd_state   <= RD_PROCESS;
            end

            RD_PROCESS: begin
                // Command word format: {opcode[31:24], addr[23:16], value[15:0]}.
                // Preserved from pre-PR-AD FT601 driver so the host-side encoding
                // stays unchanged; only the FPGA->host TX direction was reworked
                // in PR-AD. Host->FPGA byte-order parity is a future concern if
                // the host ever needs symmetric encoding across both drivers.
                cmd_data   <= rd_captured;
                cmd_opcode <= rd_captured[31:24];
                cmd_addr   <= rd_captured[23:16];
                cmd_value  <= rd_captured[15:0];
                cmd_valid  <= 1'b1;
                rd_state   <= RD_IDLE;
            end

            default: rd_state <= RD_IDLE;
        endcase

        // ================================================================
        // WRITE FSM - Bulk per-frame transfer (ft601_clk_in domain)
        // ================================================================
        // Default: no emit this cycle.
        byte_grant    = 1'b0;
        pack_emit_now = 1'b0;
        ft601_wr_n    <= 1'b1;

        if (rd_state == RD_IDLE) begin
            case (wr_state)
                WR_IDLE: begin
                    ft601_data_oe <= 1'b0;
                    pack_lane     <= 2'd0;
                    pending_word_lo <= 24'd0;

                    if (status_req_ft && ft601_rxf) begin
                        wr_state    <= WR_STATUS_SEND;
                        wr_byte_idx <= 16'd0;
                    end
                    else if (frame_ready_ft && ft601_rxf) begin
                        wr_state             <= WR_FRAME_HDR;
                        wr_byte_idx          <= 16'd0;
                        dop_range_idx        <= {RANGE_BIN_BITS{1'b0}};
                        dop_doppler_idx      <= {DOPPLER_BIN_BITS{1'b0}};
                        range_rd_idx         <= {RANGE_BIN_BITS{1'b0}};
                        range_rd_addr        <= {RANGE_BIN_BITS{1'b0}};
                        det_range_idx        <= {RANGE_BIN_BITS{1'b0}};
                        det_doppler_byte_idx <= 4'd0;
                        detect_rd_addr       <= {DETECT_BYTE_ADDR_W{1'b0}};
                        mag_rd_addr          <= {FRAME_ADDR_W{1'b0}};
                        wr_byte_phase        <= 1'b0;
                    end
                end

                // ---- Frame header: 9 bytes ----
                WR_FRAME_HDR: begin
                    if (!ft601_txe) begin
                        ft601_data_oe <= 1'b1;
                        byte_grant    = 1'b1;

                        if (wr_byte_idx[3:0] == 4'd8) begin
                            wr_byte_idx <= 16'd0;
                            wr_byte_phase <= 1'b0;
                            // PR-Z A6 (Bug B) fix: pre-load detect read pipeline
                            // if next state is WR_DETECT_DATA (skipping range/doppler).
                            det_doppler_byte_idx <= 4'd1;
                            detect_rd_addr       <= {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
                            if (stream_flags_snapshot[0])
                                wr_state <= WR_RANGE_DATA;
                            else if (stream_flags_snapshot[1])
                                wr_state <= WR_DOPPLER_DATA;
                            else if (stream_flags_snapshot[2])
                                wr_state <= WR_DETECT_DATA;
                            else
                                wr_state <= WR_FRAME_FOOTER;
                        end else begin
                            wr_byte_idx <= wr_byte_idx + 16'd1;
                        end
                    end
                end

                // ---- Range profile: 512 x 2 = 1024 bytes ----
                WR_RANGE_DATA: begin
                    if (!ft601_txe) begin
                        ft601_data_oe <= 1'b1;
                        byte_grant    = 1'b1;

                        // PR-AA: addr advance lives at end of phase 0 (MSB emit).
                        // BRAM 1-cycle read latency means addr must advance at
                        // phase 0 so next pair's MSB read sees the new cell.
                        if (!wr_byte_phase) begin
                            wr_byte_phase <= 1'b1;
                            range_rd_idx  <= range_rd_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                            range_rd_addr <= range_rd_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                        end else begin
                            wr_byte_phase <= 1'b0;
                        end

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == RANGE_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx     <= 16'd0;
                            wr_byte_phase   <= 1'b0;
                            dop_range_idx   <= {RANGE_BIN_BITS{1'b0}};
                            dop_doppler_idx <= {DOPPLER_BIN_BITS{1'b0}};
                            mag_rd_addr     <= {FRAME_ADDR_W{1'b0}};
                            det_doppler_byte_idx <= 4'd1;
                            detect_rd_addr       <= {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
                            if (stream_flags_snapshot[1])
                                wr_state <= WR_DOPPLER_DATA;
                            else if (stream_flags_snapshot[2])
                                wr_state <= WR_DETECT_DATA;
                            else
                                wr_state <= WR_FRAME_FOOTER;
                        end
                    end
                end

                // ---- Doppler magnitude: 512 x 48 x 2 = 49152 bytes ----
                WR_DOPPLER_DATA: begin
                    if (!ft601_txe) begin
                        ft601_data_oe <= 1'b1;
                        byte_grant    = 1'b1;

                        // PR-AA fix: advance address at phase 0 (MSB) so BRAM
                        // has 2 cycles before next pair's MSB read.
                        if (!wr_byte_phase) begin
                            wr_byte_phase <= 1'b1;
                            if (dop_doppler_idx == DOP_BIN_LAST) begin
                                dop_doppler_idx <= {DOPPLER_BIN_BITS{1'b0}};
                                dop_range_idx   <= dop_range_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                                mag_rd_addr     <= {dop_range_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1},
                                                    {DOPPLER_BIN_BITS{1'b0}}};
                            end else begin
                                dop_doppler_idx <= dop_doppler_idx + {{(DOPPLER_BIN_BITS-1){1'b0}}, 1'b1};
                                mag_rd_addr     <= {dop_range_idx,
                                                    dop_doppler_idx + {{(DOPPLER_BIN_BITS-1){1'b0}}, 1'b1}};
                            end
                        end else begin
                            wr_byte_phase <= 1'b0;
                        end

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == DOPPLER_MAG_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx          <= 16'd0;
                            wr_byte_phase        <= 1'b0;
                            det_range_idx        <= {RANGE_BIN_BITS{1'b0}};
                            det_doppler_byte_idx <= 4'd1;
                            detect_rd_addr       <= {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
                            if (stream_flags_snapshot[2])
                                wr_state <= WR_DETECT_DATA;
                            else
                                wr_state <= WR_FRAME_FOOTER;
                        end
                    end
                end

                // ---- Detection flags: 512 x 12 = 6144 bytes (PR-G, 2-bit dense) ----
                WR_DETECT_DATA: begin
                    if (!ft601_txe) begin
                        ft601_data_oe <= 1'b1;
                        byte_grant    = 1'b1;

                        // 1-byte per cycle (BRAM read pre-loaded at state entry)
                        if (det_doppler_byte_idx == DET_BYTE_LAST_PER_RANGE) begin
                            det_doppler_byte_idx <= 4'd0;
                            det_range_idx        <= det_range_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                            detect_rd_addr       <= {det_range_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1},
                                                     4'd0};
                        end else begin
                            det_doppler_byte_idx <= det_doppler_byte_idx + 4'd1;
                            detect_rd_addr       <= {det_range_idx,
                                                     det_doppler_byte_idx + 4'd1};
                        end

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == DETECT_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx <= 16'd0;
                            wr_state    <= WR_FRAME_FOOTER;
                        end
                    end
                end

                // ---- Frame footer: 1 byte (BE=0001 partial emit) ----
                WR_FRAME_FOOTER: begin
                    if (!ft601_txe) begin
                        ft601_data_oe <= 1'b1;
                        byte_grant    = 1'b1;
                        wr_state      <= WR_DONE;
                    end
                end

                // ---- Status packet: 34 bytes (M-5: 8 x 32-bit words) ----
                WR_STATUS_SEND: begin
                    if (!ft601_txe) begin
                        ft601_data_oe <= 1'b1;
                        byte_grant    = 1'b1;

                        if (wr_byte_idx[5:0] == STATUS_PKT_LEN - 6'd1) begin
                            wr_byte_idx <= 16'd0;
                            wr_state    <= WR_DONE;
                        end else begin
                            wr_byte_idx <= wr_byte_idx + 16'd1;
                        end
                    end
                end

                WR_DONE: begin
                    ft601_data_oe  <= 1'b0;
                    wr_done_toggle <= ~wr_done_toggle;  // AUDIT-C12
                    wr_state       <= WR_IDLE;
                    pack_lane      <= 2'd0;
                end

                default: wr_state <= WR_IDLE;
            endcase
        end

        // ================================================================
        // Pack-and-emit (runs whenever byte_grant asserts)
        // ================================================================
        // byte_grant set above by the WR FSM means: this cycle, byte_now
        // is the next byte to drop into the FT601 stream. Either capture
        // into pending_word_lo[pack_lane] and advance lane, or emit the
        // assembled word (full or partial).
        if (byte_grant) begin
            if (pack_lane == 2'd3 || is_section_end) begin
                pack_emit_now = 1'b1;
                ft601_wr_n    <= 1'b0;
                case (pack_lane)
                    2'd0: begin
                        ft601_data_out <= {24'd0, byte_now};
                        ft601_be       <= 4'b0001;
                    end
                    2'd1: begin
                        ft601_data_out <= {16'd0, byte_now, pending_word_lo[7:0]};
                        ft601_be       <= 4'b0011;
                    end
                    2'd2: begin
                        ft601_data_out <= {8'd0, byte_now, pending_word_lo[15:8], pending_word_lo[7:0]};
                        ft601_be       <= 4'b0111;
                    end
                    2'd3: begin
                        ft601_data_out <= {byte_now, pending_word_lo[23:16],
                                           pending_word_lo[15:8], pending_word_lo[7:0]};
                        ft601_be       <= 4'b1111;
                    end
                    default: ;  // all 4 lane values covered above
                endcase
                pack_lane       <= 2'd0;
                pending_word_lo <= 24'd0;
            end else begin
                case (pack_lane)
                    2'd0: pending_word_lo[7:0]   <= byte_now;
                    2'd1: pending_word_lo[15:8]  <= byte_now;
                    2'd2: pending_word_lo[23:16] <= byte_now;
                    default: ;  // unreachable
                endcase
                pack_lane <= pack_lane + 2'd1;
            end
        end
    end
end

// ============================================================================
// VESTIGIAL FT601 OUTPUTS - kept for 200T board port compatibility.
// Driven to 1 inside the always block; never carry meaningful data.
// ============================================================================
// (ft601_txe_n / ft601_rxf_n already reset to 1 and never reassigned in the
// always block above, so they stay 1 for the life of the design.)

// ============================================================================
// FT601 CLOCK OUTPUT FORWARDING (ODDR)
// ============================================================================
// Forward ft601_clk_in back out via ODDR so the forwarded pin clock matches
// the data outputs' insertion delay.
`ifndef SIMULATION
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"),
    .INIT(1'b0),
    .SRTYPE("SYNC")
) oddr_ft601_clk (
    .Q(ft601_clk_out),
    .C(ft601_clk_in),
    .CE(1'b1),
    .D1(1'b1),
    .D2(1'b0),
    .R(1'b0),
    .S(1'b0)
);
`else
assign ft601_clk_out = ft601_clk_in;
`endif

// ============================================================================
// SIMULATION ONLY: BRAM init
// ============================================================================
`ifdef SIMULATION
integer init_idx;
initial begin
    for (init_idx = 0; init_idx <= DETECT_BYTE_LAST; init_idx = init_idx + 1)
        detect_bram[init_idx] = 8'd0;
    for (init_idx = 0; init_idx < FRAME_CELLS; init_idx = init_idx + 1)
        doppler_mag_bram[init_idx] = 16'd0;
    for (init_idx = 0; init_idx < NUM_RANGE_BINS; init_idx = init_idx + 1)
        range_bram[init_idx] = 16'd0;
end
`endif

// ============================================================================
// TX-N9: payload-hold checker (simulation only)
// cmd_* outputs feed downstream CDC; safety property = only change on
// cmd_valid rise. Mirrors FT2232H driver.
// ============================================================================
`ifdef SIMULATION
reg [31:0] cmd_data_prev_n9;
reg  [7:0] cmd_opcode_prev_n9;
reg  [7:0] cmd_addr_prev_n9;
reg [15:0] cmd_value_prev_n9;
reg        cmd_valid_prev_n9;

always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n) begin
        cmd_data_prev_n9   <= 32'd0;
        cmd_opcode_prev_n9 <= 8'd0;
        cmd_addr_prev_n9   <= 8'd0;
        cmd_value_prev_n9  <= 16'd0;
        cmd_valid_prev_n9  <= 1'b0;
    end else begin
        if (!cmd_valid && !cmd_valid_prev_n9) begin
            if (cmd_data   !== cmd_data_prev_n9)
                $display("[ASSERT FAIL] TX-N9: cmd_data changed while cmd_valid=0 (%h -> %h)",
                         cmd_data_prev_n9, cmd_data);
            if (cmd_opcode !== cmd_opcode_prev_n9)
                $display("[ASSERT FAIL] TX-N9: cmd_opcode changed while cmd_valid=0 (%h -> %h)",
                         cmd_opcode_prev_n9, cmd_opcode);
            if (cmd_addr   !== cmd_addr_prev_n9)
                $display("[ASSERT FAIL] TX-N9: cmd_addr changed while cmd_valid=0 (%h -> %h)",
                         cmd_addr_prev_n9, cmd_addr);
            if (cmd_value  !== cmd_value_prev_n9)
                $display("[ASSERT FAIL] TX-N9: cmd_value changed while cmd_valid=0 (%h -> %h)",
                         cmd_value_prev_n9, cmd_value);
        end
        cmd_data_prev_n9   <= cmd_data;
        cmd_opcode_prev_n9 <= cmd_opcode;
        cmd_addr_prev_n9   <= cmd_addr;
        cmd_value_prev_n9  <= cmd_value;
        cmd_valid_prev_n9  <= cmd_valid;
    end
end
`endif

// ============================================================================
// AUDIT-S22: cfar_valid-vs-RMW-busy checker (simulation only)
// Mirrors FT2232H driver.
// ============================================================================
`ifdef SIMULATION
always @(posedge clk) begin
    if (reset_n && cfar_valid && frame_filling && !detect_clearing &&
        detect_rmw_state != 2'd0) begin
        $display("[ASSERT FAIL] AUDIT-S22: cfar_valid arrived while RMW busy (state=%0d) - detection at range_bin=%0d doppler_bin=%0d dropped",
                 detect_rmw_state, range_bin_in, doppler_bin_in);
    end
end
`endif

endmodule
