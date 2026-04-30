// ============================================================================
// radar_params.vh — Single Source of Truth for AERIS-10 FPGA Parameters
// ============================================================================
//
// ALL modules in the FPGA processing chain MUST `include this file instead of
// hardcoding range bins, segment counts, chirp samples, or timing values.
//
// This file uses `define macros (not localparam) so it can be included at any
// scope. Each consuming module should include this file inside its body and
// optionally alias macros to localparams for readability.
//
// BOARD VARIANTS:
//   SUPPORT_LONG_RANGE = 0  (50T, USB_MODE=1)  — 3 km mode only
//   SUPPORT_LONG_RANGE = 1  (200T, USB_MODE=0) — 3 km + 20 km modes
//
// RADAR MODES (runtime, via host_radar_mode register, opcode 0x01):
//   2'b00 = STM32 pass-through (production — STM32 controls chirp timing)
//   2'b01 = Auto-scan 3 km     (FPGA-timed, short chirps only)
//   2'b10 = Single-chirp debug (one long chirp per trigger)
//   2'b11 = Reserved / idle
//
// RANGE MODES (runtime, via host_range_mode register, opcode 0x20):
//   2'b00 = 3 km   (default — pass-through treats all chirps as short)
//   2'b01 = Long-range (pass-through: first half long, second half short)
//   2'b10 = Reserved
//   2'b11 = Reserved
//
// USAGE:
//   `include "radar_params.vh"
//   Then reference `RP_FFT_SIZE, `RP_NUM_RANGE_BINS, etc.
//
// PHYSICAL CONSTANTS (derived from hardware):
//   ADC clock:           400 MSPS
//   CIC decimation:      4x
//   Processing rate:     100 MSPS (post-DDC)
//   Range per sample:    c / (2 * 100e6) = 1.5 m
//   FFT size:            2048
//   Decimation factor:   4  (2048 FFT bins -> 512 output range bins)
//   Range per dec. bin:  1.5 m * 4 = 6.0 m
//   Max range (3 km):    512 * 6.0 = 3072 m
//   Carrier frequency:   10.5 GHz
//   IF frequency:        120 MHz
//
// CHIRP BANDWIDTH (Phase 1 target — currently 20 MHz, planned 30 MHz):
//   Range resolution:    c / (2 * BW)
//     20 MHz -> 7.5 m
//     30 MHz -> 5.0 m
//   NOTE: Range resolution is independent of range-per-bin. Resolution
//   determines the minimum separation between two targets; range-per-bin
//   determines the spatial sampling grid.
// ============================================================================

`ifndef RADAR_PARAMS_VH
`define RADAR_PARAMS_VH

// ============================================================================
// BOARD VARIANT — set at synthesis time, NOT runtime
// ============================================================================
// Default to 50T (conservative). Override in top-level or synthesis script:
//   +define+SUPPORT_LONG_RANGE
// or via Vivado: set_property verilog_define {SUPPORT_LONG_RANGE} [current_fileset]

// Note: SUPPORT_LONG_RANGE is a flag define (ifdef/ifndef), not a value.
// `ifndef SUPPORT_LONG_RANGE means 50T (no long range).
// `ifdef SUPPORT_LONG_RANGE means 200T (long range supported).

// ============================================================================
// FFT AND PROCESSING CONSTANTS (fixed, both modes)
// ============================================================================

`define RP_FFT_SIZE             2048    // Range FFT points per segment
`define RP_LOG2_FFT_SIZE        11      // log2(2048)
`define RP_OVERLAP_SAMPLES      128     // Overlap between adjacent segments
`define RP_SEGMENT_ADVANCE      1920    // FFT_SIZE - OVERLAP = 2048 - 128
`define RP_DECIMATION_FACTOR    4       // Range bin decimation (2048 -> 512)
`define RP_NUM_RANGE_BINS       512     // FFT_SIZE / DECIMATION_FACTOR
`define RP_RANGE_BIN_BITS       9       // ceil(log2(512))
`define RP_DOPPLER_FFT_SIZE     16      // Per sub-frame Doppler FFT (scan mode)
`define RP_DOPPLER_FFT_SIZE_TRACK 64    // Track-mode dwell N (xfft_64, single waveform)
`define RP_CHIRPS_PER_FRAME     32      // (LEGACY: scan-only 2-subframe; bumped to 48 in PR-F)
`define RP_CHIRPS_PER_SUBFRAME  16      // Chirps per Doppler sub-frame
`define RP_NUM_DOPPLER_BINS     32      // (LEGACY: 2 sub-frames * 16 = 32; bumped to 48 in PR-F)
`define RP_DATA_WIDTH           16      // ADC/processing data width

// 3-ladder waveform identity (replaces 1-bit use_long_chirp rail in PR-C onward)
// `define RP_WAVE_<NAME> values are 2-bit waveform selectors carried on
// `wave_sel[1:0]` at every chirp boundary. RESERVED is a hard error.
`define RP_WAVE_SEL_WIDTH       2
`define RP_WAVE_SHORT           2'b00   // 1 µs    (3 km build workhorse)
`define RP_WAVE_MEDIUM          2'b01   // 5 µs    (mid-range fill, 0.75–8 km)
`define RP_WAVE_LONG            2'b10   // 30 µs   (legal but unused on 50T; 200T uses for 20 km)
`define RP_WAVE_RESERVED        2'b11

// Sub-frame layout. Frame = NUM_SUBFRAMES × CHIRPS_PER_SUBFRAME chirps.
// Scan mode uses 3 sub-frames (SHORT, MEDIUM, LONG), each running its own
// N=16 Doppler FFT. Track mode pins the frame to one waveform and runs N=64.
`define RP_NUM_SUBFRAMES        3
`define RP_SUBFRAME_ID_WIDTH    2       // ceil(log2(3))
`define RP_DOPPLER_BIN_WIDTH    6       // {sub_frame[1:0], bin[3:0]}

// Adaptive-escalation detection class (CFAR output — 2-class instead of 1-flag)
// Replaces detect_flag (1 bit) when PR-F lands.
`define RP_DETECT_CLASS_WIDTH   2
`define RP_DETECT_NONE          2'b00   // below soft threshold
`define RP_DETECT_CANDIDATE     2'b01   // above soft, below confirm — host re-cues
`define RP_DETECT_CONFIRMED     2'b10   // above confirm threshold — track-eligible
`define RP_DETECT_RSVD          2'b11

// ============================================================================
// 3 KM MODE PARAMETERS (both 50T and 200T)
// ============================================================================

`define RP_LONG_CHIRP_SAMPLES_3KM   3000    // 30 us at 100 MSPS
`define RP_LONG_SEGMENTS_3KM        2       // ceil((3000-2048)/1920) + 1 = 2
`define RP_SHORT_CHIRP_SAMPLES      50      // 0.5 us at 100 MSPS (same both modes)
`define RP_SHORT_SEGMENTS           1       // Single segment for short chirp

// Derived 3 km limits
`define RP_MAX_RANGE_3KM            3072    // 512 bins * 6 m = 3072 m

// ============================================================================
// 20 KM MODE PARAMETERS (200T only — Phase 2)
// ============================================================================

`define RP_LONG_CHIRP_SAMPLES_20KM  13700   // 137 us at 100 MSPS (= listen window)
`define RP_LONG_SEGMENTS_20KM       8       // 1 + ceil((13700-2048)/1920) = 1 + 7 = 8
`define RP_OUTPUT_RANGE_BINS_20KM   4096    // 8 segments * 512 dec. bins each

// Derived 20 km limits
`define RP_MAX_RANGE_20KM           24576   // 4096 bins * 6 m = 24576 m

// ============================================================================
// MAX VALUES (for sizing buffers — compile-time, based on board variant)
// ============================================================================

`ifdef SUPPORT_LONG_RANGE
  `define RP_MAX_SEGMENTS           8
  `define RP_MAX_OUTPUT_BINS        4096
  `define RP_MAX_CHIRP_SAMPLES      13700
`else
  `define RP_MAX_SEGMENTS           2
  `define RP_MAX_OUTPUT_BINS        512
  `define RP_MAX_CHIRP_SAMPLES      3000
`endif

// ============================================================================
// BIT WIDTHS (derived from MAX values)
// ============================================================================

// Segment index: ceil(log2(MAX_SEGMENTS))
//   50T:  log2(2) = 1 bit  (use 2 for safety)
//   200T: log2(8) = 3 bits
`ifdef SUPPORT_LONG_RANGE
  `define RP_SEGMENT_IDX_WIDTH      3
  `define RP_RANGE_BIN_WIDTH_MAX    12      // ceil(log2(4096))
  `define RP_DOPPLER_MEM_ADDR_W     17      // ceil(log2(4096*32)) = 17
  `define RP_CFAR_MAG_ADDR_W        17      // ceil(log2(4096*32)) = 17
`else
  `define RP_SEGMENT_IDX_WIDTH      2
  `define RP_RANGE_BIN_WIDTH_MAX    9       // ceil(log2(512))
  `define RP_DOPPLER_MEM_ADDR_W     14      // ceil(log2(512*32)) = 14
  `define RP_CFAR_MAG_ADDR_W        14      // ceil(log2(512*32)) = 14
`endif

// Derived depths (for memory declarations)
// Usage: reg [15:0] mem [0:`RP_DOPPLER_MEM_DEPTH-1];
`define RP_DOPPLER_MEM_DEPTH    (`RP_MAX_OUTPUT_BINS * `RP_CHIRPS_PER_FRAME)
`define RP_CFAR_MAG_DEPTH       (`RP_MAX_OUTPUT_BINS * `RP_NUM_DOPPLER_BINS)

// ============================================================================
// CHIRP TIMING DEFAULTS (100 MHz clock cycles)
// ============================================================================
// Reset defaults for host-configurable timing registers.
// Match radar_mode_controller.v parameters and main.cpp STM32 defaults.
//
// 3-LADDER (3 km build): SHORT 1 µs, MEDIUM 5 µs, LONG 30 µs. Same listen
// budget across waveforms (~175 µs PRI) keeps Doppler resolution uniform.
// LONG kept on 50T as legal-but-unused so 200T spin-up doesn't need a
// second wave through the codebase.

`define RP_DEF_LONG_CHIRP_CYCLES    3000    // 30 us
`define RP_DEF_LONG_LISTEN_CYCLES   13700   // 137 us
`define RP_DEF_GUARD_CYCLES         17540   // 175.4 us
`define RP_DEF_SHORT_CHIRP_CYCLES   50      // 0.5 us — LEGACY; PR-E switches to 100 (1 µs)
`define RP_DEF_SHORT_LISTEN_CYCLES  17450   // 174.5 us
`define RP_DEF_CHIRPS_PER_ELEV      32      // LEGACY; bumped to 48 in PR-F

// 3-ladder defaults — added in PR-A, consumed by chirp_scheduler in PR-D.
`define RP_DEF_SHORT_CHIRP_CYCLES_V2   100     // 1 µs at 100 MHz (was 0.5 µs)
`define RP_DEF_SHORT_LISTEN_CYCLES_V2  17400   // PRI 175 µs - chirp - guard slack
`define RP_DEF_MEDIUM_CHIRP_CYCLES     500     // 5 µs at 100 MHz
`define RP_DEF_MEDIUM_LISTEN_CYCLES    17000   // PRI 175 µs - chirp - guard slack
// LONG defaults reuse RP_DEF_LONG_CHIRP_CYCLES / RP_DEF_LONG_LISTEN_CYCLES
`define RP_DEF_CHIRPS_PER_SUBFRAME     16      // 16 per sub-frame, 3 sub-frames = 48 frame
`define RP_DEF_SUBFRAME_ENABLE         3'b111  // SHORT|MEDIUM|LONG all on by default
`define RP_DEF_TRACK_WATCHDOG_FRAMES   8'd5    // frames without track cmd before scan-fallback
`define RP_DEF_TRACK_CHIRP_COUNT       9'd64   // default track-mode dwell N
`define RP_DEF_CFAR_ALPHA_SOFT         8'h18   // 1.5 in Q4.4 — soft threshold for candidates
                                                // (Pfa_soft ≈ 10⁻⁵; confirm Pfa ≈ 10⁻⁶ at α=3.0)

// ============================================================================
// BLIND ZONE CONSTANTS (informational, for comments and GUI)
// ============================================================================
// Long chirp blind zone:  c * 30 us / 2 = 4500 m
// Short chirp blind zone: c * 0.5 us / 2 = 75 m

`define RP_LONG_BLIND_ZONE_M        4500
`define RP_SHORT_BLIND_ZONE_M       75

// ============================================================================
// PHYSICAL CONSTANTS (integer-scaled for Verilog — use in comments/assertions)
// ============================================================================
// Range per ADC sample:     1.5 m  (stored as 15 in units of 0.1 m)
// Range per decimated bin:  6.0 m  (stored as 60 in units of 0.1 m)
// Processing rate:         100 MSPS

`define RP_RANGE_PER_SAMPLE_DM      15      // 1.5 m in decimeters
`define RP_RANGE_PER_BIN_DM         60      // 6.0 m in decimeters
`define RP_PROCESSING_RATE_MHZ      100

// ============================================================================
// AGC DEFAULTS
// ============================================================================
`define RP_DEF_AGC_TARGET           200
`define RP_DEF_AGC_ATTACK           1
`define RP_DEF_AGC_DECAY            1
`define RP_DEF_AGC_HOLDOFF          4

// ============================================================================
// CFAR DEFAULTS
// ============================================================================
`define RP_DEF_CFAR_GUARD           2
`define RP_DEF_CFAR_TRAIN           8
`define RP_DEF_CFAR_ALPHA           8'h30   // 3.0 in Q4.4
`define RP_DEF_CFAR_MODE            2'b00   // CA-CFAR

// ============================================================================
// DETECTION DEFAULTS
// ============================================================================
`define RP_DEF_DETECT_THRESHOLD     10000

// ============================================================================
// RADAR MODE ENCODING (host_radar_mode, opcode 0x01)
// ============================================================================
`define RP_MODE_STM32_PASSTHROUGH   2'b00
`define RP_MODE_AUTO_3KM            2'b01
`define RP_MODE_SINGLE_DEBUG        2'b10
`define RP_MODE_RESERVED            2'b11

// ============================================================================
// RANGE MODE ENCODING (host_range_mode, opcode 0x20)
// ============================================================================
`define RP_RANGE_MODE_3KM           2'b00
`define RP_RANGE_MODE_LONG          2'b01
`define RP_RANGE_MODE_RSVD2         2'b10
`define RP_RANGE_MODE_RSVD3         2'b11

// ============================================================================
// RADAR MODE ENCODING — TRACK extension (host_radar_mode, opcode 0x01)
// ============================================================================
// Mode 11 ("RESERVED" until PR-D) becomes TRACK mode: scheduler dwells one
// beam, one waveform, host_track_chirp_count chirps. Doppler runs xfft_64.
// RP_MODE_RESERVED below is renamed in-place for clarity.
`define RP_MODE_TRACK               2'b11

// ============================================================================
// STREAM CONTROL (host_stream_control, opcode 0x04, 6-bit)
// ============================================================================
// Bits [2:0]: Stream enable mask
//   Bit 0 = range profile stream
//   Bit 1 = doppler map stream
//   Bit 2 = cfar/detection stream
// Bits [5:3]: Stream format control
//   Bit 3 = mag_only    (0=I/Q pairs, 1=Manhattan magnitude only)
//   Bit 4 = sparse_det  (0=dense detection flags, 1=sparse detection list)
//   Bit 5 = reserved (was frame_decimate, not needed with mag-only fitting)
`define RP_STREAM_CTRL_DEFAULT      6'b001_111  // all streams, mag-only mode

`endif // RADAR_PARAMS_VH
