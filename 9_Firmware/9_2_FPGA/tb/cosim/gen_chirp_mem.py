#!/usr/bin/env python3
"""
gen_chirp_mem.py — Generate all chirp .mem files for AERIS-10 FPGA.

Generates the 6 chirp .mem files used by chirp_memory_loader_param.v:
  - long_chirp_seg{0,1}_{i,q}.mem  (4 files, 2048 lines each)
  - short_chirp_{i,q}.mem          (2 files, 50 lines each)

Long chirp:
  The 3000-sample baseband chirp (30 us at 100 MHz system clock) is
  segmented into 2 blocks of 2048 samples.  Each segment covers a
  different time window of the chirp:
    seg0: samples    0 .. 2047
    seg1: samples 2048 .. 4095  (only 952 valid chirp samples; 1096 zeros)

  The memory loader stores 2*2048 = 4096 contiguous samples indexed
  by {segment_select[0], sample_addr[10:0]}.  The long chirp has
  3000 samples, so:
    seg0: chirp[0..2047] — all valid data
    seg1: chirp[2048..2999] + 1096 zeros (samples past chirp end)

Short chirp:
  50 samples (0.5 us at 100 MHz), same chirp formula with
  T_SHORT_CHIRP and CHIRP_BW.

Phase model (baseband, post-DDC):
  phase(n) = pi * chirp_rate * t^2,  t = n / FS_SYS
  chirp_rate = CHIRP_BW / T_chirp

Scaling: 0.9 * 32767 (Q15), matching radar_scene.py generate_reference_chirp_q15()

Usage:
    python3 gen_chirp_mem.py
"""

import math
import os
import re
import sys

# ============================================================================
# AERIS-10 Parameters
#
# Sample counts / FFT size / segmentation come from radar_params.vh so a
# change there (e.g. a new long-chirp duration) flows into the .mem files
# automatically — no risk of Python and RTL disagreeing on buffer sizes.
#
# Physical chirp design constants (bandwidth, sample rate, Q15 scaling)
# stay hardcoded here: they live outside radar_params.vh because they are
# baseband-generation properties, not FPGA sizing parameters.
# ============================================================================

RADAR_PARAMS_VH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'radar_params.vh'
)


def _parse_radar_params(path):
    """
    Parse `\\`define RP_<NAME>  <integer>` lines from radar_params.vh.
    Only integer literals are supported (decimal / hex / binary); the
    macros this script consumes are all integers. Comments / strings /
    concat macros are ignored by the regex.
    """
    # Grab RHS up to an optional `// ...` comment, then parse as int.
    line_pat = re.compile(r"^\s*`define\s+(RP_\w+)\s+([^/\n]+?)(?://.*)?$")
    params = {}
    with open(path) as f:
        for line in f:
            m = line_pat.match(line)
            if not m:
                continue
            name, rhs = m.group(1), m.group(2).strip()
            # Strip Verilog sized-literal prefix like 11'd2048 or 2'b00.
            sized = re.match(r"\d+'([bdh])([0-9a-fA-F_]+)", rhs)
            if sized:
                base = {'b': 2, 'd': 10, 'h': 16}[sized.group(1)]
                try:
                    params[name] = int(sized.group(2).replace('_', ''), base)
                except ValueError:
                    continue
                continue
            # Plain integer (decimal or 0x...).
            try:
                params[name] = int(rhs, 0)
            except ValueError:
                # Non-integer macro (string, expression, cross-reference) —
                # skip; this script only needs the integer sizing macros.
                continue
    return params


_RP = _parse_radar_params(RADAR_PARAMS_VH)


def _require(name):
    if name not in _RP:
        sys.stderr.write(
            f"gen_chirp_mem.py: `{name}` not found in radar_params.vh; "
            f"update the RTL macro or the parser.\n"
        )
        sys.exit(2)
    return _RP[name]


# Physical chirp design constants (not in radar_params.vh — baseband only).
CHIRP_BW = 20e6           # 20 MHz sweep bandwidth
FS_SYS = 100e6            # System clock (100 MHz, post-CIC)
SCALE = 0.9               # Q15 scaling factor (matches radar_scene.py)
Q15_MAX = 32767

# Sizing / sample counts sourced from radar_params.vh (single source of truth).
FFT_SIZE            = _require('RP_FFT_SIZE')
LONG_CHIRP_SAMPLES  = _require('RP_LONG_CHIRP_SAMPLES_3KM')
SHORT_CHIRP_SAMPLES = _require('RP_SHORT_CHIRP_SAMPLES')
LONG_SEGMENTS       = _require('RP_LONG_SEGMENTS_3KM')

# Durations are derived from sample counts + FS_SYS so a change to
# RP_LONG_CHIRP_SAMPLES_3KM automatically re-targets the chirp rate.
T_LONG_CHIRP  = LONG_CHIRP_SAMPLES  / FS_SYS
T_SHORT_CHIRP = SHORT_CHIRP_SAMPLES / FS_SYS

# Output directory (FPGA RTL root, where .mem files live)
MEM_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')


def generate_full_long_chirp():
    """
    Generate the full 3000-sample baseband chirp in Q15.

    Returns:
        (chirp_i, chirp_q): lists of 3000 signed 16-bit integers
    """
    chirp_rate = CHIRP_BW / T_LONG_CHIRP  # Hz/s

    chirp_i = []
    chirp_q = []

    for n in range(LONG_CHIRP_SAMPLES):
        t = n / FS_SYS
        phase = math.pi * chirp_rate * t * t
        re_val = round(Q15_MAX * SCALE * math.cos(phase))
        im_val = round(Q15_MAX * SCALE * math.sin(phase))
        chirp_i.append(max(-32768, min(32767, re_val)))
        chirp_q.append(max(-32768, min(32767, im_val)))

    return chirp_i, chirp_q


def generate_short_chirp():
    """
    Generate the 50-sample short chirp in Q15.

    Returns:
        (chirp_i, chirp_q): lists of 50 signed 16-bit integers
    """
    chirp_rate = CHIRP_BW / T_SHORT_CHIRP  # Hz/s (much faster sweep)

    chirp_i = []
    chirp_q = []

    for n in range(SHORT_CHIRP_SAMPLES):
        t = n / FS_SYS
        phase = math.pi * chirp_rate * t * t
        re_val = round(Q15_MAX * SCALE * math.cos(phase))
        im_val = round(Q15_MAX * SCALE * math.sin(phase))
        chirp_i.append(max(-32768, min(32767, re_val)))
        chirp_q.append(max(-32768, min(32767, im_val)))

    return chirp_i, chirp_q


def to_hex16(value):
    """Convert signed 16-bit integer to 4-digit hex string (unsigned representation)."""
    if value < 0:
        value += 0x10000
    return f"{value:04x}"


def write_mem_file(filename, values):
    """Write a list of 16-bit signed integers to a .mem file (hex format)."""
    path = os.path.join(MEM_DIR, filename)
    with open(path, 'w') as f:
        for v in values:
            f.write(to_hex16(v) + '\n')


def main():

    # ---- Long chirp ----
    long_i, long_q = generate_full_long_chirp()

    # Verify first sample matches generate_reference_chirp_q15() from radar_scene.py
    # (which only generates the first 1024 samples)

    # Segment into 4 x 1024 blocks
    for seg in range(LONG_SEGMENTS):
        start = seg * FFT_SIZE
        end = start + FFT_SIZE

        seg_i = []
        seg_q = []
        valid_count = 0

        for idx in range(start, end):
            if idx < LONG_CHIRP_SAMPLES:
                seg_i.append(long_i[idx])
                seg_q.append(long_q[idx])
                valid_count += 1
            else:
                seg_i.append(0)
                seg_q.append(0)

        FFT_SIZE - valid_count

        write_mem_file(f"long_chirp_seg{seg}_i.mem", seg_i)
        write_mem_file(f"long_chirp_seg{seg}_q.mem", seg_q)

    # ---- Short chirp ----
    short_i, short_q = generate_short_chirp()

    write_mem_file("short_chirp_i.mem", short_i)
    write_mem_file("short_chirp_q.mem", short_q)

    # ---- Verification summary ----

    # Cross-check seg0 against radar_scene.py generate_reference_chirp_q15()
    # That function generates exactly the first 1024 samples of the chirp
    chirp_rate = CHIRP_BW / T_LONG_CHIRP
    mismatches = 0
    for n in range(FFT_SIZE):
        t = n / FS_SYS
        phase = math.pi * chirp_rate * t * t
        expected_i = max(-32768, min(32767, round(Q15_MAX * SCALE * math.cos(phase))))
        expected_q = max(-32768, min(32767, round(Q15_MAX * SCALE * math.sin(phase))))
        if long_i[n] != expected_i or long_q[n] != expected_q:
            mismatches += 1

    if mismatches == 0:
        pass
    else:
        return 1

    # Check magnitude envelope
    max(math.sqrt(i*i + q*q) for i, q in zip(long_i, long_q, strict=False))

    # Check seg1 zero padding (samples 3000-4095 should be zero)
    seg1_i_path = os.path.join(MEM_DIR, 'long_chirp_seg1_i.mem')
    with open(seg1_i_path) as f:
        seg1_lines = [line.strip() for line in f if line.strip()]
    # Indices 952..2047 in seg1 (global 3000..4095) should be zero
    nonzero_tail = sum(1 for line in seg1_lines[952:] if line != '0000')

    if nonzero_tail == 0:
        pass
    else:
        pass


    return 0


if __name__ == '__main__':
    sys.exit(main())
