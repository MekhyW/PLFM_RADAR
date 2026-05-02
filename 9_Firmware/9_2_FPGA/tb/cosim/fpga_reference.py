#!/usr/bin/env python3
r"""
Independent floating-point reference for AERIS-10 signal processing chain.

Unlike fpga_model.py (which is a bit-exact PORT of the RTL — same NCO LUT,
same twiddle ROMs, same Q15 quantization), this module computes the
algorithm using ideal numpy/scipy primitives. It is the second leg of the
T-6 three-way triangulation:

    fpga_model.py (twin)  --bit-exact--  RTL simulation
              \                            /
               \                          /
                \-- within tolerance --> /
                  fpga_reference.py
                  (this file)

Drift signal interpretation:
  * twin == reference (within tol) and twin == RTL (bit-exact)
        -> chain healthy
  * twin == reference (within tol) but twin != RTL (bit-exact)
        -> RTL diverged from spec (real RTL bug)
  * twin != reference (outside tol) but twin == RTL (bit-exact)
        -> twin and RTL share a transcription error
           (e.g. wrong NCO_SINE_LUT entry, wrong twiddle ROM, wrong window
           coefficients). This is the bug class T-6 was opened to catch.

Coverage in this revision (highest transcription risk first):
  * NCO  — numpy.cos/sin vs the 64-entry quarter-wave NCO_SINE_LUT
  * FFT  — numpy.fft.fft/ifft vs Q15 twiddle ROM butterfly
  * MF   — numpy ifft(fft(sig) * conj(fft(ref))) vs RTL pipeline
  * Doppler — numpy.fft + ideal Cheby window vs Q15 window LUT + RTL FFT

Out of scope here (lower transcription risk, deferred):
  * CIC / FIR — coefficient files are derived from independent generators
  * Mixer / RangeBinDecimator — trivially correct in both
"""

from __future__ import annotations

import numpy as np


# =============================================================================
# NCO reference — ideal complex sinusoid
# =============================================================================

def nco_reference(num_samples: int, ftw: int,
                  phase_offset_deg: float = 0.0):
    """Ideal floating-point NCO output, scaled to match Q15 fpga_model.

    The fpga_model.NCO uses a 32-bit phase accumulator stepped by ftw, with
    a 64-entry quarter-wave cos LUT scaled to ±0x7FFF. This reference uses
    numpy.cos/sin directly with no LUT.

    Args:
        num_samples: number of samples to produce
        ftw: 32-bit unsigned phase increment per sample
        fs: sample rate (Hz)
        phase_offset_deg: phase offset (degrees), default 0

    Returns:
        (cos_q15, sin_q15) — float arrays of length num_samples, in Q15 scale
    """
    ftw = int(ftw) & 0xFFFFFFFF
    n = np.arange(num_samples, dtype=np.float64)
    # Phase accumulator semantics: phase[k] = (k * ftw) mod 2^32
    # Convert to radians: 2*pi * phase / 2^32
    phase_rad = 2.0 * np.pi * (n * ftw) / (1 << 32) + np.deg2rad(phase_offset_deg)
    cos_q15 = np.cos(phase_rad) * 32767.0
    sin_q15 = np.sin(phase_rad) * 32767.0
    return cos_q15, sin_q15


# =============================================================================
# FFT reference — numpy.fft.fft / ifft
# =============================================================================

def fft_reference(in_re, in_im, n: int = 2048, inverse: bool = False):
    """Ideal floating-point FFT.

    Scaling matches the AUDIT-C10/C-8 RTL convention (LogiCORE FFT v9.1
    scaled mode + iverilog fft_engine.v with per-stage convergent >>>1):
      forward: y[k] = (1/N) * sum_n x[n] * exp(-j*2*pi*k*n/N)    (1/N applied)
      inverse: y[n] = (1/N) * sum_k X[k] * exp(+j*2*pi*k*n/N)    (1/N applied)

    Both directions apply the SCALE_SCH = [1,1,…,1] schedule (one >>>1 per
    radix-2 stage = total /N), making FWD and INV symmetric. numpy.fft.ifft
    already includes the 1/N for INV; for FWD we divide explicitly so this
    reference exactly matches the RTL output.

    Args:
        in_re/in_im: length-N int or float sequences
        n: FFT size (16, 1024, 2048)
        inverse: True for IFFT

    Returns:
        (out_re, out_im) — float arrays, no Q15 saturation
    """
    re = np.asarray(in_re, dtype=np.float64)
    im = np.asarray(in_im, dtype=np.float64)
    if len(re) != n or len(im) != n:
        raise ValueError(f"input length {len(re)} != N={n}")
    x = re + 1j * im
    y = np.fft.ifft(x) if inverse else np.fft.fft(x) / n
    return y.real.copy(), y.imag.copy()


# =============================================================================
# Matched filter reference — IFFT(FFT(sig) * conj(FFT(ref)))
# =============================================================================

def matched_filter_reference(sig_re, sig_im, ref_re, ref_im, fft_size: int = 2048):
    """Ideal range profile via FFT-domain matched filter.

    range_profile = IFFT( FFT(sig) * conj(FFT(ref)) )

    The RTL pipeline does the same operation but with Q15 twiddles, 32-bit
    accumulators, and Q15 round-and-saturate after the conjugate multiply.
    This reference is unquantized.

    Args:
        sig_re/im, ref_re/im: length-fft_size sequences (int or float)
        fft_size: matched filter FFT size (default 2048)

    Returns:
        (range_re, range_im) — float arrays, no Q15 saturation
    """
    sig_re = np.asarray(sig_re, dtype=np.float64)
    sig_im = np.asarray(sig_im, dtype=np.float64)
    ref_re = np.asarray(ref_re, dtype=np.float64)
    ref_im = np.asarray(ref_im, dtype=np.float64)
    s = sig_re + 1j * sig_im
    r = ref_re + 1j * ref_im
    # AUDIT-C10/C-8: forward FFTs are scaled /N to mirror the RTL scaled-mode
    # schedule [1,…,1]; the IFFT is also /N (numpy default). Total chain
    # downscale = /N², predictable and matched between sim and silicon.
    S = np.fft.fft(s, n=fft_size) / fft_size
    R = np.fft.fft(r, n=fft_size) / fft_size
    P = S * np.conj(R)
    p = np.fft.ifft(P)
    return p.real.copy(), p.imag.copy()


# =============================================================================
# Doppler reference — Cheby-windowed per-sub-frame 16-pt FFT
# =============================================================================

def doppler_window_ideal():
    """Production Doppler window — 16-pt Dolph-Chebyshev, 60 dB sidelobes.

    Independent reference for the Q15 LUT in fpga_model.DOPPLER_WINDOW_COEFF
    and doppler_processor.v window_coeff. Generated by scipy directly so the
    drift cosim catches transcription errors that exist identically in the
    LUT and the Python twin.
    """
    from scipy.signal.windows import chebwin
    return chebwin(16, at=60, sym=True) * 32767.0


def doppler_reference(chirp_data_i, chirp_data_q,
                      num_subframes: int = 3,
                      chirps_per_subframe: int = 16,
                      range_bins: int = 512):
    """Ideal Doppler map using ideal Cheby-60 window + numpy.fft, no Q15 quantization.

    Args:
        chirp_data_i/q: 2D arrays [chirps_per_frame][range_bins], int or float
        num_subframes: number of independent 16-pt FFTs per range bin (default 3)
        chirps_per_subframe: 16
        range_bins: 512

    Returns:
        (doppler_map_re, doppler_map_im) — 2D float arrays
            shape [range_bins][num_subframes * chirps_per_subframe]
        Sub-frame s occupies output bins [s*16 .. s*16+15].
    """
    chirp_data_i = np.asarray(chirp_data_i, dtype=np.float64)
    chirp_data_q = np.asarray(chirp_data_q, dtype=np.float64)
    chirps_per_frame = num_subframes * chirps_per_subframe
    if chirp_data_i.shape != (chirps_per_frame, range_bins):
        raise ValueError(
            f"chirp_data_i shape {chirp_data_i.shape} != "
            f"({chirps_per_frame}, {range_bins})"
        )

    win = doppler_window_ideal()  # Q15-scaled
    total_bins = num_subframes * chirps_per_subframe
    out_re = np.zeros((range_bins, total_bins), dtype=np.float64)
    out_im = np.zeros((range_bins, total_bins), dtype=np.float64)

    for rbin in range(range_bins):
        for sf in range(num_subframes):
            start = sf * chirps_per_subframe
            stop = start + chirps_per_subframe
            offset = sf * chirps_per_subframe

            # Apply window and divide by 32768 to undo the Q15 scaling so
            # the comparison is to ideal floating-point amplitudes (the RTL
            # rounds (data*win + 1<<14) >> 15 which is an approximate /32768).
            x_re = chirp_data_i[start:stop, rbin] * win / 32768.0
            x_im = chirp_data_q[start:stop, rbin] * win / 32768.0
            x = x_re + 1j * x_im

            # AUDIT-C10/C-8: xfft_16 wraps fft_engine.v which now applies the
            # /N (=/16) scaled-mode schedule per radix-2 stage. Mirror that
            # downscale in the reference so the cosim compares apples-to-apples.
            X = np.fft.fft(x) / chirps_per_subframe
            out_re[rbin, offset:offset + chirps_per_subframe] = X.real
            out_im[rbin, offset:offset + chirps_per_subframe] = X.imag

    return out_re, out_im


# =============================================================================
# Self-test (sanity checks against numpy's own analytical answers)
# =============================================================================

def _self_test():
    """Quick sanity checks."""
    # NCO: at FTW = 0x4CCCCCCD, frequency = 0.3 * fs = 120 MHz at 400 MSPS.
    cos_q15, sin_q15 = nco_reference(8, 0x4CCCCCCD)
    # First sample should be cos(0)=1, sin(0)=0 in Q15
    assert abs(cos_q15[0] - 32767.0) < 1.0, f"NCO[0].cos = {cos_q15[0]}"
    assert abs(sin_q15[0]) < 1.0, f"NCO[0].sin = {sin_q15[0]}"

    # FFT: impulse -> all bins = amplitude/N (scaled-mode schedule)
    in_re = [1000] + [0] * 15
    in_im = [0] * 16
    out_re, _out_im = fft_reference(in_re, in_im, n=16)
    for k in range(16):
        # AUDIT-C10/C-8: FWD FFT now applies /N (=/16), so each bin = 1000/16
        assert abs(out_re[k] - 1000.0 / 16.0) < 1e-9, \
            f"FFT impulse bin {k}: {out_re[k]}"

    # Doppler: zero input -> zero output
    z_i = np.zeros((48, 512))
    z_q = np.zeros((48, 512))
    d_re, d_im = doppler_reference(z_i, z_q, num_subframes=3,
                                   chirps_per_subframe=16, range_bins=512)
    assert np.max(np.abs(d_re)) < 1e-9
    assert np.max(np.abs(d_im)) < 1e-9

    # Doppler window: scipy chebwin matches our reference output
    from scipy.signal.windows import chebwin
    w_ours = doppler_window_ideal()
    w_scipy = chebwin(16, at=60, sym=True) * 32767.0
    assert np.allclose(w_ours, w_scipy)

    print("fpga_reference self-test: OK")


if __name__ == '__main__':
    _self_test()
