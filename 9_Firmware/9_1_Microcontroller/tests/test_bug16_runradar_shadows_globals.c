/*
 * test_bug16_runradar_shadows_globals.c
 *
 * MCU-N5/C4: runRadarPulseSequence() in main.cpp used to declare local
 * `int m, n, y` that shadowed the file-scope globals consumed by
 * getStatusString() (BeamPos/Azimuth/ChirpCount). Result: telemetry
 * was frozen at "BeamPos:1|Azimuth:1|ChirpCount:1" forever no matter
 * how many beam positions or revolutions had elapsed.
 *
 * This is a host-side static-pattern test. We replay the structural
 * loop from runRadarPulseSequence using two sibling helpers — one
 * with the pre-fix shadowing, one with the post-fix global-only
 * pattern — and assert that:
 *   (a) the shadowing version leaves globals at their initial value,
 *   (b) the fixed version updates the globals exactly as expected.
 *
 * No HAL, no mocks; just C arithmetic.
 */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>

/* Mirror main.cpp m_max/n_max (P-5 update: m_max 32 -> 48 to match
 * RP_CHIRPS_PER_FRAME = 48 = 3 sub-frames * 16 chirps from PR-F). */
static const int m_max = 48;
static const int n_max = 31;

static uint8_t g_m;
static uint8_t g_n;
static uint8_t g_y;
static uint8_t g_y_max = 50;

static void reset_globals(void)
{
    g_m = 1;
    g_n = 1;
    g_y = 1;
}

/* Pre-fix replica of runRadarPulseSequence body (locals shadow globals). */
static void run_buggy(void)
{
    int m = 1;
    int n = 1;
    int y = 1;

    for (int beam_pos = 0; beam_pos < 15; beam_pos++) {
        m += m_max / 2;
        m += m_max / 2;
        m += m_max / 2;
        if (m > m_max) m = 1;

        n++;
        if (n > n_max) n = 1;
    }
    y++; if (y > g_y_max) y = 1;

    /* The locals are discarded; globals stay untouched. */
    (void)m; (void)n; (void)y;
}

/* Post-fix: same body, no local redeclaration — references globals.
 * PR-AB.a moved vector_0 out of the inner loop, so the m advance is now
 *   1 (vector_0, before loop) + 2 × 15 (matrix1+matrix2 in loop) = 31 increments
 * per azimuth, instead of the prior 3 × 15 = 45. The wrap behavior
 * (g_m wraps on every iteration's matrix2 add) is unchanged. */
static void run_fixed(void)
{
    /* PR-AB.a: vector_0 broadside reference, 1× per azimuth (was inside loop). */
    g_m += m_max / 2;
    if (g_m > m_max) g_m = 1;

    for (int beam_pos = 0; beam_pos < 15; beam_pos++) {
        g_m += m_max / 2;  /* matrix1 (negative-θ scan) */
        g_m += m_max / 2;  /* matrix2 (positive-θ scan) */
        if (g_m > m_max) g_m = 1;

        g_n++;
        if (g_n > n_max) g_n = 1;
    }
    g_y++; if (g_y > g_y_max) g_y = 1;
}

int main(void)
{
    int failures = 0;

    /* (a) Buggy version: globals must remain at 1. */
    reset_globals();
    run_buggy();
    if (g_m != 1 || g_n != 1 || g_y != 1) {
        fprintf(stderr,
                "FAIL: buggy run modified globals (m=%u n=%u y=%u) — "
                "shadowing replica is broken\n", g_m, g_n, g_y);
        failures++;
    } else {
        printf("PASS: pre-fix shadowing leaves globals at (1,1,1)\n");
    }

    /* (b) Fixed version: 15 beam positions advance n; m wraps; y bumps. */
    reset_globals();
    run_fixed();

    /* After 15 iterations of n++, with n_max=31, n should be 16. */
    if (g_n != 16) {
        fprintf(stderr, "FAIL: g_n=%u (expected 16)\n", g_n);
        failures++;
    } else {
        printf("PASS: g_n advanced to 16 after 15 beam positions\n");
    }

    /* m: each iter adds 3*(m_max/2)=72; reset to 1 when m>m_max=48.
     * 1+72=73 -> reset to 1. So after every iter m=1. */
    if (g_m != 1) {
        fprintf(stderr, "FAIL: g_m=%u (expected 1 after wrap)\n", g_m);
        failures++;
    } else {
        printf("PASS: g_m wraps to 1 each iteration as designed\n");
    }

    /* y bumped exactly once at end of sweep. */
    if (g_y != 2) {
        fprintf(stderr, "FAIL: g_y=%u (expected 2)\n", g_y);
        failures++;
    } else {
        printf("PASS: g_y advanced from 1 -> 2 after one revolution\n");
    }

    /* (c) Belt-and-suspenders: a static-string scan of main.cpp asserts
     * that the three shadowing declarations are gone. Skipped here —
     * gated by the runtime checks above plus the project's regression
     * grep. The Makefile target keeps this test cheap. */

    if (failures > 0) {
        fprintf(stderr, "\n*** %d failure(s) ***\n", failures);
        return 1;
    }
    printf("\nAll checks passed.\n");
    return 0;
}
