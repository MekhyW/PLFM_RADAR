/*******************************************************************************
 * test_audit_s10_dsp_stall_polling.c
 *
 * AUDIT-S10 follow-up: MCU-side polling of gpio_dig7 (PD15 / FPGA_DIG7).
 *
 * Background: AUDIT-S10 (commit `58154a6`) split the FPGA's six-flag aggregate
 * gpio_dig5 into two MCU-visible bits: gpio_dig5 keeps signal-saturation only
 * (AGC reacts) and gpio_dig7 (PD15) carries control-fault classes
 * (range-decim watchdog | cic_fir overrun). Pre-follow-up the MCU did NOT
 * poll PD15, so DSP control faults were invisible to the recovery dispatcher
 * and accumulated until the operator noticed downstream symptoms.
 *
 * The post-fix predicate (matches checkSystemHealth section 10):
 *
 *     static uint32_t last_dsp_check = 0;
 *     static uint8_t  dsp_stall_streak = 0;
 *     if (HAL_GetTick() - last_dsp_check > 1000) {
 *         last_dsp_check = HAL_GetTick();         // commit BEFORE check
 *         bool fault = read_pd15();
 *         if (fault)  { if (dsp_stall_streak < 2) dsp_stall_streak++; }
 *         else        { dsp_stall_streak = 0; }
 *         if (dsp_stall_streak >= 2) {
 *             dsp_stall_streak = 0;               // arm for next post-recovery
 *             return ERROR_FPGA_DSP_STALL;
 *         }
 *     }
 *     return ERROR_NONE;
 *
 * Test strategy:
 *   - Extract the post-fix predicate into a pure function.
 *   - Drive it with simulated HAL_GetTick() and a controllable PD15 mock.
 *   - Verify: rate-limit holds (1 Hz cadence), 2-sample debounce blocks
 *     glitches, sustained fault fires error exactly once per assertion,
 *     last_dsp_check committed on every fired-watchdog call (AUDIT-CAL
 *     pattern), and HAL_GetTick wrap is handled correctly.
 *   - Add a counter-test using a pre-fix-style "fire on first HIGH" predicate
 *     to demonstrate the glitch-driven false-positive class the debounce
 *     guards against.
 ******************************************************************************/
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/* ---- Mock PD15 state ---- */
static bool pd15_high = false;
static int  pd15_read_count = 0;

static bool mock_read_pd15(void)
{
    pd15_read_count++;
    return pd15_high;
}

/* ============================================================================
 * Post-fix predicate (matches main.cpp section 10).
 * Returns 1 iff this call raises ERROR_FPGA_DSP_STALL.
 * ============================================================================ */
static uint32_t last_dsp_check_postfix     = 0;
static uint8_t  dsp_stall_streak_postfix   = 0;

static int dsp_watchdog_postfix(uint32_t now_tick)
{
    if (now_tick - last_dsp_check_postfix > 1000) {
        last_dsp_check_postfix = now_tick;          /* commit BEFORE check */
        bool fault = mock_read_pd15();
        if (fault) {
            if (dsp_stall_streak_postfix < 2) dsp_stall_streak_postfix++;
        } else {
            dsp_stall_streak_postfix = 0;
        }
        if (dsp_stall_streak_postfix >= 2) {
            dsp_stall_streak_postfix = 0;           /* arm for next assertion */
            return 1;                               /* ERROR_FPGA_DSP_STALL */
        }
    }
    return 0;
}

/* ============================================================================
 * Pre-fix-style predicate: fires on first HIGH read with no debounce. Kept as
 * a counter-test to demonstrate the glitch-driven false-positive that the
 * 2-sample debounce in the post-fix predicate guards against.
 * ============================================================================ */
static uint32_t last_dsp_check_prefix = 0;

static int dsp_watchdog_prefix_no_debounce(uint32_t now_tick)
{
    if (now_tick - last_dsp_check_prefix > 1000) {
        last_dsp_check_prefix = now_tick;
        if (mock_read_pd15()) {
            return 1;
        }
    }
    return 0;
}

/* ---- Test bookkeeping ---- */
static void reset_state(void)
{
    last_dsp_check_postfix    = 0;
    dsp_stall_streak_postfix  = 0;
    last_dsp_check_prefix     = 0;
    pd15_high                 = false;
    pd15_read_count           = 0;
}

int main(void)
{
    printf("=== AUDIT-S10 follow-up: PD15 polling + ERROR_FPGA_DSP_STALL ===\n");

    /* ----------------------------------------------------------------
     * T1: Healthy FPGA — PD15 stays LOW → no error across many windows.
     * Drive 60 s of 10 ms-spaced ticks; expect 0 errors and streak=0.
     * ---------------------------------------------------------------- */
    printf("  T1 healthy FPGA (PD15 LOW) — 0 errors over 60 s... ");
    reset_state();
    int errors = 0;
    for (int i = 0; i <= 6000; i++) {
        errors += dsp_watchdog_postfix((uint32_t)(i * 10));   /* 0..60 s */
    }
    assert(errors == 0);
    assert(dsp_stall_streak_postfix == 0);
    /* Polling cadence: window crosses every 1 s; with > 1000 strict, fires
       at t=1010, 2020, ..., 60060 — across [0, 60000] inclusive that's 59 polls. */
    assert(pd15_read_count == 59);
    printf("PASS (polls=%d)\n", pd15_read_count);

    /* ----------------------------------------------------------------
     * T2: Single-sample glitch — PD15 HIGH for 1 window only, LOW after.
     * Debounce must block: streak hits 1 then resets, no error.
     * ---------------------------------------------------------------- */
    printf("  T2 single-sample glitch — debounce blocks... ");
    reset_state();
    /* Cross threshold once with PD15 HIGH (glitch). */
    pd15_high = true;
    int e = dsp_watchdog_postfix(1001);
    assert(e == 0);
    assert(dsp_stall_streak_postfix == 1);
    /* Next window: PD15 back to LOW (glitch cleared). */
    pd15_high = false;
    e = dsp_watchdog_postfix(2002);
    assert(e == 0);
    assert(dsp_stall_streak_postfix == 0);
    /* Many subsequent LOW windows — no error ever. */
    for (uint32_t t = 3003; t < 60000; t += 1001) {
        assert(dsp_watchdog_postfix(t) == 0);
    }
    printf("PASS\n");

    /* ----------------------------------------------------------------
     * T3: Sustained DSP fault — PD15 HIGH for 2 consecutive windows.
     * Expect: streak reaches 2, fires ERROR_FPGA_DSP_STALL on second poll.
     * After fire, streak resets to 0 (armed for next post-recovery assertion).
     * ---------------------------------------------------------------- */
    printf("  T3 sustained fault (PD15 HIGH x2) — fires ERROR_FPGA_DSP_STALL... ");
    reset_state();
    pd15_high = true;
    /* First poll after threshold — streak=1, no error. */
    e = dsp_watchdog_postfix(1001);
    assert(e == 0);
    assert(dsp_stall_streak_postfix == 1);
    /* Second poll — streak=2, fires error, then resets to 0. */
    e = dsp_watchdog_postfix(2002);
    assert(e == 1);
    assert(dsp_stall_streak_postfix == 0);
    /* last_dsp_check committed BEFORE return — must equal 2002 even though
       we returned an error. Same AUDIT-CAL invariant as IMU watchdog. */
    assert(last_dsp_check_postfix == 2002u);
    printf("PASS\n");

    /* ----------------------------------------------------------------
     * T4: After fire, intra-window calls do NOT re-fire (rate-limit holds).
     * ---------------------------------------------------------------- */
    printf("  T4 post-fire rate-limit holds within window... ");
    /* Continue from T3 state: t=2002, fault still HIGH, streak=0. */
    /* Call again at t=2003 (only 1 ms after last poll) — under rate-limit,
       must NOT poll PD15 again. */
    int reads_before = pd15_read_count;
    e = dsp_watchdog_postfix(2003);
    assert(e == 0);
    assert(pd15_read_count == reads_before);   /* no PD15 read */
    printf("PASS\n");

    /* ----------------------------------------------------------------
     * T5: Sustained fault — error cadence is 1 per ~2 s (1 s window +
     * 2-sample debounce). Across 60 s of continuous fault, expect bounded
     * fire rate (NOT every iteration as the pre-fix path would).
     * ---------------------------------------------------------------- */
    printf("  T5 sustained fault — error rate bounded over 60 s... ");
    reset_state();
    pd15_high = true;
    int total_errors = 0;
    int total_calls  = 0;
    for (int i = 0; i <= 6000; i++) {
        if (dsp_watchdog_postfix((uint32_t)(i * 10))) total_errors++;
        total_calls++;
    }
    /* Polling at t=1010, 2020, ..., 60060 — 59 polls. Streak pattern:
       1, 2(fire+reset to 0), 1, 2(fire+reset), ... so error fires every
       2 polls. 59 polls / 2 = 29 errors (integer). Allow ±1 for boundary. */
    assert(total_calls == 6001);
    assert(total_errors >= 28 && total_errors <= 30);
    /* MCU-N1 latch at error_count > 10: under sustained fault would fire
       in ~22 s. That's acceptable — gives operator time to intervene
       before SAFE-MODE; bench-test should validate. Pre-fix without any
       polling, this fault was MCU-invisible until downstream symptoms. */
    printf("PASS (calls=%d errors=%d)\n", total_calls, total_errors);

    /* ----------------------------------------------------------------
     * T6: Counter-test — no-debounce predicate fires on first HIGH window,
     * even for a single-sample glitch. Demonstrates the false-positive class
     * the post-fix 2-sample debounce guards against.
     * ---------------------------------------------------------------- */
    printf("  T6 counter-test: no-debounce predicate false-fires on glitch... ");
    reset_state();
    pd15_high = true;
    /* Single HIGH glitch crosses threshold. */
    e = dsp_watchdog_prefix_no_debounce(1001);
    assert(e == 1);     /* false positive — bug demo */
    /* Glitch clears next window. */
    pd15_high = false;
    e = dsp_watchdog_prefix_no_debounce(2002);
    assert(e == 0);
    printf("PASS\n");

    /* ----------------------------------------------------------------
     * T7: HAL_GetTick() 32-bit wrap. Same modulo-arithmetic guarantee as
     * test_audit_imu_watchdog_cadence T6 / test_gap3_health_watchdog_cold_start T8.
     * ---------------------------------------------------------------- */
    printf("  T7 HAL_GetTick wrap (0xFFFFFF00 -> 0x00000064)... ");
    reset_state();
    pd15_high = false;
    /* Seed: prime the watchdog at t=0xFFFFFF00 (just before wrap). */
    last_dsp_check_postfix = 0xFFFFFF00u;
    /* Now ask at 0x00000064 — true elapsed = 0x164 = 356 ms, BELOW 1 s. */
    int err = dsp_watchdog_postfix(0x00000064u);
    assert(err == 0);
    assert(pd15_read_count == 0);                /* no poll */
    /* Now jump >1 s past the wrap: 0x00000064 + 1001 = 0x0000044D. */
    err = dsp_watchdog_postfix(0x0000044Du);
    assert(err == 0);                            /* PD15 LOW */
    assert(pd15_read_count == 1);                /* one poll */
    printf("PASS\n");

    printf("\n=== AUDIT-S10 follow-up: ALL TESTS PASSED ===\n\n");
    return 0;
}
