/*******************************************************************************
 * test_audit_imu_watchdog_cadence.c
 *
 * AUDIT-CAL follow-up: identical "last_X_check not updated on error path" bug
 * existed at THREE sites in checkSystemHealth() (pre commit):
 *   - AD9523 clock check  (main.cpp:693-705, 5 s rate-limit broken)
 *   - ADAR1000 comm check (main.cpp:729-749, 2 s rate-limit broken)
 *   - IMU comm check      (main.cpp:752-760, 10 s rate-limit broken)
 *
 * All three shared one fix pattern: commit `last_X_check = HAL_GetTick();`
 * BEFORE any early return inside the watchdog block, so a transient sub-check
 * failure cannot bypass the rate-limit window. This test uses the IMU branch
 * as the representative predicate (10 s window is the longest; pre-fix demo
 * is most dramatic) — the AD9523 (5 s) and ADAR1000 (2 s) sites have
 * identical control flow and are covered by code review against the test's
 * extracted predicate.
 *
 * The pre-fix watchdog block was:
 *
 *     static uint32_t last_imu_check = 0;
 *     if (HAL_GetTick() - last_imu_check > 10000) {
 *         if (!GY85_Update(&imu)) {
 *             current_error = ERROR_IMU_COMM;
 *             return current_error;            // <-- skips timestamp update
 *         }
 *         last_imu_check = HAL_GetTick();      // <-- only on success path
 *     }
 *
 * Once GY85_Update started returning false (transient I2C glitch, cable pull,
 * sensor brown-out), the 10 s rate-limit never engaged. Every iteration of
 * the main while(1) loop fired ERROR_IMU_COMM. Combined with the MCU-N1
 * `error_count > 10 -> system_emergency_state=true` latch, the radar would
 * trip into SAFE-MODE within ~10 main-loop iterations of the first IMU
 * failure — far short of the intended ~100 s (10 errors x 10 s) grace window
 * meant to give an operator time to intervene or `attemptErrorRecovery` to
 * succeed.
 *
 * Same bug class as AUDIT-CAL (BMP180 watchdog at main.cpp:771-780); same
 * fix pattern: move `last_imu_check = HAL_GetTick();` to BEFORE the early
 * return, so the rate-limit window commits on every fired-watchdog call,
 * regardless of whether the underlying check passed.
 *
 * Test strategy:
 *   - Extract the post-fix watchdog predicate into a pure function.
 *   - Drive it with a simulated HAL_GetTick() and a controllable
 *     GY85_Update() mock; assert error count tracks the 10 s rate-limit and
 *     never tracks the main-loop iteration count.
 *   - Add a counter-test using the pre-fix predicate to demonstrate the
 *     regression we are guarding against.
 ******************************************************************************/
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* ---- Mock state ---- */
static int gy85_returns_ok = 1;       /* 1=PASS, 0=FAIL  */
static int gy85_call_count = 0;       /* incremented each invocation */

static int mock_GY85_Update(void)
{
    gy85_call_count++;
    return gy85_returns_ok;
}

/* ============================================================================
 * Post-fix predicate (matches the new main.cpp:760-769 block).
 * Returns 1 iff this call raises ERROR_IMU_COMM.
 * Updates last_imu_check BEFORE the early return on GY85 failure, so a
 * flapping IMU never bypasses the rate-limit.
 * ============================================================================ */
static uint32_t last_imu_check_postfix = 0;

static int imu_watchdog_postfix(uint32_t now_tick)
{
    if (now_tick - last_imu_check_postfix > 10000) {
        last_imu_check_postfix = now_tick;        /* commit BEFORE check */
        if (!mock_GY85_Update()) {
            return 1;                             /* ERROR_IMU_COMM */
        }
    }
    return 0;
}

/* ============================================================================
 * Pre-fix predicate (matches the old main.cpp:752-760 block, kept here as a
 * counter-test). Updates last_imu_check ONLY on the success path.
 * ============================================================================ */
static uint32_t last_imu_check_prefix = 0;

static int imu_watchdog_prefix(uint32_t now_tick)
{
    if (now_tick - last_imu_check_prefix > 10000) {
        if (!mock_GY85_Update()) {
            return 1;                             /* ERROR_IMU_COMM */
        }
        last_imu_check_prefix = now_tick;         /* only on success */
    }
    return 0;
}

/* ---- Test bookkeeping ---- */
static void reset_state(void)
{
    last_imu_check_postfix = 0;
    last_imu_check_prefix  = 0;
    gy85_call_count        = 0;
    gy85_returns_ok        = 1;
}

int main(void)
{
    printf("=== AUDIT-CAL follow-up: IMU watchdog rate-limit on error path ===\n");

    /* ----------------------------------------------------------------
     * T1: Healthy IMU — only one GY85_Update per 10 s window.
     * Drive 10 s of 10 ms-spaced main-loop ticks (1000 calls).
     * Expect: 1 invocation per 10 s window (so ~1-2 across the run).
     * ---------------------------------------------------------------- */
    printf("  T1 healthy IMU — calls rate-limited to ~1 per 10 s... ");
    reset_state();
    int errors = 0;
    for (int i = 0; i <= 1000; i++) {
        errors += imu_watchdog_postfix(i * 10);   /* 0..10000 ms */
    }
    /* First call (t=0): now - last(0) = 0, so the > 10000 test is FALSE,
       no GY85 call. The window first opens at t > 10000 (ie i = 1001+). */
    assert(errors == 0);
    assert(gy85_call_count == 0);
    /* Push past the threshold once. */
    errors += imu_watchdog_postfix(10001);
    assert(errors == 0);                          /* IMU OK -> no error */
    assert(gy85_call_count == 1);                 /* one update */
    printf("PASS (gy85 calls=%d, errors=%d)\n", gy85_call_count, errors);

    /* ----------------------------------------------------------------
     * T2: First failure path commits the rate-limit window.
     * Set GY85_Update to fail. Walk the simulated tick across one window
     * boundary; expect exactly 1 error AND last_imu_check_postfix updated.
     * ---------------------------------------------------------------- */
    printf("  T2 single failure commits window... ");
    reset_state();
    gy85_returns_ok = 0;
    /* Cross threshold once. */
    int e = imu_watchdog_postfix(10001);
    assert(e == 1);
    assert(last_imu_check_postfix == 10001u);     /* ts updated despite error */
    assert(gy85_call_count == 1);
    /* Immediately ask again — must NOT re-fire (rate-limit holds). */
    e = imu_watchdog_postfix(10002);
    assert(e == 0);
    assert(gy85_call_count == 1);
    printf("PASS\n");

    /* ----------------------------------------------------------------
     * T3: Continuous failure across many main-loop iterations does NOT
     * exceed 1 error per 10 s window.
     * Drive 60 s of 10 ms-spaced calls with GY85 always failing.
     * Expect: ceil(60 / 10) = 6 errors max (one per crossed window).
     * ---------------------------------------------------------------- */
    printf("  T3 flapping IMU — errors capped at 1 per 10 s window over 60 s... ");
    reset_state();
    gy85_returns_ok = 0;
    int total_errors = 0;
    int total_calls = 0;
    for (int i = 0; i <= 6000; i++) {              /* 0..60000 ms, step 10 ms */
        if (imu_watchdog_postfix(i * 10)) total_errors++;
        total_calls++;
    }
    /* Threshold is strict > 10000, so windows fire at t=10001, 20001, ...
       Across [0, 60000] inclusive: t exceeds 10000 starting i=1001 -> first
       fire; ts becomes 10010 (not 10001 since we stepped by 10 ms). Next
       window opens at ts + 10000 -> t > 20010 -> first fire at 20020.
       Continuing pattern: 30030, 40040, 50050, 60060 (out of range).
       Across [0, 60000]: exactly 5 fires. */
    assert(total_calls == 6001);
    assert(total_errors == 5);
    /* Critical invariant: errors >>>> main-loop iterations means broken. */
    assert(total_errors < total_calls);
    /* MCU-N1 latch (error_count > 10) requires ~110 s of post-fix flapping;
       under pre-fix it would have tripped within ~10 main-loop iterations. */
    printf("PASS (calls=%d errors=%d)\n", total_calls, total_errors);

    /* ----------------------------------------------------------------
     * T4: Counter-test — pre-fix predicate fires error every main-loop
     * iteration past the first 10 s, demonstrating the bug we just fixed.
     * ---------------------------------------------------------------- */
    printf("  T4 counter-test: pre-fix predicate fires every loop... ");
    reset_state();
    gy85_returns_ok = 0;
    int prefix_errors = 0;
    int prefix_calls = 0;
    /* Prime: cross the threshold (any t > 10000), then fail thereafter. */
    for (int i = 1001; i <= 1100; i++) {           /* 100 main-loop iterations */
        if (imu_watchdog_prefix(i * 10)) prefix_errors++;
        prefix_calls++;
    }
    /* Pre-fix bug: each iteration past t=10000 returns an error, because
       last_imu_check_prefix never updates on the failure path. */
    assert(prefix_calls == 100);
    assert(prefix_errors == 100);                  /* ALL fail = bug demo */
    printf("PASS (prefix would have %d errors in %d iterations)\n",
           prefix_errors, prefix_calls);

    /* ----------------------------------------------------------------
     * T5: Recovery — when GY85 starts passing again, errors stop and the
     * window cadence resumes normally.
     * ---------------------------------------------------------------- */
    printf("  T5 recovery — IMU comes back, errors stop... ");
    reset_state();
    gy85_returns_ok = 0;
    /* One failure across first window. */
    assert(imu_watchdog_postfix(10001) == 1);
    /* Now IMU recovers. Cross next window boundary. */
    gy85_returns_ok = 1;
    assert(imu_watchdog_postfix(20002) == 0);      /* window crossed; check OK */
    assert(imu_watchdog_postfix(20003) == 0);      /* same window; not re-tested */
    /* Cross another window boundary while healthy. */
    assert(imu_watchdog_postfix(30004) == 0);
    printf("PASS\n");

    /* ----------------------------------------------------------------
     * T6: HAL_GetTick() 32-bit wrap. Same modulo-arithmetic guarantee as
     * test_gap3_health_watchdog_cold_start.c T8.
     * ---------------------------------------------------------------- */
    printf("  T6 HAL_GetTick wrap (0xFFFFFF00 -> 0x00000064)... ");
    reset_state();
    gy85_returns_ok = 1;
    /* Seed: prime the watchdog at t=0xFFFFFF00 by crossing a window from 0. */
    last_imu_check_postfix = 0xFFFFFF00u;
    /* Now ask at 0x00000064 (just after wrap). True elapsed = 0x164 = 356 ms,
       which is BELOW the 10 s threshold, so the watchdog should NOT trigger
       a GY85 call. */
    int err = imu_watchdog_postfix(0x00000064u);
    assert(err == 0);
    assert(gy85_call_count == 0);                  /* no GY85 call */
    /* Now jump >10 s past the wrap: 0x00000064 + 10001 = 0x00002775. */
    err = imu_watchdog_postfix(0x00002775u);
    assert(err == 0);                              /* IMU OK */
    assert(gy85_call_count == 1);                  /* one GY85 call */
    printf("PASS\n");

    printf("\n=== AUDIT-CAL follow-up: ALL TESTS PASSED ===\n\n");
    return 0;
}
