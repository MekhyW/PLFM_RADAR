/*******************************************************************************
 * test_mcu_a5_pa_cal_gate.c
 *
 * MCU-A5: the boot-time PA Idq calibration walks DAC_val from 126 down,
 * so mid-walk Idq readings sit well above the 2.5 A overcurrent threshold
 * by design. A channel that hits the safety-counter timeout (50 iters) can
 * also be left above the window. Without a "calibration in progress" gate,
 * the next checkSystemHealth() pass would trip ERROR_RF_PA_OVERCURRENT and
 * Emergency_Stop the whole system.
 *
 * Production fix adds a `pa_calibration_in_progress` flag set TRUE around
 * the cal walks and consulted by checkSystemHealth's Idq window. This test
 * replays the walk + post-cal completion path against the gated check and
 * asserts:
 *   - mid-walk overcurrent readings do NOT trip while the gate is set
 *   - bias-low readings do NOT trip while the gate is set
 *   - same readings DO trip once the gate is cleared
 *   - converged-in-window readings pass either way
 ******************************************************************************/
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

typedef enum {
    ERR_NONE,
    ERR_OVERCURRENT,
    ERR_BIAS,
} Err_t;

/* Replays the post-fix gated Idq window from main.cpp:checkSystemHealth */
static Err_t check_idq(const float idq[16], bool power_amplifier, bool cal_in_progress)
{
    if (!power_amplifier || cal_in_progress) return ERR_NONE;
    for (int i = 0; i < 16; i++) {
        if (idq[i] > 2.5f) return ERR_OVERCURRENT;
        if (idq[i] < 0.1f) return ERR_BIAS;
    }
    return ERR_NONE;
}

int main(void)
{
    printf("=== MCU-A5: PA-cal gate suppresses Idq window during walk ===\n");

    /* Mid-walk: every channel sitting at the DAC=126 starting current
     * (~3.5 A typical for the QPA2962 family). */
    float idq_midwalk[16];
    for (int i = 0; i < 16; i++) idq_midwalk[i] = 3.5f;

    /* Converged: every channel at the 1.680 A target (well inside window). */
    float idq_converged[16];
    for (int i = 0; i < 16; i++) idq_converged[i] = 1.680f;

    /* Mixed: ch5 left high by safety_counter timeout, others converged. */
    float idq_stuck_high[16];
    for (int i = 0; i < 16; i++) idq_stuck_high[i] = 1.680f;
    idq_stuck_high[5] = 2.85f;

    /* Bias fault: ch9 at 0.05 A (below 0.1 A floor). */
    float idq_bias_fault[16];
    for (int i = 0; i < 16; i++) idq_bias_fault[i] = 1.680f;
    idq_bias_fault[9] = 0.05f;

    /* 1. Mid-walk readings while cal IS in progress -> no trip. */
    printf("  Test 1: mid-walk 3.5 A with cal gate SET ... ");
    assert(check_idq(idq_midwalk, true, true) == ERR_NONE);
    printf("ERR_NONE, PASS\n");

    /* 2. Same readings with cal gate CLEARED -> overcurrent. */
    printf("  Test 2: mid-walk 3.5 A with cal gate CLEAR ... ");
    assert(check_idq(idq_midwalk, true, false) == ERR_OVERCURRENT);
    printf("ERR_OVERCURRENT, PASS\n");

    /* 3. Converged readings -> no trip whether gate is set or clear. */
    printf("  Test 3: converged 1.68 A regardless of gate ... ");
    assert(check_idq(idq_converged, true, true)  == ERR_NONE);
    assert(check_idq(idq_converged, true, false) == ERR_NONE);
    printf("ERR_NONE both, PASS\n");

    /* 4. Stuck-high channel after cal completes (gate clears) MUST trip
     * — this is exactly the "advisory surfaces post-cal" behaviour the
     * fix preserves. */
    printf("  Test 4: ch5 stuck high 2.85 A surfaces post-cal ... ");
    assert(check_idq(idq_stuck_high, true, true)  == ERR_NONE);
    assert(check_idq(idq_stuck_high, true, false) == ERR_OVERCURRENT);
    printf("masked during cal, trips after, PASS\n");

    /* 5. Bias fault is also gated during cal (early walk reads can dip
     * low) and surfaces after. */
    printf("  Test 5: ch9 bias 0.05 A surfaces post-cal ... ");
    assert(check_idq(idq_bias_fault, true, true)  == ERR_NONE);
    assert(check_idq(idq_bias_fault, true, false) == ERR_BIAS);
    printf("masked during cal, trips after, PASS\n");

    /* 6. PA disabled -> no trip whether or not gate is set (preserves
     * existing PowerAmplifier guard). */
    printf("  Test 6: PowerAmplifier=false short-circuits check ... ");
    assert(check_idq(idq_midwalk, false, false) == ERR_NONE);
    assert(check_idq(idq_midwalk, false, true)  == ERR_NONE);
    printf("ERR_NONE, PASS\n");

    /* 7. Pre-fix regression — the buggy path had no gate parameter. With
     * mid-walk readings of 3.5 A it would unconditionally trip
     * ERR_OVERCURRENT mid-cal, leading to Emergency_Stop. Fix prevents
     * by allowing cal_in_progress=true to mask. */
    printf("  Test 7: pre-fix would have tripped mid-walk ... ");
    /* "pre-fix" === call with cal_in_progress=false during the walk */
    assert(check_idq(idq_midwalk, true, false) == ERR_OVERCURRENT);
    printf("buggy path = trip, fixed path masks, PASS\n");

    printf("\n=== MCU-A5: ALL TESTS PASSED ===\n\n");
    return 0;
}
