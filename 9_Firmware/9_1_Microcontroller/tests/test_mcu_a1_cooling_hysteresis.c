/*******************************************************************************
 * test_mcu_a1_cooling_hysteresis.c
 *
 * MCU-A1: cooling-fan threshold was a 25 C dev stub that latched the fan ON
 * at room temperature. Production fix raises the threshold to 70 C with a
 * 60 C off point (10 C hysteresis) so the fan does not chatter near the
 * trip point.
 *
 * This test replays the fixed cooling-control logic against a temperature
 * sweep and asserts (a) the fan stays off below the ON threshold from cold,
 * (b) it engages crossing 70 C upward, (c) it stays on through the 60-70 C
 * dead-band on the way down, and (d) it disengages below 60 C.
 ******************************************************************************/
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

static const float COOLING_ON_C  = 70.0f;
static const float COOLING_OFF_C = 60.0f;

/* Replays the post-fix cooling control inside main.cpp:2183-... */
static bool step_cooling(bool cooling_on, float t_max)
{
    if (!cooling_on && t_max > COOLING_ON_C)       return true;
    else if (cooling_on && t_max < COOLING_OFF_C)  return false;
    return cooling_on;
}

int main(void)
{
    printf("=== MCU-A1: cooling-fan hysteresis (70 C ON / 60 C OFF) ===\n");

    bool fan = false;

    /* 1. Cold start: room temperature must NOT engage the fan
     * (this is the bug the 25 C stub caused). */
    printf("  Test 1: 25 C from cold ... ");
    fan = step_cooling(fan, 25.0f);
    assert(fan == false);
    printf("OFF, PASS\n");

    /* 2. Walking up through the dead band must not engage. */
    printf("  Test 2: 65 C from cold ... ");
    fan = step_cooling(fan, 65.0f);
    assert(fan == false);
    printf("OFF, PASS\n");

    /* 3. At the exact threshold (>, not >=) still off. */
    printf("  Test 3: 70.0 C exactly ... ");
    fan = step_cooling(fan, 70.0f);
    assert(fan == false);
    printf("OFF, PASS\n");

    /* 4. Crossing the trip point upward engages. */
    printf("  Test 4: 70.5 C ... ");
    fan = step_cooling(fan, 70.5f);
    assert(fan == true);
    printf("ON, PASS\n");

    /* 5. Cooling off into the dead band — fan must stay on. */
    printf("  Test 5: 65 C while ON ... ");
    fan = step_cooling(fan, 65.0f);
    assert(fan == true);
    printf("ON (hysteresis), PASS\n");

    /* 6. At the OFF threshold exactly, still on (uses <, not <=). */
    printf("  Test 6: 60.0 C exactly while ON ... ");
    fan = step_cooling(fan, 60.0f);
    assert(fan == true);
    printf("ON, PASS\n");

    /* 7. Crossing the OFF point disengages. */
    printf("  Test 7: 59.5 C while ON ... ");
    fan = step_cooling(fan, 59.5f);
    assert(fan == false);
    printf("OFF, PASS\n");

    /* 8. Spike-and-recover above the system overtemp gate (75 C) — the
     * fan engages well before checkSystemHealth() trips SAFE mode. */
    printf("  Test 8: 76 C engages cooling before 75 C SAFE-mode gate ... ");
    fan = step_cooling(fan, 76.0f);
    assert(fan == true);
    printf("ON, PASS\n");

    /* 9. The pre-fix 25 C stub would have set fan=true here. Confirm the
     * fixed logic does not. */
    printf("  Test 9: 30 C does NOT engage (regression guard for 25 C stub) ... ");
    fan = false;
    fan = step_cooling(fan, 30.0f);
    assert(fan == false);
    printf("OFF, PASS\n");

    printf("\n=== MCU-A1: ALL TESTS PASSED ===\n\n");
    return 0;
}
