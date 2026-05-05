/*******************************************************************************
 * test_bug5_fine_phase_gpio_only.c
 *
 * F-5.1: SetFinePhaseShift / SetPhaseShift use binary GPIO only.
 *
 * Schematic-verified rationale:
 *   - MCU is STM32F746ZGT7. PG7 (RX_DELADJ) and PG13 (TX_DELADJ) have no TIM3
 *     alternate function (Port G AFs are FMC/ETH/USART6/SAI2/SDMMC2 — no
 *     TIMx routes). Even configuring AF mode would have no valid AF code.
 *   - FreqSynth board: DELADJ net has only a 200 kOhm pulldown (R22 on TX,
 *     R35 on RX). No series-R + shunt-C low-pass filter exists on the board,
 *     so a PWM-to-DC scheme cannot work as built.
 *
 * Prior history:
 *   - 5fbe97f (2026-03-09 initial upload): GPIO-only with explicit TODO
 *     "// In a real system, you would generate a PWM signal on DELADJ pin".
 *   - 3979693 (2026-03-19 "Bug #5 fix"): added HAL_TIM_PWM_Start scaffolding
 *     calling htim3 (which didn't exist yet) — false-fix.
 *   - c466021 (2026-03-19 "B15 fix"): added MX_TIM3_Init to define htim3 —
 *     timer ran internally but the pin's AF mux was never enabled.
 *   - This commit: revert to binary GPIO matching the schematic's actual
 *     intent. Delete the PWM scaffolding and htim3 entirely.
 *
 * Test strategy (binary):
 *   1. duty=0          -> 1 GPIO write LOW   (no PWM API touched)
 *   2. duty=MAX        -> 1 GPIO write HIGH  (no PWM API touched)
 *   3. duty=500 (any nonzero) -> 1 GPIO write HIGH (no PWM API touched)
 *   4. RX device (1)   -> writes RX_DELADJ_Pin (PG7), not TX_DELADJ_Pin (PG13)
 ******************************************************************************/
#include "adf4382a_manager.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    ADF4382A_Manager mgr;
    int ret;

    printf("=== F-5.1: SetFinePhaseShift uses binary GPIO (no PWM) ===\n");

    /* Setup */
    spy_reset();
    ret = ADF4382A_Manager_Init(&mgr, SYNC_METHOD_TIMED);
    assert(ret == ADF4382A_MANAGER_OK);

    /* ---- Test A: duty=0 -> GPIO LOW, no PWM ---- */
    spy_reset();
    ret = ADF4382A_SetFinePhaseShift(&mgr, 0, 0);
    assert(ret == ADF4382A_MANAGER_OK);

    int gpio_writes  = spy_count_type(SPY_GPIO_WRITE);
    int pwm_starts   = spy_count_type(SPY_TIM_PWM_START);
    int pwm_stops    = spy_count_type(SPY_TIM_PWM_STOP);
    int set_compares = spy_count_type(SPY_TIM_SET_COMPARE);
    printf("  duty=0: GPIO_WRITE=%d PWM_START=%d PWM_STOP=%d SET_COMPARE=%d\n",
           gpio_writes, pwm_starts, pwm_stops, set_compares);
    assert(gpio_writes == 1);
    assert(pwm_starts == 0 && pwm_stops == 0 && set_compares == 0);

    int idx = spy_find_nth(SPY_GPIO_WRITE, 0);
    const SpyRecord *r = spy_get(idx);
    assert(r != NULL && r->value == GPIO_PIN_RESET);
    assert(r->pin == TX_DELADJ_Pin);
    printf("  PASS: duty=0 -> GPIO LOW on TX_DELADJ_Pin (PG13), no PWM\n");

    /* ---- Test B: duty=MAX -> GPIO HIGH, no PWM ---- */
    spy_reset();
    ret = ADF4382A_SetFinePhaseShift(&mgr, 0, DELADJ_MAX_DUTY_CYCLE);
    assert(ret == ADF4382A_MANAGER_OK);

    gpio_writes  = spy_count_type(SPY_GPIO_WRITE);
    pwm_starts   = spy_count_type(SPY_TIM_PWM_START);
    pwm_stops    = spy_count_type(SPY_TIM_PWM_STOP);
    set_compares = spy_count_type(SPY_TIM_SET_COMPARE);
    printf("  duty=MAX(%d): GPIO_WRITE=%d PWM_START=%d PWM_STOP=%d SET_COMPARE=%d\n",
           DELADJ_MAX_DUTY_CYCLE, gpio_writes, pwm_starts, pwm_stops, set_compares);
    assert(gpio_writes == 1);
    assert(pwm_starts == 0 && pwm_stops == 0 && set_compares == 0);

    idx = spy_find_nth(SPY_GPIO_WRITE, 0);
    r = spy_get(idx);
    assert(r != NULL && r->value == GPIO_PIN_SET);
    assert(r->pin == TX_DELADJ_Pin);
    printf("  PASS: duty=MAX -> GPIO HIGH on TX_DELADJ_Pin (PG13), no PWM\n");

    /* ---- Test C: duty=500 (intermediate) -> GPIO HIGH (binary mapping) ---- */
    spy_reset();
    ret = ADF4382A_SetFinePhaseShift(&mgr, 0, 500);
    assert(ret == ADF4382A_MANAGER_OK);

    gpio_writes  = spy_count_type(SPY_GPIO_WRITE);
    pwm_starts   = spy_count_type(SPY_TIM_PWM_START);
    set_compares = spy_count_type(SPY_TIM_SET_COMPARE);
    printf("  duty=500 (intermediate): GPIO_WRITE=%d PWM_START=%d SET_COMPARE=%d\n",
           gpio_writes, pwm_starts, set_compares);
    assert(gpio_writes == 1);
    assert(pwm_starts == 0 && set_compares == 0);

    idx = spy_find_nth(SPY_GPIO_WRITE, 0);
    r = spy_get(idx);
    assert(r != NULL && r->value == GPIO_PIN_SET);
    printf("  PASS: duty=500 -> GPIO HIGH (binary: any nonzero -> HIGH)\n");

    /* ---- Test D: RX device (1) writes RX_DELADJ_Pin, not TX ---- */
    spy_reset();
    ret = ADF4382A_SetFinePhaseShift(&mgr, 1, 750);
    assert(ret == ADF4382A_MANAGER_OK);

    idx = spy_find_nth(SPY_GPIO_WRITE, 0);
    r = spy_get(idx);
    assert(r != NULL);
    printf("  RX duty=750: pin=0x%04X (expected 0x%04X = RX_DELADJ_Pin/PG7), value=%u\n",
           r->pin, RX_DELADJ_Pin, r->value);
    assert(r->pin == RX_DELADJ_Pin);
    assert(r->value == GPIO_PIN_SET);
    printf("  PASS: RX device writes PG7 with HIGH (binary)\n");

    /* Cleanup */
    ADF4382A_Manager_Deinit(&mgr);

    printf("\n=== F-5.1: ALL TESTS PASSED (binary DELADJ) ===\n\n");
    return 0;
}
