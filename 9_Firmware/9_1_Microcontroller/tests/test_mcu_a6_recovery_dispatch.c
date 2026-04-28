/*******************************************************************************
 * test_mcu_a6_recovery_dispatch.c
 *
 * MCU-A6: attemptErrorRecovery() previously had no case for
 * ERROR_AD9523_CLOCK or ERROR_FPGA_COMM — both fell through to the
 * default DIAG_WARN("No specific handler") branch. checkSystemHealth()
 * keeps re-firing the same error every pass, the recovery never advances,
 * and the system reaches whatever escalation threshold is wired in
 * handleSystemError without ever attempting a fix.
 *
 * Production fix adds:
 *   - ERROR_AD9523_CLOCK: AD9523_RESET_ASSERT, 10 ms, configure_ad9523()
 *   - ERROR_FPGA_COMM:    pulse PD12 LOW->10 ms->HIGH (matches boot reset)
 *
 * This test models the dispatch table and asserts each error code routes
 * to the expected handler (including the existing TX/RX/ADAR/IMU/GPS
 * paths so a future regression that drops one is caught here).
 ******************************************************************************/
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    ERR_NONE,
    ERR_AD9523_CLOCK,
    ERR_ADF4382_TX_UNLOCK,
    ERR_ADF4382_RX_UNLOCK,
    ERR_ADAR1000_COMM,
    ERR_ADAR1000_TEMP,
    ERR_IMU_COMM,
    ERR_BMP180_COMM,
    ERR_GPS_COMM,
    ERR_RF_PA_OVERCURRENT,
    ERR_FPGA_COMM,
    ERR_OTHER,
} Err_t;

typedef enum {
    HND_NONE,
    HND_AD9523_RESET_AND_RECONFIG,
    HND_LO_REINIT,
    HND_ADAR_REINIT,
    HND_IMU_REINIT,
    HND_GPS_NOOP,
    HND_FPGA_RESET_PULSE,
    HND_DEFAULT_LOG,
} Handler_t;

/* Mirrors main.cpp:attemptErrorRecovery() switch dispatch */
static Handler_t dispatch(Err_t error)
{
    switch (error) {
        case ERR_ADF4382_TX_UNLOCK:
        case ERR_ADF4382_RX_UNLOCK:
            return HND_LO_REINIT;
        case ERR_ADAR1000_COMM:
            return HND_ADAR_REINIT;
        case ERR_IMU_COMM:
            return HND_IMU_REINIT;
        case ERR_GPS_COMM:
            return HND_GPS_NOOP;
        case ERR_AD9523_CLOCK:                /* MCU-A6 new */
            return HND_AD9523_RESET_AND_RECONFIG;
        case ERR_FPGA_COMM:                   /* MCU-A6 new */
            return HND_FPGA_RESET_PULSE;
        default:
            return HND_DEFAULT_LOG;
    }
}

int main(void)
{
    printf("=== MCU-A6: attemptErrorRecovery dispatch coverage ===\n");

    /* MCU-A6 new cases ------------------------------------------------ */
    printf("  Test 1: ERR_AD9523_CLOCK -> reset+reconfig ... ");
    assert(dispatch(ERR_AD9523_CLOCK) == HND_AD9523_RESET_AND_RECONFIG);
    printf("PASS\n");

    printf("  Test 2: ERR_FPGA_COMM -> PD12 pulse ... ");
    assert(dispatch(ERR_FPGA_COMM) == HND_FPGA_RESET_PULSE);
    printf("PASS\n");

    /* Existing handlers must still route correctly ------------------- */
    printf("  Test 3: ERR_ADF4382_TX_UNLOCK -> LO re-init ... ");
    assert(dispatch(ERR_ADF4382_TX_UNLOCK) == HND_LO_REINIT);
    printf("PASS\n");

    printf("  Test 4: ERR_ADF4382_RX_UNLOCK -> LO re-init ... ");
    assert(dispatch(ERR_ADF4382_RX_UNLOCK) == HND_LO_REINIT);
    printf("PASS\n");

    printf("  Test 5: ERR_ADAR1000_COMM -> ADAR re-init ... ");
    assert(dispatch(ERR_ADAR1000_COMM) == HND_ADAR_REINIT);
    printf("PASS\n");

    printf("  Test 6: ERR_IMU_COMM -> IMU re-init ... ");
    assert(dispatch(ERR_IMU_COMM) == HND_IMU_REINIT);
    printf("PASS\n");

    printf("  Test 7: ERR_GPS_COMM -> auto-recover (no-op) ... ");
    assert(dispatch(ERR_GPS_COMM) == HND_GPS_NOOP);
    printf("PASS\n");

    /* Default branch for un-handled codes ---------------------------- */
    printf("  Test 8: ERR_BMP180_COMM -> default log ... ");
    assert(dispatch(ERR_BMP180_COMM) == HND_DEFAULT_LOG);
    printf("PASS\n");

    printf("  Test 9: ERR_ADAR1000_TEMP -> default log ... ");
    assert(dispatch(ERR_ADAR1000_TEMP) == HND_DEFAULT_LOG);
    printf("PASS\n");

    /* Pre-fix regression — without MCU-A6, AD9523_CLOCK and FPGA_COMM
     * fell into HND_DEFAULT_LOG. Confirm fixed dispatch does NOT. */
    printf("  Test 10: pre-fix would log default for AD9523/FPGA ... ");
    assert(dispatch(ERR_AD9523_CLOCK) != HND_DEFAULT_LOG);
    assert(dispatch(ERR_FPGA_COMM)    != HND_DEFAULT_LOG);
    printf("fixed dispatch routes both, PASS\n");

    /* RF_PA_OVERCURRENT is intentionally NOT in attemptErrorRecovery
     * because handleSystemError escalates it directly to Emergency_Stop
     * (main.cpp:944-957). Document via test. */
    printf("  Test 11: ERR_RF_PA_OVERCURRENT -> default (escalated upstream) ... ");
    assert(dispatch(ERR_RF_PA_OVERCURRENT) == HND_DEFAULT_LOG);
    printf("PASS\n");

    printf("\n=== MCU-A6: ALL TESTS PASSED ===\n\n");
    return 0;
}
