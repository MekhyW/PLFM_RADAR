/*******************************************************************************
 * test_audit_cal_bmp180_begin.c
 *
 * AUDIT-CAL: BMP180 driver had no public init method and never called
 * `readCalibrationCoefficients()` from anywhere — `_calCoeff` ran at the
 * C++ in-class member-initializer defaults (all zeros) at runtime.
 *
 * Consequence: `computeB5(UT)` short-circuited via 0/0 (Cortex-M7 SDIV
 * with `SCB->CCR.DIV_0_TRP=0` returns 0 silently) → `getPressure()` always
 * tripped the `if (B4 == 0) return INT32_MIN;` guard. The health watchdog
 * fired ERROR_BMP180_COMM every main-loop iteration (last_bmp_check was
 * not updated on the error path), and `error_count > 10` latched
 * `system_emergency_state = true` within ~25 s of boot. The radar
 * self-shut-down for no real reason every time it powered on.
 *
 * Production fix:
 *   - Added public `bool BMP180::begin(void)` — verifies chip ID then reads
 *     the 11 factory calibration coefficients (AC1..MD at registers
 *     0xAA..0xBE, every 2 bytes). Returns true only on full success.
 *   - main.cpp BAROMETER INIT calls myBMP.begin() with up to 3 retries;
 *     on success sets bmp180_operational=true, gates altitude baseline.
 *   - Health watchdog gates BMP180 check on bmp180_operational AND updates
 *     last_bmp_check regardless of error path (no more tight-loop).
 *
 * This test models the production cal-loading loop and asserts:
 *   T1: every coefficient register is read in order AC1..MD and written
 *       to the matching _calCoeff field with correct signed/unsigned type.
 *   T2: bool semantics — begin() returns true on full success; false on
 *       chip-ID mismatch (without invoking the cal loop); false if any of
 *       the 11 read16 calls fails (early termination).
 *   T3: regression demonstration — with all-zero _calCoeff (the broken
 *       pre-fix runtime state), computeB5 returns 0 for ANY UT input,
 *       which is exactly the silent-failure mode that caused getPressure
 *       to hit the B4==0 guard and watchdog to fire SAFE-MODE.
 ******************************************************************************/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Mirror of BMP180_CAL_COEFF struct at BMP180.h:111-127.
 * Field types match exactly — three signed pairs (AC1/2/3), three unsigned
 * (AC4/5/6), then signed (B1/2, MB/C/D). The assignment in the cal loop
 * uses implicit narrowing from uint16_t (read16 result) to the matching
 * field type, so reproduce that here.
 * ------------------------------------------------------------------------- */
typedef struct {
    int16_t  AC1;
    int16_t  AC2;
    int16_t  AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t  B1;
    int16_t  B2;
    int16_t  MB;
    int16_t  MC;
    int16_t  MD;
} BMP180_CAL;

/* Calibration register addresses, BMP180.h:69-79. */
#define REG_AC1 0xAA
#define REG_AC2 0xAC
#define REG_AC3 0xAE
#define REG_AC4 0xB0
#define REG_AC5 0xB2
#define REG_AC6 0xB4
#define REG_B1  0xB6
#define REG_B2  0xB8
#define REG_MB  0xBA
#define REG_MC  0xBC
#define REG_MD  0xBE

#define REG_CHIP_ID    0xD0
#define BMP180_CHIP_ID 0x55

/* -------------------------------------------------------------------------
 * Mock I2C: programmed bytes per register, plus a "fail at register N"
 * counter to simulate mid-loop I2C failure.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint16_t regs16[256];      /* programmed read16 value per register addr */
    uint8_t  regs8[256];       /* programmed read8  value per register addr */
    int      fail_after_n;     /* -1 = never fail; else fail on (1+n)th read16 */
    int      read16_calls;
    int      read8_calls;
} MockI2C;

static bool mock_read16(MockI2C *m, uint8_t reg, uint16_t *out)
{
    m->read16_calls++;
    if (m->fail_after_n >= 0 && m->read16_calls > m->fail_after_n) return false;
    *out = m->regs16[reg];
    return true;
}

static bool mock_read8(MockI2C *m, uint8_t reg, uint8_t *out)
{
    m->read8_calls++;
    *out = m->regs8[reg];
    return true;
}

/* -------------------------------------------------------------------------
 * Mirror of BMP180::readCalibrationCoefficients() at BMP180.cpp:237-294.
 * Walks REG_AC1..REG_MD in 2-byte steps, calls read16 for each, dispatches
 * to the right field. Returns false on first read16 failure.
 * ------------------------------------------------------------------------- */
static bool readCalibrationCoefficients(MockI2C *m, BMP180_CAL *cal)
{
    uint16_t value = 0;
    for (uint8_t reg = REG_AC1; reg <= REG_MD; reg += 2) {
        if (!mock_read16(m, reg, &value)) return false;
        switch (reg) {
            case REG_AC1: cal->AC1 = (int16_t)value;  break;
            case REG_AC2: cal->AC2 = (int16_t)value;  break;
            case REG_AC3: cal->AC3 = (int16_t)value;  break;
            case REG_AC4: cal->AC4 = value;           break;
            case REG_AC5: cal->AC5 = value;           break;
            case REG_AC6: cal->AC6 = value;           break;
            case REG_B1:  cal->B1  = (int16_t)value;  break;
            case REG_B2:  cal->B2  = (int16_t)value;  break;
            case REG_MB:  cal->MB  = (int16_t)value;  break;
            case REG_MC:  cal->MC  = (int16_t)value;  break;
            case REG_MD:  cal->MD  = (int16_t)value;  break;
        }
    }
    return true;
}

/* Mirror of BMP180::readDeviceID at BMP180.cpp:217-223 — chip-ID probe. */
static uint8_t readDeviceID(MockI2C *m)
{
    uint8_t id = 0;
    if (!mock_read8(m, REG_CHIP_ID, &id)) return 0;
    if (id == BMP180_CHIP_ID) return 180;
    return 0;
}

/* Mirror of new BMP180::begin() in BMP180.cpp. */
static bool begin(MockI2C *m, BMP180_CAL *cal)
{
    if (readDeviceID(m) != 180) return false;
    return readCalibrationCoefficients(m, cal);
}

/* Mirror of BMP180::computeB5 at BMP180.cpp:393-399 — for the regression
 * demonstration in T3. */
static int32_t computeB5(const BMP180_CAL *cal, int32_t UT)
{
    int32_t X1 = ((UT - (int32_t)cal->AC6) * (int32_t)cal->AC5) >> 15;
    /* Cortex-M7 SDIV with DIV_0_TRP=0 returns 0 on divide-by-zero —
     * model that here so T3 reproduces the catastrophic silent-fail
     * behavior accurately. */
    int32_t denom = X1 + (int32_t)cal->MD;
    int32_t X2 = (denom == 0) ? 0 : (((int32_t)cal->MC << 11) / denom);
    return X1 + X2;
}

/* -------------------------------------------------------------------------
 * Bosch BMP180 datasheet sample calibration (Table 6).
 * ------------------------------------------------------------------------- */
static void program_datasheet_cal(MockI2C *m)
{
    memset(m->regs16, 0, sizeof(m->regs16));
    memset(m->regs8,  0, sizeof(m->regs8));
    m->regs16[REG_AC1] = (uint16_t)408;
    m->regs16[REG_AC2] = (uint16_t)(int16_t)-72;
    m->regs16[REG_AC3] = (uint16_t)(int16_t)-14383;
    m->regs16[REG_AC4] = 32741;
    m->regs16[REG_AC5] = 32757;
    m->regs16[REG_AC6] = 23153;
    m->regs16[REG_B1]  = (uint16_t)6190;
    m->regs16[REG_B2]  = (uint16_t)4;
    m->regs16[REG_MB]  = (uint16_t)(int16_t)-32768;
    m->regs16[REG_MC]  = (uint16_t)(int16_t)-8711;
    m->regs16[REG_MD]  = (uint16_t)2868;
    m->regs8[REG_CHIP_ID] = BMP180_CHIP_ID;
    m->fail_after_n  = -1;
    m->read16_calls  = 0;
    m->read8_calls   = 0;
}

/* -------------------------------------------------------------------------
 * T1: every coefficient register is read; values land in the right fields
 *     with the right signedness.
 * ------------------------------------------------------------------------- */
static void test_t1_cal_loading(void)
{
    printf("  T1: begin() loads all 11 cal coefficients in order ... ");
    MockI2C   m   = {0};
    BMP180_CAL cal = {0};
    program_datasheet_cal(&m);

    bool ok = begin(&m, &cal);
    assert(ok == true);

    /* Exactly one read8 (chip-ID probe) + 11 read16 (one per coeff). */
    assert(m.read8_calls  == 1);
    assert(m.read16_calls == 11);

    /* Every field matches the programmed datasheet value with correct
     * signed/unsigned interpretation. */
    assert(cal.AC1 ==    408);
    assert(cal.AC2 ==    -72);
    assert(cal.AC3 == -14383);
    assert(cal.AC4 ==  32741);
    assert(cal.AC5 ==  32757);
    assert(cal.AC6 ==  23153);
    assert(cal.B1  ==   6190);
    assert(cal.B2  ==      4);
    assert(cal.MB  == -32768);
    assert(cal.MC  ==  -8711);
    assert(cal.MD  ==   2868);

    printf("PASS\n");
}

/* -------------------------------------------------------------------------
 * T2: bool semantics for the three failure modes.
 * ------------------------------------------------------------------------- */
static void test_t2_failure_paths(void)
{
    printf("  T2: chip-mismatch / I2C-fail short-circuit semantics ... ");
    MockI2C m = {0};

    /* (a) Chip-ID mismatch: cal loop is NOT entered. */
    program_datasheet_cal(&m);
    m.regs8[REG_CHIP_ID] = 0xAA;     /* not 0x55 */
    BMP180_CAL cal_a = {0};
    bool ok_a = begin(&m, &cal_a);
    assert(ok_a == false);
    assert(m.read16_calls == 0);     /* short-circuited at chip-ID check */

    /* (b) I2C fails on the first cal read (after chip-ID). begin returns
     *     false; cal struct may have partial state but caller has been told
     *     "do not trust." */
    program_datasheet_cal(&m);
    m.fail_after_n = 0;              /* fail on the very next read16 */
    BMP180_CAL cal_b = {0};
    bool ok_b = begin(&m, &cal_b);
    assert(ok_b == false);
    assert(m.read16_calls >= 1);

    /* (c) I2C fails mid-loop (after 5 successful reads). */
    program_datasheet_cal(&m);
    m.fail_after_n = 5;              /* succeeds 5 times then fails */
    BMP180_CAL cal_c = {0};
    bool ok_c = begin(&m, &cal_c);
    assert(ok_c == false);
    assert(m.read16_calls == 6);     /* 5 OK + 1 fail */
    /* Partial state: AC1..AC5 set, AC6..MD untouched. The bool=false return
     * tells caller this struct is unsafe to use. */
    assert(cal_c.AC1 ==    408);
    assert(cal_c.AC2 ==    -72);
    assert(cal_c.AC5 ==  32757);
    assert(cal_c.AC6 ==      0); /* untouched after failure */
    assert(cal_c.MD  ==      0);

    printf("PASS\n");
}

/* -------------------------------------------------------------------------
 * T3: with zero-init _calCoeff (the runtime state under the original bug)
 *     computeB5 returns 0 for any UT — exactly the silent-fail mode that
 *     made getPressure return its sentinel and tripped the watchdog into
 *     SAFE-MODE within seconds of boot.
 * ------------------------------------------------------------------------- */
static void test_t3_zero_cal_silent_fail(void)
{
    printf("  T3: zero-cal computeB5 returns 0 for any UT (regression demo) ... ");
    BMP180_CAL zero = {0};

    /* Sweep raw UT across the full plausible range; with all-zero cal,
     * every call returns 0 (X1=0, denom=0 → SDIV-by-zero=0, X2=0). */
    int hits = 0;
    int total = 0;
    for (int32_t UT = 0; UT <= 65535; UT += 1024) {
        total++;
        if (computeB5(&zero, UT) == 0) hits++;
    }
    assert(hits == total);

    /* Sanity: with the datasheet calibration, computeB5 reproduces the
     * datasheet worked example (UT=27898 → B5=2399, T=15.0 °C). */
    BMP180_CAL good = {
        408, -72, -14383, 32741, 32757, 23153, 6190, 4, -32768, -8711, 2868
    };
    int32_t B5 = computeB5(&good, 27898);
    /* Datasheet algorithm: T = (B5 + 8) >> 4; expected T = 150 (= 15.0 °C). */
    int32_t T_tenths = (B5 + 8) >> 4;
    assert(T_tenths == 150);

    printf("PASS (zero-cal=%d/%d zeros, datasheet cal -> 15.0 C)\n", hits, total);
}

int main(void)
{
    printf("=== AUDIT-CAL: BMP180 begin() initialization + chip-ID gate ===\n");

    test_t1_cal_loading();
    test_t2_failure_paths();
    test_t3_zero_cal_silent_fail();

    printf("=== ALL PASS ===\n");
    return 0;
}
