/*
 * test_temperature.c
 *
 * Standalone host-based test harness for GetTemperature()
 * from Core/Src/main.c.
 *
 * Compile with:
 *   gcc -o test_temperature test_temperature.c -Wall -Wextra -std=c11 -lm
 *
 * Self-contained — copies the function body verbatim from main.c,
 * replacing direct pointer dereferences with volatile fake_* globals
 * that tests can override.
 *
 * Calibration addresses (from stm32f4xx_ll_adc.h and main.c):
 *   VREFINT_CAL_ADDR  = 0x1FFF7A2A
 *   TS_CAL1_ADDR      = 0x1FFF7A2C  (raw @ 30C, Vdda = 3.3V)
 *   TS_CAL2_ADDR      = 0x1FFF7A2E  (raw @ 110C, Vdda = 3.3V)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ========================================================================
 *  Calibration constants — mirrors Core/Src/main.c
 * ======================================================================== */

/* VREFINT calibration address (from stm32f4xx_ll_adc.h) */
#define VREFINT_CAL_ADDR  ((uint16_t*)0x1FFF7A2A)

/* Temperature calibration addresses (from main.c) */
#define TS_CAL1_ADDR      ((uint16_t*)0x1FFF7A2C)  /* Raw @ 30C, Vdda=3.3V */
#define TS_CAL2_ADDR      ((uint16_t*)0x1FFF7A2E)  /* Raw @ 110C, Vdda=3.3V */

#define TEMP_CAL1_TEMPC   30.0f
#define TEMP_CAL2_TEMPC   110.0f

/* ========================================================================
 *  Volatile globals for calibration values — tests override these to
 *  simulate factory-calibrated values without reading hardware addresses.
 * ======================================================================== */

static volatile uint16_t fake_vrefint_cal = 1500;   /* typical VREFINT_CAL */
static volatile uint16_t fake_ts_cal1     = 1730;   /* typical @30C */
static volatile uint16_t fake_ts_cal2     = 2160;   /* typical @110C */

/* ========================================================================
 *  Function body — copied verbatim from Core/Src/main.c lines 237-257,
 *  but modified to use fake_* globals instead of direct pointer dereferences
 *  to hardware calibration addresses.
 *
 *  Original (main.c):
 *    float vdda = 3.3f * ((float) (*VREFINT_CAL_ADDR) / (float) raw_vref);
 *    float ts_cal1 = (float) (*TS_CAL1_ADDR);
 *    float ts_cal2 = (float) (*TS_CAL2_ADDR);
 *
 *  Modified to use fake_* globals so tests can control calibration values.
 * ======================================================================== */

float GetTemperature(uint16_t raw_temp, uint16_t raw_vref) {
    if (raw_vref == 0)
        return 0.0f;

    // Use fake globals for calibration values so tests can control them
    float vdda = 3.3f * ((float)(fake_vrefint_cal) / (float)raw_vref);
    float raw_equiv_3v3 = (float)raw_temp * (vdda / 3.3f);
    float ts_cal1 = (float)(fake_ts_cal1);
    float ts_cal2 = (float)(fake_ts_cal2);
    float temperature = ((raw_equiv_3v3 - ts_cal1) * (TEMP_CAL2_TEMPC - TEMP_CAL1_TEMPC) / (ts_cal2 - ts_cal1))
            + TEMP_CAL1_TEMPC;
    return temperature;
}

/* ========================================================================
 *  Test utilities
 * ======================================================================== */

/* Floating-point comparison tolerance (default for most tests) */
#define EPSILON 1e-4f

/* Helper: return true if the absolute difference is below epsilon */
static inline bool approx_eq(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

/* Custom assert macro — prints PASS/FAIL and returns 0 from test function on failure */
#define TEST_ASSERT(cond, msg) do {                                    \
    if (!(cond)) {                                                     \
        printf("  FAIL: %s  [%s:%d]\n", msg, __FILE__, __LINE__);     \
        return 0;                                                      \
    }                                                                  \
    printf("  PASS: %s\n", msg);                                       \
} while (0)

/* ========================================================================
 *  TEST 1: raw_vref == 0 returns 0.0f immediately
 *
 *  When raw_vref is zero, the function should early-return 0.0f to avoid
 *  division by zero in the VDDA calculation.
 * ======================================================================== */
static int test_vref_zero_returns_zero(void)
{
    printf("=== TEST 1: raw_vref zero returns 0.0f ===\n");

    float result = GetTemperature(1000, 0);
    TEST_ASSERT(result == 0.0f,
                "GetTemperature with raw_vref=0 should return 0.0f");
    TEST_ASSERT(result == 0.0f && !(result > 0.0f),
                "Return value must be exactly zero, not negative zero or small positive");

    printf("  PASS: raw_vref=0 returns %.6f (expected 0.0)\n", result);
    return 1;
}

/* ========================================================================
 *  TEST 2: Known temperature — 30.0C at perfect 3.3V supply
 *
 *  With factory calibration (1730 @30C, 2160 @110C) and perfect 3.3V
 *  supply (raw_vref == fake_vrefint_cal == 1500), raw_temp == ts_cal1
 *  should produce exactly 30.0C.
 *
 *    vdda = 3.3 * (1500 / 1500) = 3.3
 *    raw_equiv_3v3 = 1730 * (3.3 / 3.3) = 1730
 *    temp = ((1730 - 1730) * 80 / 430) + 30 = 0 + 30 = 30.0
 * ======================================================================== */
static int test_known_temperature(void)
{
    printf("=== TEST 2: Known temperature returns 30.0C ===\n");

    /* Restore defaults */
    fake_ts_cal1     = 1730;
    fake_ts_cal2     = 2160;
    fake_vrefint_cal = 1500;

    float result = GetTemperature(1730, 1500);
    TEST_ASSERT(approx_eq(result, 30.0f, EPSILON),
                "GetTemperature(1730, 1500) should return 30.0C");
    TEST_ASSERT(result >= 0.0f,
                "Temperature must be non-negative");

    printf("  PASS: GetTemperature(1730, 1500) = %.6f (expected 30.0)\n", result);
    return 1;
}

/* ========================================================================
 *  TEST 3: Higher temperature — 61.63C
 *
 *  Same calibration, raw_temp = 1900 (above cal1).
 *  Expected:
 *    temp = ((1900 - 1730) * 80 / 430) + 30
 *         = (170 * 80 / 430) + 30
 *         = 31.627907 + 30
 *         = 61.627907C
 *  Allow epsilon 0.1.
 * ======================================================================== */
static int test_higher_temperature(void)
{
    printf("=== TEST 3: Higher temperature (1900 raw) ===\n");

    fake_ts_cal1     = 1730;
    fake_ts_cal2     = 2160;
    fake_vrefint_cal = 1500;

    float result = GetTemperature(1900, 1500);
    float expected = 61.6279069767f;  /* (170*80/430) + 30 */

    TEST_ASSERT(approx_eq(result, expected, 0.1f),
                "GetTemperature(1900, 1500) should return ~61.63C");
    TEST_ASSERT(result > 30.0f,
                "Temperature with raw_temp > ts_cal1 must be > 30C");

    printf("  PASS: GetTemperature(1900, 1500) = %.6f (expected %.6f)\n",
           result, expected);
    return 1;
}

/* ========================================================================
 *  TEST 4: VREF variation — same physical temperature (30°C) at lower VDDA
 *
 *  Calibration constants are the defaults (ts_cal1=1730@30C, ts_cal2=2160@110C,
 *  vrefint_cal=1500). raw_vref=1600 means VDDA = 3.3 * 1500/1600 = 3.09375V.
 *
 *  At this lower VDDA the ADC reading for the *same* physical 30°C
 *  temperature is NOT 1730 (which was captured at 3.3V), but higher:
 *      raw_temp = TS_CAL1 * 3.3 / VDDA = 1730 * 3.3 / 3.09375 ≈ 1845
 *
 *  The formula should compensate via the raw_equiv_3v3 adjustment back to
 *  ~30.0C. Tolerance 1.0C due to the compensation adjustment.
 * ======================================================================== */
static int test_vref_variation(void)
{
    printf("=== TEST 4: VREF variation compensates temperature ===\n");

    fake_ts_cal1     = 1730;
    fake_ts_cal2     = 2160;
    fake_vrefint_cal = 1500;

    /* Same physical temperature (30°C) but VDDA is lower (raw_vref=1600):
     *   VDDA = 3.3 * 1500/1600 = 3.09375V
     * At this VDDA, the ADC reading for 30°C would be:
     *   raw_temp = TS_CAL1 * 3.3 / VDDA = 1730 * 3.3 / 3.094 = 1845
     * The formula should compensate back to ~30°C. */
    float result = GetTemperature(1845, 1600);

    TEST_ASSERT(approx_eq(result, 30.0f, 1.0f),
                "With VREF variation (raw_vref=1600), GetTemperature(1845,1600) should give ~30.0C");

    printf("  PASS: GetTemperature(1845, 1600) = %.6f (expected ~30.0 within 1.0C)\n",
           result);
    return 1;
}

/* ========================================================================
 *  TEST 5: Cal1 equals Cal2 — division by zero
 *
 *  If fake_ts_cal1 == fake_ts_cal2, the denominator (ts_cal2 - ts_cal1)
 *  is zero. IEEE 754 floating-point division by zero produces +inf (or
 *  -inf depending on the sign of the numerator). The test verifies that
 *  the result is NOT finite using isfinite().
 * ======================================================================== */
static int test_cal1_equals_cal2(void)
{
    printf("=== TEST 5: Cal1 equals Cal2 (division by zero) ===\n");

    fake_ts_cal1     = 1730;
    fake_ts_cal2     = 1730;  /* same as cal1 => denominator = 0 */
    fake_vrefint_cal = 1500;

    float result = GetTemperature(2000, 1500);

    /* With denominator zero, the division yields infinity */
    TEST_ASSERT(!isfinite(result),
                "Division by zero should produce a non-finite (infinity) result");

    printf("  PASS: GetTemperature with ts_cal1==ts_cal2 returns non-finite value "
           "(isfinite=%d, value=%f)\n", isfinite(result), result);
    return 1;
}

/* ========================================================================
 *  Main — run all tests, count pass/fail
 * ======================================================================== */
int main(void)
{
    int passed = 0;
    int failed = 0;

    printf("\n=== Temperature Sensor Test Harness ===\n\n");

    if (test_vref_zero_returns_zero())   passed++; else failed++;
    if (test_known_temperature())        passed++; else failed++;
    if (test_higher_temperature())       passed++; else failed++;
    if (test_vref_variation())           passed++; else failed++;
    if (test_cal1_equals_cal2())         passed++; else failed++;

    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);

    return failed > 0 ? 1 : 0;
}
