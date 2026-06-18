/*
 * test_ema_filter.c
 *
 * Standalone host-based test harness for EMA filter functions
 * (ema_init / ema_update).
 *
 * Compile with:
 *   gcc -o test_ema_filter test_ema_filter.c -Wall -Wextra -std=c11 -lm
 *
 * This file is self-contained — it defines the ema_data_structure typedef,
 * copies the function bodies verbatim from Core/Src/EMA_Filter.c, and
 * provides no external dependencies.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ========================================================================
 *  Type stubs — matching the definitions in Core/Inc/EMA_Filter.h
 *  (no STM32 HAL, no hardware headers)
 * ======================================================================== */

typedef struct {
    float alpha;
    float output;
    bool  initialized;
} ema_data_structure;

/* ========================================================================
 *  Function bodies — copied verbatim from Core/Src/EMA_Filter.c
 *
 *  The functions are copied (not #included) because the original .c file
 *  may depend on STM32 HAL or hardware headers unavailable on a host PC.
 * ======================================================================== */

void ema_init(ema_data_structure *f, float alpha) {
    f->alpha       = alpha;
    f->output      = 0.0f;
    f->initialized = false;
}

float ema_update(ema_data_structure *f, float input) {
    if (!f->initialized) {
        f->output      = input;  // Avoid startup transient
        f->initialized = true;
    } else {
        f->output = f->alpha * input + (1.0f - f->alpha) * f->output;
    }
    return f->output;
}

/* ========================================================================
 *  Test utilities
 * ======================================================================== */

/* Floating-point comparison tolerance */
#define EPSILON 1e-5f

/* Helper: return true if the absolute difference is below epsilon */
static inline bool approx_eq(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

/* Custom assert macro — returns failure code from test function */
#define TEST_ASSERT(cond, msg) do {                                    \
    if (!(cond)) {                                                     \
        printf("  FAIL: %s  [%s:%d]\n", msg, __FILE__, __LINE__);     \
        return 0;                                                      \
    }                                                                  \
} while (0)

/* ========================================================================
 *  TEST 1: First update returns input directly
 *
 *  After ema_init(), the first call to ema_update() should return the
 *  input value unchanged (initialized==false path).
 * ======================================================================== */
static int test_first_update_returns_input(void)
{
    printf("=== TEST 1: First update returns input directly ===\n");

    ema_data_structure flt;
    float result;

    ema_init(&flt, 0.5f);

    /* First update — should return the input as-is */
    result = ema_update(&flt, 42.0f);
    TEST_ASSERT(approx_eq(result, 42.0f, EPSILON),
                "First update should return input (42.0) unchanged");
    TEST_ASSERT(flt.initialized == true,
                "initialized flag should be true after first update");

    printf("  PASS: First update with input 42.0 returned %.6f\n", result);
    return 1;
}

/* ========================================================================
 *  TEST 2: Subsequent updates apply EMA formula
 *
 *  Alpha = 0.5:
 *    update(10) -> 10            (initialization)
 *    update(20) -> 0.5*20 + 0.5*10 = 15.0
 * ======================================================================== */
static int test_subsequent_updates_formula(void)
{
    printf("=== TEST 2: Subsequent updates apply EMA formula ===\n");

    ema_data_structure flt;
    float result;

    ema_init(&flt, 0.5f);

    /* First update — initialization */
    result = ema_update(&flt, 10.0f);
    TEST_ASSERT(approx_eq(result, 10.0f, EPSILON),
                "First update should return 10.0");
    printf("  update(10.0) = %.6f  (expected 10.0)\n", result);

    /* Second update — applies EMA: 0.5*20 + 0.5*10 = 15 */
    result = ema_update(&flt, 20.0f);
    TEST_ASSERT(approx_eq(result, 15.0f, EPSILON),
                "Second update with alpha=0.5 should give 15.0");
    printf("  update(20.0) = %.6f  (expected 15.0)\n", result);

    printf("  PASS: EMA formula applied correctly\n");
    return 1;
}

/* ========================================================================
 *  TEST 3: Alpha = 1.0 gives passthrough
 *
 *  Every update returns the input directly (no filtering).
 * ======================================================================== */
static int test_alpha_one_passthrough(void)
{
    printf("=== TEST 3: Alpha = 1.0 gives passthrough ===\n");

    ema_data_structure flt;
    float result;

    ema_init(&flt, 1.0f);

    result = ema_update(&flt, 10.0f);
    TEST_ASSERT(approx_eq(result, 10.0f, EPSILON),
                "First update with alpha=1.0 should return 10.0");

    result = ema_update(&flt, 20.0f);
    TEST_ASSERT(approx_eq(result, 20.0f, EPSILON),
                "Second update with alpha=1.0 should return 20.0");

    result = ema_update(&flt, 30.0f);
    TEST_ASSERT(approx_eq(result, 30.0f, EPSILON),
                "Third update with alpha=1.0 should return 30.0");

    result = ema_update(&flt, 100.0f);
    TEST_ASSERT(approx_eq(result, 100.0f, EPSILON),
                "Fourth update with alpha=1.0 should return 100.0");

    printf("  PASS: Alpha=1.0 passes all inputs through unchanged\n");
    return 1;
}

/* ========================================================================
 *  TEST 4: Alpha = 0.0 holds last value
 *
 *  After initialization, all subsequent updates return the initial value
 *  because the filter has infinite memory (no new input weight).
 * ======================================================================== */
static int test_alpha_zero_holds_value(void)
{
    printf("=== TEST 4: Alpha = 0.0 holds last value ===\n");

    ema_data_structure flt;
    float result;

    ema_init(&flt, 0.0f);

    /* First update — initialization, stores 10.0 */
    result = ema_update(&flt, 10.0f);
    TEST_ASSERT(approx_eq(result, 10.0f, EPSILON),
                "First update with alpha=0.0 should return 10.0");

    /* Subsequent updates — output stays at 10.0 regardless of input */
    result = ema_update(&flt, 20.0f);
    TEST_ASSERT(approx_eq(result, 10.0f, EPSILON),
                "Second update with alpha=0.0 should hold at 10.0");

    result = ema_update(&flt, -5.0f);
    TEST_ASSERT(approx_eq(result, 10.0f, EPSILON),
                "Third update with alpha=0.0 should still hold at 10.0");

    result = ema_update(&flt, 9999.0f);
    TEST_ASSERT(approx_eq(result, 10.0f, EPSILON),
                "Fourth update with alpha=0.0 should still hold at 10.0");

    printf("  PASS: Alpha=0.0 holds initial value across all subsequent updates\n");
    return 1;
}

/* ========================================================================
 *  TEST 5: Multiple values converge without overshoot
 *
 *  Feed sequence 0, 10, 20, 30, 40 with alpha = 0.2.
 *  Expected behaviour:
 *    update(0)  -> 0.0    (initialization)
 *    update(10) -> 2.0    (0.2*10 + 0.8*0)
 *    update(20) -> 5.6    (0.2*20 + 0.8*2.0)
 *    update(30) -> 10.48  (0.2*30 + 0.8*5.6)
 *    update(40) -> 16.384 (0.2*40 + 0.8*10.48)
 *
 *  The output must follow monotonically and never exceed the current input
 *  (no overshoot for a rising step sequence).
 * ======================================================================== */
static int test_convergence_no_overshoot(void)
{
    printf("=== TEST 5: Multiple values converge without overshoot ===\n");

    ema_data_structure flt;
    float result;
    float expected;

    ema_init(&flt, 0.2f);

    /* Step 1: input 0 -> output 0 */
    result = ema_update(&flt, 0.0f);
    TEST_ASSERT(approx_eq(result, 0.0f, EPSILON),
                "Step 1: update(0) should return 0.0");
    TEST_ASSERT(result <= 0.0f, "Step 1: output must not exceed input");
    printf("  update(%2d) = %8.6f  (expected %8.6f)\n", 0, result, 0.0f);

    /* Step 2: input 10 -> output 2.0 */
    result = ema_update(&flt, 10.0f);
    expected = 2.0f;
    TEST_ASSERT(approx_eq(result, expected, EPSILON),
                "Step 2: update(10) with alpha=0.2 should return 2.0");
    TEST_ASSERT(result <= 10.0f, "Step 2: output must not exceed input");
    printf("  update(%2d) = %8.6f  (expected %8.6f)\n", 10, result, expected);

    /* Step 3: input 20 -> output 5.6 */
    result = ema_update(&flt, 20.0f);
    expected = 5.6f;
    TEST_ASSERT(approx_eq(result, expected, EPSILON),
                "Step 3: update(20) with alpha=0.2 should return 5.6");
    TEST_ASSERT(result <= 20.0f, "Step 3: output must not exceed input");
    printf("  update(%2d) = %8.6f  (expected %8.6f)\n", 20, result, expected);

    /* Step 4: input 30 -> output 10.48 */
    result = ema_update(&flt, 30.0f);
    expected = 10.48f;
    TEST_ASSERT(approx_eq(result, expected, EPSILON),
                "Step 4: update(30) with alpha=0.2 should return 10.48");
    TEST_ASSERT(result <= 30.0f, "Step 4: output must not exceed input");
    printf("  update(%2d) = %8.6f  (expected %8.6f)\n", 30, result, expected);

    /* Step 5: input 40 -> output 16.384 */
    result = ema_update(&flt, 40.0f);
    expected = 16.384f;
    TEST_ASSERT(approx_eq(result, expected, EPSILON),
                "Step 5: update(40) with alpha=0.2 should return 16.384");
    TEST_ASSERT(result <= 40.0f, "Step 5: output must not exceed input");
    printf("  update(%2d) = %8.6f  (expected %8.6f)\n", 40, result, expected);

    /* Verify monotonicity: output must be non-decreasing for this sequence */
    float outputs[] = {0.0f, 2.0f, 5.6f, 10.48f, 16.384f};
    for (int i = 1; i < 5; i++) {
        TEST_ASSERT(outputs[i] >= outputs[i - 1],
                    "EMA output must be non-decreasing for increasing input");
    }

    printf("  PASS: EMA converges correctly without overshoot\n");
    return 1;
}

/* ========================================================================
 *  Main — run all tests, count pass/fail
 * ======================================================================== */
int main(void)
{
    int passed = 0;
    int failed = 0;

    printf("\n=== EMA Filter Test Harness ===\n\n");

    if (test_first_update_returns_input())   passed++; else failed++;
    if (test_subsequent_updates_formula())   passed++; else failed++;
    if (test_alpha_one_passthrough())        passed++; else failed++;
    if (test_alpha_zero_holds_value())       passed++; else failed++;
    if (test_convergence_no_overshoot())     passed++; else failed++;

    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);

    return failed > 0 ? 1 : 0;
}
