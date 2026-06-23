/*
 * test_toggle_wdt.c
 *
 * Standalone host-based unit test for toggle_wdt()
 * from Core/Src/state_machine.c.
 *
 * Tests: timing guard (10 ms), WDT enabled/disabled, toggle pin count.
 *
 * Compile:
 *   gcc -o test_toggle_wdt test_toggle_wdt.c -Wall -Wextra -std=c11 -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ========================================================================
 *  HAL / GPIO stubs  (minimal — just enough for toggle_wdt)
 * ======================================================================== */

typedef uint16_t GPIO_PinState;
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET   1

#define GPIO_PIN_0  ((uint16_t)0x0001)

typedef struct { uint32_t dummy; } GPIO_TypeDef;

static GPIO_TypeDef GPIOC_dummy;
static GPIO_TypeDef * const GPIOC = &GPIOC_dummy;

#define WDT_PULSE_Pin       GPIO_PIN_0
#define WDT_PULSE_GPIO_Port GPIOC

static int hal_gpio_toggle_call_count = 0;
static uint16_t last_toggle_pin = 0;
static GPIO_TypeDef *last_toggle_port = NULL;

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    hal_gpio_toggle_call_count++;
    last_toggle_port = port;
    last_toggle_pin  = pin;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)port; (void)pin; (void)state;
}

/* ========================================================================
 *  Global state stubs  (from main.h / main.c)
 * ======================================================================== */

struct car {
    uint8_t HW_WDT_Enable;
    uint8_t front_solenoid;
    uint8_t rear_solenoid;
    uint8_t Emergency;
    uint8_t Ignition_Request;
    uint8_t rpm;
};

static struct car t24;

/* ========================================================================
 *  Fake time
 * ======================================================================== */

static uint32_t fake_time_ms = 0;

uint32_t millis(void) { return fake_time_ms; }
uint32_t HAL_GetTick(void) { return fake_time_ms; }

/* ========================================================================
 *  Function under test — copied verbatim from Core/Src/state_machine.c
 *
 *  NOTE: wdt_time promoted from block-scope static to file-scope static
 *  so test reset_globals() can clear it between tests (behavior is
 *  identical — still file-internal linkage and persistent across calls).
 * ======================================================================== */

static unsigned long wdt_time = 0;

void toggle_wdt() {
    if (millis() - wdt_time >= 10) {
        if (t24.HW_WDT_Enable == 1) {
            HAL_GPIO_TogglePin(WDT_PULSE_GPIO_Port, WDT_PULSE_Pin);
            wdt_time = millis();
        }
    }
}

/* ========================================================================
 *  Test infrastructure
 * ======================================================================== */

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do {                                     \
        if (!(cond)) {                                                  \
            printf("  FAIL: %s  (line %d)\n", msg, __LINE__);           \
            tests_failed++;                                             \
            return 0;                                                   \
        }                                                               \
        printf("  PASS: %s\n", msg);                                    \
        tests_passed++;                                                 \
    } while(0)

static void reset_globals(void)
{
    memset(&t24, 0, sizeof(t24));
    fake_time_ms = 0;
    hal_gpio_toggle_call_count = 0;
    wdt_time = 0;
}

/* ========================================================================
 *  Test cases
 * ======================================================================== */

/* TEST 1: WDT disabled — no toggle despite time passing */
static int test_wdt_disabled(void)
{
    printf("=== TEST 1: WDT disabled — no toggle ===\n");
    reset_globals();

    t24.HW_WDT_Enable = 0;
    fake_time_ms = 100;

    toggle_wdt();

    TEST_ASSERT(hal_gpio_toggle_call_count == 0,
                "no toggle when HW_WDT_Enable == 0");
    return 1;
}

/* TEST 2: WDT enabled, first call — toggles immediately (wdt_time starts at 0, millis >= 10) */
static int test_wdt_enabled_first_call(void)
{
    printf("=== TEST 2: WDT enabled, first call ===\n");
    reset_globals();

    t24.HW_WDT_Enable = 1;
    fake_time_ms = 10;

    toggle_wdt();

    TEST_ASSERT(hal_gpio_toggle_call_count == 1,
                "toggle occurs on first call with time >= 10");
    TEST_ASSERT(last_toggle_port == WDT_PULSE_GPIO_Port,
                "toggle on WDT_PULSE_GPIO_Port");
    TEST_ASSERT(last_toggle_pin == WDT_PULSE_Pin,
                "toggle on WDT_PULSE_Pin");
    return 1;
}

/* TEST 3: Two calls within 10 ms — only one toggle */
static int test_wdt_too_soon_skips(void)
{
    printf("=== TEST 3: Too soon — second call skipped ===\n");
    reset_globals();

    t24.HW_WDT_Enable = 1;
    fake_time_ms = 100;

    toggle_wdt();                 /* first: toggles */
    TEST_ASSERT(hal_gpio_toggle_call_count == 1,
                "first call toggles");

    fake_time_ms = 105;           /* only 5 ms later */
    toggle_wdt();                 /* should NOT toggle */

    TEST_ASSERT(hal_gpio_toggle_call_count == 1,
                "second call skipped (only 5 ms elapsed)");
    return 1;
}

/* TEST 4: Two calls 10 ms apart — both toggle */
static int test_wdt_second_after_10ms(void)
{
    printf("=== TEST 4: 10 ms apart — both toggle ===\n");
    reset_globals();

    t24.HW_WDT_Enable = 1;
    fake_time_ms = 100;

    toggle_wdt();
    TEST_ASSERT(hal_gpio_toggle_call_count == 1,
                "first call toggles");

    fake_time_ms = 110;           /* exactly 10 ms later */
    toggle_wdt();

    TEST_ASSERT(hal_gpio_toggle_call_count == 2,
                "second call toggles after 10 ms");
    return 1;
}

/* TEST 5: WDT disabled mid-cycle — stops toggling */
static int test_wdt_disabled_mid_cycle(void)
{
    printf("=== TEST 5: WDT disabled mid-cycle ===\n");
    reset_globals();

    t24.HW_WDT_Enable = 1;
    fake_time_ms = 100;

    toggle_wdt();
    TEST_ASSERT(hal_gpio_toggle_call_count == 1,
                "first call toggles");

    t24.HW_WDT_Enable = 0;
    fake_time_ms = 200;

    toggle_wdt();
    TEST_ASSERT(hal_gpio_toggle_call_count == 1,
                "no toggle after WDT disabled (time elapsed but WDT off)");

    t24.HW_WDT_Enable = 1;
    fake_time_ms = 300;

    toggle_wdt();
    TEST_ASSERT(hal_gpio_toggle_call_count == 2,
                "toggle resumes after WDT re-enabled");
    return 1;
}

/* ========================================================================
 *  Main
 * ======================================================================== */

int main(void)
{
    printf("========================================\n");
    printf(" toggle_wdt() Unit Tests\n");
    printf("========================================\n");

    int (*test_fn[])(void) = {
        test_wdt_disabled,
        test_wdt_enabled_first_call,
        test_wdt_too_soon_skips,
        test_wdt_second_after_10ms,
        test_wdt_disabled_mid_cycle,
    };
    const int num_tests = sizeof(test_fn) / sizeof(test_fn[0]);

    for (int i = 0; i < num_tests; i++) {
        printf("\n========================================\n");
        printf(" Test %d\n", i + 1);
        printf("========================================\n");
        int ret = test_fn[i]();
        if (ret == 0) {
            printf("\n  *** TEST %d FAILED ***\n", i + 1);
        } else {
            printf("\n  +++ TEST %d PASSED +++\n", i + 1);
        }
    }

    printf("\n========================================\n");
    printf(" RESULTS\n");
    printf("========================================\n");
    printf("  Passed assertions: %d\n", tests_passed);
    printf("  Failed assertions: %d\n", tests_failed);

    if (tests_failed > 0) {
        printf("\n  >>> SOME TESTS FAILED <<<\n");
        return 1;
    }
    printf("\n  >>> ALL TESTS PASSED <<<\n");
    return 0;
}
