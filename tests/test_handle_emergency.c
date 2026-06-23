/*
 * test_handle_emergency.c
 *
 * Standalone host-based unit test for Handle_Emergency()
 * from Core/Src/state_machine.c.
 *
 * Tests: WDT disable, ignition off, solenoids off, Emergency flag set.
 *
 * Compile:
 *   gcc -o test_handle_emergency test_handle_emergency.c -Wall -Wextra -std=c11 -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ========================================================================
 *  HAL / GPIO stubs
 * ======================================================================== */

typedef uint16_t GPIO_PinState;
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET   1

typedef struct { uint32_t dummy; } GPIO_TypeDef;

static GPIO_TypeDef front_solenoid_port_dummy;
static GPIO_TypeDef rear_solenoid_port_dummy;
static GPIO_TypeDef * const Front_Solenoid_GPIO_Port = &front_solenoid_port_dummy;
static GPIO_TypeDef * const Rear_Solenoid_GPIO_Port  = &rear_solenoid_port_dummy;

#define Front_Solenoid_Pin ((uint16_t)0x0001)
#define Rear_Solenoid_Pin  ((uint16_t)0x0002)

static int hal_gpio_write_call_count = 0;
static GPIO_TypeDef *last_write_port = NULL;
static uint16_t last_write_pin = 0;
static GPIO_PinState last_write_state = 0;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    hal_gpio_write_call_count++;
    last_write_port  = port;
    last_write_pin   = pin;
    last_write_state = state;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
}

/* ========================================================================
 *  Global state stubs
 * ======================================================================== */

struct car {
    uint8_t HW_WDT_Enable;
    uint8_t Ignition_Request;
    uint8_t Emergency;
    uint8_t front_solenoid;
    uint8_t rear_solenoid;
    uint8_t rpm;
    uint8_t ASMS;
};

static struct car t24;

/* ========================================================================
 *  Function under test — copied verbatim from Core/Src/state_machine.c
 * ======================================================================== */

void Handle_Emergency() {
    t24.HW_WDT_Enable = 0;
    t24.Ignition_Request = 0;
    t24.Emergency = 1;
    HAL_GPIO_WritePin(Front_Solenoid_GPIO_Port, Front_Solenoid_Pin,
            GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Rear_Solenoid_GPIO_Port, Rear_Solenoid_Pin,
            GPIO_PIN_RESET);
    t24.front_solenoid = 0;
    t24.rear_solenoid = 0;
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
    hal_gpio_write_call_count = 0;
}

/* ========================================================================
 *  Test cases
 * ======================================================================== */

/* TEST 1: Handle_Emergency sets all fields correctly */
static int test_emergency_sets_fields(void)
{
    printf("=== TEST 1: Handle_Emergency sets fields correctly ===\n");
    reset_globals();

    /* Arrange: set some fields to non-default values */
    t24.HW_WDT_Enable    = 1;
    t24.Ignition_Request = 1;
    t24.Emergency        = 0;
    t24.front_solenoid   = 1;
    t24.rear_solenoid    = 1;

    Handle_Emergency();

    TEST_ASSERT(t24.HW_WDT_Enable == 0,    "WDT disabled");
    TEST_ASSERT(t24.Ignition_Request == 0,  "ignition off");
    TEST_ASSERT(t24.Emergency == 1,         "Emergency flag set");
    TEST_ASSERT(t24.front_solenoid == 0,    "front solenoid off");
    TEST_ASSERT(t24.rear_solenoid == 0,     "rear solenoid off");
    return 1;
}

/* TEST 2: HAL_GPIO_WritePin called for both solenoids with RESET */
static int test_emergency_writes_gpio(void)
{
    printf("=== TEST 2: GPIO writes for both solenoids ===\n");
    reset_globals();

    Handle_Emergency();

    TEST_ASSERT(hal_gpio_write_call_count == 2,
                "two HAL_GPIO_WritePin calls");

    TEST_ASSERT(last_write_port == Rear_Solenoid_GPIO_Port,
                "last write on rear solenoid port");
    TEST_ASSERT(last_write_pin == Rear_Solenoid_Pin,
                "last write on rear solenoid pin");
    TEST_ASSERT(last_write_state == GPIO_PIN_RESET,
                "last write is RESET");
    return 1;
}

/* TEST 3: Sets Emergency=1 even when already in emergency */
static int test_emergency_idempotent(void)
{
    printf("=== TEST 3: Idempotent — emergency already active ===\n");
    reset_globals();

    t24.Emergency = 1;
    t24.HW_WDT_Enable = 0;
    t24.front_solenoid = 0;

    Handle_Emergency();

    TEST_ASSERT(t24.Emergency == 1,   "Emergency stays 1");
    TEST_ASSERT(t24.HW_WDT_Enable == 0, "WDT stays 0");
    TEST_ASSERT(t24.front_solenoid == 0, "solenoid stays 0");
    return 1;
}

/* ========================================================================
 *  Main
 * ======================================================================== */

int main(void)
{
    printf("========================================\n");
    printf(" Handle_Emergency() Unit Tests\n");
    printf("========================================\n");

    int (*test_fn[])(void) = {
        test_emergency_sets_fields,
        test_emergency_writes_gpio,
        test_emergency_idempotent,
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
