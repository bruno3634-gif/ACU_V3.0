/*
 * test_adc_callbacks.c
 * Standalone host-based unit tests for HAL_ADC_ConvCpltCallback()
 * and HAL_GPIO_EXTI_Callback() from Core/Src/main.c.
 *
 * Tests the pressure-sensor chain (raw ADC → voltage → pressure) and
 * the mission-selector button EXTI handler.
 *
 * The EMA filter is stubbed (pass-through) since it is tested separately
 * in test_ema_filter.c.  GetTemperature() is a verbatim copy of the
 * version in test_temperature.c, using volatile fake_* globals so tests
 * can control factory calibration values.
 *
 * Compile:
 *   gcc -o test_adc_callbacks test_adc_callbacks.c -Wall -Wextra -std=c11 -lm
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

/* ==================================================================
 * HAL / main.h type stubs
 * ================================================================== */

/* ADC handle -- only the address matters for testing */
typedef struct { uint32_t dummy; } ADC_HandleTypeDef;

/* TIM handle (needed for compilation even if unused) */
typedef struct { uint32_t dummy; } TIM_HandleTypeDef;

ADC_HandleTypeDef hadc1;   /* referenced in ring_buffer code (unused here) */

/* ==================================================================
 * Mission / state enums  (from main.h)
 * ================================================================== */

typedef enum {
    MANUAL,       /* 0 */
    ACCELERATION, /* 1 */
    SKIDPAD,      /* 2 */
    TRACKDRIVE,   /* 3 */
    EBS_TEST,     /* 4 */
    INSPECTION,   /* 5 */
    AUTOCROSS     /* 6 */
} current_mission_t;

typedef enum {
    AS_STATE_OFF       = 1,
    AS_STATE_READY     = 2,
    AS_STATE_DRIVING   = 3,
    AS_STATE_EMERGENCY = 4,
    AS_STATE_FINISHED  = 5
} AS_STATE_t;

/* ==================================================================
 * Data structures  (from main.h)
 * ================================================================== */

struct pressure {
    float Pneumatic;
    float Hydraulic;
};

struct speed {
    uint8_t Speed;          /* Km/h */
    uint8_t Target_Speed;   /* Km/h */
};

struct car {
    struct pressure Rear_Pressure;
    struct pressure Front_Pressure;
    uint8_t front_solenoid;
    uint8_t rear_solenoid;
    uint8_t Ignition_Status;
    uint8_t Ignition_Request;
    uint8_t ASMS;
    uint8_t Emergency;
    uint8_t Res;
    uint8_t HW_WDT_Enable;
    uint8_t ignition_pin_state;
    uint8_t SDC_feedback;
    uint8_t ASSI_state;
    uint8_t vcu_sdc;
    volatile AS_STATE_t Autonomous_State;
    volatile current_mission_t Current_Mission;
    volatile current_mission_t Jetson_mission;
    struct speed Speed;
    float chip_temp;
    int rpm;
    uint32_t VCU_LAST_TX, REAR_PRESSURE_LAST_TX;
    uint32_t JETSON_LAST_TX, DIR_ACTUATOR_LAST_TX, RES_LAST_TX;
};

/* The global car instance -- used by the callbacks under test */
struct car t24;

/* ==================================================================
 * EMA filter stubs  (test_ema_filter.c tests the real implementation)
 * ================================================================== */

typedef struct {
    float alpha;
    float output;
    bool  initialized;
} ema_data_structure;

/* Global EMA instances referenced by HAL_ADC_ConvCpltCallback */
static ema_data_structure ema_front_pressure;
static ema_data_structure ema_rear_pressure;

static void ema_init(ema_data_structure *f, float alpha)
{
    f->alpha = alpha;
    f->output = 0.0f;
    f->initialized = false;
}

/*
 * Pass-through stub -- stores the input directly as the filtered output.
 * The real EMA filter's time-constant responsiveness is tested separately
 * in test_ema_filter.c.
 */
static float ema_update(ema_data_structure *f, float input)
{
    f->output = input;
    f->initialized = true;
    return input;
}

/* ==================================================================
 * Globals from main.c
 * ================================================================== */

uint32_t ADC_Samples[4];
uint8_t mission_selector_enable = 0;

/* ==================================================================
 * GPIO pin defines  (from main.h)
 * ================================================================== */

#define GPIO_PIN_3   ((uint16_t)3)
#define GPIO_PIN_5   ((uint16_t)5)
#define MS_BTN_Pin   GPIO_PIN_3

/* ==================================================================
 * Calibration constants and stubs for GetTemperature()
 * (Matches test_temperature.c approach -- volatile fake_* globals
 *  allow tests to control factory-cal values without hardware access.)
 * ================================================================== */

/* Temperature calibration addresses (from main.c / stm32f4xx_ll_adc.h) */
#define VREFINT_CAL_ADDR  ((uint16_t*)0x1FFF7A2A)
#define TS_CAL1_ADDR      ((uint16_t*)0x1FFF7A2C)   /* Raw @ 30C, Vdda=3.3V */
#define TS_CAL2_ADDR      ((uint16_t*)0x1FFF7A2E)   /* Raw @ 110C, Vdda=3.3V */

#define TEMP_CAL1_TEMPC   30.0f
#define TEMP_CAL2_TEMPC   110.0f

/* Volatile globals that GetTemperature() uses instead of direct
 * hardware-pointer dereferences -- tests override these freely. */
static volatile uint16_t fake_vrefint_cal = 1500;   /* typical VREFINT_CAL */
static volatile uint16_t fake_ts_cal1     = 1730;   /* typical @30C */
static volatile uint16_t fake_ts_cal2     = 2160;   /* typical @110C */

/* ==================================================================
 * HAL_GetTick stub
 * ================================================================== */

static uint32_t fake_tick = 0;

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

/* ==================================================================
 * Function bodies copied VERBATIM from Core/Src/main.c
 *   (with minimal adaptation for host testability)
 * ================================================================== */

/* ------------------------------------------------------------------
 * GetTemperature -- verbatim from test_temperature.c, which is
 * itself a verbatim copy of the GetTemperature() in main.c with
 * hardware-pointer dereferences replaced by volatile fake_* globals.
 * ------------------------------------------------------------------ */
static float GetTemperature(uint16_t raw_temp, uint16_t raw_vref)
{
    if (raw_vref == 0)
        return 0.0f;

    float vdda = 3.3f * ((float)(fake_vrefint_cal) / (float)raw_vref);
    float raw_equiv_3v3 = (float)raw_temp * (vdda / 3.3f);
    float ts_cal1 = (float)(fake_ts_cal1);
    float ts_cal2 = (float)(fake_ts_cal2);
    float temperature = ((raw_equiv_3v3 - ts_cal1) * (TEMP_CAL2_TEMPC - TEMP_CAL1_TEMPC)
                        / (ts_cal2 - ts_cal1)) + TEMP_CAL1_TEMPC;
    return temperature;
}

/* ------------------------------------------------------------------
 * HAL_ADC_ConvCpltCallback -- from main.c lines 222-235.
 *
 * Adaptation: the Instance check (hadc->Instance == ADC1) is removed
 * because we do not define the full STM32 HAL ADC_TypeDef in this
 * host-based test.  The conversion math is identical to the original.
 * EMA calls use the pass-through stub defined above.
 * ------------------------------------------------------------------ */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;

    float raw_voltage_front = (ADC_Samples[0] * 3.3 / 4096) / 0.66;
    float raw_voltage_rear  = (ADC_Samples[1] * 3.3 / 4096) / 0.66;

    t24.chip_temp = GetTemperature(ADC_Samples[2], ADC_Samples[3]);

    float front_pressure = (raw_voltage_front - 0.5) / 0.4;
    float rear_pressure  = (raw_voltage_rear  - 0.5) / 0.4;

    t24.Front_Pressure.Pneumatic = ema_update(&ema_front_pressure, front_pressure);
    t24.Rear_Pressure.Pneumatic  = ema_update(&ema_rear_pressure, rear_pressure);
}

/* ------------------------------------------------------------------
 * HAL_GPIO_EXTI_Callback -- verbatim from main.c lines 348-355.
 * ------------------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MS_BTN_Pin && mission_selector_enable == 1) {
        t24.Current_Mission++;
        if (t24.Current_Mission > AUTOCROSS) {
            t24.Current_Mission = MANUAL;
        }
    }
}

/* ==================================================================
 * Test infrastructure
 * ================================================================== */

/* Floating-point comparison tolerance */
#define EPSILON 1e-4f

static inline bool approx_eq(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

static int tests_passed = 0;
static int tests_failed = 0;

/*
 * TEST_ASSERT – if condition fails, print failure, increment global
 * counter and return 0 from the enclosing test function.
 * On success, print PASS and increment passed counter.
 */
#define TEST_ASSERT(cond, msg) do {                                    \
        if (!(cond)) {                                                 \
            printf("  FAIL: %s  (line %d)\n", msg, __LINE__);         \
            tests_failed++;                                            \
            return 0;                                                  \
        }                                                              \
        printf("  PASS: %s\n", msg);                                   \
        tests_passed++;                                                \
    } while(0)

/* Reset all global state to known defaults before each test */
static void reset_state(void)
{
    memset(&t24, 0, sizeof(t24));
    memset(ADC_Samples, 0, sizeof(ADC_Samples));
    mission_selector_enable = 0;
    fake_tick = 0;

    /* Restore default calibration values */
    fake_vrefint_cal = 1500;
    fake_ts_cal1     = 1730;
    fake_ts_cal2     = 2160;

    /* Re-initialise EMA filter stubs */
    ema_init(&ema_front_pressure, 0.5f);
    ema_init(&ema_rear_pressure,  0.5f);
}

/* Helper: fire the EXTI callback with the given pin */
static inline void fire_exti(uint16_t pin)
{
    HAL_GPIO_EXTI_Callback(pin);
}

/* Helper: fire the ADC conversion complete callback */
static inline void fire_adc_cb(void)
{
    HAL_ADC_ConvCpltCallback(&hadc1);
}

/* ==================================================================
 * Test cases
 * ================================================================== */

/* ------------------------------------------------------------------
 * TEST 1: ADC computes front pressure correctly
 *
 * ADC_Samples[0] = 2048  (mid-scale = 1.65 V at ADC input)
 *   raw_voltage_front = (2048 * 3.3 / 4096) / 0.66 = 2.5 V
 *   front_pressure    = (2.5 - 0.5) / 0.4         = 5.0 bar
 * ------------------------------------------------------------------ */
static int test_adc_computes_front_pressure(void)
{
    printf("\n--- TEST 1: ADC computes front pressure ---\n");
    reset_state();

    ADC_Samples[0] = 2048;
    fire_adc_cb();

    TEST_ASSERT(approx_eq(t24.Front_Pressure.Pneumatic, 5.0f, EPSILON),
                "Front pressure = 5.0 bar for ADC sample 2048");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 2: ADC computes rear pressure correctly
 *
 * ADC_Samples[1] = 1229  (~0.9902 V at ADC input)
 *   raw_voltage_rear = (1229 * 3.3 / 4096) / 0.66 ~ 1.5003 V
 *   rear_pressure    = (1.5003 - 0.5) / 0.4       ~ 2.5006 bar
 * ------------------------------------------------------------------ */
static int test_adc_computes_rear_pressure(void)
{
    printf("\n--- TEST 2: ADC computes rear pressure ---\n");
    reset_state();

    ADC_Samples[1] = 1229;
    fire_adc_cb();

    TEST_ASSERT(approx_eq(t24.Rear_Pressure.Pneumatic, 2.5006f, 0.01f),
                "Rear pressure ~ 2.50 bar for ADC sample 1229");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 3: ADC temperature returns 0 when VREF is zero
 *
 * GetTemperature() early-returns 0.0f when raw_vref == 0 to avoid
 * division by zero.  The ADC callback stores that result.
 * ------------------------------------------------------------------ */
static int test_adc_temperature_zero_vref(void)
{
    printf("\n--- TEST 3: ADC temperature zero when VREF=0 ---\n");
    reset_state();

    ADC_Samples[2] = 2000;   /* any non-zero raw temp */
    ADC_Samples[3] = 0;      /* VREF = 0 -> division guard */
    fire_adc_cb();

    TEST_ASSERT(t24.chip_temp == 0.0f,
                "chip_temp = 0.0 when raw_vref = 0");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 4: ADC temperature at known calibration point (30.0C)
 *
 * With factory defaults (fake_ts_cal1=1730, fake_ts_cal2=2160,
 * fake_vrefint_cal=1500), raw_temp=1730 and raw_vref=1500 should
 * yield exactly 30.0C.
 * ------------------------------------------------------------------ */
static int test_adc_temperature_known(void)
{
    printf("\n--- TEST 4: ADC temperature at 30.0C calibration ---\n");
    reset_state();

    ADC_Samples[2] = 1730;   /* raw temp = TS_CAL1 */
    ADC_Samples[3] = 1500;   /* raw VREF = VREFINT_CAL  */
    fire_adc_cb();

    TEST_ASSERT(approx_eq(t24.chip_temp, 30.0f, 0.1f),
                "chip_temp ~ 30.0C at TS_CAL1 with ideal VREF");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 5: Full ADC chain updates all three outputs
 *
 * Set known values for all four ADC_Samples, fire the callback,
 * and verify front pressure, rear pressure, and chip_temp are all
 * updated to the expected values simultaneously.
 * ------------------------------------------------------------------ */
static int test_adc_full_chain(void)
{
    printf("\n--- TEST 5: Full ADC chain updates all outputs ---\n");
    reset_state();

    ADC_Samples[0] = 2048;   /* front pressure -> 5.0 bar */
    ADC_Samples[1] = 1229;   /* rear pressure  -> ~2.50 bar */
    ADC_Samples[2] = 1730;   /* chip_temp      -> 30.0C */
    ADC_Samples[3] = 1500;   /* VREF reference */
    fire_adc_cb();

    TEST_ASSERT(approx_eq(t24.Front_Pressure.Pneumatic, 5.0f, EPSILON),
                "Front pressure = 5.0 bar (full chain)");
    TEST_ASSERT(approx_eq(t24.Rear_Pressure.Pneumatic, 2.5006f, 0.01f),
                "Rear pressure ~ 2.50 bar (full chain)");
    TEST_ASSERT(approx_eq(t24.chip_temp, 30.0f, 0.1f),
                "chip_temp ~ 30.0C (full chain)");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 6: EXTI increments mission from ACCELERATION to SKIDPAD
 *
 * With mission_selector_enable=1 and Current_Mission=ACCELERATION(1),
 * firing EXTI with MS_BTN_Pin should advance to SKIDPAD(2).
 * ------------------------------------------------------------------ */
static int test_exti_increments_mission(void)
{
    printf("\n--- TEST 6: EXTI increments mission ---\n");
    reset_state();

    mission_selector_enable = 1;
    t24.Current_Mission = ACCELERATION;   /* 1 */
    fire_exti(MS_BTN_Pin);

    TEST_ASSERT(t24.Current_Mission == SKIDPAD,
                "Current_Mission == SKIDPAD after EXTI from ACCELERATION");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 7: EXTI wraps from AUTOCROSS back to MANUAL
 *
 * With mission_selector_enable=1 and Current_Mission=AUTOCROSS(6),
 * incrementing yields 7, which is > AUTOCROSS, so it wraps to
 * MANUAL(0).
 * ------------------------------------------------------------------ */
static int test_exti_wraps_to_manual(void)
{
    printf("\n--- TEST 7: EXTI wraps from AUTOCROSS to MANUAL ---\n");
    reset_state();

    mission_selector_enable = 1;
    t24.Current_Mission = AUTOCROSS;   /* 6 */
    fire_exti(MS_BTN_Pin);

    TEST_ASSERT(t24.Current_Mission == MANUAL,
                "Current_Mission wraps from AUTOCROSS to MANUAL");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 8: EXTI ignored when mission_selector_enable == 0
 *
 * With the selector disabled, pressing MS_BTN should leave the
 * mission unchanged.
 * ------------------------------------------------------------------ */
static int test_exti_ignored_when_disabled(void)
{
    printf("\n--- TEST 8: EXTI ignored when selector disabled ---\n");
    reset_state();

    mission_selector_enable = 0;
    t24.Current_Mission = MANUAL;
    fire_exti(MS_BTN_Pin);

    TEST_ASSERT(t24.Current_Mission == MANUAL,
                "Current_Mission unchanged when selector disabled");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 9: EXTI ignores pins other than MS_BTN_Pin
 *
 * Even with the selector enabled, a different EXTI line (GPIO_PIN_5)
 * must not change the mission.
 * ------------------------------------------------------------------ */
static int test_exti_ignores_other_pins(void)
{
    printf("\n--- TEST 9: EXTI ignores other GPIO pins ---\n");
    reset_state();

    mission_selector_enable = 1;
    t24.Current_Mission = TRACKDRIVE;   /* 3 */
    fire_exti(GPIO_PIN_5);              /* not MS_BTN_Pin (GPIO_PIN_3) */

    TEST_ASSERT(t24.Current_Mission == TRACKDRIVE,
                "Current_Mission unchanged for non-MS_BTN pin");

    return 1;
}

/* ------------------------------------------------------------------
 * TEST 10: EXTI full cycle through all missions and wrap
 *
 * Start at MANUAL(0).  Fire EXTI 8 times and verify the sequence
 * of transitions:
 *   MANUAL→ACCEL→SKIDPAD→TRACKDRIVE→EBS_TEST→INSPECTION→AUTOCROSS
 *   →(wrap)→MANUAL
 * After 8 presses the final state should be ACCELERATION (second
 * lap).
 * ------------------------------------------------------------------ */
static int test_exti_full_cycle(void)
{
    printf("\n--- TEST 10: EXTI full mission cycle ---\n");
    reset_state();

    mission_selector_enable = 1;
    t24.Current_Mission = MANUAL;

    fire_exti(MS_BTN_Pin);
    TEST_ASSERT(t24.Current_Mission == ACCELERATION,
                "After press 1: ACCELERATION");

    fire_exti(MS_BTN_Pin);
    TEST_ASSERT(t24.Current_Mission == SKIDPAD,
                "After press 2: SKIDPAD");

    fire_exti(MS_BTN_Pin);
    TEST_ASSERT(t24.Current_Mission == TRACKDRIVE,
                "After press 3: TRACKDRIVE");

    fire_exti(MS_BTN_Pin);
    TEST_ASSERT(t24.Current_Mission == EBS_TEST,
                "After press 4: EBS_TEST");

    fire_exti(MS_BTN_Pin);
    TEST_ASSERT(t24.Current_Mission == INSPECTION,
                "After press 5: INSPECTION");

    fire_exti(MS_BTN_Pin);
    TEST_ASSERT(t24.Current_Mission == AUTOCROSS,
                "After press 6: AUTOCROSS");

    /* 7th press: value becomes 7, which > AUTOCROSS(6), so wrap */
    fire_exti(MS_BTN_Pin);
    TEST_ASSERT(t24.Current_Mission == MANUAL,
                "After press 7: MANUAL (wrap)");

    /* 8th press: now back to ACCELERATION */
    fire_exti(MS_BTN_Pin);
    TEST_ASSERT(t24.Current_Mission == ACCELERATION,
                "After press 8: ACCELERATION (second lap)");

    return 1;
}

/* ==================================================================
 * Main — run all tests, report summary, exit 0 on full pass.
 * ================================================================== */
int main(void)
{
    printf("========================================\n");
    printf(" ADC & EXTI Callback Unit Tests\n");
    printf("========================================\n");

    int (*test_fn[])(void) = {
        test_adc_computes_front_pressure,
        test_adc_computes_rear_pressure,
        test_adc_temperature_zero_vref,
        test_adc_temperature_known,
        test_adc_full_chain,
        test_exti_increments_mission,
        test_exti_wraps_to_manual,
        test_exti_ignored_when_disabled,
        test_exti_ignores_other_pins,
        test_exti_full_cycle,
    };
    const char *test_names[] = {
        "TEST 1:  ADC computes front pressure",
        "TEST 2:  ADC computes rear pressure",
        "TEST 3:  ADC temperature zero when VREF=0",
        "TEST 4:  ADC temperature at 30.0C calibration",
        "TEST 5:  Full ADC chain updates all outputs",
        "TEST 6:  EXTI increments mission",
        "TEST 7:  EXTI wraps from AUTOCROSS to MANUAL",
        "TEST 8:  EXTI ignored when selector disabled",
        "TEST 9:  EXTI ignores other GPIO pins",
        "TEST 10: EXTI full mission cycle",
    };
    const int num_tests = sizeof(test_fn) / sizeof(test_fn[0]);

    for (int i = 0; i < num_tests; i++) {
        printf("\n========================================\n");
        printf(" %s\n", test_names[i]);
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
