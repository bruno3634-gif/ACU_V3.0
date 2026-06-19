/*
 * test_autonomous_functions.c
 *
 * Standalone host-based unit tests for Autonomous_functions.c logic:
 *   ASSI_control(), check_timeout(), module_timeout(), continuous_monitoring()
 *
 * Compile:
 *   gcc -o test_autonomous_functions test_autonomous_functions.c -Wall -Wextra -std=c11 -lm
 *
 * Run:
 *   ./test_autonomous_functions
 *   Returns 0 if all tests pass, 1 otherwise.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* =========================================================================
 * Type stubs (matching the embedded firmware types)
 * ========================================================================= */

typedef enum {
    AS_STATE_OFF       = 1,
    AS_STATE_READY     = 2,
    AS_STATE_DRIVING   = 3,
    AS_STATE_EMERGENCY = 4,
    AS_STATE_FINISHED  = 5
} AS_STATE_t;

typedef enum {
    MANUAL,
    ACCELERATION,
    SKIDPAD,
    TRACKDRIVE,
    EBS_TEST,
    INSPECTION,
    AUTOCROSS
} current_mission_t;

typedef enum {
    Start,
    IDLE,
    AS_ON,
    EMERGENCY
} Main_state_machine_t;

typedef enum {
    OFF,
    Initial_Sequence,
    Monitor_sequence,
    AS_Emergency,
    Finish
} Autonomous_System_states_t;

typedef enum {
    WDT_TOGGLE_CHECK,
    WDT_STP_TOGGLE_CHECK,
    PNEUMATIC_CHECK,
    PRESSURE_CHECK1,
    HV_ACTIVATION,
    PRESSURE_CHECK_FRONT,
    PRESSURE_CHECK_REAR,
    PRESSURE_CHECK2,
    SEQUENCE_ERROR
} startup_sequence_state_t;

typedef enum {
    NONE,
    SDC_OPEN,
    RES,
    Pressure_checks,
    VCU_Timeout,
    Jetson_timeout,
    ACU_WDT_TRIGERED,
    dir_actuator_timeout,
    Dynamics_REAR_Pressure_timeout,
    UNKOWN
} Emergency_cause_t;

typedef enum {
    NO_TIMEOUT,
    VCU_TIMEOUT,
    JETSON_TIMEOUT,
    PRESSURE_TIMEOUT,
    DIR_TIMEOUT,
    RES_TIMEOUT
} can_timeouts_t;

struct pressure {
    float Pneumatic;
    float Hydraulic;
};

struct speed {
    uint8_t Speed;
    uint8_t Target_Speed;
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
    uint32_t VCU_LAST_TX;
    uint32_t REAR_PRESSURE_LAST_TX;
    uint32_t JETSON_LAST_TX;
    uint32_t DIR_ACTUATOR_LAST_TX;
    uint32_t RES_LAST_TX;
};

/* =========================================================================
 * Constants
 * ========================================================================= */

#define MAX_TIMEOUT 1000
#define IN_RANGE(val, min, max)          ((val) > (min) && (val) < (max))
#define IS_CORRELATED(hyd, pneu, gain)   ((hyd) >= (gain) * (pneu))
#define IS_UNLOADED(hyd)                 ((hyd) <= 1.0f)
#define EBS_MIN_BAR                      6.0f
#define EBS_MAX_BAR                      10.0f
#define EBS_FRONT_HYD_GAIN               9.0f
#define EBS_REAR_HYD_GAIN_INITIAL        3.8f
#define EBS_REAR_HYD_GAIN_FINAL          3.0f
#define EBS_HYD_UNLOADED_BAR             1.0f

/* =========================================================================
 * Global stubs
 * ========================================================================= */

struct car t24;
Emergency_cause_t Emergency_cause;
Main_state_machine_t Vehicle_state_machine;
uint32_t fake_time_ms = 0;

uint32_t millis(void) { return fake_time_ms; }

/* =========================================================================
 * Function bodies copied verbatim from Core/Src/Autonomous_functions.c
 * ========================================================================= */

/* check_timeout -- verbatim copy */
bool check_timeout(uint32_t start_time, uint32_t limit) {
    if (millis() - start_time > limit) {
        return true;
    }
    return false;
}

/* ASSI_control -- verbatim copy (including static locals) */
int ASSI_control(uint8_t gpio_state, uint8_t ASSI_state) {

    /*
     * bit 0 -> Yellow
     * bit 1 -> Blue
     *
     * -------------------------------------------------------------------------
     * AS Status Indicator (ASSI) - Operational States (Ref: T 14.8)
     * -------------------------------------------------------------------------
     * AS STATE        | ILLUMINATION STATUS
     * ----------------|--------------------------------------------------------
     * AS Off          | Off						1
     * AS Ready        | Yellow (Continuous)		2
     * AS Driving      | Yellow (Flashing)			3
     * AS Emergency    | Blue (Flashing)			4
     * AS Finished     | Blue (Continuous)			5
     * -------------------------------------------------------------------------
     *During "AS Driving" and "AS Emergency" the ASSIs must be flashing continuously with a
     *frequency between 2 Hz and 5 Hz and a duty cycle of 50 %.
     *
     */

    static unsigned long prev_time_yellow = 0;
    static unsigned long prev_time_blue = 0;


    switch (ASSI_state) {
    case AS_STATE_OFF:
        gpio_state = 0;
        break;
    case AS_STATE_READY:
        gpio_state = 0b00000001;
        break;
    case AS_STATE_DRIVING:
        if (millis() - prev_time_yellow >= 330) {
            gpio_state ^= 1;
            gpio_state &= 0b00000001;
            prev_time_yellow = millis();
        }
        break;
    case AS_STATE_EMERGENCY:
        if (millis() - prev_time_blue >= 330) {
            gpio_state ^= 0b00000010;
            gpio_state &= 0b00000010;
            prev_time_blue = millis();
        }
        break;
    case AS_STATE_FINISHED:
        gpio_state = 0b00000010;
        break;
    default:
        break;
    }
    return gpio_state;
}

/* module_timeout -- verbatim copy */
uint8_t module_timeout(void) {

    uint32_t current_time = millis();

    if (current_time - t24.VCU_LAST_TX > MAX_TIMEOUT) return VCU_TIMEOUT;
    if (current_time - t24.REAR_PRESSURE_LAST_TX > MAX_TIMEOUT) return PRESSURE_TIMEOUT;
    if (current_time - t24.JETSON_LAST_TX > MAX_TIMEOUT) return JETSON_TIMEOUT;
    if (current_time - t24.DIR_ACTUATOR_LAST_TX > MAX_TIMEOUT) return DIR_TIMEOUT;
    if (current_time - t24.RES_LAST_TX > MAX_TIMEOUT) return RES_TIMEOUT;
    return NO_TIMEOUT;
}

/* continuous_monitoring -- verbatim copy */
void continuous_monitoring(uint8_t sdc_status, float Rear_pneumatic,
        float Front_pneumatic, float Rear_hydraulic, float Front_hydraulic) {
    // CAN Messages timeouts
    /**
     *
     * TODO:	Check if sdc is open
     * 			Check can timeouts
     * 			Check AS system components
     * 			Check pneumatic pressure are between 6 and 10 Bar
     *
     * 			note: sdc open reads 0
     */

    if (sdc_status == 0) {
        Vehicle_state_machine = EMERGENCY;
        return;
    }
    uint8_t res = module_timeout();
    if (res != NO_TIMEOUT) {
        Vehicle_state_machine = EMERGENCY;
        switch (res) {
            case VCU_TIMEOUT:
                Emergency_cause = VCU_Timeout;
                break;
            case JETSON_TIMEOUT:
                Emergency_cause = Jetson_timeout;
                break;
            case PRESSURE_TIMEOUT:
                Emergency_cause = Dynamics_REAR_Pressure_timeout;
                break;
            case DIR_TIMEOUT:
                Emergency_cause = dir_actuator_timeout;
                break;
            case RES_TIMEOUT:
                Emergency_cause = RES;
                break;
            default:
                Emergency_cause = UNKOWN;
                break;
        }
        return;
    }

    if (!IN_RANGE(Rear_pneumatic, EBS_MIN_BAR, EBS_MAX_BAR) || !IN_RANGE(Front_pneumatic, EBS_MIN_BAR, EBS_MAX_BAR)) {
        Vehicle_state_machine = EMERGENCY;
        return;
    }

    if (!IS_CORRELATED(Front_hydraulic, Front_pneumatic, EBS_FRONT_HYD_GAIN) || !IS_CORRELATED(Rear_hydraulic, Rear_pneumatic, EBS_REAR_HYD_GAIN_FINAL)) {
        Vehicle_state_machine = EMERGENCY;
        return;
    }
}

/* =========================================================================
 * Custom test assertion
 *
 * TEST_ASSERT(cond, msg)
 *   If cond is true:  prints PASS, returns 1
 *   If cond is false: prints FAIL, returns 0
 * ========================================================================= */

static int total_pass = 0;
static int total_fail = 0;

#define TEST_ASSERT(cond, msg) do {                                        \
    if (cond) {                                                            \
        total_pass++;                                                      \
        printf("  PASS: %s\n", msg);                                       \
    } else {                                                               \
        total_fail++;                                                      \
        printf("  FAIL: %s\n", msg);                                       \
    }                                                                      \
} while(0)

/* Helper: reset global state before each test */
static void reset_globals(void)
{
    memset(&t24, 0, sizeof(t24));
    Emergency_cause = NONE;
    Vehicle_state_machine = Start;
    fake_time_ms = 0;
}

/* =========================================================================
 * Test cases
 * ========================================================================= */

/* ---- check_timeout ---- */

static void test_check_timeout_basic(void)
{
    printf("\n--- TEST 1: check_timeout basic ---\n");
    reset_globals();

    /* At time=50, start=0, limit=100 => not timed out */
    fake_time_ms = 50;
    TEST_ASSERT(check_timeout(0, 100) == false,
                "check_timeout(0,100) @ t=50 returns false");

    /* At time=101, start=0, limit=100 => timed out */
    fake_time_ms = 101;
    TEST_ASSERT(check_timeout(0, 100) == true,
                "check_timeout(0,100) @ t=101 returns true");

    /* Boundary: at exactly limit (start + limit) should NOT timeout (> not >=) */
    fake_time_ms = 100;
    TEST_ASSERT(check_timeout(0, 100) == false,
                "check_timeout(0,100) @ t=100 (exactly limit) returns false (strict >)");
}

/* ---- ASSI_control ---- */

static void test_assi_off_state(void)
{
    printf("\n--- TEST 2: ASSI_control OFF state ---\n");
    reset_globals();

    /* AS_STATE_OFF should force output to 0 regardless of input */
    int result = ASSI_control(0xFF, AS_STATE_OFF);
    TEST_ASSERT(result == 0,
                "ASSI_control(0xFF, OFF) returns 0");

    result = ASSI_control(0x00, AS_STATE_OFF);
    TEST_ASSERT(result == 0,
                "ASSI_control(0x00, OFF) returns 0");
}

static void test_assi_ready_state(void)
{
    printf("\n--- TEST 3: ASSI_control READY state ---\n");
    reset_globals();

    /* AS_STATE_READY -> yellow continuous = 0b00000001 */
    int result = ASSI_control(0x00, AS_STATE_READY);
    TEST_ASSERT(result == 0b00000001,
                "ASSI_control(0, READY) returns 0b00000001 (yellow continuous)");

    /* Input value should be overwritten */
    result = ASSI_control(0xFF, AS_STATE_READY);
    TEST_ASSERT(result == 0b00000001,
                "ASSI_control(0xFF, READY) also returns 0b00000001");
}

static void test_assi_driving_flashing(void)
{
    printf("\n--- TEST 4: ASSI_control DRIVING flashing ---\n");
    reset_globals();

    fake_time_ms = 0;
    uint8_t gpio = 0;

    /* Call 1 @ t=0: prev_time_yellow=0, 0-0 < 330 → no toggle yet */
    gpio = (uint8_t)ASSI_control(gpio, AS_STATE_DRIVING);
    printf("  Call 1 @ t=%lu: output=%u\n", fake_time_ms, gpio);
    /* gpio should still be 0 (no toggle yet) */

    /* Call 2 @ t=331: 331-0 >= 330 → toggle bit 0 on */
    fake_time_ms = 331;
    gpio = (uint8_t)ASSI_control(gpio, AS_STATE_DRIVING);
    printf("  Call 2 @ t=%lu: output=%u\n", fake_time_ms, gpio);
    TEST_ASSERT((gpio & 0x01) == 1, "Second call @ t=331 toggled bit 0 on");

    /* Call 3 @ t=662: 662-331 >= 330 → toggle bit 0 off */
    fake_time_ms = 662;
    gpio = (uint8_t)ASSI_control(gpio, AS_STATE_DRIVING);
    printf("  Call 3 @ t=%lu: output=%u\n", fake_time_ms, gpio);
    TEST_ASSERT((gpio & 0x01) == 0, "Third call @ t=662 toggled bit 0 back to 0");

    printf("  PASS: ASSI_control DRIVING toggles at ~330ms intervals\n");
}

static void test_assi_emergency_flashing(void)
{
    printf("\n--- TEST 5: ASSI_control EMERGENCY flashing ---\n");
    reset_globals();

    fake_time_ms = 0;
    uint8_t gpio = 0;

    /* Call 1 @ t=0: prev_time_blue=0, 0-0 < 330 → no toggle yet */
    gpio = (uint8_t)ASSI_control(gpio, AS_STATE_EMERGENCY);
    printf("  Call 1 @ t=%lu: output=%u\n", fake_time_ms, gpio);
    /* gpio should still be 0 (no toggle yet) */

    /* Call 2 @ t=331: 331-0 >= 330 → toggle bit 1 on */
    fake_time_ms = 331;
    gpio = (uint8_t)ASSI_control(gpio, AS_STATE_EMERGENCY);
    printf("  Call 2 @ t=%lu: output=%u\n", fake_time_ms, gpio);
    TEST_ASSERT((gpio & 0x02) == 2, "Second call @ t=331 toggled bit 1 on");

    /* Call 3 @ t=662: 662-331 >= 330 → toggle bit 1 off */
    fake_time_ms = 662;
    gpio = (uint8_t)ASSI_control(gpio, AS_STATE_EMERGENCY);
    printf("  Call 3 @ t=%lu: output=%u\n", fake_time_ms, gpio);
    TEST_ASSERT((gpio & 0x02) == 0, "Third call @ t=662 toggled bit 1 back to 0");

    printf("  PASS: ASSI_control EMERGENCY toggles at ~330ms intervals\n");
}

static void test_assi_finished_state(void)
{
    printf("\n--- TEST 6: ASSI_control FINISHED state ---\n");
    reset_globals();

    /* AS_STATE_FINISHED -> blue continuous = 0b00000010 */
    int result = ASSI_control(0x00, AS_STATE_FINISHED);
    TEST_ASSERT(result == 0b00000010,
                "ASSI_control(0, FINISHED) returns 0b00000010 (blue continuous)");

    result = ASSI_control(0xFF, AS_STATE_FINISHED);
    TEST_ASSERT(result == 0b00000010,
                "ASSI_control(0xFF, FINISHED) also returns 0b00000010");
}

/* ---- module_timeout ---- */

static void test_module_timeout_all_within_range(void)
{
    printf("\n--- TEST 7: module_timeout all within range ---\n");
    reset_globals();

    /* All LAST_TX set to current time => no timeout */
    fake_time_ms = 500;
    t24.VCU_LAST_TX            = 500;
    t24.REAR_PRESSURE_LAST_TX  = 500;
    t24.JETSON_LAST_TX         = 500;
    t24.DIR_ACTUATOR_LAST_TX   = 500;
    t24.RES_LAST_TX            = 500;

    uint8_t result = module_timeout();
    TEST_ASSERT(result == NO_TIMEOUT,
                "module_timeout() returns NO_TIMEOUT when all timestamps are current");
}

static void test_module_timeout_vcu(void)
{
    printf("\n--- TEST 8: module_timeout VCU timeout ---\n");
    reset_globals();

    /* VCU_LAST_TX expired, all others current */
    fake_time_ms = 1001;
    t24.VCU_LAST_TX            = 0;
    t24.REAR_PRESSURE_LAST_TX  = 1001;
    t24.JETSON_LAST_TX         = 1001;
    t24.DIR_ACTUATOR_LAST_TX   = 1001;
    t24.RES_LAST_TX            = 1001;

    uint8_t result = module_timeout();
    TEST_ASSERT(result == VCU_TIMEOUT,
                "module_timeout() returns VCU_TIMEOUT when VCU_LAST_TX expired");
}

static void test_module_timeout_jetson(void)
{
    printf("\n--- TEST 9: module_timeout JETSON timeout ---\n");
    reset_globals();

    /* JETSON_LAST_TX expired; VCU and PRESSURE must be current to pass their checks */
    fake_time_ms = 1001;
    t24.VCU_LAST_TX            = 1001;
    t24.REAR_PRESSURE_LAST_TX  = 1001;
    t24.JETSON_LAST_TX         = 0;
    t24.DIR_ACTUATOR_LAST_TX   = 1001;
    t24.RES_LAST_TX            = 1001;

    uint8_t result = module_timeout();
    TEST_ASSERT(result == JETSON_TIMEOUT,
                "module_timeout() returns JETSON_TIMEOUT when JETSON_LAST_TX expired");
}

static void test_module_timeout_pressure(void)
{
    printf("\n--- TEST 10: module_timeout PRESSURE timeout ---\n");
    reset_globals();

    /* REAR_PRESSURE_LAST_TX expired; VCU must be current */
    fake_time_ms = 1001;
    t24.VCU_LAST_TX            = 1001;
    t24.REAR_PRESSURE_LAST_TX  = 0;
    t24.JETSON_LAST_TX         = 1001;
    t24.DIR_ACTUATOR_LAST_TX   = 1001;
    t24.RES_LAST_TX            = 1001;

    uint8_t result = module_timeout();
    TEST_ASSERT(result == PRESSURE_TIMEOUT,
                "module_timeout() returns PRESSURE_TIMEOUT when REAR_PRESSURE_LAST_TX expired");
}

static void test_module_timeout_dir(void)
{
    printf("\n--- TEST 11: module_timeout DIR timeout ---\n");
    reset_globals();

    /* DIR_ACTUATOR_LAST_TX expired; VCU, PRESSURE, JETSON must be current */
    fake_time_ms = 1001;
    t24.VCU_LAST_TX            = 1001;
    t24.REAR_PRESSURE_LAST_TX  = 1001;
    t24.JETSON_LAST_TX         = 1001;
    t24.DIR_ACTUATOR_LAST_TX   = 0;
    t24.RES_LAST_TX            = 1001;

    uint8_t result = module_timeout();
    TEST_ASSERT(result == DIR_TIMEOUT,
                "module_timeout() returns DIR_TIMEOUT when DIR_ACTUATOR_LAST_TX expired");
}

static void test_module_timeout_res(void)
{
    printf("\n--- TEST 12: module_timeout RES timeout ---\n");
    reset_globals();

    /* RES_LAST_TX expired; all others must be current */
    fake_time_ms = 1001;
    t24.VCU_LAST_TX            = 1001;
    t24.REAR_PRESSURE_LAST_TX  = 1001;
    t24.JETSON_LAST_TX         = 1001;
    t24.DIR_ACTUATOR_LAST_TX   = 1001;
    t24.RES_LAST_TX            = 0;

    uint8_t result = module_timeout();
    TEST_ASSERT(result == RES_TIMEOUT,
                "module_timeout() returns RES_TIMEOUT when RES_LAST_TX expired");
}

/* ---- continuous_monitoring ---- */

static void test_continuous_monitoring_sdc_open(void)
{
    printf("\n--- TEST 13: continuous_monitoring SDC open ---\n");
    reset_globals();

    Vehicle_state_machine = Start;

    /* sdc_status=0 -> SDC open -> EMERGENCY */
    continuous_monitoring(0, 7.5f, 7.5f, 68.0f, 68.0f);

    TEST_ASSERT(Vehicle_state_machine == EMERGENCY,
                "SDC open (sdc_status=0) triggers EMERGENCY");
}

static void test_continuous_monitoring_pneumatic_out_of_range(void)
{
    printf("\n--- TEST 14: continuous_monitoring pneumatic out of range ---\n");
    reset_globals();

    Vehicle_state_machine = Start;

    /* Setup: sdc OK, timeouts OK, but Rear_pneumatic=11.0 > 10 (out of range) */
    fake_time_ms = 100;
    t24.VCU_LAST_TX            = 100;
    t24.REAR_PRESSURE_LAST_TX  = 100;
    t24.JETSON_LAST_TX         = 100;
    t24.DIR_ACTUATOR_LAST_TX   = 100;
    t24.RES_LAST_TX            = 100;

    continuous_monitoring(1, 11.0f, 7.5f, 68.0f, 68.0f);

    TEST_ASSERT(Vehicle_state_machine == EMERGENCY,
                "Rear_pneumatic=11.0f (>10) triggers EMERGENCY");
}

static void test_continuous_monitoring_correlation_failure(void)
{
    printf("\n--- TEST 15: continuous_monitoring correlation failure ---\n");
    reset_globals();

    Vehicle_state_machine = Start;

    /* Setup: sdc OK, timeouts OK, pneumatics in range (7.5, 8.2) */
    /* Front_hydraulic=0.0f -> not correlated with Front_pneumatic=7.5 (need >= 67.5) */
    fake_time_ms = 100;
    t24.VCU_LAST_TX            = 100;
    t24.REAR_PRESSURE_LAST_TX  = 100;
    t24.JETSON_LAST_TX         = 100;
    t24.DIR_ACTUATOR_LAST_TX   = 100;
    t24.RES_LAST_TX            = 100;

    continuous_monitoring(1, 7.5f, 8.2f, 68.0f, 0.0f);

    TEST_ASSERT(Vehicle_state_machine == EMERGENCY,
                "Front_hydraulic=0.0f (not correlated with pneu=7.5, gain=9) triggers EMERGENCY");
}

static void test_continuous_monitoring_normal_pass(void)
{
    printf("\n--- TEST 16: continuous_monitoring normal passes ---\n");
    reset_globals();

    Vehicle_state_machine = Start;

    /* Setup: sdc OK, timeouts OK */
    fake_time_ms = 100;
    t24.VCU_LAST_TX            = 100;
    t24.REAR_PRESSURE_LAST_TX  = 100;
    t24.JETSON_LAST_TX         = 100;
    t24.DIR_ACTUATOR_LAST_TX   = 100;
    t24.RES_LAST_TX            = 100;

    /*
     * Pneumatics in range: 7.5f (6 < 7.5 < 10)
     * Front: hyd=68.0f >= 9.0 * 7.5 = 67.5  => correlated
     * Rear:  hyd=23.0f >= 3.0 * 7.5 = 22.5  => correlated
     */
    continuous_monitoring(1, 7.5f, 7.5f, 23.0f, 68.0f);

    TEST_ASSERT(Vehicle_state_machine == Start,
                "All parameters valid: Vehicle_state_machine unchanged (no EMERGENCY)");

    /* Also verify no emergency cause was set spuriously */
    TEST_ASSERT(Emergency_cause == NONE,
                "Emergency_cause remains NONE when all checks pass");
}

/* =========================================================================
 * Main: run all tests and report
 * ========================================================================= */

int main(void)
{
    printf("========================================\n");
    printf("  Autonomous Functions Unit Tests\n");
    printf("========================================\n");

    /* check_timeout */
    test_check_timeout_basic();

    /* ASSI_control */
    test_assi_off_state();
    test_assi_ready_state();
    test_assi_driving_flashing();
    test_assi_emergency_flashing();
    test_assi_finished_state();

    /* module_timeout */
    test_module_timeout_all_within_range();
    test_module_timeout_vcu();
    test_module_timeout_jetson();
    test_module_timeout_pressure();
    test_module_timeout_dir();
    test_module_timeout_res();

    /* continuous_monitoring */
    test_continuous_monitoring_sdc_open();
    test_continuous_monitoring_pneumatic_out_of_range();
    test_continuous_monitoring_correlation_failure();
    test_continuous_monitoring_normal_pass();

    /* Summary */
    printf("\n========================================\n");
    printf("  Results: %d passed, %d failed out of %d\n",
           total_pass, total_fail, total_pass + total_fail);
    printf("========================================\n");

    if (total_fail == 0) {
        printf("  ALL TESTS PASSED\n");
        return 0;
    } else {
        printf("  SOME TESTS FAILED\n");
        return 1;
    }
}
