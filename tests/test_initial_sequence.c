/*
 * test_initial_sequence.c
 *
 * Standalone host-based test harness for initial_sequence() and check_timeout().
 *
 * Compile with:
 *   gcc -o test_initial_sequence test_initial_sequence.c -Wall -Wextra -std=c11
 *
 * This file is self-contained — it defines its own type stubs (no STM32 HAL),
 * provides a fake millis(), and copies the function-under-test bodies from
 * Core/Src/Autonomous_functions.c (which cannot be #included because it
 * depends on stm32f4xx_hal.h and other hardware headers).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================
 *  Type stubs — matching the definitions in Core/Inc/main.h
 *  (no HAL types, no hardware headers)
 * ======================================================================== */

typedef enum {
    AS_STATE_OFF       = 1,
    AS_STATE_READY     = 2,
    AS_STATE_DRIVING   = 3,
    AS_STATE_EMERGENCY = 4,
    AS_STATE_FINISHED  = 5
} AS_STATE_t;

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
    Start, IDLE, AS_ON, EMERGENCY
} Main_state_machine_t;

typedef enum {
    OFF,
    Initial_Sequence,
    Monitor_sequence,
    AS_Emergency,
    Finish
} Autonomous_System_states_t;

typedef enum {
    WDT_TOGGLE_CHECK     = 0,
    WDT_STP_TOGGLE_CHECK = 1,
    PNEUMATIC_CHECK      = 2,
    PRESSURE_CHECK1      = 3,
    HV_ACTIVATION        = 4,
    PRESSURE_CHECK_FRONT = 5,
    PRESSURE_CHECK_REAR  = 6,
    PRESSURE_CHECK2      = 7,
    SEQUENCE_ERROR       = 8
} startup_sequence_state_t;

/* The following enums are referenced by the originals but are NOT used
 * by the copied functions. We define them only for completeness. */
typedef enum {
    NONE, SDC_OPEN, RES, Pressure_checks, VCU_Timeout, Jetson_timeout,
    ACU_WDT_TRIGERED, dir_actuator_timeout, Dynamics_REAR_Pressure_timeout, UNKOWN
} Emergency_cause_t;

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
    uint32_t VCU_LAST_TX, REAR_PRESSURE_LAST_TX, JETSON_LAST_TX, DIR_ACTUATOR_LAST_TX;
};

/* ========================================================================
 *  Macros — copied from Autonomous_functions.h and Autonomous_functions.c
 *  All SKIP flags are 0 (normal path) for testing.
 * ======================================================================== */

/* Skip flags for debugging (all disabled for tests) */
#define SKIP_WDT_CHECK              0
#define SKIP_PNEUMATIC_CHECK        0
#define SKIP_PRESSURE_CHECK1        0
#define SKIP_IGNITION_CHECK         0
#define SKIP_PRESSURE_FRONT_CHECK   0
#define SKIP_PRESSURE_REAR_CHECK    0
#define SKIP_PRESSURE_CHECK2        0

/* Timing constants */
#define TIMEOUT_WDT_MS          5000
#define TIMEOUT_SOLENOID_MS     5000
#define SOLENOID_MIN_DELAY_MS   1000

/* EBS pressure thresholds */
#define EBS_MIN_BAR         6.0f
#define EBS_MAX_BAR         10.0f
#define EBS_FRONT_HYD_GAIN          9.0f
#define EBS_REAR_HYD_GAIN_INITIAL   3.8f
#define EBS_REAR_HYD_GAIN_FINAL     3.0f
#define EBS_HYD_UNLOADED_BAR        1.0f

/* Logic macros */
#define IN_RANGE(val, min, max)       ((val) > (min) && (val) < (max))
#define IS_CORRELATED(hyd, pneu, gain) ((hyd) >= (gain) * (pneu))
#define IS_UNLOADED(hyd)              ((hyd) <= EBS_HYD_UNLOADED_BAR)

/* ========================================================================
 *  Fake time — stub for millis()
 * ======================================================================== */

uint32_t fake_time_ms = 0;

uint32_t millis(void) {
    return fake_time_ms;
}

/* ========================================================================
 *  Global variables — the same names and types as in the firmware
 * ======================================================================== */

struct car                          t24;
startup_sequence_state_t            startup_sequence_state;
Main_state_machine_t                Vehicle_state_machine;
Autonomous_System_states_t          Autonomous_state;
Emergency_cause_t                   Emergency_cause;   /* not used by test fns */

/* File-scope static that matches the one in Autonomous_functions.c */
static uint32_t state_timer = 0;

/* ========================================================================
 *  Function bodies — copied verbatim from Core/Src/Autonomous_functions.c
 *
 *  Only check_timeout() and initial_sequence() are needed for the tests.
 *  The functions are copied (not #included) because the original .c file
 *  depends on the STM32 HAL and hardware headers unavailable on a host PC.
 * ======================================================================== */

bool check_timeout(uint32_t start_time, uint32_t limit) {
    if (millis() - start_time > limit) {
        return true;
    }
    return false;
}

void initial_sequence(struct car *t24,
                      startup_sequence_state_t *seq_status,
                      Main_state_machine_t *Vehicle_state_machine) {
    switch (*seq_status) {
        case WDT_TOGGLE_CHECK:
            if (t24->SDC_feedback == 0) {
                t24->HW_WDT_Enable = 0;
                state_timer = millis();
                *seq_status = WDT_STP_TOGGLE_CHECK;
            }
            break;

        case WDT_STP_TOGGLE_CHECK:
#if SKIP_WDT_CHECK
            t24->HW_WDT_Enable = 1;
            *seq_status = PNEUMATIC_CHECK;
            break;
#endif
            if (t24->SDC_feedback == 1) {
                t24->HW_WDT_Enable = 1;
                *seq_status = PNEUMATIC_CHECK;
            } else if (check_timeout(state_timer, TIMEOUT_WDT_MS)) {
                *seq_status = SEQUENCE_ERROR;
            }
            break;

        case PNEUMATIC_CHECK:
#if SKIP_PNEUMATIC_CHECK
            *seq_status = PRESSURE_CHECK1;
            break;
#endif
            if (IN_RANGE(t24->Front_Pressure.Pneumatic, EBS_MIN_BAR, EBS_MAX_BAR)
                && IN_RANGE(t24->Rear_Pressure.Pneumatic, EBS_MIN_BAR, EBS_MAX_BAR)) {
                *seq_status = PRESSURE_CHECK1;
            } else {
                *seq_status = SEQUENCE_ERROR;
            }
            break;

        case PRESSURE_CHECK1:
#if SKIP_PRESSURE_CHECK1
            *seq_status = HV_ACTIVATION;
            break;
#endif
            if (IS_CORRELATED(t24->Front_Pressure.Hydraulic,
                              t24->Front_Pressure.Pneumatic,
                              EBS_FRONT_HYD_GAIN)
                && IS_CORRELATED(t24->Rear_Pressure.Hydraulic,
                                 t24->Rear_Pressure.Pneumatic,
                                 EBS_REAR_HYD_GAIN_INITIAL)) {
                *seq_status = HV_ACTIVATION;
            } else {
                *seq_status = SEQUENCE_ERROR;
            }
            break;

        case HV_ACTIVATION:
            t24->Ignition_Request = t24->ignition_pin_state;
#if SKIP_IGNITION_CHECK
            *seq_status = PRESSURE_CHECK_FRONT;
            state_timer = millis();
            break;
#endif
            if (t24->Ignition_Status == 1) {
                *seq_status = PRESSURE_CHECK_FRONT;
                state_timer = millis();
            }
            break;

        case PRESSURE_CHECK_FRONT:
            t24->front_solenoid = 1;
            t24->rear_solenoid = 0;
#if SKIP_PRESSURE_FRONT_CHECK
            state_timer = millis();
            *seq_status = PRESSURE_CHECK_REAR;
            break;
#endif
            if (IS_CORRELATED(t24->Front_Pressure.Hydraulic,
                              t24->Front_Pressure.Pneumatic,
                              EBS_FRONT_HYD_GAIN)
                && IS_UNLOADED(t24->Rear_Pressure.Hydraulic)
                && check_timeout(state_timer, SOLENOID_MIN_DELAY_MS)) {
                *seq_status = PRESSURE_CHECK_REAR;
                state_timer = millis();
            } else if (check_timeout(state_timer, TIMEOUT_SOLENOID_MS)) {
                *seq_status = SEQUENCE_ERROR;
            }
            break;

        case PRESSURE_CHECK_REAR:
            t24->front_solenoid = 0;
            t24->rear_solenoid = 1;
#if SKIP_PRESSURE_REAR_CHECK
            state_timer = millis();
            *seq_status = PRESSURE_CHECK2;
            break;
#endif
            if (IS_CORRELATED(t24->Rear_Pressure.Hydraulic,
                              t24->Rear_Pressure.Pneumatic,
                              EBS_REAR_HYD_GAIN_FINAL)
                && IS_UNLOADED(t24->Front_Pressure.Hydraulic)
                && check_timeout(state_timer, SOLENOID_MIN_DELAY_MS)) {
                *seq_status = PRESSURE_CHECK2;
                state_timer = millis();
            } else if (check_timeout(state_timer, TIMEOUT_SOLENOID_MS)) {
                *seq_status = SEQUENCE_ERROR;
            }
            break;

        case PRESSURE_CHECK2:
            t24->front_solenoid = 0;
            t24->rear_solenoid = 0;
#if SKIP_PRESSURE_CHECK2
            t24->Autonomous_State = AS_STATE_READY;
            break;
#endif
            if (IS_CORRELATED(t24->Rear_Pressure.Hydraulic,
                              t24->Rear_Pressure.Pneumatic,
                              EBS_REAR_HYD_GAIN_FINAL)
                && IS_CORRELATED(t24->Front_Pressure.Hydraulic,
                                 t24->Front_Pressure.Pneumatic,
                                 EBS_FRONT_HYD_GAIN)) {
                t24->Autonomous_State = AS_STATE_READY;
            } else if (check_timeout(state_timer, TIMEOUT_SOLENOID_MS)) {
                *seq_status = SEQUENCE_ERROR;
            }
            break;

        case SEQUENCE_ERROR:
            *Vehicle_state_machine = EMERGENCY;
            break;

        default:
            *seq_status = SEQUENCE_ERROR;
            break;
    }
}

/* ========================================================================
 *  Test utilities
 * ======================================================================== */

/* Reset all global state for a fresh test */
static void reset_globals(void)
{
    memset(&t24, 0, sizeof(t24));
    startup_sequence_state = WDT_TOGGLE_CHECK;
    Vehicle_state_machine  = Start;
    Autonomous_state       = Initial_Sequence;
    Emergency_cause        = NONE;
    fake_time_ms           = 0;
    state_timer            = 0;

    /* Solenoid requests must start at 0 (reset by memset already) */
    /* Ignition pin state: default to 1 so Ignition_Request gets a
     * sensible value in HV_ACTIVATION */
    t24.ignition_pin_state = 1;
}

/* Custom assert macro — returns failure code from test function */
#define TEST_ASSERT(cond, msg) do {                                    \
    if (!(cond)) {                                                     \
        printf("  FAIL: %s  [%s:%d]\n", msg, __FILE__, __LINE__);     \
        return 0;                                                      \
    }                                                                  \
} while (0)

/* ========================================================================
 *  TEST 1: Normal successful startup
 *
 *  Walks the entire sequence from WDT_TOGGLE_CHECK to AS_STATE_READY.
 * ======================================================================== */
static int test_normal_startup(void)
{
    printf("=== TEST 1: Normal successful startup ===\n");

    reset_globals();

    /* --- WDT_TOGGLE_CHECK -> WDT_STP_TOGGLE_CHECK --- */
    t24.SDC_feedback = 0;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == WDT_STP_TOGGLE_CHECK,
                "Expected WDT_STP_TOGGLE_CHECK after SDC=0");
    TEST_ASSERT(t24.HW_WDT_Enable == 0,
                "WDT should be disabled while SDC open");

    /* --- WDT_STP_TOGGLE_CHECK -> PNEUMATIC_CHECK --- */
    fake_time_ms = 5;   /* small advance so we aren't at time 0 */
    t24.SDC_feedback = 1;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == PNEUMATIC_CHECK,
                "Expected PNEUMATIC_CHECK after SDC closes");
    TEST_ASSERT(t24.HW_WDT_Enable == 1,
                "WDT should be enabled after SDC closes");

    /* --- PNEUMATIC_CHECK -> PRESSURE_CHECK1 --- */
    t24.Front_Pressure.Pneumatic = 7.5f;   /* within (6, 10) */
    t24.Rear_Pressure.Pneumatic  = 8.2f;   /* within (6, 10) */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK1,
                "Expected PRESSURE_CHECK1 after pneumatic in range");

    /* --- PRESSURE_CHECK1 -> HV_ACTIVATION --- */
    /* Front: 9 * 7.5 = 67.5,  68.0 >= 67.5 -> correlated */
    /* Rear:  3.8 * 8.2 = 31.16, 32.0 >= 31.16 -> correlated */
    t24.Front_Pressure.Hydraulic = 68.0f;
    t24.Rear_Pressure.Hydraulic  = 32.0f;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == HV_ACTIVATION,
                "Expected HV_ACTIVATION after hydraulic correlation");

    /* --- HV_ACTIVATION -> PRESSURE_CHECK_FRONT --- */
    t24.Ignition_Status = 1;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK_FRONT,
                "Expected PRESSURE_CHECK_FRONT after ignition on");

    /* Second call actually executes the PRESSURE_CHECK_FRONT body,
     * which sets front_solenoid = 1 */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    uint32_t t_front_entered = state_timer;
    _Bool front_solenoid_on  = (t24.front_solenoid == 1);
    _Bool rear_solenoid_off  = (t24.rear_solenoid == 0);
    TEST_ASSERT(front_solenoid_on,  "Solenoid1 should be ON in PRESSURE_CHECK_FRONT");
    TEST_ASSERT(rear_solenoid_off,  "Solenoid2 should be OFF in PRESSURE_CHECK_FRONT");

    /* --- PRESSURE_CHECK_FRONT -> PRESSURE_CHECK_REAR --- */
    t24.Front_Pressure.Hydraulic = 68.0f;   /* still correlated */
    t24.Rear_Pressure.Hydraulic  = 0.5f;    /* <= 1.0 -> unloaded */
    fake_time_ms = t_front_entered + 1001;  /* past SOLENOID_MIN_DELAY_MS */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK_REAR,
                "Expected PRESSURE_CHECK_REAR after front passes");

    /* Execute PRESSURE_CHECK_REAR body to set Sol1=0, Sol2=1 */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    uint32_t t_rear_entered = state_timer;
    _Bool front_solenoid_off2 = (t24.front_solenoid == 0);
    _Bool rear_solenoid_on2   = (t24.rear_solenoid == 1);
    TEST_ASSERT(front_solenoid_off2, "Solenoid1 should be OFF in PRESSURE_CHECK_REAR");
    TEST_ASSERT(rear_solenoid_on2,   "Solenoid2 should be ON in PRESSURE_CHECK_REAR");

    /* --- PRESSURE_CHECK_REAR -> PRESSURE_CHECK2 --- */
    /* Rear gain is now 3.0: 3.0 * 8.2 = 24.6, 25.0 >= 24.6 -> correlated */
    t24.Rear_Pressure.Hydraulic  = 25.0f;
    t24.Front_Pressure.Hydraulic = 0.3f;    /* <= 1.0 -> unloaded */
    fake_time_ms = t_rear_entered + 1001;   /* past SOLENOID_MIN_DELAY_MS */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK2,
                "Expected PRESSURE_CHECK2 after rear passes");

    /* Execute PRESSURE_CHECK2 body to set both solenoids off */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    _Bool both_solenoids_off = (t24.front_solenoid == 0)
                            && (t24.rear_solenoid == 0);
    TEST_ASSERT(both_solenoids_off,
                "Both solenoids should be OFF in PRESSURE_CHECK2");

    /* --- PRESSURE_CHECK2 -> AS_STATE_READY --- */
    t24.Rear_Pressure.Hydraulic  = 25.0f;   /* still correlated (24.6) */
    t24.Front_Pressure.Hydraulic = 68.0f;   /* still correlated (67.5) */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(t24.Autonomous_State == AS_STATE_READY,
                "Expected Autonomous_State == AS_STATE_READY at end of sequence");
    /* Note: startup_sequence_state stays at PRESSURE_CHECK2 — it is the
     * terminal success state; the caller (Handle_autonomous_state) checks
     * Autonomous_State to transition to Monitor_sequence. */

    printf("  PASS: Normal startup completed all 8 stages correctly\n");
    return 1;
}

/* ========================================================================
 *  TEST 2: WDT timeout
 *
 *  SDC stays open (SDC_feedback=0) past the 5 s WDT timeout.
 * ======================================================================== */
static int test_wdt_timeout(void)
{
    printf("=== TEST 2: WDT timeout ===\n");

    reset_globals();

    /* WDT_TOGGLE_CHECK -> WDT_STP_TOGGLE_CHECK (SDC open) */
    t24.SDC_feedback = 0;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == WDT_STP_TOGGLE_CHECK,
                "Expected WDT_STP_TOGGLE_CHECK");

    /* Stay in WDT_STP_TOGGLE_CHECK with SDC still open — advance past 5 s */
    fake_time_ms = 5001;   /* strictly > 5000 */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == SEQUENCE_ERROR,
                "Expected SEQUENCE_ERROR after WDT timeout");

    /* SEQUENCE_ERROR -> Vehicle_state_machine == EMERGENCY */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(Vehicle_state_machine == EMERGENCY,
                "Expected EMERGENCY after SEQUENCE_ERROR propagation");

    printf("  PASS: WDT timeout correctly triggers EMERGENCY\n");
    return 1;
}

/* ========================================================================
 *  TEST 3: Pneumatic out of range -> immediate error
 *
 *  Front pneumatic exceeds EBS_MAX_BAR (10.0).
 * ======================================================================== */
static int test_pneumatic_out_of_range(void)
{
    printf("=== TEST 3: Pneumatic out of range -> immediate error ===\n");

    reset_globals();

    /* Reach PNEUMATIC_CHECK first */
    t24.SDC_feedback = 0;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* now WDT_STP_TOGGLE_CHECK */

    fake_time_ms = 5;
    t24.SDC_feedback = 1;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* now PNEUMATIC_CHECK */

    /* Set front pneumatic above 10 bar (out of range) */
    t24.Front_Pressure.Pneumatic = 11.0f;   /* > 10.0 -> out of range */
    t24.Rear_Pressure.Pneumatic  = 8.2f;    /* OK */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == SEQUENCE_ERROR,
                "Expected SEQUENCE_ERROR when pneumatic out of range");

    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(Vehicle_state_machine == EMERGENCY,
                "Expected EMERGENCY after pneumatic error");

    printf("  PASS: Pneumatic out of range correctly triggers EMERGENCY\n");
    return 1;
}

/* ========================================================================
 *  TEST 4: Solenoid front timeout
 *
 *  Reach PRESSURE_CHECK_FRONT, keep Rear_Hydraulic > 1.0 (not unloaded),
 *  advance past 5000 ms solenoid timeout.
 * ======================================================================== */
static int test_solenoid_front_timeout(void)
{
    printf("=== TEST 4: Solenoid front timeout ===\n");

    reset_globals();

    /* --- Fast-forward through the early stages to PRESSURE_CHECK_FRONT --- */
    t24.SDC_feedback = 0;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* WDT_STP_TOGGLE_CHECK */

    fake_time_ms = 5;
    t24.SDC_feedback = 1;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* PNEUMATIC_CHECK */

    t24.Front_Pressure.Pneumatic = 7.5f;
    t24.Rear_Pressure.Pneumatic  = 8.2f;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* PRESSURE_CHECK1 */

    t24.Front_Pressure.Hydraulic = 68.0f;
    t24.Rear_Pressure.Hydraulic  = 32.0f;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* HV_ACTIVATION */

    t24.Ignition_Status = 1;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* PRESSURE_CHECK_FRONT — state_timer reset here */
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK_FRONT,
                "Expected PRESSURE_CHECK_FRONT");
    uint32_t solenoid_timer_start = state_timer;

    /* Keep Rear_Hydraulic > 1.0 so IS_UNLOADED fails -> cannot advance */
    t24.Rear_Pressure.Hydraulic  = 32.0f;   /* not unloaded */
    t24.Front_Pressure.Hydraulic = 68.0f;   /* correlated (but irrelevant) */

    /* Advance past the 5000 ms solenoid timeout */
    fake_time_ms = solenoid_timer_start + 5001;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(startup_sequence_state == SEQUENCE_ERROR,
                "Expected SEQUENCE_ERROR after solenoid front timeout");

    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    TEST_ASSERT(Vehicle_state_machine == EMERGENCY,
                "Expected EMERGENCY after solenoid timeout");

    printf("  PASS: Solenoid front timeout correctly triggers EMERGENCY\n");
    return 1;
}

/* ========================================================================
 *  TEST 5: Handle_autonomous_state transitions to Monitor_sequence
 *
 *  After initial_sequence sets t24.Autonomous_State = AS_STATE_READY,
 *  the Handle_autonomous_state logic transitions Autonomous_state from
 *  Initial_Sequence to Monitor_sequence.
 * ======================================================================== */
static int test_handle_state_transition(void)
{
    printf("=== TEST 5: Handle_autonomous_state -> Monitor_sequence ===\n");

    reset_globals();
    Autonomous_state = Initial_Sequence;  /* set by Handle_state AS_ON path */

    /* Run the full successful startup (same steps as TEST 1) */
    t24.SDC_feedback = 0;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* WDT_STP_TOGGLE_CHECK */

    fake_time_ms = 5;
    t24.SDC_feedback = 1;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* PNEUMATIC_CHECK */

    t24.Front_Pressure.Pneumatic = 7.5f;
    t24.Rear_Pressure.Pneumatic  = 8.2f;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* PRESSURE_CHECK1 */

    t24.Front_Pressure.Hydraulic = 68.0f;
    t24.Rear_Pressure.Hydraulic  = 32.0f;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* HV_ACTIVATION */

    t24.Ignition_Status = 1;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    /* PRESSURE_CHECK_FRONT */

    {
        uint32_t tf = state_timer;
        t24.Front_Pressure.Hydraulic = 68.0f;
        t24.Rear_Pressure.Hydraulic  = 0.5f;
        fake_time_ms = tf + 1001;
        initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    }
    /* PRESSURE_CHECK_REAR */

    {
        uint32_t tr = state_timer;
        t24.Rear_Pressure.Hydraulic  = 25.0f;
        t24.Front_Pressure.Hydraulic = 0.3f;
        fake_time_ms = tr + 1001;
        initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    }
    /* PRESSURE_CHECK2 */

    t24.Rear_Pressure.Hydraulic  = 25.0f;
    t24.Front_Pressure.Hydraulic = 68.0f;
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);

    /* Sanity: initial_sequence must have set AS_STATE_READY */
    TEST_ASSERT(t24.Autonomous_State == AS_STATE_READY,
                "initial_sequence should set Autonomous_State = AS_STATE_READY");
    TEST_ASSERT(Autonomous_state == Initial_Sequence,
                "Autonomous_state should still be Initial_Sequence before transition");

    /* --- Simulate Handle_autonomous_state logic (from state_machine.c) ---
     *   case Initial_Sequence:
     *       initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
     *       if (t24.Autonomous_State == AS_STATE_READY) {
     *           Autonomous_state = Monitor_sequence;
     *       }
     *       break;
     */
    initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
    if (t24.Autonomous_State == AS_STATE_READY) {
        Autonomous_state = Monitor_sequence;
    }

    TEST_ASSERT(Autonomous_state == Monitor_sequence,
                "Autonomous_state should transition to Monitor_sequence");
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK2,
                "startup_sequence should remain at PRESSURE_CHECK2");

    printf("  PASS: Handle_autonomous_state correctly transitions to Monitor_sequence\n");
    return 1;
}

/* ========================================================================
 *  Main — run all tests, count pass/fail
 * ======================================================================== */
int main(void)
{
    int passed = 0;
    int failed = 0;

    printf("\n=== Initial Sequence Test Harness ===\n\n");

    if (test_normal_startup())           passed++; else failed++;
    if (test_wdt_timeout())              passed++; else failed++;
    if (test_pneumatic_out_of_range())   passed++; else failed++;
    if (test_solenoid_front_timeout())   passed++; else failed++;
    if (test_handle_state_transition())  passed++; else failed++;

    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);

    return failed > 0 ? 1 : 0;
}
