/*
 * test_state_machine.c
 *
 * Standalone host-based unit test for:
 *   - Handle_autonomous_state()  from Core/Src/state_machine.c
 *   - Handle_state()             from Core/Src/state_machine.c
 *   - dbc_decode()               from Core/Src/APP.c
 *
 * Compile with:
 *   gcc -o test_state_machine test_state_machine.c -Wall -Wextra -std=c11 -lm
 *
 * This file is self-contained — it defines its own type stubs (no STM32 HAL),
 * provides a fake millis() and HAL_GetTick(), and copies the function-under-test
 * bodies verbatim from the firmware source files (which cannot be #included
 * because they depend on stm32f4xx_hal.h and other hardware headers).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

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

typedef enum {
    NONE, SDC_OPEN, RES, Pressure_checks, VCU_Timeout, Jetson_timeout,
    ACU_WDT_TRIGERED, dir_actuator_timeout, Dynamics_REAR_Pressure_timeout, UNKOWN
} Emergency_cause_t;

typedef enum {
    NO_TIMEOUT    = 0,
    VCU_TIMEOUT   = 1,
    JETSON_TIMEOUT = 2,
    PRESSURE_TIMEOUT = 3,
    DIR_TIMEOUT   = 4,
    RES_TIMEOUT   = 5
} can_timeouts_t;

struct pressure {
    float Pneumatic;
    float Hydraulic;
};

struct speed {
    uint8_t Speed;
    uint8_t Target_Speed;
};

/* Minimal HAL CAN header stubs */
typedef struct {
    uint32_t StdId;
} CAN_RxHeaderTypeDef;

struct can_queue {
    uint32_t TX_MAILBOX;
    uint32_t can_tx_header;              /* stub — no HAL type needed */
    CAN_RxHeaderTypeDef can_rx_header;
    uint8_t tx_data[8];
    uint32_t arrival_time;
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
    uint32_t VCU_LAST_TX, REAR_PRESSURE_LAST_TX, JETSON_LAST_TX,
             DIR_ACTUATOR_LAST_TX, RES_LAST_TX;
};

/* ========================================================================
 *  Macros — copied from Autonomous_functions.h / Autonomous_functions.c
 *  All SKIP flags are 0 (normal path) for testing.
 * ======================================================================== */

#define SKIP_WDT_CHECK              0
#define SKIP_PNEUMATIC_CHECK        0
#define SKIP_PRESSURE_CHECK1        0
#define SKIP_IGNITION_CHECK         0
#define SKIP_PRESSURE_FRONT_CHECK   0
#define SKIP_PRESSURE_REAR_CHECK    0
#define SKIP_PRESSURE_CHECK2        0

#define MAX_TIMEOUT         1000
#define TIMEOUT_WDT_MS      5000
#define TIMEOUT_SOLENOID_MS 5000
#define SOLENOID_MIN_DELAY_MS 1000

#define EBS_MIN_BAR               6.0f
#define EBS_MAX_BAR               10.0f
#define EBS_FRONT_HYD_GAIN        9.0f
#define EBS_REAR_HYD_GAIN_INITIAL 3.8f
#define EBS_REAR_HYD_GAIN_FINAL   3.0f
#define EBS_HYD_UNLOADED_BAR      1.0f

#define IN_RANGE(val, min, max)       ((val) > (min) && (val) < (max))
#define IS_CORRELATED(hyd, pneu, gain) ((hyd) >= (gain) * (pneu))
#define IS_UNLOADED(hyd)              ((hyd) <= EBS_HYD_UNLOADED_BAR)

/* ========================================================================
 *  CAN frame IDs — from autonomous_t26.h (test-local values)
 * ======================================================================== */

#define AUTONOMOUS_T26_AQT7_FRAME_ID             0x111
#define AUTONOMOUS_T26_AQT7_LENGTH               2
#define AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID     0x222
#define AUTONOMOUS_T26_VCU_IGN_R2_D_LENGTH       8
#define AUTONOMOUS_T26_JETSON_FRAME_ID           0x333
#define AUTONOMOUS_T26_JETSON_LENGTH             8
#define AUTONOMOUS_T26_VCU_RPM_FRAME_ID          0x444
#define AUTONOMOUS_T26_VCU_RPM_LENGTH            2
#define AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID 0x555
#define AUTONOMOUS_T26_RES_FRAME_ID              0x666

/* ========================================================================
 *  CAN DBC signal unpack / decode stubs (simple byte unpacking)
 * ======================================================================== */

struct autonomous_t26_aqt7_t {
    uint16_t rear_brk_press;
};
void autonomous_t26_aqt7_unpack(struct autonomous_t26_aqt7_t *dst,
                                 const uint8_t *src, int size) {
    (void)size;
    dst->rear_brk_press = (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}
double autonomous_t26_aqt7_rear_brk_press_decode(uint16_t val) {
    return (double)val * 0.1;
}

struct autonomous_t26_vcu_ign_r2_d_t {
    uint8_t ignition_manual;
    uint8_t r2d_manual;
    uint8_t ignition_auto;
    uint8_t r2d_auto;
    uint8_t shutdown_signal;
    uint8_t vcu_state;
    uint8_t r2_d_button_raw;
    uint8_t ignition_switch_raw;
};
void autonomous_t26_vcu_ign_r2_d_unpack(
    struct autonomous_t26_vcu_ign_r2_d_t *dst,
    const uint8_t *src, int size) {
    (void)size;
    dst->ignition_manual     = src[0];
    dst->r2d_manual          = src[1];
    dst->ignition_auto       = src[2];
    dst->r2d_auto            = src[3];
    dst->shutdown_signal     = src[4];
    dst->vcu_state           = src[5];
    dst->r2_d_button_raw     = src[6];
    dst->ignition_switch_raw = src[7];
}
double autonomous_t26_vcu_ign_r2_d_ignition_auto_decode(uint8_t val) {
    return (double)val;
}
double autonomous_t26_vcu_ign_r2_d_shutdown_signal_decode(uint8_t val) {
    return (double)val;
}

struct autonomous_t26_jetson_t {
    uint8_t as_state;
    uint8_t as_mission;
};
void autonomous_t26_jetson_unpack(struct autonomous_t26_jetson_t *dst,
                                   const uint8_t *src, int size) {
    (void)size;
    dst->as_state   = src[0];
    dst->as_mission = src[1];
}
double autonomous_t26_jetson_as_state_decode(uint8_t val) {
    return (double)val;
}
double autonomous_t26_jetson_as_mission_decode(uint8_t val) {
    return (double)val;
}

struct autonomous_t26_vcu_rpm_t {
    int16_t rpm_actual;
};
void autonomous_t26_vcu_rpm_unpack(struct autonomous_t26_vcu_rpm_t *dst,
                                    const uint8_t *src, int size) {
    (void)size;
    dst->rpm_actual = (int16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}
double autonomous_t26_vcu_rpm_rpm_actual_decode(int16_t val) {
    return (double)val;
}

/* ========================================================================
 *  Fake time — stub for millis() and HAL_GetTick()
 * ======================================================================== */

uint32_t fake_time_ms = 0;

uint32_t millis(void) {
    return fake_time_ms;
}

uint32_t HAL_GetTick(void) {
    return fake_time_ms;
}

/* ========================================================================
 *  Global variables — same names and types as in the firmware
 * ======================================================================== */

struct car                            t24;
Main_state_machine_t                  Vehicle_state_machine;
Autonomous_System_states_t            Autonomous_state;
startup_sequence_state_t              startup_sequence_state;
Emergency_cause_t                     Emergency_cause;
struct can_queue                      can_rx_data;

/* Ring buffer stub — just enough for can_rx_ringbuffer references in dbc_decode */
struct ring {
    uint32_t head;
    uint32_t tail;
    uint32_t counter;
    struct can_queue queue[4];
};
struct ring can_rx_ringbuffer;

/* File-scope static that matches the one in Autonomous_functions.c */
static uint32_t state_timer = 0;

/*
 * File-scope replacements for function-level statics in the test copies.
 * Exposed here so reset_globals() can clear them between tests.
 */
static uint32_t _test_mismatch_tick      = 0;
static uint8_t  _test_mismatch_active    = 0;
static uint8_t  _test_as_on_first_time   = 0;

/* ========================================================================
 *  Function bodies — copied verbatim from Core/Src/Autonomous_functions.c
 *
 *  check_timeout(), initial_sequence(), continuous_monitoring(),
 *  module_timeout().
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

/* Forward declaration */
uint8_t module_timeout(void);

void continuous_monitoring(uint8_t sdc_status, float Rear_pneumatic,
                           float Front_pneumatic, float Rear_hydraulic,
                           float Front_hydraulic) {
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

    if (!IN_RANGE(Rear_pneumatic, EBS_MIN_BAR, EBS_MAX_BAR)
        || !IN_RANGE(Front_pneumatic, EBS_MIN_BAR, EBS_MAX_BAR)) {
        Vehicle_state_machine = EMERGENCY;
        return;
    }

    if (!IS_CORRELATED(Front_hydraulic, Front_pneumatic, EBS_FRONT_HYD_GAIN)
        || !IS_CORRELATED(Rear_hydraulic, Rear_pneumatic, EBS_REAR_HYD_GAIN_FINAL)) {
        Vehicle_state_machine = EMERGENCY;
        return;
    }
}

uint8_t module_timeout(void) {
    uint32_t current_time = millis();
    if (current_time - t24.VCU_LAST_TX > MAX_TIMEOUT) return VCU_TIMEOUT;
    if (current_time - t24.REAR_PRESSURE_LAST_TX > MAX_TIMEOUT) return PRESSURE_TIMEOUT;
    if (current_time - t24.JETSON_LAST_TX > MAX_TIMEOUT) return JETSON_TIMEOUT;
    if (current_time - t24.DIR_ACTUATOR_LAST_TX > MAX_TIMEOUT) return DIR_TIMEOUT;
    if (current_time - t24.RES_LAST_TX > MAX_TIMEOUT) return RES_TIMEOUT;
    return NO_TIMEOUT;
}

/* ========================================================================
 *  Function bodies — copied verbatim from Core/Src/state_machine.c
 *
 *  Handle_autonomous_state(), Handle_state().
 *  Handle_Emergency()  — STUB (no HAL calls).
 * ======================================================================== */

void Handle_autonomous_state(void) {
    switch (Autonomous_state) {
        case Initial_Sequence:
            initial_sequence(&t24, &startup_sequence_state, &Vehicle_state_machine);
            if (t24.Autonomous_State == AS_STATE_READY) {
                Autonomous_state = Monitor_sequence;
            }
            break;
        case Monitor_sequence:
            continuous_monitoring(t24.SDC_feedback,
                                  t24.Rear_Pressure.Pneumatic,
                                  t24.Front_Pressure.Pneumatic,
                                  t24.Rear_Pressure.Hydraulic,
                                  t24.Front_Pressure.Hydraulic);
        {
            /* Debounce: require 1 s of sustained mismatch before EMERGENCY */
            if (t24.Current_Mission != t24.Jetson_mission) {
                if (!_test_mismatch_active) {
                    _test_mismatch_active = 1;
                    _test_mismatch_tick = millis();
                } else if (millis() - _test_mismatch_tick >= 1000) {
                    Vehicle_state_machine = EMERGENCY;
                }
            } else {
                _test_mismatch_active = 0;  /* mission match restored */
            }
        }
            if (t24.Autonomous_State == AS_STATE_FINISHED) {
                Autonomous_state = Finish;
            }
            break;
        case Finish:
            if (t24.rpm <= 10) {
                if (t24.ASMS == 0) {
                    t24.ASSI_state = 4;
                    t24.front_solenoid = 0;
                    t24.rear_solenoid = 0;
                    if (!t24.ASMS) {
                        Vehicle_state_machine = IDLE;
                    }
                }
            }
            break;
        case AS_Emergency:
            Vehicle_state_machine = EMERGENCY;
            break;
        default:
            Vehicle_state_machine = EMERGENCY;
            break;
    }
}

/* Stub — no HAL_GPIO_WritePin calls */
void Handle_Emergency(void) {
    t24.HW_WDT_Enable  = 0;
    t24.Ignition_Request = 0;
    t24.Emergency      = 1;
    t24.front_solenoid = 0;
    t24.rear_solenoid  = 0;
}

void Handle_state(uint8_t prev_asms_state) {
    switch (Vehicle_state_machine) {
        case Start:
            t24.HW_WDT_Enable = 1;
            Vehicle_state_machine = IDLE;
            break;
        case IDLE:
            _test_as_on_first_time = 0;
            if (t24.ASMS == 1 && prev_asms_state == 0
                && t24.ignition_pin_state == 0) {
                Vehicle_state_machine = AS_ON;
                _test_as_on_first_time = 0;
            }
            break;
        case AS_ON:
            if (!_test_as_on_first_time) {
                startup_sequence_state = WDT_TOGGLE_CHECK;
                Autonomous_state       = Initial_Sequence;
                _test_as_on_first_time = 1;
            }
            Handle_autonomous_state();
            break;
        case EMERGENCY:
            Handle_Emergency();
            if (t24.ASMS == 0 && t24.rpm < 10) {
                Vehicle_state_machine = IDLE;
            }
            break;
        default:
            Vehicle_state_machine = EMERGENCY;
            break;
    }
}

/* ========================================================================
 *  Function body — copied verbatim from Core/Src/APP.c
 *
 *  dbc_decode()  (with can_rx_data as the sole input).
 * ======================================================================== */

void dbc_decode(void) {
    switch (can_rx_data.can_rx_header.StdId) {
        case AUTONOMOUS_T26_AQT7_FRAME_ID: {
            struct autonomous_t26_aqt7_t rear_dynamics;
            autonomous_t26_aqt7_unpack(&rear_dynamics, can_rx_data.tx_data,
                                       AUTONOMOUS_T26_AQT7_LENGTH);
            t24.Rear_Pressure.Hydraulic =
                (float)autonomous_t26_aqt7_rear_brk_press_decode(
                    rear_dynamics.rear_brk_press);
            t24.REAR_PRESSURE_LAST_TX =
                can_rx_ringbuffer.queue[can_rx_ringbuffer.tail].arrival_time;
            break;
        }
        case AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID: {
            struct autonomous_t26_vcu_ign_r2_d_t vcu_data;
            autonomous_t26_vcu_ign_r2_d_unpack(
                &vcu_data, can_rx_data.tx_data,
                AUTONOMOUS_T26_VCU_IGN_R2_D_LENGTH);
            t24.Ignition_Status = (uint8_t)
                autonomous_t26_vcu_ign_r2_d_ignition_auto_decode(
                    vcu_data.ignition_auto);
            t24.vcu_sdc = (uint8_t)
                autonomous_t26_vcu_ign_r2_d_shutdown_signal_decode(
                    vcu_data.shutdown_signal);
            t24.VCU_LAST_TX = HAL_GetTick();
            break;
        }
        case AUTONOMOUS_T26_JETSON_FRAME_ID: {
            struct autonomous_t26_jetson_t jetson_data;
            autonomous_t26_jetson_unpack(
                &jetson_data, can_rx_data.tx_data,
                AUTONOMOUS_T26_JETSON_LENGTH);
            t24.Autonomous_State = (AS_STATE_t)
                autonomous_t26_jetson_as_state_decode(jetson_data.as_state);
            t24.Jetson_mission = (current_mission_t)
                autonomous_t26_jetson_as_mission_decode(jetson_data.as_mission);
            t24.JETSON_LAST_TX = HAL_GetTick();
            break;
        }
        case AUTONOMOUS_T26_VCU_RPM_FRAME_ID: {
            struct autonomous_t26_vcu_rpm_t vcu_rpm;
            autonomous_t26_vcu_rpm_unpack(
                &vcu_rpm, can_rx_data.tx_data,
                AUTONOMOUS_T26_VCU_RPM_LENGTH);
            t24.rpm = (int)
                autonomous_t26_vcu_rpm_rpm_actual_decode(vcu_rpm.rpm_actual);
            break;
        }
        case AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID:
            t24.DIR_ACTUATOR_LAST_TX = HAL_GetTick();
            break;
        case AUTONOMOUS_T26_RES_FRAME_ID:
            t24.RES_LAST_TX = HAL_GetTick();
            break;
        default:
            break;
    }
}

/* ========================================================================
 *  Test utilities
 * ======================================================================== */

static void reset_globals(void) {
    memset(&t24, 0, sizeof(t24));
    Vehicle_state_machine  = Start;
    Autonomous_state       = OFF;
    startup_sequence_state = WDT_TOGGLE_CHECK;
    Emergency_cause        = NONE;
    fake_time_ms           = 0;
    state_timer            = 0;
    memset(&can_rx_data, 0, sizeof(can_rx_data));
    memset(&can_rx_ringbuffer, 0, sizeof(can_rx_ringbuffer));

    /* Solenoid requests must start at 0 (reset by memset already). */
    /* Ignition pin state: default to 1 so Ignition_Request gets a
     * sensible value in HV_ACTIVATION */
    t24.ignition_pin_state = 1;

    /* Clear file-scope statics used in function test copies */
    _test_mismatch_tick      = 0;
    _test_mismatch_active    = 0;
    _test_as_on_first_time   = 0;
}

/* Custom assert macro — returns failure code from test function */
#define TEST_ASSERT(cond, msg) do {                                    \
    if (!(cond)) {                                                     \
        printf("  FAIL: %s  [%s:%d]\n", msg, __FILE__, __LINE__);     \
        return 0;                                                      \
    }                                                                  \
} while (0)

/* ========================================================================
 *  TEST 1: Handle_autonomous_state Initial_Sequence -> Monitor_sequence
 *
 *  Set Autonomous_state = Initial_Sequence, startup_sequence_state =
 *  WDT_TOGGLE_CHECK.  Walk through the full initial_sequence using
 *  repeated Handle_autonomous_state() calls with proper pressures and
 *  timing (same pattern as test_initial_sequence.c test_normal_startup).
 *  After AS_STATE_READY is reached, verify Autonomous_state transitions
 *  to Monitor_sequence.
 * ======================================================================== */

static int test_handle_auto_init_to_monitor(void)
{
    printf("=== TEST 1: Handle_autonomous_state Initial_Sequence -> Monitor_sequence ===\n");

    reset_globals();
    Autonomous_state       = Initial_Sequence;
    startup_sequence_state = WDT_TOGGLE_CHECK;

    /* --- WDT_TOGGLE_CHECK -> WDT_STP_TOGGLE_CHECK --- */
    t24.SDC_feedback = 0;
    Handle_autonomous_state();
    TEST_ASSERT(startup_sequence_state == WDT_STP_TOGGLE_CHECK,
                "Expected WDT_STP_TOGGLE_CHECK after SDC=0");
    TEST_ASSERT(t24.HW_WDT_Enable == 0,
                "WDT should be disabled while SDC open");

    /* --- WDT_STP_TOGGLE_CHECK -> PNEUMATIC_CHECK --- */
    fake_time_ms = 5;
    t24.SDC_feedback = 1;
    Handle_autonomous_state();
    TEST_ASSERT(startup_sequence_state == PNEUMATIC_CHECK,
                "Expected PNEUMATIC_CHECK after SDC closes");
    TEST_ASSERT(t24.HW_WDT_Enable == 1,
                "WDT should be enabled after SDC closes");

    /* --- PNEUMATIC_CHECK -> PRESSURE_CHECK1 --- */
    t24.Front_Pressure.Pneumatic = 7.5f;
    t24.Rear_Pressure.Pneumatic  = 8.2f;
    Handle_autonomous_state();
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK1,
                "Expected PRESSURE_CHECK1 after pneumatic in range");

    /* --- PRESSURE_CHECK1 -> HV_ACTIVATION --- */
    t24.Front_Pressure.Hydraulic = 68.0f;
    t24.Rear_Pressure.Hydraulic  = 32.0f;
    Handle_autonomous_state();
    TEST_ASSERT(startup_sequence_state == HV_ACTIVATION,
                "Expected HV_ACTIVATION after hydraulic correlation");

    /* --- HV_ACTIVATION -> PRESSURE_CHECK_FRONT --- */
    t24.Ignition_Status = 1;
    Handle_autonomous_state();
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK_FRONT,
                "Expected PRESSURE_CHECK_FRONT after ignition on");

    /* Execute PRESSURE_CHECK_FRONT body */
    Handle_autonomous_state();
    uint32_t t_front_entered = state_timer;
    TEST_ASSERT(t24.front_solenoid == 1,
                "Solenoid1 should be ON in PRESSURE_CHECK_FRONT");
    TEST_ASSERT(t24.rear_solenoid == 0,
                "Solenoid2 should be OFF in PRESSURE_CHECK_FRONT");

    /* --- PRESSURE_CHECK_FRONT -> PRESSURE_CHECK_REAR --- */
    t24.Front_Pressure.Hydraulic = 68.0f;   /* still correlated */
    t24.Rear_Pressure.Hydraulic  = 0.5f;    /* <= 1.0 -> unloaded */
    fake_time_ms = t_front_entered + 1001;  /* past SOLENOID_MIN_DELAY_MS */
    Handle_autonomous_state();
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK_REAR,
                "Expected PRESSURE_CHECK_REAR after front passes");

    /* Execute PRESSURE_CHECK_REAR body */
    Handle_autonomous_state();
    uint32_t t_rear_entered = state_timer;
    TEST_ASSERT(t24.front_solenoid == 0,
                "Solenoid1 should be OFF in PRESSURE_CHECK_REAR");
    TEST_ASSERT(t24.rear_solenoid == 1,
                "Solenoid2 should be ON in PRESSURE_CHECK_REAR");

    /* --- PRESSURE_CHECK_REAR -> PRESSURE_CHECK2 --- */
    t24.Rear_Pressure.Hydraulic  = 25.0f;   /* 3.0*8.2=24.6 -> correlated */
    t24.Front_Pressure.Hydraulic = 0.3f;    /* <= 1.0 -> unloaded */
    fake_time_ms = t_rear_entered + 1001;   /* past SOLENOID_MIN_DELAY_MS */
    Handle_autonomous_state();
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK2,
                "Expected PRESSURE_CHECK2 after rear passes");

    /* Execute PRESSURE_CHECK2 body (front not yet correlated) */
    Handle_autonomous_state();
    TEST_ASSERT(t24.front_solenoid == 0,
                "Solenoid1 should be OFF in PRESSURE_CHECK2");
    TEST_ASSERT(t24.rear_solenoid == 0,
                "Solenoid2 should be OFF in PRESSURE_CHECK2");

    /* --- PRESSURE_CHECK2 -> AS_STATE_READY, then Monitor_sequence --- */
    t24.Rear_Pressure.Hydraulic  = 25.0f;   /* still correlated */
    t24.Front_Pressure.Hydraulic = 68.0f;   /* now correlated */
    Handle_autonomous_state();
    TEST_ASSERT(t24.Autonomous_State == AS_STATE_READY,
                "Expected Autonomous_State == AS_STATE_READY");
    TEST_ASSERT(Autonomous_state == Monitor_sequence,
                "Expected Autonomous_state transition to Monitor_sequence");
    TEST_ASSERT(startup_sequence_state == PRESSURE_CHECK2,
                "startup_sequence should remain at PRESSURE_CHECK2");

    printf("  PASS: Full initial sequence + Monitor_sequence transition\n");
    return 1;
}

/* ========================================================================
 *  TEST 2: Handle_state Start -> IDLE
 *
 *  Set Vehicle_state_machine=Start, call Handle_state(0).  Verify it
 *  transitions to IDLE and HW_WDT_Enable=1.
 * ======================================================================== */

static int test_handle_state_start_to_idle(void)
{
    printf("=== TEST 2: Handle_state Start -> IDLE ===\n");

    reset_globals();
    Vehicle_state_machine = Start;

    Handle_state(0);

    TEST_ASSERT(Vehicle_state_machine == IDLE,
                "Expected IDLE after Start");
    TEST_ASSERT(t24.HW_WDT_Enable == 1,
                "HW_WDT_Enable should be set to 1 in Start");

    printf("  PASS: Start -> IDLE with WDT enabled\n");
    return 1;
}

/* ========================================================================
 *  TEST 3: Handle_state IDLE -> AS_ON
 *
 *  Set Vehicle_state_machine=IDLE, t24.ASMS=1 with prev_asms_state=0
 *  (rising edge), t24.ignition_pin_state=0.  Call Handle_state(0).
 *  Verify transitions to AS_ON.
 * ======================================================================== */

static int test_handle_state_idle_to_as_on(void)
{
    printf("=== TEST 3: Handle_state IDLE -> AS_ON ===\n");

    reset_globals();
    Vehicle_state_machine  = IDLE;
    t24.ASMS               = 1;
    t24.ignition_pin_state = 0;   /* ASMS button directly controls AS_ON */

    /* prev_asms_state == 0 -> rising edge detected */
    Handle_state(0);

    TEST_ASSERT(Vehicle_state_machine == AS_ON,
                "Expected AS_ON after rising edge of ASMS with ignition_pin_state=0");

    printf("  PASS: IDLE -> AS_ON on ASMS rising edge\n");
    return 1;
}

/* ========================================================================
 *  TEST 4: Handle_state EMERGENCY recovery
 *
 *  Set Vehicle_state_machine=EMERGENCY, t24.ASMS=1 (prevent recovery on
 *  first call), t24.rpm=5.  First call: verify Emergency=1 set but state
 *  stays EMERGENCY because ASMS=1.  Then set ASMS=0, second call: verify
 *  recovery to IDLE.
 * ======================================================================== */

static int test_handle_state_emergency_recovery(void)
{
    printf("=== TEST 4: Handle_state EMERGENCY recovery ===\n");

    reset_globals();
    Vehicle_state_machine = EMERGENCY;
    t24.ASMS              = 1;   /* prevent recovery on first call */
    t24.rpm               = 5;   /* below threshold */

    /* First call: EMERGENCY state, Handle_Emergency fires */
    Handle_state(0);

    TEST_ASSERT(t24.Emergency == 1,
                "Handle_Emergency should set Emergency=1");
    TEST_ASSERT(Vehicle_state_machine == EMERGENCY,
                "Should stay in EMERGENCY because ASMS==1 (not 0)");

    /* Second call: now ASMS=0, condition should recover */
    t24.ASMS = 0;
    Handle_state(0);

    TEST_ASSERT(Vehicle_state_machine == IDLE,
                "Should recover to IDLE when ASMS=0 and rpm<10");

    /* Third call: verify it stays IDLE */
    Handle_state(0);
    TEST_ASSERT(Vehicle_state_machine == IDLE,
                "Should remain IDLE after recovery");

    printf("  PASS: EMERGENCY correctly recovers to IDLE\n");
    return 1;
}

/* ========================================================================
 *  TEST 5: dbc_decode AQT7 rear pressure
 *
 *  Set StdId = 0x111 (AUTONOMOUS_T26_AQT7_FRAME_ID).
 *  Put known rear_brk_press bytes (1234 -> 0x04D2).
 *  Call dbc_decode().  Verify t24.Rear_Pressure.Hydraulic == 123.4f.
 * ======================================================================== */

static int test_dbc_decode_aqt7(void)
{
    printf("=== TEST 5: dbc_decode AQT7 rear pressure ===\n");

    reset_globals();

    can_rx_data.can_rx_header.StdId = AUTONOMOUS_T26_AQT7_FRAME_ID;
    /* rear_brk_press = 1234 -> 0x04D2 */
    can_rx_data.tx_data[0] = 0xD2;
    can_rx_data.tx_data[1] = 0x04;

    dbc_decode();

    /* 1234 * 0.1 = 123.4 */
    float expected = 123.4f;
    float actual   = t24.Rear_Pressure.Hydraulic;
    TEST_ASSERT(fabsf(actual - expected) < 0.001f,
                "Rear_Pressure.Hydraulic should be 123.4");

    printf("  PASS: AQT7 decoded Rear_Pressure.Hydraulic = %.1f\n",
           (double)actual);
    return 1;
}

/* ========================================================================
 *  TEST 6: dbc_decode VCU_IGN_R2_D
 *
 *  Set StdId = 0x222, bytes[2]=1 (ignition_auto), bytes[4]=1
 *  (shutdown_signal).  Call dbc_decode().  Verify t24.Ignition_Status=1
 *  and t24.vcu_sdc=1.
 * ======================================================================== */

static int test_dbc_decode_vcu_ign(void)
{
    printf("=== TEST 6: dbc_decode VCU_IGN_R2_D ===\n");

    reset_globals();

    can_rx_data.can_rx_header.StdId = AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID;
    can_rx_data.tx_data[2] = 1;   /* ignition_auto = 1 */
    can_rx_data.tx_data[4] = 1;   /* shutdown_signal = 1 */

    dbc_decode();

    TEST_ASSERT(t24.Ignition_Status == 1,
                "Ignition_Status should be 1");
    TEST_ASSERT(t24.vcu_sdc == 1,
                "vcu_sdc should be 1");

    printf("  PASS: VCU_IGN_R2_D decoded Ignition_Status=1, vcu_sdc=1\n");
    return 1;
}

/* ========================================================================
 *  TEST 7: dbc_decode JETSON mission
 *
 *  Set StdId = 0x333, bytes[0]=2 (as_state=READY), bytes[1]=3
 *  (as_mission=TRACKDRIVE).  Call dbc_decode().  Verify
 *  t24.Autonomous_State = AS_STATE_READY, t24.Jetson_mission = TRACKDRIVE.
 * ======================================================================== */

static int test_dbc_decode_jetson(void)
{
    printf("=== TEST 7: dbc_decode JETSON mission ===\n");

    reset_globals();

    can_rx_data.can_rx_header.StdId = AUTONOMOUS_T26_JETSON_FRAME_ID;
    can_rx_data.tx_data[0] = 2;   /* as_state = 2 = READY */
    can_rx_data.tx_data[1] = 3;   /* as_mission = 3 = TRACKDRIVE */

    dbc_decode();

    TEST_ASSERT(t24.Autonomous_State == AS_STATE_READY,
                "Autonomous_State should be AS_STATE_READY (2)");
    TEST_ASSERT(t24.Jetson_mission == TRACKDRIVE,
                "Jetson_mission should be TRACKDRIVE (3)");

    printf("  PASS: JETSON decoded AS_STATE_READY, TRACKDRIVE\n");
    return 1;
}

/* ========================================================================
 *  TEST 8: dbc_decode VCU_RPM
 *
 *  Set StdId = 0x444, bytes = {0xB8, 0x0B} for rpm=3000 (0x0BB8).
 *  Call dbc_decode().  Verify t24.rpm = 3000.
 * ======================================================================== */

static int test_dbc_decode_vcu_rpm(void)
{
    printf("=== TEST 8: dbc_decode VCU_RPM ===\n");

    reset_globals();

    can_rx_data.can_rx_header.StdId = AUTONOMOUS_T26_VCU_RPM_FRAME_ID;
    /* 3000 RPM = 0x0BB8 in little-endian */
    can_rx_data.tx_data[0] = 0xB8;
    can_rx_data.tx_data[1] = 0x0B;

    dbc_decode();

    TEST_ASSERT(t24.rpm == 3000,
                "rpm should be 3000");

    printf("  PASS: VCU_RPM decoded rpm = %d\n", t24.rpm);
    return 1;
}

/* ========================================================================
 *  TEST 9: Mission mismatch debounce — no immediate EMERGENCY
 *
 *  Set up a mission mismatch (ACCELERATION vs SKIDPAD) while in
 *  Monitor_sequence with all monitoring conditions passing.
 *  Verify that the debounce waits at least 1 s before escalating.
 * ======================================================================== */

static int test_mission_mismatch_no_immediate_emergency(void)
{
    printf("=== TEST 9: Mission mismatch — no immediate EMERGENCY ===\n");

    reset_globals();

    Autonomous_state          = Monitor_sequence;
    t24.Current_Mission       = ACCELERATION;   /* 1 */
    t24.Jetson_mission        = SKIDPAD;        /* 2 — mismatch */
    t24.SDC_feedback          = 1;
    /* Good pressures so continuous_monitoring() passes */
    t24.Front_Pressure.Pneumatic  = 7.5f;
    t24.Rear_Pressure.Pneumatic   = 8.2f;
    t24.Front_Pressure.Hydraulic  = 68.0f;      /* 9.0 * 7.5 = 67.5 */
    t24.Rear_Pressure.Hydraulic   = 25.0f;      /* 3.0 * 8.2 = 24.6 */
    /* Keep timeouts happy — all LAST_TX match fake_time_ms (0) */

    /* Call at t = 0 ms — debounce should just start */
    fake_time_ms = 0;
    Handle_autonomous_state();
    TEST_ASSERT(Vehicle_state_machine != EMERGENCY,
                "Should NOT trigger EMERGENCY at t=0 (debounce just started)");

    /* Advance to t = 500 ms — still inside the 1 s window */
    fake_time_ms = 500;
    Handle_autonomous_state();
    TEST_ASSERT(Vehicle_state_machine != EMERGENCY,
                "Should NOT trigger EMERGENCY at t=500 ms (debounce < 1 s)");

    printf("  PASS: No premature EMERGENCY from mission mismatch debounce\n");
    return 1;
}

/* ========================================================================
 *  TEST 10: Mission mismatch debounce — triggers after 1 s
 *
 *  Same setup as TEST 9, but advance past the 1 s threshold and verify
 *  that Vehicle_state_machine becomes EMERGENCY.
 * ======================================================================== */

static int test_mission_mismatch_triggers_after_1s(void)
{
    printf("=== TEST 10: Mission mismatch — triggers after 1 s ===\n");

    reset_globals();

    Autonomous_state          = Monitor_sequence;
    t24.Current_Mission       = ACCELERATION;
    t24.Jetson_mission        = SKIDPAD;
    t24.SDC_feedback          = 1;
    t24.Front_Pressure.Pneumatic  = 7.5f;
    t24.Rear_Pressure.Pneumatic   = 8.2f;
    t24.Front_Pressure.Hydraulic  = 68.0f;
    t24.Rear_Pressure.Hydraulic   = 25.0f;

    /* Call at t = 0 — debounce starts */
    fake_time_ms = 0;
    Handle_autonomous_state();
    TEST_ASSERT(Vehicle_state_machine != EMERGENCY,
                "Not EMERGENCY at t=0");

    /* Advance to exactly 1 s — threshold reached */
    fake_time_ms = 1000;
    Handle_autonomous_state();
    TEST_ASSERT(Vehicle_state_machine == EMERGENCY,
                "Should be EMERGENCY at t=1000 ms (debounce expired)");

    printf("  PASS: EMERGENCY triggered after 1 s debounce\n");
    return 1;
}

/* ========================================================================
 *  TEST 11: Mission mismatch cleared before timeout
 *
 *  Start with a mismatch, then restore matching missions before the
 *  1 s debounce expires.  Verify that EMERGENCY is never entered and
 *  the debounce resets.
 * ======================================================================== */

static int test_mission_mismatch_clears_before_timeout(void)
{
    printf("=== TEST 11: Mission mismatch — cleared before timeout ===\n");

    reset_globals();

    /* Prevent false module_timeout() — CAN timestamps must stay recent
     * relative to fake_time_ms advances throughout the test.
     */
    t24.VCU_LAST_TX = fake_time_ms;
    t24.REAR_PRESSURE_LAST_TX = fake_time_ms;
    t24.JETSON_LAST_TX = fake_time_ms;
    t24.DIR_ACTUATOR_LAST_TX = fake_time_ms;
    t24.RES_LAST_TX = fake_time_ms;

    Autonomous_state          = Monitor_sequence;
    t24.Current_Mission       = ACCELERATION;
    t24.Jetson_mission        = SKIDPAD;
    t24.SDC_feedback          = 1;
    t24.Front_Pressure.Pneumatic  = 7.5f;
    t24.Rear_Pressure.Pneumatic   = 8.2f;
    t24.Front_Pressure.Hydraulic  = 68.0f;
    t24.Rear_Pressure.Hydraulic   = 25.0f;

    /* Call at t = 0 — debounce starts */
    fake_time_ms = 0;
    t24.VCU_LAST_TX = fake_time_ms;
    t24.REAR_PRESSURE_LAST_TX = fake_time_ms;
    t24.JETSON_LAST_TX = fake_time_ms;
    t24.DIR_ACTUATOR_LAST_TX = fake_time_ms;
    t24.RES_LAST_TX = fake_time_ms;
    Handle_autonomous_state();
    TEST_ASSERT(Vehicle_state_machine != EMERGENCY,
                "Not EMERGENCY at t=0");

    /* At t = 500 ms, clear the mismatch */
    fake_time_ms = 500;
    t24.VCU_LAST_TX = fake_time_ms;
    t24.REAR_PRESSURE_LAST_TX = fake_time_ms;
    t24.JETSON_LAST_TX = fake_time_ms;
    t24.DIR_ACTUATOR_LAST_TX = fake_time_ms;
    t24.RES_LAST_TX = fake_time_ms;
    t24.Current_Mission = t24.Jetson_mission;   /* now both SKIDPAD */
    Handle_autonomous_state();
    TEST_ASSERT(Vehicle_state_machine != EMERGENCY,
                "Not EMERGENCY after mismatch cleared at t=500 ms");

    /* At t = 2000 ms, verify still no EMERGENCY (debounce was reset) */
    fake_time_ms = 2000;
    t24.VCU_LAST_TX = fake_time_ms;
    t24.REAR_PRESSURE_LAST_TX = fake_time_ms;
    t24.JETSON_LAST_TX = fake_time_ms;
    t24.DIR_ACTUATOR_LAST_TX = fake_time_ms;
    t24.RES_LAST_TX = fake_time_ms;
    Handle_autonomous_state();
    TEST_ASSERT(Vehicle_state_machine != EMERGENCY,
                "Not EMERGENCY at t=2000 ms (debounce was reset when mismatch cleared)");

    printf("  PASS: Debounce correctly reset after mismatch cleared\n");
    return 1;
}

/* ========================================================================
 *  TEST 12: Mission mismatch does not block Finish transition
 *
 *  Start with a mission mismatch active.  Set Autonomous_State to
 *  AS_STATE_FINISHED before the debounce expires.  Verify that
 *  Autonomous_state transitions to Finish (not blocked by debounce).
 * ======================================================================== */

static int test_mission_mismatch_does_not_block_finish(void)
{
    printf("=== TEST 12: Mission mismatch — does not block Finish ===\n");

    reset_globals();

    Autonomous_state          = Monitor_sequence;
    t24.Current_Mission       = ACCELERATION;
    t24.Jetson_mission        = SKIDPAD;
    t24.SDC_feedback          = 1;
    t24.Front_Pressure.Pneumatic  = 7.5f;
    t24.Rear_Pressure.Pneumatic   = 8.2f;
    t24.Front_Pressure.Hydraulic  = 68.0f;
    t24.Rear_Pressure.Hydraulic   = 25.0f;

    /* Call at t = 0 — debounce starts, Autonomous_State still driving */
    t24.Autonomous_State = AS_STATE_DRIVING;
    fake_time_ms = 0;
    Handle_autonomous_state();
    TEST_ASSERT(Vehicle_state_machine != EMERGENCY,
                "Not EMERGENCY at t=0");
    TEST_ASSERT(Autonomous_state == Monitor_sequence,
                "Should remain in Monitor_sequence while driving");

    /* At t = 500 ms, set finished — must transition even with mismatch */
    fake_time_ms = 500;
    t24.Autonomous_State = AS_STATE_FINISHED;
    Handle_autonomous_state();
    TEST_ASSERT(Autonomous_state == Finish,
                "Should transition to Finish despite active mismatch debounce");
    TEST_ASSERT(Vehicle_state_machine != EMERGENCY,
                "EMERGENCY should not be set when Finish takes precedence");

    printf("  PASS: Finish transition not blocked by mission mismatch debounce\n");
    return 1;
}

/* ========================================================================
 *  Main — run all 12 tests, count pass/fail
 * ======================================================================== */

int main(void)
{
    int passed = 0;
    int failed = 0;

    printf("\n=== State Machine / dbc_decode Test Harness ===\n\n");

    if (test_handle_auto_init_to_monitor())              passed++; else failed++;
    if (test_handle_state_start_to_idle())                passed++; else failed++;
    if (test_handle_state_idle_to_as_on())                passed++; else failed++;
    if (test_handle_state_emergency_recovery())           passed++; else failed++;
    if (test_dbc_decode_aqt7())                           passed++; else failed++;
    if (test_dbc_decode_vcu_ign())                        passed++; else failed++;
    if (test_dbc_decode_jetson())                         passed++; else failed++;
    if (test_dbc_decode_vcu_rpm())                        passed++; else failed++;
    if (test_mission_mismatch_no_immediate_emergency())   passed++; else failed++;
    if (test_mission_mismatch_triggers_after_1s())        passed++; else failed++;
    if (test_mission_mismatch_clears_before_timeout())    passed++; else failed++;
    if (test_mission_mismatch_does_not_block_finish())    passed++; else failed++;

    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);

    return failed > 0 ? 1 : 0;
}
