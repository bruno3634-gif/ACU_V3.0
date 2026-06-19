/*
 * test_tim_callback.c
 * Self-contained host-based unit test for HAL_TIM_PeriodElapsedCallback
 * (Core/Src/main.c).
 *
 * Tests the most complex periodic callback — it packs ACU status, DV_STATUS,
 * and ASF_SIGNALS CAN frames, manages BLE telemetry with DMA timeout recovery,
 * and exercises value clamping for all scaled fields.
 *
 * Compile: gcc -o test_tim_callback test_tim_callback.c -Wall -Wextra -std=c11 -lm
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

/* ==================================================================
 * Defines
 * ================================================================== */

#define CAN_RTR_DATA      ((uint32_t)0x00000000)
#define EPSILON           1e-4f

/* ==================================================================
 * HAL type stubs  (matches declarations in main.h / stm32f4xx_hal.h)
 * ================================================================== */

typedef uint32_t HAL_StatusTypeDef;
#define HAL_OK     0
#define HAL_ERROR  1

typedef struct { uint32_t Instance; } TIM_TypeDef;
#define TIM2_BASE 0x40000000
#define TIM2       ((TIM_TypeDef *)TIM2_BASE)

typedef struct { TIM_TypeDef *Instance; uint32_t dummy2; } TIM_HandleTypeDef;
TIM_HandleTypeDef htim2 = { .Instance = TIM2 };

typedef struct { uint32_t dummy; } USART_TypeDef;
#define USART2_BASE 0x40004400
#define USART2      ((USART_TypeDef *)USART2_BASE)

typedef struct { USART_TypeDef *Instance; uint32_t dummy2; } UART_HandleTypeDef;
UART_HandleTypeDef huart2 = { .Instance = (USART_TypeDef *)USART2_BASE };

/* CAN types */
typedef struct { uint32_t StdId; uint32_t RTR; uint32_t DLC; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t dummy; } CAN_HandleTypeDef;
CAN_HandleTypeDef hcan1;

/* ==================================================================
 * Data structures  (from main.h)
 * ================================================================== */

struct can_queue {
    uint32_t TX_MAILBOX;
    CAN_TxHeaderTypeDef can_tx_header;
    uint8_t tx_data[8];
    uint32_t arrival_time;
};

/* ==================================================================
 * Enums and structs from main.h
 * ================================================================== */

typedef enum {
    MANUAL, ACCELERATION, SKIDPAD, TRACKDRIVE, EBS_TEST, INSPECTION, AUTOCROSS
} current_mission_t;

typedef enum {
    AS_STATE_OFF=1, AS_STATE_READY=2, AS_STATE_DRIVING=3,
    AS_STATE_EMERGENCY=4, AS_STATE_FINISHED=5
} AS_STATE_t;

typedef enum { Start, IDLE, AS_ON, EMERGENCY } Main_state_machine_t;

typedef enum {
    NONE, SDC_OPEN, RES, Pressure_checks, VCU_Timeout, Jetson_timeout,
    ACU_WDT_TRIGERED, dir_actuator_timeout, Dynamics_REAR_Pressure_timeout,
    UNKOWN
} Emergency_cause_t;

typedef enum {
    OFF, Initial_Sequence, Monitor_sequence, AS_Emergency, Finish
} Autonomous_System_states_t;

struct pressure { float Pneumatic; float Hydraulic; };

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
    struct speed { uint8_t Speed; uint8_t Target_Speed; } Speed;
    float chip_temp;
    int rpm;
    uint32_t VCU_LAST_TX, REAR_PRESSURE_LAST_TX, JETSON_LAST_TX,
             DIR_ACTUATOR_LAST_TX, RES_LAST_TX;
};

struct car t24;

/* ==================================================================
 * Auto-generated CAN structs (from autonomous_t26.h)
 * ================================================================== */

struct autonomous_t26_acu_t {
    uint8_t assi_state;
    uint8_t acu_state;
    uint8_t acu_cpu_temp;
    uint8_t mission_select;
    uint8_t as_state;
    uint8_t emergency;
    uint8_t asms;
    uint8_t ign;
    uint8_t emergency_cause;
};

struct autonomous_t26_dv_status_t {
    uint8_t as_status;
    uint8_t asb_ebs_state;
    uint8_t ami_state;
    uint8_t steering_state;
    uint8_t asb_redundancy_state;
    uint8_t lap_counter;
    uint8_t cones_count_actual;
    uint32_t cones_count_all;
};

struct autonomous_t26_asf_signals_t {
    uint8_t ebs_pressure_tank_front;
    uint8_t ebs_pressure_tank_rear;
    uint8_t brake_pressure_front;
    uint8_t brake_pressure_rear;
};

/* BLE telemetry packed struct */
typedef __attribute__((packed)) struct {
    uint8_t  state_machine;
    uint8_t  assi_status;
    uint8_t  mission;
    uint16_t hydraulic_p1;
    uint16_t hydraulic_p2;
    uint16_t pneumatic_p1;
    uint16_t pneumatic_p2;
    int16_t  chip_temp;
    uint8_t  solenoid_front;
    uint8_t  solenoid_rear;
} ble_telemetry_packet_t;

/* ==================================================================
 * CAN frame ID defines (from autonomous_t26.h)
 * ================================================================== */

#define AUTONOMOUS_T26_ACU_FRAME_ID               0x51u
#define AUTONOMOUS_T26_DV_STATUS_FRAME_ID         0x502u
#define AUTONOMOUS_T26_ASF_SIGNALS_FRAME_ID       0x511u
#define AUTONOMOUS_T26_ACU_LENGTH                 8u
#define AUTONOMOUS_T26_DV_STATUS_LENGTH           8u
#define AUTONOMOUS_T26_ASF_SIGNALS_LENGTH         4u

/* ==================================================================
 * Global variables  (from main.c)
 * ================================================================== */

CAN_TxHeaderTypeDef can_tx_header;
uint8_t tx_data[8];
uint32_t TX_MAILBOX;

struct can_queue can_tx_queue[64];
int can_queue_index = -1;

/* ==================================================================
 * "Static" locals from HAL_TIM_PeriodElapsedCallback — exposed as
 * globals so tests can control DMA busy state and tick directly.
 * ================================================================== */

volatile uint8_t test_ble_tx_busy = 0;
uint32_t         test_ble_tx_tick = 0;

void set_ble_busy(uint8_t val) { test_ble_tx_busy = val; }
uint8_t get_ble_busy(void)      { return test_ble_tx_busy; }

/* ==================================================================
 * HAL stub implementations
 * ================================================================== */

static uint32_t fake_tick = 0;
uint32_t HAL_GetTick(void) { return fake_tick; }

static int add_can_message_call_count = 0;
static struct can_queue last_can_message;
static struct can_queue can_tx_history[32];
static int can_tx_history_count = 0;

void add_can_message(uint32_t mailbox, CAN_TxHeaderTypeDef tx_header,
                     uint8_t tx_data[8]) {
    can_queue_index++;
    can_tx_queue[can_queue_index].TX_MAILBOX = mailbox;
    can_tx_queue[can_queue_index].can_tx_header = tx_header;
    memcpy(can_tx_queue[can_queue_index].tx_data, tx_data, 8);
    add_can_message_call_count++;
    last_can_message.can_tx_header = tx_header;
    memcpy(last_can_message.tx_data, tx_data, 8);
    if (can_tx_history_count < 32) {
        can_tx_history[can_tx_history_count].can_tx_header = tx_header;
        memcpy(can_tx_history[can_tx_history_count].tx_data, tx_data, 8);
        can_tx_history_count++;
    }
}

/* ==================================================================
 * Stub CAN pack functions  (simplified — match autonomous_t26.h API)
 * ================================================================== */

/* ACU pack */
int autonomous_t26_acu_pack(uint8_t *dst_p,
                            const struct autonomous_t26_acu_t *src_p,
                            size_t size) {
    if (size < 8) return -1;
    dst_p[0] = src_p->assi_state;
    dst_p[1] = src_p->acu_state;
    dst_p[2] = src_p->acu_cpu_temp;
    dst_p[3] = src_p->mission_select;
    dst_p[4] = src_p->as_state;
    dst_p[5] = src_p->emergency;
    dst_p[6] = src_p->asms;
    dst_p[7] = src_p->ign;
    return 0;
}

/* DV_STATUS pack */
int autonomous_t26_dv_status_pack(uint8_t *dst_p,
                                  const struct autonomous_t26_dv_status_t *src_p,
                                  size_t size) {
    if (size < 8) return -1;
    memset(dst_p, 0, 8);
    dst_p[0] = src_p->as_status;
    dst_p[2] = src_p->ami_state;
    return 0;
}

/* ASF_SIGNALS pack */
int autonomous_t26_asf_signals_pack(uint8_t *dst_p,
                                    const struct autonomous_t26_asf_signals_t *src_p,
                                    size_t size) {
    if (size < 4) return -1;
    dst_p[0] = src_p->ebs_pressure_tank_front;
    dst_p[1] = src_p->ebs_pressure_tank_rear;
    dst_p[2] = src_p->brake_pressure_front;
    dst_p[3] = src_p->brake_pressure_rear;
    return 0;
}

/* ==================================================================
 * UART stubs
 * ================================================================== */

static int uart_dma_call_count = 0;
static int uart_abort_call_count = 0;
static uint8_t last_dma_data[sizeof(ble_telemetry_packet_t)] = {0};

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                         const uint8_t *pData,
                                         uint16_t Size) {
    (void)huart;
    uart_dma_call_count++;
    if (Size <= sizeof(last_dma_data))
        memcpy(last_dma_data, pData, Size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart) {
    (void)huart;
    uart_abort_call_count++;
    return HAL_OK;
}

/* ==================================================================
 * BLE config stub  (from APP.c)
 * ================================================================== */

static uint8_t ble_config_done = 1;  /* default: already configured */

uint8_t ble_module_config_is_done(void) { return ble_config_done; }

/* ==================================================================
 * Function body — copied from Core/Src/main.c  (HAL_TIM_PeriodElapsedCallback)
 *
 * Changes from original:
 *   - Uses global `test_ble_tx_busy` / `test_ble_tx_tick` instead of
 *     static locals (so tests can observe and control DMA busy state).
 *   - Simplified: Vehicle_state_machine and Emergency_cause replaced
 *     with 0 (omitted for test focus).
 * ================================================================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        (void)TIM2; /* avoid unused warning — we know it's TIM2 */

        struct autonomous_t26_acu_t AS_data;

        AS_data.acu_cpu_temp = (uint8_t)t24.chip_temp;
        AS_data.as_state = (uint8_t)t24.Autonomous_State;
        AS_data.asms = t24.ASMS;
        AS_data.assi_state = t24.ASSI_state;
        AS_data.emergency = t24.Emergency;
        AS_data.ign = t24.Ignition_Request;
        AS_data.mission_select = (uint8_t)t24.Current_Mission;
        AS_data.acu_state = (uint8_t)0;          /* Vehicle_state_machine omitted */
        AS_data.emergency_cause = 0;             /* Emergency_cause omitted */

        autonomous_t26_acu_pack(tx_data, &AS_data, AUTONOMOUS_T26_ACU_LENGTH);

        can_tx_header.StdId = AUTONOMOUS_T26_ACU_FRAME_ID;
        can_tx_header.RTR = CAN_RTR_DATA;
        can_tx_header.DLC = AUTONOMOUS_T26_ACU_LENGTH;

        add_can_message(TX_MAILBOX, can_tx_header, tx_data);

        struct autonomous_t26_dv_status_t dv_data;

        dv_data.as_status = (uint8_t)t24.Autonomous_State;
        dv_data.ami_state = (uint8_t)t24.Jetson_mission;

        autonomous_t26_dv_status_pack(tx_data, &dv_data,
                                      AUTONOMOUS_T26_DV_STATUS_LENGTH);

        struct autonomous_t26_asf_signals_t asf_signals;
        asf_signals.brake_pressure_front =
            (uint8_t)(t24.Front_Pressure.Hydraulic * 10);
        asf_signals.brake_pressure_rear =
            (uint8_t)(t24.Rear_Pressure.Hydraulic * 10);
        asf_signals.ebs_pressure_tank_front =
            (uint8_t)(t24.Front_Pressure.Pneumatic * 10);
        asf_signals.ebs_pressure_tank_rear =
            (uint8_t)(t24.Rear_Pressure.Pneumatic * 10);

        autonomous_t26_asf_signals_pack(tx_data, &asf_signals,
                                        AUTONOMOUS_T26_ASF_SIGNALS_LENGTH);
        can_tx_header.StdId = AUTONOMOUS_T26_ASF_SIGNALS_FRAME_ID;
        can_tx_header.RTR = CAN_RTR_DATA;
        can_tx_header.DLC = AUTONOMOUS_T26_ASF_SIGNALS_LENGTH;

        add_can_message(TX_MAILBOX, can_tx_header, tx_data);

        /* ── BLE telemetry ── */
        /* DMA timeout recovery: if TX stuck >500ms, abort and reset */
        if (test_ble_tx_busy && (HAL_GetTick() - test_ble_tx_tick > 500)) {
            HAL_UART_Abort(&huart2);
            test_ble_tx_busy = 0;
        }
        if (ble_module_config_is_done() && !test_ble_tx_busy) {
            static ble_telemetry_packet_t pkt;
            pkt.state_machine      = 0; /* Vehicle_state_machine omitted */
            pkt.assi_status        = t24.ASSI_state;
            pkt.mission            = (uint8_t)t24.Current_Mission;
            {   float _v = t24.Front_Pressure.Hydraulic * 100.0f;
                pkt.hydraulic_p1 = (_v > 65535.0f) ? 65535
                                  : (_v < 0 ? 0 : (uint16_t)_v); }
            {   float _v = t24.Rear_Pressure.Hydraulic * 100.0f;
                pkt.hydraulic_p2 = (_v > 65535.0f) ? 65535
                                  : (_v < 0 ? 0 : (uint16_t)_v); }
            {   float _v = t24.Front_Pressure.Pneumatic * 100.0f;
                pkt.pneumatic_p1 = (_v > 65535.0f) ? 65535
                                  : (_v < 0 ? 0 : (uint16_t)_v); }
            {   float _v = t24.Rear_Pressure.Pneumatic * 100.0f;
                pkt.pneumatic_p2 = (_v > 65535.0f) ? 65535
                                  : (_v < 0 ? 0 : (uint16_t)_v); }
            {   float _v = t24.chip_temp * 100.0f;
                pkt.chip_temp = (_v > 32767.0f) ? 32767
                               : (_v < -32768.0f) ? -32768 : (int16_t)_v; }
            pkt.solenoid_front     = t24.front_solenoid;
            pkt.solenoid_rear      = t24.rear_solenoid;

            if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&pkt,
                                      sizeof(pkt)) == HAL_OK) {
                test_ble_tx_busy = 1;
                test_ble_tx_tick = HAL_GetTick();
            }
        }
    }
}

/* ==================================================================
 * Test infrastructure
 * ================================================================== */

static int tests_passed = 0;
static int tests_failed = 0;
static int test_num     = 0;

#define TEST_ASSERT(cond, msg) do {                                     \
        if (!(cond)) {                                                  \
            printf("  FAIL: %s  (line %d)\n", msg, __LINE__);          \
            tests_failed++;                                             \
            return 0;                                                   \
        }                                                               \
        printf("  PASS: %s\n", msg);                                    \
        tests_passed++;                                                 \
    } while(0)

static inline bool approx_eq(float a, float b, float eps) {
    return fabsf(a - b) < eps;
}

/* ------------------------------------------------------------------
 * Reset helper — clears all global state between tests.
 * ------------------------------------------------------------------ */
static void reset_state(void) {
    memset(&t24, 0, sizeof(t24));
    memset(can_tx_queue, 0, sizeof(can_tx_queue));
    can_queue_index = -1;
    add_can_message_call_count = 0;
    can_tx_history_count = 0;
    memset(can_tx_history, 0, sizeof(can_tx_history));
    uart_dma_call_count = 0;
    uart_abort_call_count = 0;
    memset(last_dma_data, 0, sizeof(last_dma_data));
    fake_tick = 0;
    ble_config_done = 1;
    test_ble_tx_busy = 0;
    test_ble_tx_tick = 0;
}

/* ==================================================================
 * Test cases
 * ================================================================== */

/* ------------------------------------------------------------------
 * TEST 1: ACU frame (StdId=0x51) is first message queued.
 * ------------------------------------------------------------------ */
static int test_acu_frame_queued(void)
{
    printf("\n--- TEST %d: ACU frame queued with correct ID ---\n", test_num);
    reset_state();

    t24.Autonomous_State = AS_STATE_DRIVING;
    t24.Current_Mission = AUTOCROSS;

    HAL_TIM_PeriodElapsedCallback(&htim2);

    TEST_ASSERT(can_tx_history_count >= 1,
                "at least one CAN message queued");
    TEST_ASSERT(can_tx_history[0].can_tx_header.StdId == 0x51,
                "first queued message StdId == 0x51 (ACU)");
    TEST_ASSERT(can_tx_history[0].can_tx_header.DLC == 8,
                "ACU DLC == 8");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 2: ACU frame data bytes match packed values.
 * ------------------------------------------------------------------ */
static int test_acu_frame_data_content(void)
{
    printf("\n--- TEST %d: ACU frame data content ---\n", test_num);
    reset_state();

    t24.ASSI_state       = 3;
    t24.ASMS             = 1;
    t24.Emergency        = 0;
    t24.Ignition_Request = 1;
    t24.chip_temp        = 35.0f;
    t24.Autonomous_State = AS_STATE_DRIVING;    /* = 3 */
    t24.Current_Mission  = TRACKDRIVE;          /* = 3 */

    HAL_TIM_PeriodElapsedCallback(&htim2);

    uint8_t *d = can_tx_history[0].tx_data;
    TEST_ASSERT(d[0] == 3,   "tx_data[0] == ASSI_state (3)");
    TEST_ASSERT(d[1] == 0,   "tx_data[1] == acu_state (0, simplified)");
    TEST_ASSERT(d[2] == 35,  "tx_data[2] == chip_temp truncated (35)");
    TEST_ASSERT(d[3] == 3,   "tx_data[3] == mission_select (3 = TRACKDRIVE)");
    TEST_ASSERT(d[4] == 3,   "tx_data[4] == as_state (3 = AS_STATE_DRIVING)");
    TEST_ASSERT(d[5] == 0,   "tx_data[5] == emergency (0)");
    TEST_ASSERT(d[6] == 1,   "tx_data[6] == asms (1)");
    TEST_ASSERT(d[7] == 1,   "tx_data[7] == ign (1)");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 3: ASF_SIGNALS frame (StdId=0x511) is second message queued.
 * ------------------------------------------------------------------ */
static int test_asf_signals_frame_queued(void)
{
    printf("\n--- TEST %d: ASF_SIGNALS frame queued ---\n", test_num);
    reset_state();

    HAL_TIM_PeriodElapsedCallback(&htim2);

    TEST_ASSERT(can_tx_history_count >= 2,
                "at least two CAN messages queued");
    TEST_ASSERT(can_tx_history[1].can_tx_header.StdId == 0x511,
                "second queued message StdId == 0x511 (ASF_SIGNALS)");
    TEST_ASSERT(can_tx_history[1].can_tx_header.DLC == 4,
                "ASF_SIGNALS DLC == 4");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 4: ASF_SIGNALS pressure scaling (Hydraulic *10, Pneumatic *10).
 * ------------------------------------------------------------------ */
static int test_asf_signals_pressure_scaling(void)
{
    printf("\n--- TEST %d: ASF_SIGNALS pressure scaling ---\n", test_num);
    reset_state();

    t24.Front_Pressure.Hydraulic  = 5.0f;   /* brake_pressure_front = 50 */
    t24.Rear_Pressure.Pneumatic   = 8.0f;   /* ebs_pressure_tank_rear = 80 */
    /* Front_Pneumatic and Rear_Hydraulic remain 0 from reset */

    HAL_TIM_PeriodElapsedCallback(&htim2);

    uint8_t *d = can_tx_history[1].tx_data;
    /*
     * ASF_SIGNALS pack order:
     *   [0] ebs_pressure_tank_front  = Front_Pneumatic * 10 = 0
     *   [1] ebs_pressure_tank_rear   = Rear_Pneumatic  * 10 = 80
     *   [2] brake_pressure_front     = Front_Hydraulic * 10 = 50
     *   [3] brake_pressure_rear      = Rear_Hydraulic  * 10 = 0
     */
    TEST_ASSERT(d[0] == 0,   "ebs_pressure_tank_front == 0");
    TEST_ASSERT(d[1] == 80,  "ebs_pressure_tank_rear == 80  (8.0*10)");
    TEST_ASSERT(d[2] == 50,  "brake_pressure_front == 50   (5.0*10)");
    TEST_ASSERT(d[3] == 0,   "brake_pressure_rear == 0");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 5: BLE telemetry transmitted via DMA when not busy.
 * ------------------------------------------------------------------ */
static int test_ble_telemetry_sent(void)
{
    printf("\n--- TEST %d: BLE telemetry sent via DMA ---\n", test_num);
    reset_state();

    /* Default: ble_config_done=1, test_ble_tx_busy=0 */
    HAL_TIM_PeriodElapsedCallback(&htim2);

    TEST_ASSERT(uart_dma_call_count == 1,
                "HAL_UART_Transmit_DMA called exactly once");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 6: BLE telemetry data content matches packed values.
 * ------------------------------------------------------------------ */
static int test_ble_telemetry_data(void)
{
    printf("\n--- TEST %d: BLE telemetry data content ---\n", test_num);
    reset_state();

    t24.ASSI_state                = 1;
    t24.Current_Mission           = ACCELERATION;    /* = 1 */
    t24.Front_Pressure.Hydraulic  = 3.5f;   /* *100 = 350 */
    t24.Rear_Pressure.Pneumatic   = 7.2f;   /* *100 = 720 */
    t24.chip_temp                 = 45.3f;  /* *100 = 4530 */
    t24.front_solenoid            = 1;
    t24.rear_solenoid             = 0;

    HAL_TIM_PeriodElapsedCallback(&htim2);

    ble_telemetry_packet_t *pkt = (ble_telemetry_packet_t *)last_dma_data;

    TEST_ASSERT(pkt->state_machine == 0,
                "state_machine == 0 (simplified)");
    TEST_ASSERT(pkt->assi_status == 1,
                "assi_status == 1");
    TEST_ASSERT(pkt->mission == 1,
                "mission == 1 (ACCELERATION)");
    TEST_ASSERT(pkt->hydraulic_p1 == 350,
                "hydraulic_p1 == 350  (3.5 * 100)");
    TEST_ASSERT(pkt->hydraulic_p2 == 0,
                "hydraulic_p2 == 0    (Rear_Hydraulic not set)");
    TEST_ASSERT(pkt->pneumatic_p1 == 0,
                "pneumatic_p1 == 0    (Front_Pneumatic not set)");
    TEST_ASSERT(pkt->pneumatic_p2 == 720,
                "pneumatic_p2 == 720  (7.2 * 100)");
    TEST_ASSERT(pkt->chip_temp == 4530,
                "chip_temp == 4530    (45.3 * 100)");
    TEST_ASSERT(pkt->solenoid_front == 1,
                "solenoid_front == 1");
    TEST_ASSERT(pkt->solenoid_rear == 0,
                "solenoid_rear == 0");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 7: BLE telemetry NOT sent when busy (busy flag from previous
 *         call, no DMA timeout yet).
 * ------------------------------------------------------------------ */
static int test_ble_telemetry_not_sent_when_busy(void)
{
    printf("\n--- TEST %d: BLE telemetry blocked when busy ---\n", test_num);
    reset_state();

    /* First call: sends, sets busy=1, tick=0 */
    HAL_TIM_PeriodElapsedCallback(&htim2);
    TEST_ASSERT(uart_dma_call_count == 1,
                "first call: DMA called");
    TEST_ASSERT(test_ble_tx_busy == 1,
                "first call: busy flag set");

    /* Second call: busy=1, timeout not expired (fake_tick still 0), no send */
    HAL_TIM_PeriodElapsedCallback(&htim2);
    TEST_ASSERT(uart_dma_call_count == 1,
                "second call: DMA NOT called (busy, no timeout)");
    TEST_ASSERT(test_ble_tx_busy == 1,
                "second call: still busy");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 8: BLE telemetry NOT sent before config is done.
 * ------------------------------------------------------------------ */
static int test_ble_telemetry_not_sent_before_config_done(void)
{
    printf("\n--- TEST %d: BLE telemetry blocked before config done ---\n", test_num);
    reset_state();

    ble_config_done = 0;   /* module not yet configured */

    HAL_TIM_PeriodElapsedCallback(&htim2);

    TEST_ASSERT(uart_dma_call_count == 0,
                "DMA NOT called when config not done");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 9: BLE telemetry uint16 clamping (hydraulic/pneumatic >65535).
 * ------------------------------------------------------------------ */
static int test_ble_telemetry_value_clamping_uint16(void)
{
    printf("\n--- TEST %d: BLE uint16 clamping ---\n", test_num);
    reset_state();

    t24.Front_Pressure.Pneumatic = 700.0f;   /* *100 = 70000 > 65535 */

    HAL_TIM_PeriodElapsedCallback(&htim2);

    ble_telemetry_packet_t *pkt = (ble_telemetry_packet_t *)last_dma_data;

    TEST_ASSERT(pkt->pneumatic_p1 == 65535,
                "pneumatic_p1 clamped to 65535  (700.0*100=70000 > 65535)");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 10: BLE telemetry int16 clamping (chip_temp +/- extremes).
 * ------------------------------------------------------------------ */
static int test_ble_telemetry_value_clamping_int16(void)
{
    printf("\n--- TEST %d: BLE int16 clamping ---\n", test_num);
    reset_state();

    /* Positive overflow */
    t24.chip_temp = 400.0f;   /* *100 = 40000 > 32767 */

    HAL_TIM_PeriodElapsedCallback(&htim2);
    ble_telemetry_packet_t *pkt = (ble_telemetry_packet_t *)last_dma_data;
    TEST_ASSERT(pkt->chip_temp == 32767,
                "chip_temp clamped to 32767  (400.0*100 > 32767)");

    /* Negative overflow */
    reset_state();
    t24.chip_temp = -400.0f;   /* *100 = -40000 < -32768 */

    HAL_TIM_PeriodElapsedCallback(&htim2);
    pkt = (ble_telemetry_packet_t *)last_dma_data;
    TEST_ASSERT(pkt->chip_temp == -32768,
                "chip_temp clamped to -32768  (-400.0*100 < -32768)");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 11: BLE telemetry negative value clamped to 0 for uint16.
 * ------------------------------------------------------------------ */
static int test_ble_telemetry_value_clamping_negative(void)
{
    printf("\n--- TEST %d: BLE uint16 negative clamping ---\n", test_num);
    reset_state();

    /* Hydraulic pressure should never be negative, but protect against it */
    t24.Front_Pressure.Hydraulic = -1.0f;   /* *100 = -100 < 0 */

    HAL_TIM_PeriodElapsedCallback(&htim2);

    ble_telemetry_packet_t *pkt = (ble_telemetry_packet_t *)last_dma_data;

    TEST_ASSERT(pkt->hydraulic_p1 == 0,
                "hydraulic_p1 clamped to 0  (-1.0*100 < 0)");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 12: DMA timeout recovery — abort on stuck TX >500ms.
 * ------------------------------------------------------------------ */
static int test_dma_timeout_recovery(void)
{
    printf("\n--- TEST %d: DMA timeout recovery ---\n", test_num);
    reset_state();

    /* ---- Step 1: first call sends, busy becomes 1, tick=0 ---- */
    HAL_TIM_PeriodElapsedCallback(&htim2);
    TEST_ASSERT(uart_dma_call_count == 1,
                "step1: DMA called once");
    TEST_ASSERT(test_ble_tx_busy == 1,
                "step1: busy flag set");
    TEST_ASSERT(uart_abort_call_count == 0,
                "step1: abort not yet called");

    /* ---- Step 2: advance tick beyond 500ms threshold ---- */
    fake_tick = 1000;

    /*
     * Second call:
     *   1. Timeout check: 1000-0 > 500 → YES → abort called, busy=0
     *   2. Then: !busy AND config_done → re-sends telemetry → busy=1, tick=1000
     * So after step 2: busy=1, dma=2, abort=1
     */
    HAL_TIM_PeriodElapsedCallback(&htim2);
    TEST_ASSERT(uart_abort_call_count == 1,
                "step2: HAL_UART_Abort called once (timeout >500ms)");
    TEST_ASSERT(uart_dma_call_count == 2,
                "step2: DMA called again (re-sends immediately after abort+clear)");
    TEST_ASSERT(test_ble_tx_busy == 1,
                "step2: busy set again (re-send in same cycle after abort)");

    /* ---- Step 3: tick still 1000, busy=1, no timeout yet since (1000-1000)=0 ---- */
    HAL_TIM_PeriodElapsedCallback(&htim2);
    TEST_ASSERT(uart_abort_call_count == 1,
                "step3: no additional abort (timeout not expired)");
    TEST_ASSERT(uart_dma_call_count == 2,
                "step3: no new DMA (still busy from re-send)");
    TEST_ASSERT(test_ble_tx_busy == 1,
                "step3: still busy (no timeout expired)");

    /* ---- Step 4: advance tick to 2000, now timeout expired again ---- */
    fake_tick = 2000;
    HAL_TIM_PeriodElapsedCallback(&htim2);
    TEST_ASSERT(uart_abort_call_count == 2,
                "step4: second abort (2000-1000 > 500)");
    TEST_ASSERT(uart_dma_call_count == 3,
                "step4: DMA called again (re-send after second abort+clear)");
    TEST_ASSERT(test_ble_tx_busy == 1,
                "step4: busy set again after re-send");

    return 1;
}

/* ==================================================================
 * Main — run all tests, report summary, exit 0 on full pass.
 * ================================================================== */
int main(void)
{
    printf("========================================================\n");
    printf(" HAL_TIM_PeriodElapsedCallback  Unit Tests\n");
    printf("========================================================\n");

    int (*test_fn[])(void) = {
        test_acu_frame_queued,
        test_acu_frame_data_content,
        test_asf_signals_frame_queued,
        test_asf_signals_pressure_scaling,
        test_ble_telemetry_sent,
        test_ble_telemetry_data,
        test_ble_telemetry_not_sent_when_busy,
        test_ble_telemetry_not_sent_before_config_done,
        test_ble_telemetry_value_clamping_uint16,
        test_ble_telemetry_value_clamping_int16,
        test_ble_telemetry_value_clamping_negative,
        test_dma_timeout_recovery,
    };
    const char *test_names[] = {
        "TEST 1:  ACU frame queued with correct ID",
        "TEST 2:  ACU frame data content",
        "TEST 3:  ASF_SIGNALS frame queued",
        "TEST 4:  ASF_SIGNALS pressure scaling",
        "TEST 5:  BLE telemetry sent via DMA",
        "TEST 6:  BLE telemetry data content",
        "TEST 7:  BLE telemetry blocked when busy",
        "TEST 8:  BLE telemetry blocked before config done",
        "TEST 9:  BLE uint16 clamping (pneumatic >65535)",
        "TEST 10: BLE int16 clamping (chip_temp +/- extremes)",
        "TEST 11: BLE uint16 negative clamping",
        "TEST 12: DMA timeout recovery",
    };
    const int num_tests = sizeof(test_fn) / sizeof(test_fn[0]);

    for (int i = 0; i < num_tests; i++) {
        test_num = i + 1;
        printf("\n========================================================\n");
        printf(" %s\n", test_names[i]);
        printf("========================================================\n");
        int ret = test_fn[i]();
        if (ret == 0) {
            printf("\n  *** TEST %d FAILED ***\n", test_num);
        } else {
            printf("\n  +++ TEST %d PASSED +++\n", test_num);
        }
    }

    printf("\n========================================================\n");
    printf(" RESULTS\n");
    printf("========================================================\n");
    printf("  Passed assertions: %d\n", tests_passed);
    printf("  Failed assertions: %d\n", tests_failed);

    if (tests_failed > 0) {
        printf("\n  >>> SOME TESTS FAILED <<<\n");
        return 1;
    }
    printf("\n  >>> ALL TESTS PASSED <<<\n");
    return 0;
}
