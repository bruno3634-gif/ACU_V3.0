/*
 * test_ble_handler.c
 *
 * Standalone host-based unit test for ble_handler.c:
 *   ble_handler_init(), ble_handler(), ble_send_binary(),
 *   HAL_UART_TxCpltCallback()
 *
 * Tests state machine transitions, command parsing, DMA timeout recovery,
 * busy flag, binary/text telemetry, log flush, and pause/resume.
 *
 * Compile:
 *   gcc -o test_ble_handler test_ble_handler.c -Wall -Wextra -std=c11 -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

/* ========================================================================
 *  Architecture stubs (__* primask helpers)
 * ======================================================================== */

static uint32_t irq_enabled = 1;
uint32_t __get_PRIMASK(void) { return irq_enabled ? 0 : 1; }
void     __disable_irq(void) { irq_enabled = 0; }
void     __set_PRIMASK(uint32_t v) { irq_enabled = (v == 0) ? 1 : 0; }

/* ========================================================================
 *  HAL type stubs
 * ======================================================================== */

typedef uint32_t HAL_StatusTypeDef;
#define HAL_OK      0
#define HAL_ERROR   1
#define HAL_BUSY    2

typedef struct { uint32_t Instance; } USART_TypeDef;
#define USART2 ((USART_TypeDef *)0x40004400)

typedef struct {
    USART_TypeDef *Instance;
    struct { uint32_t *cndtr; } hdmarx;
    uint32_t dummy;
} UART_HandleTypeDef;

static UART_HandleTypeDef huart2_stub = { .Instance = USART2 };

UART_HandleTypeDef huart2 = { .Instance = USART2 };

/* ========================================================================
 *  DMA stub — track buffer and counter for __HAL_DMA_GET_COUNTER
 * ======================================================================== */

static uint8_t  *dma_rx_buf    = NULL;
static uint16_t  dma_rx_len    = 0;
static uint16_t  dma_rx_written = 0;   /* how many bytes written to dma_rx_buf */

uint32_t __HAL_DMA_GET_COUNTER(uint32_t *cndtr)
{
    (void)cndtr;
    if (!dma_rx_buf) return 0;
    return dma_rx_len - dma_rx_written;
}

/* Helper: inject RX data into the circular buffer for the test */
static void inject_rx_data(const uint8_t *data, uint16_t len)
{
    memcpy(dma_rx_buf, data, len);
    dma_rx_written = len;
}

/* ========================================================================
 *  HAL UART stubs
 * ======================================================================== */

static int hal_uart_tx_dma_call_count = 0;
static int hal_uart_rx_dma_call_count = 0;
static int hal_uart_abort_tx_call_count = 0;
static HAL_StatusTypeDef hal_uart_tx_dma_result = HAL_OK;
static uint8_t *last_tx_buf = NULL;
static uint16_t last_tx_len = 0;

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *pData, uint16_t Size)
{
    (void)huart;
    hal_uart_tx_dma_call_count++;
    last_tx_buf = pData;
    last_tx_len = Size;
    return hal_uart_tx_dma_result;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart,
                                       uint8_t *pData, uint16_t Size)
{
    (void)huart;
    dma_rx_buf = pData;
    dma_rx_len = Size;
    dma_rx_written = 0;
    hal_uart_rx_dma_call_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
{
    (void)huart;
    hal_uart_abort_tx_call_count++;
    return HAL_OK;
}

/* ========================================================================
 *  Other stubs: ble_module_config_is_done
 * ======================================================================== */

static int config_done_result = 0;
int ble_module_config_is_done(void) { return config_done_result; }

/* ========================================================================
 *  Time stubs
 * ======================================================================== */

static uint32_t fake_time_ms = 0;
uint32_t millis(void)     { return fake_time_ms; }
uint32_t HAL_GetTick(void) { return fake_time_ms; }

/* ========================================================================
 *  Type definitions ble_handler.c depends on (from main.h)
 * ======================================================================== */

typedef enum {
    Start, IDLE, AS_ON, EMERGENCY
} Main_state_machine_t;

typedef enum {
    AS_STATE_OFF = 1, AS_STATE_READY = 2, AS_STATE_DRIVING = 3,
    AS_STATE_EMERGENCY = 4, AS_STATE_FINISHED = 5
} AS_STATE_t;

typedef enum {
    MANUAL, ACCELERATION, SKIDPAD, TRACKDRIVE, EBS_TEST,
    INSPECTION, AUTOCROSS
} current_mission_t;

typedef enum {
    BLE_IDLE, BLE_WAIT_CONFIG, BLE_BRIDGE
} BLE_STATE_MACHINE_t;

struct pressure { float Pneumatic; float Hydraulic; };
struct speed   { uint8_t Speed; uint8_t Target_Speed; };

struct car {
    AS_STATE_t      Autonomous_State;
    current_mission_t Current_Mission;
    current_mission_t Jetson_mission;
    struct pressure  Front_Pressure;
    struct pressure  Rear_Pressure;
    struct speed     Speed;
    uint8_t  front_solenoid;
    uint8_t  rear_solenoid;
    int      rpm;
    double   chip_temp;
    uint8_t  SDC_feedback;
    uint8_t  Ignition_Status;
    uint8_t  ASMS;
    uint8_t  ASSI_state;
    uint8_t  Emergency;
    uint8_t  Res;
};

static struct car t24;
Main_state_machine_t Vehicle_state_machine = Start;

/* ========================================================================
 *  ble_handler.c function bodies — copied verbatim
 * ======================================================================== */

/* Re-include the file content via #define trick — but we can't because
 * it has hardware #includes we don't have.  So we copy the relevant
 * function bodies below.
 *
 * Implementation note: We copy the full ble_handler.c content through
 * a section-based approach, with some defines pre-set.
 */

#define BLE_RX_BUF_SIZE  256
#define BLE_TX_BUF_SIZE  512

/* We'll define these static buffers ourselves (same as ble_handler.c) */
static uint8_t  dma_rx[BLE_RX_BUF_SIZE];
static uint16_t rx_head = 0;
static uint8_t  ble_tx_buf[BLE_TX_BUF_SIZE];

volatile uint8_t ble_tx_busy = 0;
static uint32_t  ble_tx_tick = 0;

static BLE_STATE_MACHINE_t ble_state     = BLE_IDLE;
static uint32_t            state_tick    = 0;

static uint8_t  logs_paused   = 0;
static uint8_t  flushing      = 0;
static uint8_t  flush_index   = 0;

#define MAX_EVENTS  20

/* ── Internal helpers ── */

static void ble_send_dma(const char *s) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ble_tx_busy) {
        __set_PRIMASK(primask);
        return;
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
    size_t len = strlen(s);
    if (len >= sizeof(ble_tx_buf)) {
        len = sizeof(ble_tx_buf) - 1;
    }
    memcpy(ble_tx_buf, s, len);
    ble_tx_buf[len] = '\0';
    if (HAL_UART_Transmit_DMA(&huart2, ble_tx_buf, len) != HAL_OK) {
        ble_tx_busy = 0;
    }
}

static void go_to(BLE_STATE_MACHINE_t s) {
    ble_state  = s;
    state_tick = millis();
}

static uint8_t timed_out(uint32_t limit_ms) {
    return (millis() - state_tick) > limit_ms;
}

static uint8_t rx_contains(const char *needle) {
    uint16_t write_head = BLE_RX_BUF_SIZE -
        __HAL_DMA_GET_COUNTER(huart2.hdmarx.cndtr);

    char tmp[BLE_RX_BUF_SIZE + 1];
    uint16_t n = 0, i = rx_head;

    while (i != write_head) {
        tmp[n++] = dma_rx[i];
        i = (i + 1) % BLE_RX_BUF_SIZE;
    }
    tmp[n] = '\0';

    if (strstr(tmp, needle)) {
        rx_head = write_head;
        return 1;
    }
    return 0;
}

static uint16_t rx_read(char *dst, uint16_t max) {
    uint16_t write_head = BLE_RX_BUF_SIZE -
        __HAL_DMA_GET_COUNTER(huart2.hdmarx.cndtr);

    uint16_t n = 0, i = rx_head;
    while (i != write_head && n < max - 1) {
        dst[n++] = dma_rx[i];
        i = (i + 1) % BLE_RX_BUF_SIZE;
    }
    dst[n] = '\0';
    rx_head = write_head;
    return n;
}

static const char* state_name(Main_state_machine_t s) {
    switch(s) {
        case Start: return "START";
        case IDLE:  return "IDLE";
        case AS_ON: return "AS_ON";
        case EMERGENCY: return "EMERGENCY";
        default: return "?";
    }
}

static const char* as_state_name(AS_STATE_t s) {
    switch(s) {
        case AS_STATE_OFF:       return "OFF";
        case AS_STATE_READY:     return "READY";
        case AS_STATE_DRIVING:   return "DRIVING";
        case AS_STATE_EMERGENCY: return "EMERGENCY";
        case AS_STATE_FINISHED:  return "FINISHED";
        default: return "?";
    }
}

static const char* mission_name(current_mission_t m) {
    switch(m) {
        case MANUAL:       return "MANUAL";
        case ACCELERATION: return "ACCELERATION";
        case SKIDPAD:      return "SKIDPAD";
        case TRACKDRIVE:   return "TRACKDRIVE";
        case EBS_TEST:     return "EBS_TEST";
        case INSPECTION:   return "INSPECTION";
        case AUTOCROSS:    return "AUTOCROSS";
        default: return "?";
    }
}

static const char* assi_state_name(uint8_t a) {
    switch(a) {
        case AS_STATE_OFF:       return "OFF";
        case AS_STATE_READY:     return "YELLOW";
        case AS_STATE_DRIVING:   return "YELLOW FLASH";
        case AS_STATE_EMERGENCY: return "BLUE FLASH";
        case AS_STATE_FINISHED:  return "BLUE";
        default: return "?";
    }
}

static void flush_one_record(uint8_t index) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ble_tx_busy) {
        __set_PRIMASK(primask);
        return;
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
    int len = snprintf((char*)ble_tx_buf, sizeof(ble_tx_buf),
                       "SLOT:%u (EEPROM not yet implemented)\r\n---\r\n",
                       index);
    if (HAL_UART_Transmit_DMA(&huart2, ble_tx_buf, len) != HAL_OK) {
        ble_tx_busy = 0;
    }
}

static void send_telemetry_log(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ble_tx_busy) {
        __set_PRIMASK(primask);
        return;
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
    int len = snprintf((char*)ble_tx_buf, sizeof(ble_tx_buf),
        "\r\n"
        "====================================\r\n"
        "      ACU V3.0 TELEMETRY\r\n"
        "====================================\r\n"
        "State: %s | AS: %s | Mission: %s\r\n"
        "Front:  Pneu=%.1f bar  Hyd=%.1f bar\r\n"
        "Rear:   Pneu=%.1f bar  Hyd=%.1f bar\r\n"
        "Sol:    Front=%s  Rear=%s\r\n"
        "Speed:  %u km/h  |  RPM: %d\r\n"
        "Temp:   %.1f C\r\n"
        "SDC:    %s  |  Ignition: %s\r\n"
        "ASMS:   %s  |  ASSI: %s\r\n"
        "Emergency: %s  |  Res: %u\r\n"
        "====================================\r\n",
        state_name(Vehicle_state_machine),
        as_state_name(t24.Autonomous_State),
        mission_name(t24.Current_Mission),
        t24.Front_Pressure.Pneumatic,
        t24.Front_Pressure.Hydraulic,
        t24.Rear_Pressure.Pneumatic,
        t24.Rear_Pressure.Hydraulic,
        t24.front_solenoid ? "ON " : "OFF",
        t24.rear_solenoid  ? "ON " : "OFF",
        (unsigned int)t24.Speed.Speed, t24.rpm,
        (double)t24.chip_temp,
        t24.SDC_feedback ? "OK" : "OPEN",
        t24.Ignition_Status ? "ON" : "OFF",
        t24.ASMS ? "ON" : "OFF",
        assi_state_name(t24.ASSI_state),
        t24.Emergency ? "ACTIVE" : "NONE",
        (unsigned int)t24.Res
    );
    if (HAL_UART_Transmit_DMA(&huart2, ble_tx_buf, len) != HAL_OK) {
        ble_tx_busy = 0;
    }
}

HAL_StatusTypeDef ble_send_binary(const uint8_t *data, uint16_t len) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ble_tx_busy) {
        __set_PRIMASK(primask);
        return HAL_BUSY;
    }
    ble_tx_busy = 1;
    ble_tx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data, len) != HAL_OK) {
        ble_tx_busy = 0;
        return HAL_ERROR;
    }
    return HAL_OK;
}

void ble_handler_init(void) {
    HAL_UART_Receive_DMA(&huart2, dma_rx, BLE_RX_BUF_SIZE);
    go_to(BLE_WAIT_CONFIG);
}

void ble_handler(void) {
    switch (ble_state) {

    case BLE_WAIT_CONFIG:
        if (ble_module_config_is_done()) {
            go_to(BLE_BRIDGE);
        }
        break;

    case BLE_BRIDGE: {
        if (ble_tx_busy && (HAL_GetTick() - ble_tx_tick > 500)) {
            HAL_UART_AbortTransmit(&huart2);
            ble_tx_busy = 0;
        }

        if (rx_contains("stop\r\n")) {
            logs_paused = 1;
        } else if (rx_contains("start\r\n")) {
            logs_paused = 0;
            flushing    = 0;
        } else if (rx_contains("flush\r\n")) {
            logs_paused  = 1;
            flushing     = 1;
            flush_index  = 0;
            ble_send_dma("=== EVENT LOG ===\r\n");
        }

        if (flushing) {
            if (flush_index < MAX_EVENTS) {
                flush_one_record(flush_index);
                flush_index++;
            } else {
                ble_send_dma("=== END LOG ===\r\n");
                flushing    = 0;
                logs_paused = 0;
            }
            break;
        }

        if (!logs_paused) {
            static uint32_t log_tick = 0;
            if (millis() - log_tick > 1000) {
                send_telemetry_log();
                log_tick = millis();
            }
        }
        break;
    }

    default:
        go_to(BLE_BRIDGE);
        break;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        ble_tx_busy = 0;
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

static void reset_ble_globals(void)
{
    memset(&t24, 0, sizeof(t24));
    Vehicle_state_machine = Start;
    memset(dma_rx, 0, sizeof(dma_rx));
    rx_head = 0;
    dma_rx_written = 0;
    ble_tx_busy = 0;
    ble_tx_tick = 0;
    ble_state = BLE_IDLE;
    state_tick = 0;
    logs_paused = 0;
    flushing = 0;
    flush_index = 0;
    fake_time_ms = 0;
    config_done_result = 0;
    hal_uart_tx_dma_call_count = 0;
    hal_uart_rx_dma_call_count = 0;
    hal_uart_abort_tx_call_count = 0;
    hal_uart_tx_dma_result = HAL_OK;
    irq_enabled = 1;
    /* Reset static inside ble_handler: log_tick */
    /* Force reset by re-initializing handler */
    ble_handler_init();
}

/* ========================================================================
 *  Test cases
 * ======================================================================== */

/* TEST 1: ble_handler_init starts RX DMA and goes to BLE_WAIT_CONFIG */
static int test_init_sets_state(void)
{
    printf("=== TEST 1: ble_handler_init ===\n");
    reset_ble_globals();

    TEST_ASSERT(hal_uart_rx_dma_call_count >= 1,
                "HAL_UART_Receive_DMA called on init");
    TEST_ASSERT(ble_state == BLE_WAIT_CONFIG,
                "state == BLE_WAIT_CONFIG after init");
    return 1;
}

/* TEST 2: BLE_WAIT_CONFIG -> BLE_BRIDGE after config done */
static int test_config_done_transition(void)
{
    printf("=== TEST 2: Config done -> BLE_BRIDGE ===\n");
    reset_ble_globals();

    /* Still in BLE_WAIT_CONFIG, config not done */
    ble_handler();
    TEST_ASSERT(ble_state == BLE_WAIT_CONFIG,
                "stays BLE_WAIT_CONFIG when config not done");

    /* Mark config done */
    config_done_result = 1;
    ble_handler();
    TEST_ASSERT(ble_state == BLE_BRIDGE,
                "transitions to BLE_BRIDGE when config done");
    return 1;
}

/* TEST 3: DMA timeout recovery aborts TX after >500ms */
static int test_dma_timeout_recovery(void)
{
    printf("=== TEST 3: DMA timeout recovery ===\n");
    reset_ble_globals();
    config_done_result = 1;
    ble_handler(); /* enter BLE_BRIDGE */

    ble_tx_busy = 1;
    ble_tx_tick = 100;
    fake_time_ms = 601; /* 501 ms later */

    ble_handler();

    TEST_ASSERT(hal_uart_abort_tx_call_count >= 1,
                "HAL_UART_AbortTransmit called on timeout");
    TEST_ASSERT(ble_tx_busy == 0,
                "ble_tx_busy cleared after timeout recovery");
    return 1;
}

/* TEST 4: Busy flag is not cleared before 500ms timeout */
static int test_dma_no_timeout_before_500ms(void)
{
    printf("=== TEST 4: No timeout before 500ms ===\n");
    reset_ble_globals();
    config_done_result = 1;
    ble_handler();

    ble_tx_busy = 1;
    ble_tx_tick = 100;
    fake_time_ms = 500; /* exactly 400ms — less than 500 */

    ble_handler();

    TEST_ASSERT(hal_uart_abort_tx_call_count == 0,
                "no abort before 500ms timeout");
    TEST_ASSERT(ble_tx_busy == 1,
                "busy still set");
    return 1;
}

/* TEST 5: "stop\r\n" command pauses logs */
static int test_stop_command(void)
{
    printf("=== TEST 5: stop command ===\n");
    reset_ble_globals();
    config_done_result = 1;
    ble_handler(); /* enter BLE_BRIDGE */

    logs_paused = 0;
    inject_rx_data((const uint8_t*)"stop\r\n", 7);

    ble_handler();

    TEST_ASSERT(logs_paused == 1,
                "logs_paused set after stop command");
    return 1;
}

/* TEST 6: "start\r\n" command resumes logs */
static int test_start_command(void)
{
    printf("=== TEST 6: start command ===\n");
    reset_ble_globals();
    config_done_result = 1;
    ble_handler();

    logs_paused = 1;
    flushing    = 1;
    inject_rx_data((const uint8_t*)"start\r\n", 8);

    ble_handler();

    TEST_ASSERT(logs_paused == 0,
                "logs_paused cleared after start command");
    TEST_ASSERT(flushing == 0,
                "flushing cleared after start command");
    return 1;
}

/* TEST 7: "flush\r\n" triggers log flush sequence */
static int test_flush_command(void)
{
    printf("=== TEST 7: flush command ===\n");
    reset_ble_globals();
    config_done_result = 1;
    ble_handler();

    inject_rx_data((const uint8_t*)"flush\r\n", 9);

    ble_handler();

    TEST_ASSERT(logs_paused == 1,
                "logs_paused set after flush");
    TEST_ASSERT(flushing == 1,
                "flushing set after flush");
    TEST_ASSERT(flush_index == 1,
                "flush_index advanced after first record");
    TEST_ASSERT(hal_uart_tx_dma_call_count >= 1,
                "DMA TX called for event log header");
    return 1;
}

/* TEST 8: Flush completes after MAX_EVENTS records */
static int test_flush_completes(void)
{
    printf("=== TEST 8: Flush completes ===\n");
    reset_ble_globals();
    config_done_result = 1;
    ble_handler();

    /* Start flush */
    inject_rx_data((const uint8_t*)"flush\r\n", 9);
    ble_handler();

    /* Clear injected data so rx_contains doesn't re-trigger */
    dma_rx_written = 0;
    memset(dma_rx, 0, sizeof(dma_rx));
    rx_head = 0;

    /* Drain all MAX_EVENTS records */
    for (int i = 0; i < MAX_EVENTS; i++) {
        /* Simulate TX complete between ticks */
        HAL_UART_TxCpltCallback(&huart2);
        fake_time_ms += 10;
        ble_handler();
    }

    /* After all records, the next call should send END LOG and exit flush */
    HAL_UART_TxCpltCallback(&huart2);
    ble_handler();

    TEST_ASSERT(flushing == 0,
                "flushing cleared after all records sent");
    TEST_ASSERT(logs_paused == 0,
                "logs_paused cleared after flush done");
    return 1;
}

/* TEST 9: ble_send_binary returns OK and sets busy */
static int test_send_binary_ok(void)
{
    printf("=== TEST 9: ble_send_binary success ===\n");
    reset_ble_globals();
    ble_tx_busy = 0;

    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};

    HAL_StatusTypeDef ret = ble_send_binary(data, 5);

    TEST_ASSERT(ret == HAL_OK,
                "ble_send_binary returns HAL_OK");
    TEST_ASSERT(ble_tx_busy == 1,
                "ble_tx_busy set");
    TEST_ASSERT(hal_uart_tx_dma_call_count >= 1,
                "DMA transmit called");
    return 1;
}

/* TEST 10: ble_send_binary returns BUSY when busy */
static int test_send_binary_busy(void)
{
    printf("=== TEST 10: ble_send_binary busy ===\n");
    reset_ble_globals();
    ble_tx_busy = 1;

    uint8_t data[] = {0x01, 0x02, 0x03};

    HAL_StatusTypeDef ret = ble_send_binary(data, 3);

    TEST_ASSERT(ret == HAL_BUSY,
                "ble_send_binary returns HAL_BUSY when busy");
    return 1;
}

/* TEST 11: HAL_UART_TxCpltCallback clears busy flag */
static int test_tx_cplt_callback(void)
{
    printf("=== TEST 11: TxCpltCallback clears busy ===\n");
    reset_ble_globals();
    ble_tx_busy = 1;

    HAL_UART_TxCpltCallback(&huart2);

    TEST_ASSERT(ble_tx_busy == 0,
                "ble_tx_busy cleared by callback");
    return 1;
}

/* TEST 12: Telemetry log sends when not paused and time > 1s */
static int test_telemetry_log_sends(void)
{
    printf("=== TEST 12: Telemetry log periodic send ===\n");
    reset_ble_globals();
    config_done_result = 1;
    ble_handler();

    logs_paused = 0;
    fake_time_ms = 1001; /* > 1s since last send (which was at time 0) */

    int count_before = hal_uart_tx_dma_call_count;
    ble_handler();

    TEST_ASSERT(hal_uart_tx_dma_call_count > count_before,
                "DMA TX called for telemetry log");
    return 1;
}

/* TEST 13: Telemetry log skipped when paused */
static int test_telemetry_log_paused(void)
{
    printf("=== TEST 13: Telemetry log skipped when paused ===\n");
    reset_ble_globals();
    config_done_result = 1;
    ble_handler();

    logs_paused = 1;
    fake_time_ms = 2000;

    int count_before = hal_uart_tx_dma_call_count;
    ble_handler();

    TEST_ASSERT(hal_uart_tx_dma_call_count == count_before,
                "no DMA TX when logs_paused");
    return 1;
}

/* ========================================================================
 *  Main
 * ======================================================================== */

int main(void)
{
    printf("========================================\n");
    printf(" ble_handler.c Unit Tests\n");
    printf("========================================\n");

    int (*test_fn[])(void) = {
        test_init_sets_state,
        test_config_done_transition,
        test_dma_timeout_recovery,
        test_dma_no_timeout_before_500ms,
        test_stop_command,
        test_start_command,
        test_flush_command,
        test_flush_completes,
        test_send_binary_ok,
        test_send_binary_busy,
        test_tx_cplt_callback,
        test_telemetry_log_sends,
        test_telemetry_log_paused,
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
