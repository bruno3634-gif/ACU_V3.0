/*
 * test_ble_config.c
 *
 * Standalone host-based test harness for RN4871 BLE module configuration
 * state machine (ble_module_config_start / ble_module_config_tick /
 * ble_module_config_is_done).
 *
 * Compile with:
 *   gcc -o test_ble_config test_ble_config.c -Wall -Wextra -std=c11
 *
 * This file is self-contained — it provides HAL type stubs, copies the
 * function bodies verbatim from Core/Src/APP.c, and has no external
 * hardware dependencies.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ========================================================================
 *  HAL type stubs — matching the STM32 HAL API surface used by APP.c
 *  (no actual hardware headers)
 * ======================================================================== */

typedef uint32_t HAL_StatusTypeDef;
#define HAL_OK     0
#define HAL_ERROR  1

typedef struct { uint32_t dummy; } UART_HandleTypeDef;
UART_HandleTypeDef huart2;

/* ========================================================================
 *  Stub tracking for HAL_UART_Transmit and HAL_GetTick
 * ======================================================================== */

static uint32_t fake_tick = 0;
static int      transmit_call_count = 0;
static uint8_t  last_transmit_data[64] = {0};
static uint16_t last_transmit_size = 0;

uint32_t HAL_GetTick(void) { return fake_tick; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    uint8_t *pData, uint16_t Size,
                                    uint32_t Timeout)
{
    (void)huart; (void)Timeout;
    transmit_call_count++;
    last_transmit_size = Size;
    if (Size > 64) Size = 64;
    memcpy(last_transmit_data, pData, Size);
    return HAL_OK;
}

/* Reset all stubs between tests */
static void reset_stubs(void)
{
    fake_tick = 0;
    transmit_call_count = 0;
    memset(last_transmit_data, 0, sizeof(last_transmit_data));
    last_transmit_size = 0;
}

/* ========================================================================
 *  Function bodies — copied verbatim from Core/Src/APP.c
 *
 *  The functions are copied (not #included) because the original .c file
 *  depends on STM32 HAL and hardware headers unavailable on a host PC.
 * ======================================================================== */

/* RN4871 configuration commands — transparent serial bridge mode */
#define BLE_CFG_NUM_CMDS  3

static const char* ble_cfg_cmds[BLE_CFG_NUM_CMDS] = {
    "SN,ACU_V3\r\n",
    "SR,01\r\n",
    "R,1\r\n",
};

static uint8_t      ble_cfg_state = 0;
static uint8_t      ble_cfg_index = 0;
static uint32_t     ble_cfg_tick  = 0;

void ble_module_config_start(void)
{
    ble_cfg_state = 1;
    ble_cfg_index = 0;
    ble_cfg_tick  = HAL_GetTick();
}

void ble_module_config_tick(void)
{
    uint32_t now = HAL_GetTick();

    switch (ble_cfg_state) {

    case 0: /* idle */
    case 5: /* done */
        break;

    case 1: /* ENTER_CMD: send $$$ */
        HAL_UART_Transmit(&huart2, (uint8_t*)"$$$", 3, 10);
        ble_cfg_tick = now;
        ble_cfg_state = 2;
        break;

    case 2: /* WAIT_ENTER: wait 500ms after $$$ for CMD> prompt */
        if (now - ble_cfg_tick >= 500) {
            ble_cfg_index = 0;
            ble_cfg_tick = now;
            ble_cfg_state = 3;
        }
        break;

    case 3: /* SEND_CMD: send next configuration command */
        if (ble_cfg_index < BLE_CFG_NUM_CMDS) {
            HAL_UART_Transmit(&huart2,
                              (uint8_t*)ble_cfg_cmds[ble_cfg_index],
                              strlen(ble_cfg_cmds[ble_cfg_index]), 10);
            ble_cfg_tick = now;
            ble_cfg_state = 4;
        } else {
            ble_cfg_state = 5;   /* all done */
        }
        break;

    case 4: /* WAIT_CMD: wait 500ms between commands for AOK response */
        if (now - ble_cfg_tick >= 500) {
            ble_cfg_index++;
            ble_cfg_tick = now;
            ble_cfg_state = 3;
        }
        break;
    }
}

uint8_t ble_module_config_is_done(void)
{
    return (ble_cfg_state == 5);
}

/* ========================================================================
 *  Test utilities
 * ======================================================================== */

/* Custom assert macro — returns 0 (failure) from test function on failure */
#define TEST_ASSERT(cond, msg) do {                                    \
    if (!(cond)) {                                                     \
        printf("  FAIL: %s  [%s:%d]\n", msg, __FILE__, __LINE__);     \
        return 0;                                                      \
    }                                                                  \
    printf("  PASS: %s\n", msg);                                       \
} while (0)

/* ========================================================================
 *  TEST 1: Initial state is idle (not done)
 *
 *  Before calling anything, ble_module_config_is_done() returns 0 and
 *  the internal state is 0 (idle).
 * ======================================================================== */
static int test_initial_state(void)
{
    printf("=== TEST 1: Initial state is idle (not done) ===\n");

    /* Reset module-level static state to default (idle) */
    ble_cfg_state = 0;
    ble_cfg_index = 0;
    ble_cfg_tick  = 0;

    TEST_ASSERT(ble_module_config_is_done() == 0,
                "is_done returns 0 before any operation");
    TEST_ASSERT(ble_cfg_state == 0,
                "internal state is 0 (idle) on init");

    printf("  PASS: Initial state correctly idle\n");
    return 1;
}

/* ========================================================================
 *  TEST 2: Start transitions to state 1
 *
 *  After ble_module_config_start(), the state should be 1 (ENTER_CMD)
 *  and the tick should be captured.
 * ======================================================================== */
static int test_start_transitions(void)
{
    printf("=== TEST 2: Start transitions to state 1 ===\n");

    reset_stubs();
    ble_cfg_state = 0;
    ble_cfg_index = 0;
    ble_cfg_tick  = 0;
    fake_tick = 1000;

    ble_module_config_start();

    TEST_ASSERT(ble_cfg_state == 1,
                "state should be 1 (ENTER_CMD) after start");
    TEST_ASSERT(ble_cfg_index == 0,
                "index should be 0 after start");
    TEST_ASSERT(ble_cfg_tick == 1000,
                "tick should be captured from HAL_GetTick()");

    printf("  PASS: Start correctly transitions to ENTER_CMD state\n");
    return 1;
}

/* ========================================================================
 *  TEST 3: First tick sends "$$$" and transitions to state 2
 *
 *  When called in state 1 (ENTER_CMD), tick() must transmit the 3-byte
 *  "$$$" escape sequence and advance to state 2 (WAIT_ENTER).
 * ======================================================================== */
static int test_start_sends_dollar_dollar(void)
{
    printf("=== TEST 3: First tick sends \"$$$\" and transitions to state 2 ===\n");

    reset_stubs();
    ble_cfg_state = 0;
    ble_cfg_index = 0;
    ble_cfg_tick  = 0;

    /* Start the configuration process */
    ble_module_config_start();       /* state -> 1 */

    /* First tick — should send $$$ and go to state 2 */
    ble_module_config_tick();

    TEST_ASSERT(ble_cfg_state == 2,
                "state should be 2 (WAIT_ENTER) after first tick");
    TEST_ASSERT(transmit_call_count == 1,
                "exactly one UART transmit occurred");
    TEST_ASSERT(last_transmit_size == 3,
                "transmitted 3 bytes ($$$)");
    TEST_ASSERT(memcmp(last_transmit_data, "$$$", 3) == 0,
                "transmitted data is \"$$$\"");

    printf("  PASS: First tick sent \"$$$\" and advanced to WAIT_ENTER\n");
    return 1;
}

/* ========================================================================
 *  TEST 4: Wait in state 2 before 500ms elapses
 *
 *  While in state 2 (WAIT_ENTER), calling tick() before 500ms have passed
 *  must NOT trigger a transmit or advance the state.
 * ======================================================================== */
static int test_wait_enter_early(void)
{
    printf("=== TEST 4: Wait in state 2 before 500ms elapses ===\n");

    reset_stubs();
    ble_cfg_state = 0;
    ble_cfg_index = 0;
    ble_cfg_tick  = 0;
    fake_tick = 10000;

    /* Start and do the first tick to get to state 2 */
    ble_module_config_start();
    ble_module_config_tick();        /* state -> 2, tick -> 10000 */

    /* Advance time by only 100ms — not enough to exit WAIT_ENTER */
    fake_tick += 100;
    ble_module_config_tick();

    TEST_ASSERT(ble_cfg_state == 2,
                "state should still be 2 (WAIT_ENTER) before 500ms");
    TEST_ASSERT(transmit_call_count == 1,
                "no additional UART transmit occurred (still 1 from $$$)");

    /* Advance time by another 100ms — still only 200ms total */
    fake_tick += 100;
    ble_module_config_tick();

    TEST_ASSERT(ble_cfg_state == 2,
                "state should still be 2 at 200ms");

    printf("  PASS: State 2 correctly waits until 500ms elapses\n");
    return 1;
}

/* ========================================================================
 *  TEST 5: After 500ms in state 2, transition to state 3 with index 0
 *
 *  At exactly (or beyond) 500ms, tick() must transition from WAIT_ENTER
 *  to SEND_CMD (state 3) and reset index to 0.
 * ======================================================================== */
static int test_wait_enter_complete(void)
{
    printf("=== TEST 5: After 500ms in state 2, transition to state 3 ===\n");

    reset_stubs();
    ble_cfg_state = 0;
    ble_cfg_index = 99;              /* set non-zero to verify reset */
    ble_cfg_tick  = 0;
    fake_tick = 20000;

    /* Start and do the first tick to get to state 2 */
    ble_module_config_start();
    ble_module_config_tick();        /* state -> 2 */

    /* Advance time by exactly 500ms */
    fake_tick += 500;
    ble_module_config_tick();

    TEST_ASSERT(ble_cfg_state == 3,
                "state should be 3 (SEND_CMD) after 500ms wait");
    TEST_ASSERT(ble_cfg_index == 0,
                "index should be reset to 0 after WAIT_ENTER completes");

    printf("  PASS: Wait completed, transitioned to SEND_CMD with index=0\n");
    return 1;
}

/* ========================================================================
 *  TEST 6: In state 3, send the first configuration command
 *
 *  When in state 3 (SEND_CMD) with index=0, tick() must transmit the
 *  first command string (SN,ACU_V3) and transition to state 4 (WAIT_CMD).
 * ======================================================================== */
static int test_send_first_command(void)
{
    printf("=== TEST 6: Send first configuration command ===\n");

    reset_stubs();
    ble_cfg_state = 0;
    ble_cfg_index = 0;
    ble_cfg_tick  = 0;
    fake_tick = 30000;

    /* Start and drive to state 3 via start->tick->wait500->tick */
    ble_module_config_start();
    ble_module_config_tick();        /* state -> 2 */
    fake_tick += 500;
    ble_module_config_tick();        /* state -> 3, index=0 */

    /* Now in SEND_CMD — tick should send cmd[0] */
    ble_module_config_tick();

    const char *expected = ble_cfg_cmds[0];
    uint16_t    expected_len = (uint16_t)strlen(expected);

    TEST_ASSERT(ble_cfg_state == 4,
                "state should be 4 (WAIT_CMD) after sending command");
    TEST_ASSERT(transmit_call_count == 2,
                "two UART transmits total ($$$ + cmd0)");
    TEST_ASSERT(last_transmit_size == expected_len,
                "transmitted length matches first command");
    TEST_ASSERT(memcmp(last_transmit_data, expected, expected_len) == 0,
                "transmitted data matches first command string");

    printf("  PASS: First command \"%s\" sent, state -> WAIT_CMD\n",
           ble_cfg_cmds[0]);
    return 1;
}

/* ========================================================================
 *  TEST 7: Full configuration cycle — all 3 commands sent in order
 *
 *  Drive the full state machine (3 commands, done state = 5):
 *    start -> state1 -> tick -> state2 -> wait500 -> state3
 *    -> send_cmd0 -> state4 -> wait500 -> state3
 *    -> send_cmd1 -> state4 -> wait500 -> state3
 *    -> send_cmd2 -> state4 -> wait500 -> state3
 *    -> tick (index == BLE_CFG_NUM_CMDS, so state -> 5)
 *
 *  Total UART transmits = 1 ($$$) + 3 (commands) = 4.
 * ======================================================================== */
static int test_full_config_cycle(void)
{
    printf("=== TEST 7: Full configuration cycle (3 commands) ===\n");

    reset_stubs();
    ble_cfg_state = 0;
    ble_cfg_index = 0;
    ble_cfg_tick  = 0;
    fake_tick = 1000;

    /* ---- Phase 1: enter command mode ---- */
    ble_module_config_start();             /* state -> 1 */
    TEST_ASSERT(ble_cfg_state == 1, "after start -> state 1");

    ble_module_config_tick();              /* state 1: send $$$, state -> 2 */
    TEST_ASSERT(ble_cfg_state == 2, "after tick -> state 2 (WAIT_ENTER)");
    TEST_ASSERT(transmit_call_count == 1, "transmit #1 = $$$");

    /* ---- Phase 2: wait 500ms for CMD> prompt ---- */
    fake_tick += 500;
    ble_module_config_tick();              /* state 2: time done, state -> 3 */
    TEST_ASSERT(ble_cfg_state == 3, "after 500ms -> state 3 (SEND_CMD)");
    TEST_ASSERT(ble_cfg_index == 0, "index reset to 0");

    /* ---- Phase 3: send all 3 commands with 500ms between each ---- */
    for (int i = 0; i < BLE_CFG_NUM_CMDS; i++) {
        /* In state 3: send the current command */
        ble_module_config_tick();          /* state 3: send cmd[i], state -> 4 */
        TEST_ASSERT(ble_cfg_state == 4,
                    "state is 4 (WAIT_CMD) after sending command");

        /* Verify transmit count grows by one each command */
        {
            int expected_tx = 2 + i;  /* 1 ($$$) + 1 (cmd0) + i additional */
            if (transmit_call_count != expected_tx) {
                printf("  FAIL: transmit count expected %d, got %d  [%s:%d]\n",
                       expected_tx, transmit_call_count, __FILE__, __LINE__);
                return 0;
            }
            printf("  PASS: transmit count after cmd %d = %d\n",
                   i, transmit_call_count);
        }

        /* Verify the last transmitted data matches this command */
        const char *cmd = ble_cfg_cmds[i];
        uint16_t    cmd_len = (uint16_t)strlen(cmd);
        if (last_transmit_size != cmd_len) {
            printf("  FAIL: cmd %d length expected %d, got %d  [%s:%d]\n",
                   i, cmd_len, last_transmit_size, __FILE__, __LINE__);
            return 0;
        }
        printf("  PASS: cmd %d length = %d\n", i, cmd_len);

        if (memcmp(last_transmit_data, cmd, cmd_len) != 0) {
            printf("  FAIL: cmd %d data mismatch  [%s:%d]\n",
                   i, __FILE__, __LINE__);
            return 0;
        }
        printf("  PASS: cmd %d data matches expected string\n", i);

        /* If this is not the last command, wait 500ms to prepare next */
        if (i < BLE_CFG_NUM_CMDS - 1) {
            fake_tick += 500;
            ble_module_config_tick();      /* state 4: index++, state -> 3 */
            TEST_ASSERT(ble_cfg_state == 3,
                        "after wait -> state 3 for next command");

            if (ble_cfg_index != i + 1) {
                printf("  FAIL: expected index %d, got %d  [%s:%d]\n",
                       i + 1, ble_cfg_index, __FILE__, __LINE__);
                return 0;
            }
            printf("  PASS: index incremented to %d\n", i + 1);
        }
    }

    /* ---- Phase 4: after last command (cmd2), wait 500ms ---- */
    fake_tick += 500;
    ble_module_config_tick();              /* state 4 -> state 3 */
    TEST_ASSERT(ble_cfg_state == 3,
                "after last wait -> state 3 (SEND_CMD)");

    if (ble_cfg_index != BLE_CFG_NUM_CMDS) {
        printf("  FAIL: expected index %d, got %d  [%s:%d]\n",
               BLE_CFG_NUM_CMDS, ble_cfg_index, __FILE__, __LINE__);
        return 0;
    }
    printf("  PASS: index == BLE_CFG_NUM_CMDS (%d)\n", BLE_CFG_NUM_CMDS);

    /* One more tick — index >= BLE_CFG_NUM_CMDS, so state -> 5 */
    ble_module_config_tick();              /* state 3: all done, state -> 5 */
    TEST_ASSERT(ble_cfg_state == 5,
                "final state should be 5 (done)");

    /* Verify total transmits: 1 ($$$) + 3 (commands) = 4 */
    {
        int expected_total = 1 + BLE_CFG_NUM_CMDS;
        if (transmit_call_count != expected_total) {
            printf("  FAIL: total transmits expected %d, got %d  [%s:%d]\n",
                   expected_total, transmit_call_count, __FILE__, __LINE__);
            return 0;
        }
        printf("  PASS: total transmits = %d ($$$ + %d commands)\n",
               expected_total, BLE_CFG_NUM_CMDS);
    }

    printf("  PASS: Full cycle completed with %d transmits, state=%d\n",
           1 + BLE_CFG_NUM_CMDS, ble_cfg_state);
    return 1;
}

/* ========================================================================
 *  TEST 8: is_done returns true after full cycle completes
 *
 *  After all 3 commands have been sent and the state machine has reached
 *  state 5, ble_module_config_is_done() must return 1 (true).
 * ======================================================================== */
static int test_is_done_after_full_cycle(void)
{
    printf("=== TEST 8: is_done returns true after full cycle ===\n");

    reset_stubs();
    ble_cfg_state = 0;
    ble_cfg_index = 0;
    ble_cfg_tick  = 0;
    fake_tick = 50000;

    /* Drive the full cycle manually to completion */
    ble_module_config_start();
    ble_module_config_tick();              /* state -> 2 */
    fake_tick += 500;
    ble_module_config_tick();              /* state -> 3, index=0 */

    for (int i = 0; i < BLE_CFG_NUM_CMDS; i++) {
        ble_module_config_tick();          /* send cmd[i], state -> 4 */
        if (i < BLE_CFG_NUM_CMDS - 1) {
            fake_tick += 500;
            ble_module_config_tick();      /* state -> 3, index++ */
        }
    }

    /* Final wait + tick to push past all commands */
    fake_tick += 500;
    ble_module_config_tick();              /* state 4 -> 3 (index now == NUM) */
    ble_module_config_tick();              /* state 3 -> 5 (done) */

    TEST_ASSERT(ble_module_config_is_done() == 1,
                "is_done returns 1 after full cycle completes");
    TEST_ASSERT(ble_cfg_state == 5,
                "internal state is 5 (done)");

    printf("  PASS: is_done correctly reports configuration complete\n");
    return 1;
}

/* ========================================================================
 *  Main — run all tests, count pass/fail
 * ======================================================================== */
int main(void)
{
    int passed = 0;
    int failed = 0;

    printf("\n=== BLE Module Configuration Test Harness ===\n\n");

    if (test_initial_state())              passed++; else failed++;
    if (test_start_transitions())          passed++; else failed++;
    if (test_start_sends_dollar_dollar())  passed++; else failed++;
    if (test_wait_enter_early())           passed++; else failed++;
    if (test_wait_enter_complete())        passed++; else failed++;
    if (test_send_first_command())         passed++; else failed++;
    if (test_full_config_cycle())          passed++; else failed++;
    if (test_is_done_after_full_cycle())   passed++; else failed++;

    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);

    return failed > 0 ? 1 : 0;
}
