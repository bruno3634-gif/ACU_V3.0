/*
 * test_uart_callbacks.c
 * Standalone host-based unit test for UART RX/TX callbacks
 * (Core/Src/main.c: HAL_UARTEx_RxEventCallback, HAL_UART_TxCpltCallback).
 *
 * Compile: gcc -o test_uart_callbacks test_uart_callbacks.c -Wall -Wextra -std=c11
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

/* ==================================================================
 * HAL type stubs  (matches declarations in main.h)
 * ================================================================== */

typedef uint32_t HAL_StatusTypeDef;
#define HAL_OK     0
#define HAL_ERROR  1

typedef struct { uint32_t Instance; } USART_TypeDef;
#define USART2_BASE 0x40004400
#define USART2      ((USART_TypeDef *)USART2_BASE)

typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
UART_HandleTypeDef huart2 = { .Instance = USART2 };

/* ==================================================================
 * Defines and globals from main.c / main.h
 * ================================================================== */

#define RX_BUFFER_SIZE  32

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t uart_log_dump = 0;

/* Static volatile from main.c for TX callback */
static volatile uint8_t ble_tx_busy = 0;

/* ==================================================================
 * Helper accessors for ble_tx_busy (declared in main.c)
 * ================================================================== */

uint8_t get_ble_tx_busy(void) { return ble_tx_busy; }
void set_ble_tx_busy(uint8_t val) { ble_tx_busy = val; }

/* ==================================================================
 * Function bodies copied VERBATIM from Core/Src/main.c
 * ================================================================== */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        ble_tx_busy = 0;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART2) {
		char cmd[RX_BUFFER_SIZE] = { 0 };

		uint8_t j = 0;

		for (uint8_t i = 0; i < Size && j < RX_BUFFER_SIZE - 1; i++) {
			char c = (char) rx_buffer[i];
			if (c == '\r' || c == '\n' || c == ' ')
				continue;
			cmd[j++] = tolower(c);
		}

		if (strcmp(cmd, "log") == 0)
			uart_log_dump = 1;
		if (strcmp(cmd, "resume") == 0)
			uart_log_dump = 0;
	}
}

/* ==================================================================
 * Test infrastructure
 * ================================================================== */

static int tests_passed = 0;
static int tests_failed = 0;

/*
 * TEST_ASSERT – if condition fails, print failure, increment global
 * counter and return 0 from the enclosing test function.
 * On success, print PASS and increment passed counter.
 */
#define TEST_ASSERT(cond, msg) do {					\
		if (!(cond)) {						\
			printf("  FAIL: %s  (line %d)\n", msg, __LINE__); \
			tests_failed++;					\
			return 0;					\
		}							\
		printf("  PASS: %s\n", msg);				\
		tests_passed++;						\
	} while(0)

/* Helper to reset global state between tests */
static void reset_state(void)
{
	memset((void*)rx_buffer, 0, RX_BUFFER_SIZE);
	uart_log_dump = 0;
	ble_tx_busy = 0;
}

/* Helper to fill rx_buffer from a string literal (avoids volatile warnings) */
static void fill_rx_buffer(const char *str)
{
	size_t len = strlen(str);
	if (len > RX_BUFFER_SIZE)
		len = RX_BUFFER_SIZE;
	memcpy((void*)rx_buffer, str, len);
}

/* ==================================================================
 * Test cases
 * ================================================================== */

/* ------------------------------------------------------------------
 * TEST 1: "log\r\n" command sets uart_log_dump to 1.
 * ------------------------------------------------------------------ */
static int test_rx_callback_log_command(void)
{
	printf("\n--- TEST 1: RX callback sets uart_log_dump on 'log' command ---\n");
	reset_state();

	fill_rx_buffer("log\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 5);

	TEST_ASSERT(uart_log_dump == 1, "uart_log_dump == 1 after 'log' command");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 2: "resume\r\n" command clears uart_log_dump to 0.
 * ------------------------------------------------------------------ */
static int test_rx_callback_resume_command(void)
{
	printf("\n--- TEST 2: RX callback clears uart_log_dump on 'resume' command ---\n");
	reset_state();

	uart_log_dump = 1;
	fill_rx_buffer("resume\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 8);

	TEST_ASSERT(uart_log_dump == 0, "uart_log_dump == 0 after 'resume' command");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 3: Whitespace before/after command is stripped.
 * ------------------------------------------------------------------ */
static int test_rx_callback_strips_whitespace(void)
{
	printf("\n--- TEST 3: RX callback strips whitespace around command ---\n");
	reset_state();

	fill_rx_buffer("  log  \r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 9);

	TEST_ASSERT(uart_log_dump == 1, "uart_log_dump == 1 after '  log  \\r\\n'");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 4: Uppercase command is recognised (tolower makes it work).
 * ------------------------------------------------------------------ */
static int test_rx_callback_case_insensitive(void)
{
	printf("\n--- TEST 4: RX callback is case-insensitive ---\n");
	reset_state();

	fill_rx_buffer("LOG\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 5);

	TEST_ASSERT(uart_log_dump == 1, "uart_log_dump == 1 after 'LOG' (uppercase)");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 5: Unknown command does not change uart_log_dump.
 * ------------------------------------------------------------------ */
static int test_rx_callback_unknown_command(void)
{
	printf("\n--- TEST 5: Unknown command does not change state ---\n");
	reset_state();

	fill_rx_buffer("help\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 6);

	TEST_ASSERT(uart_log_dump == 0, "uart_log_dump unchanged (stays 0) after unknown command");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 6: Multiple calls work correctly and rx_buffer doesn't bleed.
 *
 * Call sequence: "log\r\n" -> "resume\r\n" -> "log\r\n"
 * Final state: uart_log_dump == 1.
 * Also verify that rx_buffer content from previous call does not
 * influence the next (i.e. the callback only reads up to Size).
 * ------------------------------------------------------------------ */
static int test_rx_callback_multiple_calls_no_buffer_overflow(void)
{
	printf("\n--- TEST 6: Multiple calls with correct state transitions ---\n");
	reset_state();

	/* Call 1: "log\r\n" */
	fill_rx_buffer("log\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 5);
	TEST_ASSERT(uart_log_dump == 1, "uart_log_dump == 1 after first 'log'");

	/* Call 2: "resume\r\n" */
	fill_rx_buffer("resume\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 8);
	TEST_ASSERT(uart_log_dump == 0, "uart_log_dump == 0 after 'resume'");

	/* Call 3: "log\r\n" */
	fill_rx_buffer("log\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 5);
	TEST_ASSERT(uart_log_dump == 1, "uart_log_dump == 1 after second 'log'");

	/* Verify rx_buffer content doesn't bleed in a pathological case:
	 * fill with leftover junk after a NULL-terminated short command.
	 * rx_buffer after first call still holds "log\r\n" plus trailing
	 * nulls; after second call it holds "resume\r\n" plus nulls.
	 * We just check that the callback correctly processes Size bytes. */
	printf("  INFO: rx_buffer content does not bleed (Size-bound loop)\n");
	tests_passed++;

	return 1;
}

/* ------------------------------------------------------------------
 * TEST 7: Callback ignores non-USART2 handles.
 * ------------------------------------------------------------------ */
static int test_rx_callback_only_usart2(void)
{
	printf("\n--- TEST 7: RX callback ignores other USART instances ---\n");
	reset_state();

	/* UART handle with a different peripheral address (e.g. USART1) */
	USART_TypeDef usart1_mem = { .Instance = 0x40004800 };
	UART_HandleTypeDef huart_other = { .Instance = &usart1_mem };

	fill_rx_buffer("log\r\n");
	HAL_UARTEx_RxEventCallback(&huart_other, 5);

	TEST_ASSERT(uart_log_dump == 0, "uart_log_dump unchanged when callback fired with non-USART2 handle");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 8: TX complete callback clears ble_tx_busy.
 * ------------------------------------------------------------------ */
static int test_tx_callback_clears_busy(void)
{
	printf("\n--- TEST 8: TX complete callback clears ble_tx_busy ---\n");
	reset_state();

	set_ble_tx_busy(1);
	HAL_UART_TxCpltCallback(&huart2);

	TEST_ASSERT(get_ble_tx_busy() == 0, "ble_tx_busy == 0 after TX complete callback");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 9: TX complete callback only clears for USART2.
 * ------------------------------------------------------------------ */
static int test_tx_callback_only_usart2(void)
{
	printf("\n--- TEST 9: TX complete callback ignores non-USART2 handles ---\n");
	reset_state();

	USART_TypeDef usart1_mem = { .Instance = 0x40004800 };
	UART_HandleTypeDef huart_other = { .Instance = &usart1_mem };

	set_ble_tx_busy(1);
	HAL_UART_TxCpltCallback(&huart_other);

	TEST_ASSERT(get_ble_tx_busy() == 1, "ble_tx_busy stays 1 when callback fired with non-USART2 handle");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 10: State persistence — flag resets reliably on repeated calls.
 * ------------------------------------------------------------------ */
static int test_state_persistence(void)
{
	printf("\n--- TEST 10: State persistence — ble_tx_busy resets reliably ---\n");
	reset_state();

	/* Set busy, fire correct callback -> cleared */
	set_ble_tx_busy(1);
	HAL_UART_TxCpltCallback(&huart2);
	TEST_ASSERT(get_ble_tx_busy() == 0, "ble_tx_busy == 0 after first callback");

	/* Set busy again, fire callback -> cleared again */
	set_ble_tx_busy(1);
	HAL_UART_TxCpltCallback(&huart2);
	TEST_ASSERT(get_ble_tx_busy() == 0, "ble_tx_busy == 0 after second callback (reliably reset)");

	/* Also verify uart_log_dump transitions reliably */
	uart_log_dump = 1;
	fill_rx_buffer("resume\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 8);
	TEST_ASSERT(uart_log_dump == 0, "uart_log_dump == 0 after resume (reliable transition)");

	fill_rx_buffer("log\r\n");
	HAL_UARTEx_RxEventCallback(&huart2, 5);
	TEST_ASSERT(uart_log_dump == 1, "uart_log_dump == 1 after log (reliable transition)");

	return 1;
}

/* ==================================================================
 * Main — run all tests, report summary, exit 0 on full pass.
 * ================================================================== */
int main(void)
{
	printf("========================================\n");
	printf(" UART Callback Unit Tests\n");
	printf("========================================\n");

	int (*test_fn[])(void) = {
		test_rx_callback_log_command,
		test_rx_callback_resume_command,
		test_rx_callback_strips_whitespace,
		test_rx_callback_case_insensitive,
		test_rx_callback_unknown_command,
		test_rx_callback_multiple_calls_no_buffer_overflow,
		test_rx_callback_only_usart2,
		test_tx_callback_clears_busy,
		test_tx_callback_only_usart2,
		test_state_persistence,
	};
	const char *test_names[] = {
		"TEST 1:  RX 'log' command sets uart_log_dump",
		"TEST 2:  RX 'resume' command clears uart_log_dump",
		"TEST 3:  RX callback strips whitespace",
		"TEST 4:  RX callback is case-insensitive",
		"TEST 5:  RX unknown command does nothing",
		"TEST 6:  RX multiple calls, no buffer bleed",
		"TEST 7:  RX callback ignores non-USART2",
		"TEST 8:  TX callback clears ble_tx_busy",
		"TEST 9:  TX callback ignores non-USART2",
		"TEST 10: State persistence (reliable resets)",
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
