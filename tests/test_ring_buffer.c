/*
 * test_ring_buffer.c
 * Standalone host-based unit test for CAN ring buffer (Core/Src/ring_buffer.c).
 *
 * Tests only the RX pop path + push path.  TX-pop path is compiled but
 * exercised only shallowly via stubs.
 *
 * Compile: gcc -o test_ring_buffer test_ring_buffer.c -Wall -Wextra -std=c11
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* ==================================================================
 * HAL / main.h type stubs  (matches declarations in main.h)
 * ================================================================== */

typedef uint32_t HAL_StatusTypeDef;
#define HAL_OK     0
#define HAL_ERROR  1

typedef struct { uint32_t StdId; uint32_t RTR; uint32_t DLC; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t StdId; uint32_t RTR; uint32_t DLC; } CAN_RxHeaderTypeDef;

/* Dummy CAN handle – only the address is passed to stubs, contents unused */
typedef struct { uint32_t dummy; } CAN_HandleTypeDef;

/* The extern reference from ring_buffer.h */
CAN_HandleTypeDef hcan1;

/* ==================================================================
 * HAL stub implementations
 * ================================================================== */

/*
 * HAL_GetTick stub – returns a fixed non-zero value so arrival_time
 * tests are meaningful.
 */
uint32_t HAL_GetTick(void)
{
	return 12345;
}

/*
 * HAL_CAN_GetTxMailboxesFreeLevel stub – always says at least one
 * mailbox is free (TX path compiles but is not deeply tested).
 */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)
{
	(void)hcan;
	return 1;
}

/*
 * HAL_CAN_AddTxMessage stub – claims success immediately so the TX
 * pop path advances the ring buffer (not deeply tested).
 */
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
				       CAN_TxHeaderTypeDef *pHeader,
				       uint8_t aData[],
				       uint32_t *pTxMailbox)
{
	(void)hcan;
	(void)pHeader;
	(void)aData;
	(void)pTxMailbox;
	return HAL_OK;
}

/* ==================================================================
 * Data structures  (from main.h / ring_buffer.h)
 * ================================================================== */

struct can_queue {
	uint32_t TX_MAILBOX;
	CAN_TxHeaderTypeDef can_tx_header;
	CAN_RxHeaderTypeDef can_rx_header;
	uint8_t tx_data[8];
	uint32_t arrival_time;
};

#define MAX_SIZE 1024

struct ring {
	uint32_t head;
	uint32_t tail;
	uint32_t counter;
	struct can_queue queue[MAX_SIZE];
};

/* ==================================================================
 * Function bodies copied VERBATIM from Core/Src/ring_buffer.c
 * ================================================================== */

void can_buffer_init(struct ring *ring_buffer)
{
	ring_buffer->counter = 0;
	ring_buffer->head = 0;
	ring_buffer->tail = 0;
}

void can_buffer_push(struct ring *ring_buffer, CAN_TxHeaderTypeDef  tx_header,
		uint8_t data[8])
{
	if (ring_buffer->counter >= MAX_SIZE) {
		return;
	}
	ring_buffer->queue[ring_buffer->head].can_tx_header = tx_header;
	memcpy(ring_buffer->queue[ring_buffer->head].tx_data, data, 8);

	ring_buffer->head = (ring_buffer->head + 1) % MAX_SIZE;
	ring_buffer->counter++;
}

void can_rx_buffer_push(struct ring *ring_buffer, CAN_RxHeaderTypeDef  tx_header,
		uint8_t data[8])
{
	if (ring_buffer->counter >= MAX_SIZE) {
		return;
	}
	ring_buffer->queue[ring_buffer->head].arrival_time = HAL_GetTick();
	ring_buffer->queue[ring_buffer->head].can_rx_header = tx_header;
	memcpy(ring_buffer->queue[ring_buffer->head].tx_data, data, 8);

	ring_buffer->head = (ring_buffer->head + 1) % MAX_SIZE;
	ring_buffer->counter++;
}

void can_buffer_pop(struct ring *ring_buffer, uint8_t tx_or_rx,struct can_queue *can_rx)
{
	if (ring_buffer->counter == 0) {
		return;
	}

	uint32_t mailbox;
	HAL_StatusTypeDef result = HAL_ERROR;

	if(tx_or_rx){
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
			result = HAL_CAN_AddTxMessage(&hcan1,
					&ring_buffer->queue[ring_buffer->tail].can_tx_header,
					ring_buffer->queue[ring_buffer->tail].tx_data, &mailbox);
		}
		if(result == HAL_OK){
			ring_buffer->tail = (ring_buffer->tail + 1) % MAX_SIZE;
			ring_buffer->counter--;
		}
	}else {
		memcpy(can_rx, &ring_buffer->queue[ring_buffer->tail], sizeof(ring_buffer->queue[ring_buffer->tail]));
		memset(&ring_buffer->queue[ring_buffer->tail], 0, sizeof(ring_buffer->queue[ring_buffer->tail]));
		ring_buffer->tail = (ring_buffer->tail + 1) % MAX_SIZE;
		ring_buffer->counter--;
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

/* ==================================================================
 * Test cases
 * ================================================================== */

/* ------------------------------------------------------------------
 * TEST 1: Init sets head, tail, counter to zero.
 * ------------------------------------------------------------------ */
static int test_init(void)
{
	printf("\n--- TEST 1: Init sets all to zero ---\n");
	struct ring buf;
	can_buffer_init(&buf);
	TEST_ASSERT(buf.head == 0,    "head == 0 after init");
	TEST_ASSERT(buf.tail == 0,    "tail == 0 after init");
	TEST_ASSERT(buf.counter == 0, "counter == 0 after init");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 2: Push increments counter.
 * ------------------------------------------------------------------ */
static int test_push_increments_counter(void)
{
	printf("\n--- TEST 2: Push increments counter ---\n");
	struct ring buf;
	can_buffer_init(&buf);

	CAN_TxHeaderTypeDef hdr = { .StdId = 0x123, .RTR = 0, .DLC = 8 };
	uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
	can_buffer_push(&buf, hdr, data);
	TEST_ASSERT(buf.counter == 1, "counter == 1 after one push");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 3: Push and RX-pop returns the same data.
 * ------------------------------------------------------------------ */
static int test_push_and_rx_pop(void)
{
	printf("\n--- TEST 3: Push and RX-pop returns same data ---\n");
	struct ring buf;
	can_buffer_init(&buf);

	/* Push a message with known content */
	uint32_t expected_stdid = 0x456;
	CAN_TxHeaderTypeDef hdr = { .StdId = expected_stdid, .RTR = 0, .DLC = 8 };
	uint8_t data[8] = {10, 20, 30, 40, 50, 60, 70, 80};
	can_buffer_push(&buf, hdr, data);

	/* Pop via RX path */
	struct can_queue output;
	memset(&output, 0, sizeof(output));
	can_buffer_pop(&buf, 0, &output);

	TEST_ASSERT(output.can_tx_header.StdId == expected_stdid,
		    "StdId matches after RX pop");
	TEST_ASSERT(output.tx_data[0] == 10, "data[0] == 10");
	TEST_ASSERT(output.tx_data[1] == 20, "data[1] == 20");
	TEST_ASSERT(output.tx_data[2] == 30, "data[2] == 30");
	TEST_ASSERT(output.tx_data[3] == 40, "data[3] == 40");
	TEST_ASSERT(output.tx_data[4] == 50, "data[4] == 50");
	TEST_ASSERT(output.tx_data[5] == 60, "data[5] == 60");
	TEST_ASSERT(output.tx_data[6] == 70, "data[6] == 70");
	TEST_ASSERT(output.tx_data[7] == 80, "data[7] == 80");
	TEST_ASSERT(buf.counter == 0, "counter back to 0 after pop");
	TEST_ASSERT(buf.head == buf.tail, "head == tail when empty");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 4: Multiple pushes without pop.
 * ------------------------------------------------------------------ */
static int test_multiple_pushes(void)
{
	printf("\n--- TEST 4: Multiple pushes without pop ---\n");
	struct ring buf;
	can_buffer_init(&buf);

	for (int i = 0; i < 5; i++) {
		CAN_TxHeaderTypeDef hdr = { .StdId = (uint32_t)i, .RTR = 0, .DLC = 8 };
		uint8_t data[8] = {0};
		can_buffer_push(&buf, hdr, data);
	}
	TEST_ASSERT(buf.counter == 5, "counter == 5 after 5 pushes");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 5: Full buffer drops excess data.
 * ------------------------------------------------------------------ */
static int test_full_buffer_drops(void)
{
	printf("\n--- TEST 5: Full buffer drops data ---\n");
	struct ring buf;
	can_buffer_init(&buf);

	for (int i = 0; i < MAX_SIZE + 10; i++) {
		CAN_TxHeaderTypeDef hdr = { .StdId = (uint32_t)i, .RTR = 0, .DLC = 8 };
		uint8_t data[8] = {0};
		can_buffer_push(&buf, hdr, data);
	}
	TEST_ASSERT(buf.counter == MAX_SIZE,
		    "counter == MAX_SIZE after overflow pushes");
	TEST_ASSERT(buf.head == buf.tail,
		    "head == tail when full (wrapped around)");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 6: Empty buffer pop does nothing.
 * ------------------------------------------------------------------ */
static int test_empty_pop(void)
{
	printf("\n--- TEST 6: Empty buffer pop does nothing ---\n");
	struct ring buf;
	can_buffer_init(&buf);

	struct can_queue output;
	memset(&output, 0xAA, sizeof(output));

	/* Pop from empty buffer (RX path) */
	can_buffer_pop(&buf, 0, &output);
	TEST_ASSERT(buf.counter == 0, "counter still 0 after pop on empty");

	/* Pop from empty buffer (TX path) – should also be a no-op */
	memset(&output, 0xBB, sizeof(output));
	can_buffer_pop(&buf, 1, &output);
	TEST_ASSERT(buf.counter == 0, "counter still 0 after TX-pop on empty");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 7: Wrap-around behavior.
 *
 * Fill the buffer, drain half, push a few more, then verify the
 * oldest remaining message is the one we expect (correct tail index).
 * ------------------------------------------------------------------ */
static int test_wrap_around(void)
{
	printf("\n--- TEST 7: Wrap-around behavior ---\n");
	struct ring buf;
	can_buffer_init(&buf);

	/* 1) Fill buffer completely */
	for (int i = 0; i < MAX_SIZE; i++) {
		CAN_TxHeaderTypeDef hdr = { .StdId = (uint32_t)i,
					    .RTR  = 0,
					    .DLC  = 8 };
		uint8_t data[8] = { (uint8_t)i, (uint8_t)(i+1),
				    (uint8_t)(i+2), (uint8_t)(i+3),
				    (uint8_t)(i+4), (uint8_t)(i+5),
				    (uint8_t)(i+6), (uint8_t)(i+7) };
		can_buffer_push(&buf, hdr, data);
	}
	TEST_ASSERT(buf.counter == MAX_SIZE,
		    "counter == MAX_SIZE after fill");
	TEST_ASSERT(buf.head == 0,
		    "head wrapped to 0 after fill");

	/* 2) Drain half via RX pop */
	for (int i = 0; i < MAX_SIZE / 2; i++) {
		struct can_queue out;
		can_buffer_pop(&buf, 0, &out);
		/* Each popped message should carry its index as StdId */
		if (out.can_tx_header.StdId != (uint32_t)i) {
			printf("  FAIL: pop %d returned StdId %lu, expected %u (line %d)\n",
			       i, (unsigned long)out.can_tx_header.StdId, i, __LINE__);
			tests_failed++;
			return 0;
		}
		tests_passed++;
		printf("  PASS: pop %d StdId == %d\n", i, i);
	}
	TEST_ASSERT(buf.tail == MAX_SIZE / 2,
		    "tail advanced to MAX_SIZE/2 after draining half");
	TEST_ASSERT(buf.counter == MAX_SIZE / 2,
		    "counter == MAX_SIZE/2 after draining half");

	/* 3) Push 3 more messages — they overwrite the cleared slots
	 *    at the beginning of the array. */
	for (int i = 0; i < 3; i++) {
		CAN_TxHeaderTypeDef hdr = { .StdId = (uint32_t)(1000 + i),
					    .RTR  = 0,
					    .DLC  = 8 };
		uint8_t data[8] = {0};
		can_buffer_push(&buf, hdr, data);
	}
	/* head should now be 3 (three slots reclaimed) */
	TEST_ASSERT(buf.head == 3,
		    "head == 3 after pushing 3 more into freed slots");

	/* 4) Verify the oldest remaining message — it should be at
	 *    tail = MAX_SIZE/2, with StdId = MAX_SIZE/2 (the first
	 *    message that was NOT popped). */
	struct can_queue oldest;
	memset(&oldest, 0, sizeof(oldest));
	can_buffer_pop(&buf, 0, &oldest);
	TEST_ASSERT(oldest.can_tx_header.StdId == MAX_SIZE / 2,
		    "oldest remaining message has StdId == MAX_SIZE/2");
	/* Verify data bytes also match what we originally pushed */
	TEST_ASSERT(oldest.tx_data[0] == (uint8_t)(MAX_SIZE / 2),
		    "oldest data[0] matches original push");
	TEST_ASSERT(oldest.tx_data[1] == (uint8_t)(MAX_SIZE / 2 + 1),
		    "oldest data[1] matches original push");
	TEST_ASSERT(oldest.tx_data[7] == (uint8_t)(MAX_SIZE / 2 + 7),
		    "oldest data[7] matches original push");

	/* After that pop: tail should be MAX_SIZE/2 + 1,
	 * counter should be (MAX_SIZE/2) - 1 + 3 = MAX_SIZE/2 + 2
	 * (drained half = MAX_SIZE/2, pushed 3 more, popped 1 more). */
	TEST_ASSERT(buf.tail == MAX_SIZE / 2 + 1,
		    "tail == MAX_SIZE/2 + 1 after popping oldest");
	TEST_ASSERT(buf.counter == MAX_SIZE / 2 + 2,
		    "counter == MAX_SIZE/2 + 2 after draining, pushing 3, popping 1");
	return 1;
}

/* ------------------------------------------------------------------
 * TEST 8: RX push stores arrival_time.
 * ------------------------------------------------------------------ */
static int test_rx_push_arrival_time(void)
{
	printf("\n--- TEST 8: RX push stores arrival_time ---\n");
	struct ring buf;
	can_buffer_init(&buf);

	CAN_RxHeaderTypeDef rx_hdr = { .StdId = 0x789, .RTR = 0, .DLC = 8 };
	uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	can_rx_buffer_push(&buf, rx_hdr, data);

	TEST_ASSERT(buf.counter == 1, "counter == 1 after rx push");
	TEST_ASSERT(buf.queue[buf.head == 0 ? MAX_SIZE - 1 : buf.head - 1].arrival_time == 12345,
		    "arrival_time == HAL_GetTick() stub value");

	/* Also verify via pop */
	struct can_queue output;
	memset(&output, 0, sizeof(output));
	can_buffer_pop(&buf, 0, &output);
	TEST_ASSERT(output.arrival_time == 12345,
		    "popped arrival_time == 12345 (HAL_GetTick stub)");
	TEST_ASSERT(output.can_rx_header.StdId == 0x789,
		    "RxHeader.StdId preserved through pop");
	TEST_ASSERT(output.tx_data[0] == 1, "data[0] preserved");
	TEST_ASSERT(output.tx_data[7] == 8, "data[7] preserved");
	return 1;
}

/* ==================================================================
 * Main — run all tests, report summary, exit 0 on full pass.
 * ================================================================== */
int main(void)
{
	printf("========================================\n");
	printf(" CAN Ring Buffer Unit Tests\n");
	printf("========================================\n");

	int (*test_fn[])(void) = {
		test_init,
		test_push_increments_counter,
		test_push_and_rx_pop,
		test_multiple_pushes,
		test_full_buffer_drops,
		test_empty_pop,
		test_wrap_around,
		test_rx_push_arrival_time,
	};
	const char *test_names[] = {
		"TEST 1: Init sets all to zero",
		"TEST 2: Push increments counter",
		"TEST 3: Push and RX-pop returns same data",
		"TEST 4: Multiple pushes without pop",
		"TEST 5: Full buffer drops data",
		"TEST 6: Empty buffer pop does nothing",
		"TEST 7: Wrap-around behavior",
		"TEST 8: RX push stores arrival_time",
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
