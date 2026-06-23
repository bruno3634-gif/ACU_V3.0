/*
 * test_can_queue.c
 * Standalone host-based unit test for software CAN TX queue
 * (Core/Src/hardware_abstaction.c: add_can_message / handle_can_tx).
 *
 * Tests the array-based transmit queue (not ring buffer).  Verifies
 * enqueue, dequeue, drain-on-reset, no-op on empty/mailbox-full, and
 * data integrity through the HAL stub layer.
 *
 * Compile: gcc -o test_can_queue test_can_queue.c -Wall -Wextra -std=c11
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

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

/* The extern reference from hardware_abstraction.h  (main.c defines it) */
CAN_HandleTypeDef hcan1;

/* ==================================================================
 * CAN queue struct  (from main.h)
 * ================================================================== */

struct can_queue {
    uint32_t TX_MAILBOX;
    CAN_TxHeaderTypeDef can_tx_header;
    CAN_RxHeaderTypeDef can_rx_header;
    uint8_t tx_data[8];
    uint32_t arrival_time;          /* not used in TX path, but present */
};

/* ==================================================================
 * HAL stub implementations  (with test-controllable state)
 * ================================================================== */

/* Track remaining free mailboxes – tests can set this */
static uint32_t fake_free_mailboxes = 3;   /* max 3 on STM32F4 */
static int add_tx_message_call_count = 0;
static struct can_queue last_added_message; /* stores last msg passed to HAL_CAN_AddTxMessage */

uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    return fake_free_mailboxes;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
                                       CAN_TxHeaderTypeDef *pHeader,
                                       uint8_t aData[],
                                       uint32_t *pTxMailbox)
{
    (void)hcan;
    add_tx_message_call_count++;
    last_added_message.can_tx_header = *pHeader;
    memcpy(last_added_message.tx_data, aData, 8);
    return HAL_OK;
}

static void reset_stubs(void)
{
    fake_free_mailboxes = 3;
    add_tx_message_call_count = 0;
    memset(&last_added_message, 0, sizeof(last_added_message));
}

/* ==================================================================
 * Global variables  (from main.c)  – used by the functions under test
 * ================================================================== */

struct can_queue can_tx_queue[64];
int can_queue_index = -1;

/* These are passed to add_can_message but we instantiate them locally
 * in each test; the main.c globals of the same name are unused here. */
uint32_t TX_MAILBOX = 0;
CAN_TxHeaderTypeDef can_tx_header;
uint8_t tx_data[8];

/* ==================================================================
 * Function bodies copied VERBATIM from Core/Src/hardware_abstaction.c
 * ================================================================== */

void add_can_message(uint32_t mailbox, CAN_TxHeaderTypeDef tx_header,
        uint8_t tx_data[8]) {
    if (can_queue_index >= 63) {
        return;  /* queue full, drop message */
    }
    can_queue_index++;
    can_tx_queue[can_queue_index].TX_MAILBOX = mailbox;
    can_tx_queue[can_queue_index].can_tx_header = tx_header;
    memcpy(can_tx_queue[can_queue_index].tx_data, tx_data, 8);
}

void handle_can_tx(void) {
    static uint8_t tx_index = 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && can_queue_index > -1) {
        HAL_CAN_AddTxMessage(&hcan1, &can_tx_queue[tx_index].can_tx_header,
                can_tx_queue[tx_index].tx_data,
                &can_tx_queue[tx_index].TX_MAILBOX);
        tx_index++;
        if (can_queue_index == (tx_index - 1)) {
            can_queue_index = -1;
            tx_index = 0;
        }
    }
}

/* ==================================================================
 * Test helpers
 * ================================================================== */

/*
 * reset_queue – prepare a clean state before each test.
 *
 * Resets the global queue index, clears queue memory, and resets the
 * HAL stubs.  Also forces handle_can_tx()'s internal static tx_index
 * back to zero by engineering a full drain cycle: one dummy message
 * is enqueued and then handle_can_tx() is called up to 256 times
 * (worst case for uint8_t wrap) until the queue resets.
 *
 * This works regardless of the prior value of tx_index because every
 * call to handle_can_tx() increments tx_index, and the reset
 * condition (can_queue_index == tx_index - 1) with index == 0 is
 * guaranteed to fire when tx_index wraps to 1.
 */
static void reset_queue(void)
{
    can_queue_index = -1;
    memset(can_tx_queue, 0, sizeof(can_tx_queue));
    reset_stubs();

    /* Force tx_index back to 0 by creating a guaranteed drain cycle.
     * Add one dummy message at index 0, then keep calling
     * handle_can_tx() until the condition resets the queue.
     */
    {
        CAN_TxHeaderTypeDef hdr = { .StdId = 0, .RTR = 0, .DLC = 8 };
        uint8_t data[8] = {0};
        can_queue_index = 0;
        can_tx_queue[0].TX_MAILBOX = 0;
        can_tx_queue[0].can_tx_header = hdr;
        memcpy(can_tx_queue[0].tx_data, data, 8);
        fake_free_mailboxes = 1;

        /* Call repeatedly.  Every call increments tx_index (uint8_t).
         * Reset fires when can_queue_index (0) == (tx_index - 1),
         * i.e. when tx_index == 1.  If tx_index was already 0 the
         * first call drives it to 1 and the condition matches.
         * If tx_index was N > 0 we keep incrementing until uint8_t
         * wraps back to 1 (at most 256 iterations).
         */
        for (int i = 0; i < 256; i++) {
            handle_can_tx();
            if (can_queue_index == -1) {
                break;
            }
        }
    }

    /* Finalise: can_queue_index must be -1, tx_index is 0 */
    can_queue_index = -1;
    memset(can_tx_queue, 0, sizeof(can_tx_queue));
    reset_stubs();
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
#define TEST_ASSERT(cond, msg) do {                                     \
        if (!(cond)) {                                                  \
            printf("  FAIL: %s  (line %d)\n", msg, __LINE__);           \
            tests_failed++;                                             \
            return 0;                                                   \
        }                                                               \
        printf("  PASS: %s\n", msg);                                    \
        tests_passed++;                                                 \
    } while(0)

/* ==================================================================
 * Test cases
 * ================================================================== */

/* ------------------------------------------------------------------
 * TEST 1: Empty queue – handle_can_tx() does nothing.
 * ------------------------------------------------------------------ */
static int test_empty_queue(void)
{
    printf("\n--- TEST 1: Empty queue does nothing ---\n");
    reset_queue();

    /* can_queue_index is already -1 from reset_queue */
    TEST_ASSERT(can_queue_index == -1, "can_queue_index == -1 initially");

    /* Call handle_can_tx – should be a no-op */
    handle_can_tx();

    TEST_ASSERT(add_tx_message_call_count == 0,
                "HAL_CAN_AddTxMessage NOT called on empty queue");
    TEST_ASSERT(can_queue_index == -1,
                "can_queue_index still -1 after no-op");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 2: Add one message – verify queue state.
 * ------------------------------------------------------------------ */
static int test_add_one_message(void)
{
    printf("\n--- TEST 2: Add one message ---\n");
    reset_queue();

    CAN_TxHeaderTypeDef hdr = { .StdId = 0x123, .RTR = 0, .DLC = 8 };
    uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};

    add_can_message(1, hdr, data);

    TEST_ASSERT(can_queue_index == 0, "can_queue_index == 0 after first add");
    TEST_ASSERT(can_tx_queue[0].TX_MAILBOX == 1, "mailbox stored correctly");
    TEST_ASSERT(can_tx_queue[0].can_tx_header.StdId == 0x123,
                "StdId stored correctly");
    TEST_ASSERT(can_tx_queue[0].can_tx_header.DLC == 8,
                "DLC stored correctly");
    TEST_ASSERT(can_tx_queue[0].tx_data[0] == 0xAA, "data[0] == 0xAA");
    TEST_ASSERT(can_tx_queue[0].tx_data[7] == 0x22, "data[7] == 0x22");

    /* Cleanup: drain the one message */
    fake_free_mailboxes = 3;
    handle_can_tx();
    TEST_ASSERT(add_tx_message_call_count == 1, "drain sent 1 message");
    TEST_ASSERT(can_queue_index == -1, "queue empty after drain");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 3: Add two messages – both stored correctly.
 * ------------------------------------------------------------------ */
static int test_add_two_messages(void)
{
    printf("\n--- TEST 3: Add two messages ---\n");
    reset_queue();

    CAN_TxHeaderTypeDef hdr1 = { .StdId = 0x100, .RTR = 0, .DLC = 8 };
    uint8_t data1[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    CAN_TxHeaderTypeDef hdr2 = { .StdId = 0x200, .RTR = 0, .DLC = 8 };
    uint8_t data2[8] = {9, 10, 11, 12, 13, 14, 15, 16};

    add_can_message(0, hdr1, data1);
    add_can_message(1, hdr2, data2);

    TEST_ASSERT(can_queue_index == 1, "can_queue_index == 1 after two adds");
    TEST_ASSERT(can_tx_queue[0].can_tx_header.StdId == 0x100,
                "msg0 StdId == 0x100");
    TEST_ASSERT(can_tx_queue[1].can_tx_header.StdId == 0x200,
                "msg1 StdId == 0x200");
    TEST_ASSERT(can_tx_queue[0].tx_data[0] == 1, "msg0 data[0] == 1");
    TEST_ASSERT(can_tx_queue[1].tx_data[0] == 9, "msg1 data[0] == 9");
    TEST_ASSERT(can_tx_queue[0].TX_MAILBOX == 0, "msg0 mailbox == 0");
    TEST_ASSERT(can_tx_queue[1].TX_MAILBOX == 1, "msg1 mailbox == 1");

    /* Cleanup: drain both messages */
    fake_free_mailboxes = 3;
    handle_can_tx();
    handle_can_tx();
    TEST_ASSERT(add_tx_message_call_count == 2, "drain sent 2 messages");
    TEST_ASSERT(can_queue_index == -1, "queue empty after drain");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 4: handle_can_tx sends the first message.
 * ------------------------------------------------------------------ */
static int test_handle_tx_sends_first(void)
{
    printf("\n--- TEST 4: handle_can_tx sends first message ---\n");
    reset_queue();

    CAN_TxHeaderTypeDef hdr = { .StdId = 0x555, .RTR = 0, .DLC = 8 };
    uint8_t data[8] = {10, 20, 30, 40, 50, 60, 70, 80};

    add_can_message(2, hdr, data);
    TEST_ASSERT(can_queue_index == 0, "can_queue_index == 0 after add");

    /* Call handle_can_tx – should send the one queued message */
    fake_free_mailboxes = 3;
    handle_can_tx();

    TEST_ASSERT(add_tx_message_call_count == 1,
                "HAL_CAN_AddTxMessage called once");
    TEST_ASSERT(last_added_message.can_tx_header.StdId == 0x555,
                "sent message has StdId == 0x555");
    TEST_ASSERT(last_added_message.tx_data[0] == 10,
                "sent data[0] == 10");
    TEST_ASSERT(last_added_message.tx_data[7] == 80,
                "sent data[7] == 80");
    /* Queue should reset after sending the only message */
    TEST_ASSERT(can_queue_index == -1,
                "queue reset after draining single message");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 5: Multiple calls to handle_can_tx drain the queue.
 * ------------------------------------------------------------------ */
static int test_handle_tx_drains_queue(void)
{
    printf("\n--- TEST 5: handle_can_tx drains all messages ---\n");
    reset_queue();

    CAN_TxHeaderTypeDef hdr = { .StdId = 0x111, .RTR = 0, .DLC = 8 };
    uint8_t data[8] = {0};

    /* Add 3 messages */
    for (int i = 0; i < 3; i++) {
        hdr.StdId = 0x111 + i;
        data[0] = (uint8_t)(i + 1);
        add_can_message((uint32_t)i, hdr, data);
    }
    TEST_ASSERT(can_queue_index == 2, "can_queue_index == 2 after 3 adds");

    /* Call handle_can_tx 3 times – each call sends one message */
    for (int call = 1; call <= 3; call++) {
        handle_can_tx();
        /* Use explicit check (not TEST_ASSERT) for dynamic message */
        if (add_tx_message_call_count != call) {
            printf("  FAIL: call count %d != expected %d  (line %d)\n",
                   add_tx_message_call_count, call, __LINE__);
            tests_failed++;
            return 0;
        }
        printf("  PASS: call count == %d\n", call);
        tests_passed++;
    }
    TEST_ASSERT(add_tx_message_call_count == 3, "total 3 HAL calls made");
    TEST_ASSERT(can_queue_index == -1,
                "queue index reset to -1 after full drain");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 6: No free mailboxes – handle_can_tx does not send.
 * ------------------------------------------------------------------ */
static int test_handle_tx_no_mailbox(void)
{
    printf("\n--- TEST 6: No mailboxes prevents send ---\n");
    reset_queue();

    CAN_TxHeaderTypeDef hdr = { .StdId = 0x333, .RTR = 0, .DLC = 8 };
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};

    add_can_message(0, hdr, data);
    TEST_ASSERT(can_queue_index == 0, "one message queued");

    /* Set 0 free mailboxes – handle_can_tx should bail */
    fake_free_mailboxes = 0;
    handle_can_tx();

    TEST_ASSERT(add_tx_message_call_count == 0,
                "no HAL_CAN_AddTxMessage call when mailboxes == 0");
    TEST_ASSERT(can_queue_index == 0,
                "can_queue_index unchanged (message not sent)");

    /* Cleanup: restore mailboxes and drain */
    fake_free_mailboxes = 3;
    handle_can_tx();
    TEST_ASSERT(add_tx_message_call_count == 1, "drain succeeded after restoring mailboxes");
    TEST_ASSERT(can_queue_index == -1, "queue empty after drain");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 7: Queue overflow – no crash when exceeding array size.
 * ------------------------------------------------------------------ */
static int test_queue_overflow_behavior(void)
{
    printf("\n--- TEST 7: Queue overflow behavior ---\n");
    reset_queue();

    CAN_TxHeaderTypeDef hdr = { .StdId = 0, .RTR = 0, .DLC = 8 };
    uint8_t data[8] = {0};

    /* Add exactly 64 messages (max array capacity) */
    for (int i = 0; i < 64; i++) {
        hdr.StdId = (uint32_t)i;
        data[0] = (uint8_t)i;
        add_can_message((uint32_t)i, hdr, data);
    }
    TEST_ASSERT(can_queue_index == 63,
                "can_queue_index == 63 after 64 adds (last valid index)");

    /* Add one more – should be dropped (bounds check) */
    hdr.StdId = 0xFFFF;
    data[0] = 0xFF;
    add_can_message(99, hdr, data);
    /* can_queue_index stays at 63 (overflow dropped) */
    TEST_ASSERT(can_queue_index == 63,
                "can_queue_index == 63 after 65th add (dropped by bounds check)");

    /* Drain all 64 messages */
    for (int i = 0; i < 64; i++) {
        handle_can_tx();
        if (can_queue_index == -1) {
            break;
        }
    }
    TEST_ASSERT(can_queue_index == -1,
                "queue fully drained after overflow");
    /* We should have sent 64 messages (65th was dropped) */
    TEST_ASSERT(add_tx_message_call_count == 64,
                "64 messages transmitted (65th dropped)");
    return 1;
}

/* ------------------------------------------------------------------
 * TEST 8: Data integrity – correct first message sent with distinct
 *         StdId values and data patterns.
 * ------------------------------------------------------------------ */
static int test_data_integrity(void)
{
    printf("\n--- TEST 8: Data integrity through queue ---\n");
    reset_queue();

    CAN_TxHeaderTypeDef hdr;
    uint8_t data[8];

    /* Push three messages with distinct IDs and data patterns */
    hdr = (CAN_TxHeaderTypeDef){ .StdId = 0x100, .RTR = 0, .DLC = 8 };
    uint8_t d1[8] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};
    memcpy(data, d1, 8);
    add_can_message(0, hdr, data);

    hdr = (CAN_TxHeaderTypeDef){ .StdId = 0x200, .RTR = 0, .DLC = 8 };
    uint8_t d2[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    memcpy(data, d2, 8);
    add_can_message(1, hdr, data);

    hdr = (CAN_TxHeaderTypeDef){ .StdId = 0x300, .RTR = 0, .DLC = 8 };
    uint8_t d3[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
    memcpy(data, d3, 8);
    add_can_message(2, hdr, data);

    TEST_ASSERT(can_queue_index == 2, "3 messages queued");

    /* Call handle_can_tx once – should send the FIRST message (0x100) */
    fake_free_mailboxes = 3;
    handle_can_tx();

    TEST_ASSERT(add_tx_message_call_count == 1,
                "one message sent");
    TEST_ASSERT(last_added_message.can_tx_header.StdId == 0x100,
                "first message sent has StdId == 0x100");
    TEST_ASSERT(last_added_message.tx_data[0] == 0x10,
                "first message data[0] == 0x10");
    TEST_ASSERT(last_added_message.tx_data[3] == 0x40,
                "first message data[3] == 0x40");
    TEST_ASSERT(last_added_message.tx_data[7] == 0x80,
                "first message data[7] == 0x80");

    /* Drain remaining 2 messages */
    handle_can_tx();
    handle_can_tx();

    TEST_ASSERT(add_tx_message_call_count == 3, "all 3 messages sent");
    TEST_ASSERT(can_queue_index == -1, "queue empty after drain");

    /* Verify the last sent message was the third one */
    TEST_ASSERT(last_added_message.can_tx_header.StdId == 0x300,
                "last message sent has StdId == 0x300");
    TEST_ASSERT(last_added_message.tx_data[0] == 0xDE,
                "last message data[0] == 0xDE");
    TEST_ASSERT(last_added_message.tx_data[3] == 0xEF,
                "last message data[3] == 0xEF");
    return 1;
}

/* ==================================================================
 * Main – run all tests, report summary, exit 0 on full pass.
 * ================================================================== */
int main(void)
{
    printf("========================================\n");
    printf(" CAN Queue (array-based TX) Unit Tests\n");
    printf("========================================\n");

    int (*test_fn[])(void) = {
        test_empty_queue,
        test_add_one_message,
        test_add_two_messages,
        test_handle_tx_sends_first,
        test_handle_tx_drains_queue,
        test_handle_tx_no_mailbox,
        test_queue_overflow_behavior,
        test_data_integrity,
    };
    const char *test_names[] = {
        "TEST 1: Empty queue does nothing",
        "TEST 2: Add one message",
        "TEST 3: Add two messages",
        "TEST 4: handle_can_tx sends first message",
        "TEST 5: handle_can_tx drains all messages",
        "TEST 6: No mailboxes prevents send",
        "TEST 7: Queue overflow behavior",
        "TEST 8: Data integrity through queue",
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
