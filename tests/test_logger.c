/*
 * test_logger.c
 * Standalone host-based unit test for EEPROM circular buffer logger
 * (Core/Src/logger.c).
 *
 * Compile: gcc -o test_logger test_logger.c -Wall -Wextra -std=c11
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ==================================================================
 * Enum type stubs  (from Core/Inc/main.h)
 * ================================================================== */

typedef enum {
	AS_STATE_OFF = 1,
	AS_STATE_READY = 2,
	AS_STATE_DRIVING = 3,
	AS_STATE_EMERGENCY = 4,
	AS_STATE_FINISHED = 5
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

/* ==================================================================
 * EE24 stub handle  (simplified flat-memory EEPROM)
 * ================================================================== */

typedef struct {
	uint8_t memory[4096];
} EE24_HandleTypeDef;

/* Global EE24 instance — zero-initialised at load time */
static EE24_HandleTypeDef ee;

/* ==================================================================
 * Logger constants and types  (from Core/Inc/logger.h)
 * ================================================================== */

#define LOG_MAGIC        0xDEADBEEF
#define LOG_HEADER_ADDR  0
#define LOG_DATA_ADDR    8
#define LOG_MAX_ENTRIES  19

typedef struct {
	uint32_t magic;   /* 0xDEADBEEF */
	uint8_t  head;    /* oldest entry index */
	uint8_t  tail;    /* next write index */
	uint8_t  count;   /* valid entries (0-19) */
	uint8_t  pad;     /* alignment */
} log_header_t;        /* 8 bytes */

typedef struct {
	uint32_t can_hw_state;
	uint32_t timestamp;
	uint32_t pc;
	uint32_t lr;
	uint32_t psr;
	AS_STATE_t as_state;
	current_mission_t mission;
	Emergency_cause_t emergency_cause;
	startup_sequence_state_t startup_state;
	float front_pressure_pneumatic;
	float rear_pressure_pneumatic;
	float front_pressure_hydraulic;
	float rear_pressure_hydraulic;
	uint8_t asms;
	uint8_t sdc_feedback;
	uint8_t pad[2];                 /* keep 4-byte aligned */
} log_entry_t;

/* ==================================================================
 * EE24 stub functions — read/write from flat memory buffer
 * ================================================================== */

bool EE24_Read(EE24_HandleTypeDef *ee, uint32_t addr, uint8_t *data, uint32_t len, uint32_t timeout)
{
	(void)timeout;
	if (addr + len > sizeof(ee->memory))
		return false;
	memcpy(data, &ee->memory[addr], len);
	return true;
}

bool EE24_Write(EE24_HandleTypeDef *ee, uint32_t addr, const uint8_t *data, uint32_t len, uint32_t timeout)
{
	(void)timeout;
	if (addr + len > sizeof(ee->memory))
		return false;
	memcpy(&ee->memory[addr], data, len);
	return true;
}

/* ==================================================================
 * Logger function bodies — copied VERBATIM from Core/Src/logger.c
 * ================================================================== */

bool eeprom_log_init(EE24_HandleTypeDef *ee) {
	log_header_t header = {0};

	if (!EE24_Read(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000))
		return false;

	if (header.magic != LOG_MAGIC) {
		header.magic = LOG_MAGIC;
		header.head  = 0;
		header.tail  = 0;
		header.count = 0;
		return EE24_Write(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000);
	}

	return true;
}

bool eeprom_log_write(EE24_HandleTypeDef *ee, log_entry_t *entry) {
	log_header_t header = {0};

	if (!EE24_Read(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000))
		return false;

	if (header.magic != LOG_MAGIC)
		return false;

	uint32_t addr = LOG_DATA_ADDR + (header.tail * sizeof(log_entry_t));
	if (!EE24_Write(ee, addr, (uint8_t*)entry, sizeof(log_entry_t), 1000))
		return false;

	header.tail = (header.tail + 1) % LOG_MAX_ENTRIES;

	if (header.count == LOG_MAX_ENTRIES)
		header.head = (header.head + 1) % LOG_MAX_ENTRIES;
	else
		header.count++;

	return EE24_Write(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000);
}

// index 0 = oldest, index count-1 = newest
bool eeprom_log_read(EE24_HandleTypeDef *ee, uint8_t index, log_entry_t *entry) {
	log_header_t header = {0};

	if (!EE24_Read(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000))
		return false;

	if (header.magic != LOG_MAGIC)
		return false;

	if (index >= header.count)
		return false;

	uint8_t physical = (header.head + index) % LOG_MAX_ENTRIES;
	uint32_t addr = LOG_DATA_ADDR + (physical * sizeof(log_entry_t));

	return EE24_Read(ee, addr, (uint8_t*)entry, sizeof(log_entry_t), 1000);
}

// optional but useful
bool eeprom_log_clear(EE24_HandleTypeDef *ee) {
	log_header_t header = {0};
	header.magic = LOG_MAGIC;
	return EE24_Write(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000);
}

// get count without reading entries
bool eeprom_log_count(EE24_HandleTypeDef *ee, uint8_t *count) {
	log_header_t header = {0};

	if (!EE24_Read(ee, LOG_HEADER_ADDR, (uint8_t*)&header, sizeof(log_header_t), 1000))
		return false;

	if (header.magic != LOG_MAGIC)
		return false;

	*count = header.count;
	return true;
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

/* Re-initialise the EEPROM stub to a clean (zeroed) state */
static void reset_ee(void)
{
	memset(&ee, 0, sizeof(ee));
}

/* Helper: build a log_entry_t with a given fill value */
static log_entry_t make_entry(uint32_t val)
{
	log_entry_t e;
	memset(&e, 0, sizeof(e));
	e.can_hw_state  = val;
	e.timestamp     = val + 1000;
	e.pc            = val + 0x08000000;
	e.lr            = val + 0x08001000;
	e.psr           = val + 0x01000000;
	e.as_state      = (val % 5) + 1;            /* wrap within AS_STATE_t range */
	e.mission       = (current_mission_t)(val % 7);
	e.emergency_cause = (Emergency_cause_t)(val % 10);
	e.startup_state = (startup_sequence_state_t)(val % 9);
	e.front_pressure_pneumatic  = 7.5f + (float)val * 0.1f;
	e.rear_pressure_pneumatic   = 8.0f + (float)val * 0.1f;
	e.front_pressure_hydraulic  = 65.0f + (float)val * 0.5f;
	e.rear_pressure_hydraulic   = 70.0f + (float)val * 0.5f;
	e.asms          = (uint8_t)(val & 0x01);
	e.sdc_feedback  = (uint8_t)((val >> 1) & 0x01);
	return e;
}

/* Helper: verify all fields of a log_entry_t against what make_entry(val) would produce */
static int verify_entry(uint32_t val, const log_entry_t *e, const char *label)
{
	char buf[128];

	snprintf(buf, sizeof(buf), "%s: can_hw_state == %u",      label, (unsigned)val);
	TEST_ASSERT(e->can_hw_state == val, buf);

	snprintf(buf, sizeof(buf), "%s: timestamp == %u",         label, (unsigned)(val + 1000));
	TEST_ASSERT(e->timestamp == val + 1000, buf);

	snprintf(buf, sizeof(buf), "%s: pc == 0x%08X",            label, (unsigned)(val + 0x08000000));
	TEST_ASSERT(e->pc == val + 0x08000000, buf);

	snprintf(buf, sizeof(buf), "%s: lr == 0x%08X",            label, (unsigned)(val + 0x08001000));
	TEST_ASSERT(e->lr == val + 0x08001000, buf);

	snprintf(buf, sizeof(buf), "%s: psr == 0x%08X",           label, (unsigned)(val + 0x01000000));
	TEST_ASSERT(e->psr == val + 0x01000000, buf);

	snprintf(buf, sizeof(buf), "%s: as_state == %d",          label, (int)((val % 5) + 1));
	TEST_ASSERT(e->as_state == (AS_STATE_t)((val % 5) + 1), buf);

	snprintf(buf, sizeof(buf), "%s: mission == %d",           label, (int)(val % 7));
	TEST_ASSERT(e->mission == (current_mission_t)(val % 7), buf);

	snprintf(buf, sizeof(buf), "%s: emergency_cause == %d",   label, (int)(val % 10));
	TEST_ASSERT(e->emergency_cause == (Emergency_cause_t)(val % 10), buf);

	snprintf(buf, sizeof(buf), "%s: startup_state == %d",     label, (int)(val % 9));
	TEST_ASSERT(e->startup_state == (startup_sequence_state_t)(val % 9), buf);

	snprintf(buf, sizeof(buf), "%s: front_pressure_pneumatic == %.1f", label, 7.5f + (float)val * 0.1f);
	TEST_ASSERT(e->front_pressure_pneumatic == 7.5f + (float)val * 0.1f, buf);

	snprintf(buf, sizeof(buf), "%s: rear_pressure_pneumatic == %.1f",  label, 8.0f + (float)val * 0.1f);
	TEST_ASSERT(e->rear_pressure_pneumatic == 8.0f + (float)val * 0.1f, buf);

	snprintf(buf, sizeof(buf), "%s: front_pressure_hydraulic == %.1f", label, 65.0f + (float)val * 0.5f);
	TEST_ASSERT(e->front_pressure_hydraulic == 65.0f + (float)val * 0.5f, buf);

	snprintf(buf, sizeof(buf), "%s: rear_pressure_hydraulic == %.1f",  label, 70.0f + (float)val * 0.5f);
	TEST_ASSERT(e->rear_pressure_hydraulic == 70.0f + (float)val * 0.5f, buf);

	snprintf(buf, sizeof(buf), "%s: asms == %u",              label, (unsigned)(val & 0x01));
	TEST_ASSERT(e->asms == (uint8_t)(val & 0x01), buf);

	snprintf(buf, sizeof(buf), "%s: sdc_feedback == %u",      label, (unsigned)((val >> 1) & 0x01));
	TEST_ASSERT(e->sdc_feedback == (uint8_t)((val >> 1) & 0x01), buf);

	return 1;   /* all checks passed */
}

/* ==================================================================
 * Test cases
 * ================================================================== */

/* ------------------------------------------------------------------
 * TEST 1: Init creates fresh log.
 *
 * On a zeroed EE24, eeprom_log_init() should write magic and
 * zero the head/tail/count fields.
 * ------------------------------------------------------------------ */
static int test1_init_creates_fresh_log(void)
{
	printf("\n--- TEST 1: Init creates fresh log ---\n");

	reset_ee();

	bool ok = eeprom_log_init(&ee);
	TEST_ASSERT(ok == true, "eeprom_log_init returns true on fresh EEPROM");

	log_header_t hdr;
	memset(&hdr, 0, sizeof(hdr));
	bool read_ok = EE24_Read(&ee, LOG_HEADER_ADDR, (uint8_t*)&hdr, sizeof(hdr), 100);
	TEST_ASSERT(read_ok == true,          "Read back header after init");
	TEST_ASSERT(hdr.magic == LOG_MAGIC,   "magic == LOG_MAGIC after init");
	TEST_ASSERT(hdr.head  == 0,           "head == 0 after init");
	TEST_ASSERT(hdr.tail  == 0,           "tail == 0 after init");
	TEST_ASSERT(hdr.count == 0,           "count == 0 after init");

	return 1;
}

/* ------------------------------------------------------------------
 * TEST 2: Init detects existing log.
 *
 * Calling init twice: the second call should detect the existing
 * magic and return true without re-writing the header.
 * ------------------------------------------------------------------ */
static int test2_init_detects_existing_log(void)
{
	printf("\n--- TEST 2: Init detects existing log ---\n");

	reset_ee();

	/* First init — writes the header */
	bool ok1 = eeprom_log_init(&ee);
	TEST_ASSERT(ok1 == true, "first eeprom_log_init succeeds");

	/* Read the header and capture its content to detect any re-write */
	log_header_t snap;
	EE24_Read(&ee, LOG_HEADER_ADDR, (uint8_t*)&snap, sizeof(snap), 100);

	/* Second init — should detect magic and return true */
	bool ok2 = eeprom_log_init(&ee);
	TEST_ASSERT(ok2 == true, "second eeprom_log_init returns true");

	/* Verify header is unchanged (not re-written) */
	log_header_t after;
	EE24_Read(&ee, LOG_HEADER_ADDR, (uint8_t*)&after, sizeof(after), 100);
	TEST_ASSERT(after.magic == LOG_MAGIC, "magic preserved");
	TEST_ASSERT(after.head  == snap.head,  "head unchanged");
	TEST_ASSERT(after.tail  == snap.tail,  "tail unchanged");
	TEST_ASSERT(after.count == snap.count, "count unchanged");

	return 1;
}

/* ------------------------------------------------------------------
 * TEST 3: Write and read back.
 *
 * Write a single entry, read index 0, and confirm every field
 * matches what was written.
 * ------------------------------------------------------------------ */
static int test3_write_and_read_back(void)
{
	printf("\n--- TEST 3: Write and read back ---\n");

	reset_ee();
	eeprom_log_init(&ee);

	log_entry_t w = make_entry(42);
	bool wrote = eeprom_log_write(&ee, &w);
	TEST_ASSERT(wrote == true, "eeprom_log_write returns true");

	log_entry_t r;
	memset(&r, 0, sizeof(r));
	bool read_ok = eeprom_log_read(&ee, 0, &r);
	TEST_ASSERT(read_ok == true, "eeprom_log_read(0) returns true");

	/* Verify every field */
	if (!verify_entry(42, &r, "single entry")) {
		return 0;
	}

	/* Confirm count == 1 */
	uint8_t cnt;
	bool cnt_ok = eeprom_log_count(&ee, &cnt);
	TEST_ASSERT(cnt_ok == true, "eeprom_log_count returns true");
	TEST_ASSERT(cnt == 1,        "count == 1 after one write");

	return 1;
}

/* ------------------------------------------------------------------
 * TEST 4: Write multiple entries.
 *
 * Write 5 entries with distinct values, read them all back in
 * order (index 0 oldest → index 4 newest), verify each.
 * ------------------------------------------------------------------ */
static int test4_write_multiple_entries(void)
{
	printf("\n--- TEST 4: Write multiple entries ---\n");

	reset_ee();
	eeprom_log_init(&ee);

	const int N = 5;
	for (int i = 0; i < N; i++) {
		log_entry_t w = make_entry((uint32_t)(100 + i));
		bool wrote = eeprom_log_write(&ee, &w);
		/* early-exit helper isn't available in a loop; check inline */
		if (!wrote) {
			printf("  FAIL: eeprom_log_write(%d) returned false (line %d)\n", i, __LINE__);
			tests_failed++;
			return 0;
		}
		printf("  PASS: eeprom_log_write(%d) returned true\n", i);
		tests_passed++;
	}

	/* Read back in order */
	for (int i = 0; i < N; i++) {
		log_entry_t r;
		memset(&r, 0, sizeof(r));
		bool read_ok = eeprom_log_read(&ee, (uint8_t)i, &r);
		if (!read_ok) {
			printf("  FAIL: eeprom_log_read(%d) returned false (line %d)\n", i, __LINE__);
			tests_failed++;
			return 0;
		}
		printf("  PASS: eeprom_log_read(%d) returned true\n", i);
		tests_passed++;

		char label[32];
		snprintf(label, sizeof(label), "entry %d", i);
		if (!verify_entry((uint32_t)(100 + i), &r, label)) {
			return 0;
		}
	}

	/* Verify total count */
	uint8_t cnt;
	eeprom_log_count(&ee, &cnt);
	TEST_ASSERT(cnt == N, "count == 5 after five writes");

	return 1;
}

/* ------------------------------------------------------------------
 * TEST 5: Circular overwrite.
 *
 * Write LOG_MAX_ENTRIES + 2 entries (21 with LOG_MAX_ENTRIES = 19).
 * - Index 0 should return the OLDEST surviving entry (the 3rd write,
 *   since the first 2 were overwritten).
 * - Index (LOG_MAX_ENTRIES-1) should return the newest entry.
 * - Count must remain LOG_MAX_ENTRIES.
 * ------------------------------------------------------------------ */
static int test5_circular_overwrite(void)
{
	printf("\n--- TEST 5: Circular overwrite ---\n");

	reset_ee();
	eeprom_log_init(&ee);

	int total = LOG_MAX_ENTRIES + 2;   /* 21 */

	for (int i = 0; i < total; i++) {
		log_entry_t w = make_entry((uint32_t)i);
		bool wrote = eeprom_log_write(&ee, &w);
		if (!wrote) {
			printf("  FAIL: eeprom_log_write(%d) returned false (line %d)\n", i, __LINE__);
			tests_failed++;
			return 0;
		}
		printf("  PASS: eeprom_log_write(%d) returned true\n", i);
		tests_passed++;
	}

	/* Count must be LOG_MAX_ENTRIES */
	uint8_t cnt;
	eeprom_log_count(&ee, &cnt);
	TEST_ASSERT(cnt == LOG_MAX_ENTRIES, "count == LOG_MAX_ENTRIES after overwrite");

	/* Index 0 should be the oldest surviving entry — write index 2 */
	log_entry_t oldest;
	memset(&oldest, 0, sizeof(oldest));
	bool read_oldest = eeprom_log_read(&ee, 0, &oldest);
	TEST_ASSERT(read_oldest == true, "eeprom_log_read(0) succeeds after overwrite");
	TEST_ASSERT(oldest.can_hw_state == 2,
		    "index 0 returns oldest surviving entry (write #2)");

	/* Index (LOG_MAX_ENTRIES-1) should be the newest — write index 20 */
	log_entry_t newest;
	memset(&newest, 0, sizeof(newest));
	bool read_newest = eeprom_log_read(&ee, LOG_MAX_ENTRIES - 1, &newest);
	TEST_ASSERT(read_newest == true, "eeprom_log_read(18) succeeds after overwrite");
	TEST_ASSERT(newest.can_hw_state == (uint32_t)(total - 1),
		    "index 18 returns newest entry (write #20)");

	return 1;
}

/* ------------------------------------------------------------------
 * TEST 6: Read out-of-range.
 *
 * After writing 3 entries, reading index 5 must return false.
 * ------------------------------------------------------------------ */
static int test6_read_out_of_range(void)
{
	printf("\n--- TEST 6: Read out-of-range ---\n");

	reset_ee();
	eeprom_log_init(&ee);

	for (int i = 0; i < 3; i++) {
		log_entry_t w = make_entry((uint32_t)(10 + i));
		eeprom_log_write(&ee, &w);
	}

	log_entry_t r;
	memset(&r, 0, sizeof(r));
	bool read_ok = eeprom_log_read(&ee, 5, &r);
	TEST_ASSERT(read_ok == false, "eeprom_log_read(5) returns false when count < 5");

	return 1;
}

/* ------------------------------------------------------------------
 * TEST 7: Count function.
 *
 * After writing 7 entries, eeprom_log_count must return 7.
 * ------------------------------------------------------------------ */
static int test7_count_function(void)
{
	printf("\n--- TEST 7: Count function ---\n");

	reset_ee();
	eeprom_log_init(&ee);

	for (int i = 0; i < 7; i++) {
		log_entry_t w = make_entry((uint32_t)(200 + i));
		eeprom_log_write(&ee, &w);
	}

	uint8_t cnt;
	bool ok = eeprom_log_count(&ee, &cnt);
	TEST_ASSERT(ok == true, "eeprom_log_count returns true");
	TEST_ASSERT(cnt == 7,   "count == 7 after seven writes");

	return 1;
}

/* ------------------------------------------------------------------
 * TEST 8: Clear function.
 *
 * After writing entries, call clear. Verify count is 0 and magic
 * is preserved.
 * ------------------------------------------------------------------ */
static int test8_clear_function(void)
{
	printf("\n--- TEST 8: Clear function ---\n");

	reset_ee();
	eeprom_log_init(&ee);

	/* Write a few entries */
	for (int i = 0; i < 4; i++) {
		log_entry_t w = make_entry((uint32_t)(300 + i));
		eeprom_log_write(&ee, &w);
	}

	uint8_t cnt_before;
	eeprom_log_count(&ee, &cnt_before);
	TEST_ASSERT(cnt_before == 4, "count == 4 before clear");

	/* Clear */
	bool cleared = eeprom_log_clear(&ee);
	TEST_ASSERT(cleared == true, "eeprom_log_clear returns true");

	/* Count must now be 0 */
	uint8_t cnt_after;
	bool cnt_ok = eeprom_log_count(&ee, &cnt_after);
	TEST_ASSERT(cnt_ok == true,  "eeprom_log_count returns true after clear");
	TEST_ASSERT(cnt_after == 0,  "count == 0 after clear");

	/* Magic must still be intact */
	log_header_t hdr;
	EE24_Read(&ee, LOG_HEADER_ADDR, (uint8_t*)&hdr, sizeof(hdr), 100);
	TEST_ASSERT(hdr.magic == LOG_MAGIC, "magic preserved after clear");

	/* Verify we can write again after clear */
	log_entry_t w2 = make_entry(999);
	bool wrote = eeprom_log_write(&ee, &w2);
	TEST_ASSERT(wrote == true, "can write again after clear");

	log_entry_t r;
	memset(&r, 0, sizeof(r));
	bool read_ok = eeprom_log_read(&ee, 0, &r);
	TEST_ASSERT(read_ok == true,           "can read after clear+write");
	TEST_ASSERT(r.can_hw_state == 999,     "re-written entry data intact");

	return 1;
}

/* ==================================================================
 * Main — run all tests, report summary, exit 0 on full pass.
 * ================================================================== */

int main(void)
{
	printf("========================================\n");
	printf(" EEPROM Circular Buffer Logger Unit Tests\n");
	printf("========================================\n");

	int (*test_fn[])(void) = {
		test1_init_creates_fresh_log,
		test2_init_detects_existing_log,
		test3_write_and_read_back,
		test4_write_multiple_entries,
		test5_circular_overwrite,
		test6_read_out_of_range,
		test7_count_function,
		test8_clear_function,
	};
	const char *test_names[] = {
		"TEST 1: Init creates fresh log",
		"TEST 2: Init detects existing log",
		"TEST 3: Write and read back",
		"TEST 4: Write multiple entries",
		"TEST 5: Circular overwrite",
		"TEST 6: Read out-of-range",
		"TEST 7: Count function",
		"TEST 8: Clear function",
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
