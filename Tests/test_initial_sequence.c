/*
 * test_initial_sequence.c
 *
 *  Unit tests for the initial_sequence() state machine.
 *  Compiles and runs entirely on the host (no STM32 hardware required).
 *
 *  Build and run:
 *      cd Tests && make
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* Pull in the types and the function under test without any HAL dependency. */
#include "../Core/Inc/Autonomous_functions.h"

/* ── Minimal test framework ──────────────────────────────────────────────── */

static int tests_run    = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT(expr)                                                    \
    do {                                                                \
        tests_run++;                                                    \
        if (expr) {                                                     \
            tests_passed++;                                             \
        } else {                                                        \
            tests_failed++;                                             \
            printf("  FAIL  %s:%d  (%s)\n", __FILE__, __LINE__, #expr);\
        }                                                               \
    } while (0)

#define TEST(name) \
    static void name(void); \
    static void name##_run(void) { printf("  %-55s", #name); name(); \
        printf("%s\n", tests_failed == 0 ? "OK" : ""); } \
    static void name(void)

/* ── Helper: default inputs ──────────────────────────────────────────────── */

static initial_seq_inputs_t make_inputs(void)
{
    initial_seq_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.SDC_feedback        = 0;
    in.front_pneumatic_kPa = 0.0f;
    in.rear_pneumatic_kPa  = 0.0f;
    in.ignition_status     = 0;
    in.timestamp_ms        = 0;
    return in;
}

static initial_seq_outputs_t make_outputs(void)
{
    initial_seq_outputs_t out;
    out.HW_WDT_Enable     = 1;
    out.Ignition_Request  = 0;
    out.vehicle_state     = AS_ON;  /* arbitrary non-EMERGENCY initial value */
    out.sequence_complete = 0;
    return out;
}

static initial_seq_ctx_t make_ctx(void)
{
    initial_seq_ctx_t ctx;
    ctx.state              = Watchdog_check;
    ctx.state_entry_time_ms = 0;
    return ctx;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Watchdog_check tests
 * ══════════════════════════════════════════════════════════════════════════ */

TEST(test_watchdog_activates_wdt_and_advances_when_sdc_closed)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    in.SDC_feedback = 1;
    in.timestamp_ms = 100;

    initial_sequence(&ctx, &in, &out);

    /* WDT must be ACTIVE (1) — confirmed working, keep it running. */
    ASSERT(out.HW_WDT_Enable == 1);
    ASSERT(ctx.state == Pressure_check);
    ASSERT(ctx.state_entry_time_ms == 100);
    ASSERT(out.sequence_complete == 0);
    ASSERT(out.vehicle_state != EMERGENCY);
}

TEST(test_watchdog_activates_wdt_while_sdc_open)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    /* SDC still open, but well within the 500 ms window. */
    in.SDC_feedback = 0;
    in.timestamp_ms = 100;   /* 100 ms elapsed from entry at t=0 */

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == Watchdog_check);    /* should not advance or error */
    ASSERT(out.HW_WDT_Enable == 1);         /* WDT must be activated each tick */
    ASSERT(out.vehicle_state != EMERGENCY);
}

TEST(test_watchdog_sdc_open_at_timeout_goes_to_error)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    /* Exactly at the timeout boundary (>= 500 ms). */
    in.SDC_feedback = 0;
    in.timestamp_ms = INITIAL_SEQ_SDC_TIMEOUT_MS;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == Error_state);
}

TEST(test_watchdog_sdc_open_just_before_timeout_stays)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    /* One millisecond before timeout: should NOT go to error yet. */
    in.SDC_feedback = 0;
    in.timestamp_ms = INITIAL_SEQ_SDC_TIMEOUT_MS - 1;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == Watchdog_check);
    ASSERT(out.vehicle_state != EMERGENCY);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Pressure_check tests
 * ══════════════════════════════════════════════════════════════════════════ */

TEST(test_pressure_ok_advances_to_hv_activation)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Pressure_check;
    in.front_pneumatic_kPa = INITIAL_SEQ_MIN_PNEUMATIC_KPA;
    in.rear_pneumatic_kPa  = INITIAL_SEQ_MIN_PNEUMATIC_KPA;
    in.timestamp_ms        = 200;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == HV_activation);
    ASSERT(ctx.state_entry_time_ms == 200);
    ASSERT(out.vehicle_state != EMERGENCY);
}

TEST(test_pressure_front_low_goes_to_error)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Pressure_check;
    in.front_pneumatic_kPa = INITIAL_SEQ_MIN_PNEUMATIC_KPA - 1.0f;
    in.rear_pneumatic_kPa  = INITIAL_SEQ_MIN_PNEUMATIC_KPA;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == Error_state);
}

TEST(test_pressure_rear_low_goes_to_error)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Pressure_check;
    in.front_pneumatic_kPa = INITIAL_SEQ_MIN_PNEUMATIC_KPA;
    in.rear_pneumatic_kPa  = INITIAL_SEQ_MIN_PNEUMATIC_KPA - 1.0f;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == Error_state);
}

TEST(test_pressure_both_above_threshold_pass)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Pressure_check;
    in.front_pneumatic_kPa = INITIAL_SEQ_MIN_PNEUMATIC_KPA + 50.0f;
    in.rear_pneumatic_kPa  = INITIAL_SEQ_MIN_PNEUMATIC_KPA + 100.0f;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == HV_activation);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  HV_activation tests
 * ══════════════════════════════════════════════════════════════════════════ */

TEST(test_hv_activation_sets_ignition_request)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = HV_activation;
    ctx.state_entry_time_ms = 0;
    in.ignition_status = 0;
    in.timestamp_ms    = 100;

    initial_sequence(&ctx, &in, &out);

    ASSERT(out.Ignition_Request == 1);
    ASSERT(ctx.state == HV_activation); /* still waiting */
}

TEST(test_hv_activation_ignition_confirmed_advances)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = HV_activation;
    ctx.state_entry_time_ms = 0;
    in.ignition_status = 1;   /* HV up */
    in.timestamp_ms    = 1000;

    initial_sequence(&ctx, &in, &out);

    ASSERT(out.Ignition_Request == 1);
    ASSERT(ctx.state == Pressure_correlation_check);
    ASSERT(ctx.state_entry_time_ms == 1000);
}

TEST(test_hv_activation_timeout_goes_to_error)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = HV_activation;
    ctx.state_entry_time_ms = 0;
    in.ignition_status = 0;
    in.timestamp_ms    = INITIAL_SEQ_HV_TIMEOUT_MS; /* exactly at timeout */

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == Error_state);
}

TEST(test_hv_activation_just_before_timeout_waits)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = HV_activation;
    ctx.state_entry_time_ms = 0;
    in.ignition_status = 0;
    in.timestamp_ms    = INITIAL_SEQ_HV_TIMEOUT_MS - 1;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == HV_activation);
    ASSERT(out.vehicle_state != EMERGENCY);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Pressure_correlation_check tests
 * ══════════════════════════════════════════════════════════════════════════ */

TEST(test_pressure_correlation_within_tolerance_completes)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Pressure_correlation_check;
    in.front_pneumatic_kPa = 600.0f;
    in.rear_pneumatic_kPa  = 600.0f + INITIAL_SEQ_PRESSURE_CORR_TOL_KPA;

    initial_sequence(&ctx, &in, &out);

    ASSERT(out.sequence_complete == 1);
    ASSERT(out.vehicle_state != EMERGENCY);
}

TEST(test_pressure_correlation_zero_difference_completes)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Pressure_correlation_check;
    in.front_pneumatic_kPa = 650.0f;
    in.rear_pneumatic_kPa  = 650.0f;

    initial_sequence(&ctx, &in, &out);

    ASSERT(out.sequence_complete == 1);
}

TEST(test_pressure_correlation_excess_difference_goes_to_error)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Pressure_correlation_check;
    in.front_pneumatic_kPa = 700.0f;
    in.rear_pneumatic_kPa  = 700.0f - (INITIAL_SEQ_PRESSURE_CORR_TOL_KPA + 1.0f);

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == Error_state);
    ASSERT(out.sequence_complete == 0);
}

TEST(test_pressure_correlation_reverse_direction_also_checked)
{
    /* Rear pressure higher than front by more than tolerance. */
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Pressure_correlation_check;
    in.front_pneumatic_kPa = 600.0f;
    in.rear_pneumatic_kPa  = 600.0f + INITIAL_SEQ_PRESSURE_CORR_TOL_KPA + 1.0f;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == Error_state);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Error_state tests
 * ══════════════════════════════════════════════════════════════════════════ */

TEST(test_error_state_sets_vehicle_emergency)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    ctx.state = Error_state;
    out.vehicle_state = AS_ON;   /* must be overwritten */

    initial_sequence(&ctx, &in, &out);

    ASSERT(out.vehicle_state == EMERGENCY);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Full happy-path integration test
 * ══════════════════════════════════════════════════════════════════════════ */

TEST(test_full_happy_path)
{
    initial_seq_ctx_t     ctx = make_ctx();
    initial_seq_inputs_t  in  = make_inputs();
    initial_seq_outputs_t out = make_outputs();

    /* --- Tick 1: Watchdog_check, SDC not yet closed ------------------- */
    in.timestamp_ms = 50;
    initial_sequence(&ctx, &in, &out);
    ASSERT(ctx.state == Watchdog_check);

    /* --- Tick 2: Watchdog_check, SDC closes at t=100 ms --------------- */
    in.SDC_feedback = 1;
    in.timestamp_ms = 100;
    out = make_outputs();
    initial_sequence(&ctx, &in, &out);
    ASSERT(ctx.state == Pressure_check);
    ASSERT(out.HW_WDT_Enable == 1);  /* WDT stays active after confirmation */

    /* --- Tick 3: Pressure_check, sufficient pressure ------------------ */
    in.SDC_feedback        = 1;
    in.front_pneumatic_kPa = 600.0f;
    in.rear_pneumatic_kPa  = 610.0f;
    in.timestamp_ms        = 200;
    out = make_outputs();
    initial_sequence(&ctx, &in, &out);
    ASSERT(ctx.state == HV_activation);

    /* --- Tick 4: HV_activation, waiting for ignition ------------------ */
    in.ignition_status = 0;
    in.timestamp_ms    = 300;
    out = make_outputs();
    initial_sequence(&ctx, &in, &out);
    ASSERT(ctx.state == HV_activation);
    ASSERT(out.Ignition_Request == 1);

    /* --- Tick 5: HV_activation, ignition confirmed -------------------- */
    in.ignition_status = 1;
    in.timestamp_ms    = 400;
    out = make_outputs();
    initial_sequence(&ctx, &in, &out);
    ASSERT(ctx.state == Pressure_correlation_check);

    /* --- Tick 6: Pressure_correlation_check, pressures correlated ----- */
    in.front_pneumatic_kPa = 600.0f;
    in.rear_pneumatic_kPa  = 620.0f;  /* 20 kPa difference, within 100 kPa */
    in.timestamp_ms        = 500;
    out = make_outputs();
    initial_sequence(&ctx, &in, &out);
    ASSERT(out.sequence_complete == 1);
    ASSERT(out.vehicle_state != EMERGENCY);
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("\n=== initial_sequence() unit tests ===\n\n");

    /* Watchdog_check */
    test_watchdog_activates_wdt_and_advances_when_sdc_closed_run();
    test_watchdog_activates_wdt_while_sdc_open_run();
    test_watchdog_sdc_open_at_timeout_goes_to_error_run();
    test_watchdog_sdc_open_just_before_timeout_stays_run();

    /* Pressure_check */
    test_pressure_ok_advances_to_hv_activation_run();
    test_pressure_front_low_goes_to_error_run();
    test_pressure_rear_low_goes_to_error_run();
    test_pressure_both_above_threshold_pass_run();

    /* HV_activation */
    test_hv_activation_sets_ignition_request_run();
    test_hv_activation_ignition_confirmed_advances_run();
    test_hv_activation_timeout_goes_to_error_run();
    test_hv_activation_just_before_timeout_waits_run();

    /* Pressure_correlation_check */
    test_pressure_correlation_within_tolerance_completes_run();
    test_pressure_correlation_zero_difference_completes_run();
    test_pressure_correlation_excess_difference_goes_to_error_run();
    test_pressure_correlation_reverse_direction_also_checked_run();

    /* Error_state */
    test_error_state_sets_vehicle_emergency_run();

    /* Integration */
    test_full_happy_path_run();

    printf("\n=== Results: %d/%d passed", tests_passed, tests_run);
    if (tests_failed) {
        printf(", %d FAILED", tests_failed);
    }
    printf(" ===\n\n");

    return (tests_failed == 0) ? 0 : 1;
}
