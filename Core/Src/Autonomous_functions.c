/*
 * Autonomous_functions.c
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#include "Autonomous_functions.h"

/*
 * HOW initial_sequence() WORKS
 * ─────────────────────────────────────────────────────────────────────────────
 * The car must pass a fixed sequence of checks before it is allowed to drive
 * autonomously.  The sequence has five steps (states):
 *
 *   1. Watchdog_check           – verify the hardware watchdog (WDT) is alive
 *   2. Pressure_check           – verify brake pressure is high enough
 *   3. HV_activation            – turn on high voltage and wait for confirmation
 *   4. Pressure_correlation_check – verify front and rear pressures match
 *   5. Error_state              – something went wrong; put car in EMERGENCY
 *
 * The function is called repeatedly (once per control loop tick).
 * Each call does exactly one step and returns immediately — it never blocks.
 *
 * The three arguments carry information in and out:
 *
 *   ctx     – "memory" between calls: which step we are on and when we entered it.
 *             Before the very first call set:  ctx.state = Watchdog_check;
 *                                              ctx.state_entry_time_ms = 0;
 *
 *   inputs  – everything read from the car this tick (SDC, pressures, ignition …).
 *             Fill this from real sensors before calling the function.
 *
 *   outputs – what the function wants to write to the car this tick
 *             (WDT enable, ignition request, vehicle state, done flag).
 *             Pre-fill with the current car state; the function only changes
 *             the fields that need to change.
 *
 * Happy path:
 *   Watchdog_check → Pressure_check → HV_activation
 *                 → Pressure_correlation_check → outputs->sequence_complete = 1
 *
 * Any failure goes straight to Error_state, which sets EMERGENCY.
 * ─────────────────────────────────────────────────────────────────────────────
 */
void initial_sequence(initial_seq_ctx_t *ctx,
                      const initial_seq_inputs_t *inputs,
                      initial_seq_outputs_t *outputs)
{
    switch (ctx->state) {

    /* ── STEP 1: Watchdog_check ───────────────────────────────────────────
     * What we do   : Enable the hardware watchdog (WDT) so it starts pulsing.
     * How we know it works : the SDC (Shutdown Circuit) line closes when the
     *                        WDT pulse is received correctly by the hardware.
     * Pass condition : SDC_feedback == 1 → WDT is alive → go to Pressure_check.
     * Fail condition : SDC never closes within INITIAL_SEQ_SDC_TIMEOUT_MS (500 ms)
     *                  → go to Error_state.
     * ──────────────────────────────────────────────────────────────────── */
    case Watchdog_check:
        /* Turn the watchdog on (must stay on for the rest of the run). */
        outputs->HW_WDT_Enable = 1;

        if (inputs->SDC_feedback == 1) {
            /* SDC closed → the WDT pulse reached the hardware. Move on. */
            ctx->state = Pressure_check;
            ctx->state_entry_time_ms = inputs->timestamp_ms;

        } else if ((inputs->timestamp_ms - ctx->state_entry_time_ms) >=
                   INITIAL_SEQ_SDC_TIMEOUT_MS) {
            /* 500 ms passed and the SDC never closed → WDT fault → EMERGENCY. */
            ctx->state = Error_state;
        }
        /* If neither condition is true, stay in Watchdog_check and try again
         * next tick. */
        break;

    /* ── STEP 2: Pressure_check ───────────────────────────────────────────
     * What we do   : Read front and rear pneumatic brake pressures.
     * Pass condition : both are at least INITIAL_SEQ_MIN_PNEUMATIC_KPA (500 kPa)
     *                  → brakes have enough pressure → go to HV_activation.
     * Fail condition : either sensor reads below 500 kPa → cannot brake safely
     *                  → go to Error_state.
     * ──────────────────────────────────────────────────────────────────── */
    case Pressure_check:
        if (inputs->front_pneumatic_kPa >= INITIAL_SEQ_MIN_PNEUMATIC_KPA &&
            inputs->rear_pneumatic_kPa  >= INITIAL_SEQ_MIN_PNEUMATIC_KPA) {
            /* Both brakes have enough pressure — safe to power up the HV. */
            ctx->state = HV_activation;
            ctx->state_entry_time_ms = inputs->timestamp_ms;
        } else {
            /* At least one brake lacks pressure — cannot stop the car safely. */
            ctx->state = Error_state;
        }
        break;

    /* ── STEP 3: HV_activation ────────────────────────────────────────────
     * What we do   : Ask the VCU (vehicle control unit) to turn on High Voltage.
     *                We keep sending Ignition_Request = 1 every tick until the
     *                VCU confirms it is up (ignition_status == 1).
     * Pass condition : VCU confirms HV is on → go to Pressure_correlation_check.
     * Fail condition : HV not confirmed within INITIAL_SEQ_HV_TIMEOUT_MS (5 000 ms)
     *                  → go to Error_state.
     * ──────────────────────────────────────────────────────────────────── */
    case HV_activation:
        /* Keep requesting HV every tick until the VCU confirms it. */
        outputs->Ignition_Request = 1;

        if (inputs->ignition_status == 1) {
            /* VCU says HV is active. */
            ctx->state = Pressure_correlation_check;
            ctx->state_entry_time_ms = inputs->timestamp_ms;

        } else if ((inputs->timestamp_ms - ctx->state_entry_time_ms) >=
                   INITIAL_SEQ_HV_TIMEOUT_MS) {
            /* 5 s passed and HV still not confirmed → fault. */
            ctx->state = Error_state;
        }
        break;

    /* ── STEP 4: Pressure_correlation_check ──────────────────────────────
     * What we do   : Compare front and rear brake pressures.
     *                They should both reflect the same physical system so they
     *                must be close to each other.
     * Pass condition : |front − rear| ≤ INITIAL_SEQ_PRESSURE_CORR_TOL_KPA (100 kPa)
     *                  → sensors agree → sequence_complete = 1 (all checks passed).
     * Fail condition : difference > 100 kPa → sensor or brake fault → Error_state.
     * ──────────────────────────────────────────────────────────────────── */
    case Pressure_correlation_check: {
        /* Absolute difference between front and rear pressure. */
        float diff = inputs->front_pneumatic_kPa - inputs->rear_pneumatic_kPa;
        if (diff < 0.0f) {
            diff = -diff;  /* make it positive (absolute value) */
        }

        if (diff <= INITIAL_SEQ_PRESSURE_CORR_TOL_KPA) {
            /* Pressures agree within 100 kPa — all checks passed. */
            outputs->sequence_complete = 1;
        } else {
            /* Large mismatch between front and rear → possible sensor or brake fault. */
            ctx->state = Error_state;
        }
        break;
    }

    /* ── Error_state ──────────────────────────────────────────────────────
     * Reached from any step that fails.
     * Sets EMERGENCY on the vehicle state so the main loop can react.
     * ──────────────────────────────────────────────────────────────────── */
    case Error_state:
        outputs->vehicle_state = EMERGENCY;
        break;

    default:
        /* Unknown state — treat as an error to be safe. */
        outputs->vehicle_state = EMERGENCY;
        break;
    }
}

void continuous_monitoring(void)
{
}
