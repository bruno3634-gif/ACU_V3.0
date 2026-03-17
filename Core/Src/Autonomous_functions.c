/*
 * Autonomous_functions.c
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#include "Autonomous_functions.h"

/**
 * @brief Run one tick of the initial-sequence state machine.
 *
 * State transitions:
 *
 *  Watchdog_check
 *    ├─ activate WDT every tick (HW_WDT_Enable = 1)
 *    ├─ SDC feedback == 1: WDT pulse confirmed working → advance to Pressure_check
 *    └─ timeout (>= INITIAL_SEQ_SDC_TIMEOUT_MS) without SDC feedback
 *         → Error_state
 *
 *  Pressure_check
 *    ├─ front AND rear pneumatic pressure ≥ INITIAL_SEQ_MIN_PNEUMATIC_KPA
 *    │    → advance to HV_activation
 *    └─ pressure insufficient
 *         → Error_state
 *
 *  HV_activation
 *    ├─ request ignition (Ignition_Request = 1) every tick
 *    ├─ ignition confirmed (ignition_status == 1)
 *    │    → advance to Pressure_correlation_check
 *    └─ timeout (> INITIAL_SEQ_HV_TIMEOUT_MS) without HV coming up
 *         → Error_state
 *
 *  Pressure_correlation_check
 *    ├─ |front_pneumatic − rear_pneumatic| ≤ INITIAL_SEQ_PRESSURE_CORR_TOL_KPA
 *    │    → sequence_complete = 1  (caller advances Autonomous_state)
 *    └─ pressures out of correlation
 *         → Error_state
 *
 *  Error_state
 *    → set vehicle_state = EMERGENCY
 */
void initial_sequence(initial_seq_ctx_t *ctx,
                      const initial_seq_inputs_t *inputs,
                      initial_seq_outputs_t *outputs)
{
    switch (ctx->state) {

    case Watchdog_check:
        /* Activate (or keep active) the hardware watchdog every tick. */
        outputs->HW_WDT_Enable = 1;
        if (inputs->SDC_feedback == 1) {
            /* SDC closed: WDT pulse confirmed working. Advance without
             * disabling the watchdog — it must remain active throughout. */
            ctx->state = Pressure_check;
            ctx->state_entry_time_ms = inputs->timestamp_ms;
        } else if ((inputs->timestamp_ms - ctx->state_entry_time_ms) >=
                   INITIAL_SEQ_SDC_TIMEOUT_MS) {
            /* SDC did not close within the allowed window: WDT not working. */
            ctx->state = Error_state;
        }
        break;

    case Pressure_check:
        if (inputs->front_pneumatic_kPa >= INITIAL_SEQ_MIN_PNEUMATIC_KPA &&
            inputs->rear_pneumatic_kPa  >= INITIAL_SEQ_MIN_PNEUMATIC_KPA) {
            /* Sufficient brake pressure: safe to activate HV. */
            ctx->state = HV_activation;
            ctx->state_entry_time_ms = inputs->timestamp_ms;
        } else {
            /* Insufficient pressure: cannot proceed safely. */
            ctx->state = Error_state;
        }
        break;

    case HV_activation:
        /* Assert ignition request every tick until confirmed or timed out. */
        outputs->Ignition_Request = 1;
        if (inputs->ignition_status == 1) {
            /* HV is now active. */
            ctx->state = Pressure_correlation_check;
            ctx->state_entry_time_ms = inputs->timestamp_ms;
        } else if ((inputs->timestamp_ms - ctx->state_entry_time_ms) >=
                   INITIAL_SEQ_HV_TIMEOUT_MS) {
            /* HV did not come up within the allowed window. */
            ctx->state = Error_state;
        }
        break;

    case Pressure_correlation_check: {
        float diff = inputs->front_pneumatic_kPa - inputs->rear_pneumatic_kPa;
        if (diff < 0.0f) {
            diff = -diff;
        }
        if (diff <= INITIAL_SEQ_PRESSURE_CORR_TOL_KPA) {
            /* Front and rear pressures are correlated: sequence complete. */
            outputs->sequence_complete = 1;
        } else {
            /* Pressure mismatch: unsafe to continue. */
            ctx->state = Error_state;
        }
        break;
    }

    case Error_state:
        outputs->vehicle_state = EMERGENCY;
        break;

    default:
        outputs->vehicle_state = EMERGENCY;
        break;
    }
}

void continuous_monitoring(void)
{
}
