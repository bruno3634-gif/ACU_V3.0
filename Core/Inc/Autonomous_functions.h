/*
 * Autonomous_functions.h
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#ifndef INC_AUTONOMOUS_FUNCTIONS_H_
#define INC_AUTONOMOUS_FUNCTIONS_H_

#include "main.h"

/*
 * initial_sequence() — run one step of the car's startup safety check.
 *
 * Call this function once per control loop tick (e.g. every 10 ms).
 * It works through a fixed list of safety checks, one step at a time:
 *
 *   Step 1 – Watchdog_check            : is the hardware watchdog alive?
 *   Step 2 – Pressure_check            : do the brakes have enough pressure?
 *   Step 3 – HV_activation             : turn on high voltage; wait for confirmation.
 *   Step 4 – Pressure_correlation_check: do front and rear pressures match?
 *   → All passed: outputs->sequence_complete = 1
 *   → Any failure: Error_state → outputs->vehicle_state = EMERGENCY
 *
 * Arguments
 * ─────────
 *  ctx     The function's "memory" between calls.
 *          Keeps track of which step we are on and how long we have been there.
 *          Before the very first call, set:
 *              ctx->state              = Watchdog_check;
 *              ctx->state_entry_time_ms = 0;
 *
 *  inputs  Everything read from the car before this call
 *          (SDC line, brake pressures, ignition status, current time).
 *          Fill this struct from real sensors each tick, then pass it in.
 *
 *  outputs What the function wants to write to the car after this call
 *          (WDT enable signal, ignition request, vehicle state, done flag).
 *          Pre-fill with the current car state; the function only changes
 *          the fields that need updating.
 */
void initial_sequence(initial_seq_ctx_t *ctx,
                      const initial_seq_inputs_t *inputs,
                      initial_seq_outputs_t *outputs);

void continuous_monitoring(void);

#endif /* INC_AUTONOMOUS_FUNCTIONS_H_ */
