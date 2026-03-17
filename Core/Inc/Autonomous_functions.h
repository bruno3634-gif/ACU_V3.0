/*
 * Autonomous_functions.h
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#ifndef INC_AUTONOMOUS_FUNCTIONS_H_
#define INC_AUTONOMOUS_FUNCTIONS_H_

#include "app_types.h"

/**
 * @brief  Run one tick of the initial-sequence state machine.
 *
 * The function is fully hardware-agnostic: every input comes from @p inputs
 * and every output is written to @p outputs.  The caller is responsible for
 * reading hardware inputs before the call and applying the outputs afterward.
 *
 * @param[in,out] ctx     Persistent state between ticks.  Initialise to
 *                        {Watchdog_check, 0} before the first call.
 * @param[in]     inputs  Snapshot of all sensor / status values for this tick.
 * @param[in,out] outputs Result fields to be applied to the vehicle state.
 *                        Caller must pre-fill all fields with the current
 *                        system state; only changed fields are modified.
 */
void initial_sequence(initial_seq_ctx_t *ctx,
                      const initial_seq_inputs_t *inputs,
                      initial_seq_outputs_t *outputs);

void continuous_monitoring(void);

#endif /* INC_AUTONOMOUS_FUNCTIONS_H_ */
