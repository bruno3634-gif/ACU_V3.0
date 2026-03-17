/*
 * definitions.h
 *
 *  Created on: Mar 17, 2026
 *      Author: bruno
 *
 *  Types and constants for the initial-sequence state machine.
 *
 *  This file is included at the bottom of main.h (after all application enums
 *  and structs are defined) so that the initial_seq_* types below can
 *  reference Main_state_machine_t and startup_sequence_state_t.
 *
 *  For host-side unit tests the include chain is:
 *      test → Autonomous_functions.h → main.h → stm32f4xx_hal.h (stub)
 *                                             → (type defs)
 *                                             → definitions.h (this file)
 */

#ifndef INC_DEFINITIONS_H_
#define INC_DEFINITIONS_H_

#include <stdint.h>

/* ── Threshold constants for the initial sequence ───────────────────────── */

/** Maximum time (ms) to wait for SDC to close during Watchdog_check. */
#define INITIAL_SEQ_SDC_TIMEOUT_MS          500U

/** Maximum time (ms) to wait for HV (ignition) to become active. */
#define INITIAL_SEQ_HV_TIMEOUT_MS           5000U

/** Minimum pneumatic pressure (kPa) required before activating HV. */
#define INITIAL_SEQ_MIN_PNEUMATIC_KPA       500.0f

/** Maximum allowed difference (kPa) between front and rear pneumatic
 *  pressure during the correlation check. */
#define INITIAL_SEQ_PRESSURE_CORR_TOL_KPA   100.0f

/* ── Initial-sequence context and I/O types ──────────────────────────────── */

/**
 * @brief Persistent context for the initial-sequence state machine.
 *
 * Must be initialised to {Watchdog_check, 0} before the first call.
 * The caller owns this struct and passes it by pointer on every tick.
 */
typedef struct {
  startup_sequence_state_t state;      /**< Current sub-state. */
  uint32_t state_entry_time_ms;        /**< Timestamp (ms) when state was entered. */
} initial_seq_ctx_t;

/**
 * @brief Snapshot of all inputs needed by the initial-sequence state machine.
 *
 * All values are read from hardware before calling initial_sequence() and
 * passed here, keeping the function hardware-free.
 */
typedef struct {
  uint8_t  SDC_feedback;           /**< 1 = SDC (shutdown circuit) closed. */
  float    front_pneumatic_kPa;    /**< Front pneumatic brake pressure (kPa). */
  float    rear_pneumatic_kPa;     /**< Rear  pneumatic brake pressure (kPa). */
  uint8_t  ignition_status;        /**< 1 = HV active (received from VCU). */
  uint32_t timestamp_ms;           /**< Current time in milliseconds. */
} initial_seq_inputs_t;

/**
 * @brief Outputs produced by the initial-sequence state machine on each tick.
 *
 * Callers must initialise every field to the current system state before
 * calling initial_sequence(), so that unchanged fields retain their value.
 */
typedef struct {
  uint8_t               HW_WDT_Enable;     /**< 1 = keep toggling WDT. */
  uint8_t               Ignition_Request;  /**< 1 = request HV activation. */
  Main_state_machine_t  vehicle_state;     /**< May be set to EMERGENCY. */
  uint8_t               sequence_complete; /**< 1 = initial sequence passed. */
} initial_seq_outputs_t;

#endif /* INC_DEFINITIONS_H_ */
