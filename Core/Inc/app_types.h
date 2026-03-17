/*
 * app_types.h
 *
 *  Created on: Mar 17, 2026
 *      Author: bruno
 *
 *  Application-level type definitions with no hardware (HAL) dependencies.
 *  This file can be included by both firmware code and host-side unit tests.
 */

#ifndef INC_APP_TYPES_H_
#define INC_APP_TYPES_H_

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

/* ── Enumerations ────────────────────────────────────────────────────────── */

typedef enum
{
  AS_STATE_OFF      = 1,
  AS_STATE_READY    = 2,
  AS_STATE_DRIVING  = 3,
  AS_STATE_EMERGENCY= 4,
  AS_STATE_FINISHED = 5
} AS_STATE_t;

typedef enum
{
  MANUAL,
  ACCELERATION,
  SKIDPAD,
  TRACKDRIVE,
  EBS_TEST,
  INSPECTION,
  AUTOCROSS
} current_mission_t;

typedef enum {
  Start,
  IDLE,
  AS_ON,
  EMERGENCY
} Main_state_machine_t;

typedef enum {
  OFF,
  Initial_Sequence,
  Monitor_sequence,
  Finish,
  AS_Emergency
} Autonomous_System_states_t;

typedef enum {
  Watchdog_check,
  Pressure_check,
  HV_activation,
  Pressure_correlation_check,
  Error_state
} startup_sequence_state_t;

/* ── Vehicle data structures ─────────────────────────────────────────────── */

struct pressure {
  float Pneumatic;
  float Hydraulic;
};

struct speed {
  uint8_t Speed;         /* km/h */
  uint8_t Target_Speed;  /* km/h */
};

struct car {
  struct pressure Rear_Pressure;
  struct pressure Front_Pressure;
  uint8_t Ignition_Status;          /* 0 = OFF, 1 = ON  (real, from VCU) */
  uint8_t Ignition_Request;         /* 0 = OFF, 1 = ON  (request from ACU) */
  uint8_t ASMS;                     /* 0 = OFF, 1 = ON */
  uint8_t Emergency;                /* 0 = No,  1 = Emergency */
  uint8_t Res;                      /* 0 = not active, 1 = ON, 2 = Emergency */
  uint8_t HW_WDT_Enable;
  volatile AS_STATE_t Autonomous_State;
  volatile current_mission_t Current_Mission;
  struct speed Speed;
  uint8_t SDC_feedback;
  float chip_temp;
};

/* ── Initial-sequence context and I/O types ──────────────────────────────── */

/**
 * @brief Persistent context for the initial-sequence state machine.
 *
 * Must be zero-initialised before the first call (state = Watchdog_check,
 * state_entry_time_ms = 0).  The caller owns this struct and passes it by
 * pointer on every tick.
 */
typedef struct {
  startup_sequence_state_t state;     /**< Current sub-state. */
  uint32_t state_entry_time_ms;       /**< Timestamp (ms) when state was entered. */
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
  uint8_t               HW_WDT_Enable;    /**< 1 = keep toggling WDT. */
  uint8_t               Ignition_Request; /**< 1 = request HV activation. */
  Main_state_machine_t  vehicle_state;    /**< May be set to EMERGENCY. */
  uint8_t               sequence_complete;/**< 1 = initial sequence passed. */
} initial_seq_outputs_t;

#endif /* INC_APP_TYPES_H_ */
