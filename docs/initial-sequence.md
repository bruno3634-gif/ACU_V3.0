# Initial Sequence — State Machine Reference

The **Initial Sequence** is the safety-critical startup check the ACU must pass before an autonomous mission is allowed to begin. It runs as a sub-state of the `AS_ON` vehicle state and is ticked on every iteration of the main loop.

---

## Purpose

Before the car is allowed to drive autonomously, the ACU must verify that:

1. The **hardware watchdog is alive** (confirmed by the Shutdown Circuit closing).
2. The **pneumatic brake pressure** is sufficient to stop the car.
3. The **High Voltage system** is active (confirmed by the VCU).
4. Front and rear brake pressures are **correlated** (sensors agree — no sensor fault).

Only when all four checks pass in order does the ACU set `sequence_complete = 1` and allow the autonomous mission to proceed.

---

## State diagram

```
                      ┌──────────────────────────────────────────────────────────┐
                      │  Error_state                                              │
                      │  • sets vehicle_state = EMERGENCY                         │
                      └──────────────────────────────────────────────────────────┘
                            ▲           ▲              ▲              ▲
                         timeout    low pressure    timeout       out of tol.
                         500 ms                    5 000 ms
                            │           │              │              │
┌──────────────┐    SDC=1   │  P≥500kPa │  ign_status  │  |Δp|≤100   │
│Watchdog_check├───────────►│Pressure_check├───────────►│HV_activation├───────────►│Pressure_correlation_check│
│HW_WDT_En = 1│            │(front+rear)│              │Ign_Req = 1  │              │    → sequence_complete=1  │
└──────────────┘            └────────────┘              └─────────────┘              └───────────────────────────┘
```

---

## States in detail

### 1 · `Watchdog_check`

**Goal:** prove the hardware watchdog circuit is alive by observing the Shutdown Circuit (SDC) close in response to the WDT pulses.

| | |
|---|---|
| **Entry action** | `HW_WDT_Enable = 1` every tick — keeps the WDT pulse running |
| **Advance condition** | `SDC_feedback == 1` (SDC has closed, confirming WDT is pulsing) |
| **Fail condition** | `SDC_feedback` still 0 after **500 ms** |
| **On advance** | Move to `Pressure_check`; WDT stays active |
| **On fail** | Move to `Error_state` |

**How it works on hardware:**  
`toggle_wdt()` toggles `WDT_PULSE` (PC0) every 10 ms while `HW_WDT_Enable == 1`.  
The external watchdog circuit only closes the SDC if it sees a valid pulse within its window.  
`SDC_FEEDBACK` (PC1) is read to confirm the SDC is closed.

---

### 2 · `Pressure_check`

**Goal:** verify both front and rear pneumatic brake circuits have enough pressure to stop the car before HV is enabled.

| | |
|---|---|
| **Advance condition** | `front_pneumatic_kPa ≥ 500 kPa` **AND** `rear_pneumatic_kPa ≥ 500 kPa` |
| **Fail condition** | Either sensor reads below 500 kPa |
| **On advance** | Move to `HV_activation` |
| **On fail** | Move to `Error_state` immediately (no timeout — pressure must already be present) |

> **Threshold:** `INITIAL_SEQ_MIN_PNEUMATIC_KPA = 500.0 kPa` (defined in `app_types.h`)

---

### 3 · `HV_activation`

**Goal:** request and confirm that the High Voltage (accumulator) system is active, as reported by the VCU over CAN.

| | |
|---|---|
| **Repeated action** | `Ignition_Request = 1` every tick — keeps requesting HV from VCU |
| **Advance condition** | `ignition_status == 1` (VCU confirms HV is active) |
| **Fail condition** | `ignition_status` still 0 after **5 000 ms** |
| **On advance** | Move to `Pressure_correlation_check` |
| **On fail** | Move to `Error_state` |

> **Timeout:** `INITIAL_SEQ_HV_TIMEOUT_MS = 5000 ms` (defined in `app_types.h`)

---

### 4 · `Pressure_correlation_check`

**Goal:** cross-check front and rear pneumatic pressure sensors against each other. A large difference indicates a sensor fault or a ruptured circuit.

| | |
|---|---|
| **Advance condition** | `|front_pneumatic_kPa − rear_pneumatic_kPa| ≤ 100 kPa` |
| **Fail condition** | Difference exceeds 100 kPa |
| **On advance** | `sequence_complete = 1` — caller (`Handle_autonomous_state`) moves to `Monitor_sequence` |
| **On fail** | Move to `Error_state` |

> **Tolerance:** `INITIAL_SEQ_PRESSURE_CORR_TOL_KPA = 100.0 kPa` (defined in `app_types.h`)

---

### 5 · `Error_state`

**Goal:** assert a system emergency and halt autonomous operation.

| | |
|---|---|
| **Action every tick** | `vehicle_state = EMERGENCY` |
| **Effect** | `Handle_Emergency()` is called: WDT stops, ignition cut, solenoids opened |

---

## Data flow

The state machine is **hardware-free** — all inputs are read by the caller before the call, and all outputs are applied by the caller after.

```
main loop
  └─ Handle_state()
       └─ Handle_autonomous_state()   [in main.c]
             │
             │  reads from hardware → initial_seq_inputs_t
             │  (SDC_feedback, pressures, ignition_status, HAL_GetTick())
             ▼
         initial_sequence()           [in Autonomous_functions.c]
             │
             │  writes → initial_seq_outputs_t
             │  (HW_WDT_Enable, Ignition_Request, vehicle_state, sequence_complete)
             ▼
          apply outputs to t24 struct & state machines
```

### Input struct (`initial_seq_inputs_t`)

| Field | Source | Meaning |
|---|---|---|
| `SDC_feedback` | `SDC_FEEDBACK_Pin` (PC1) | 1 = SDC closed (WDT confirmed) |
| `front_pneumatic_kPa` | ADC / CAN | Front pneumatic pressure |
| `rear_pneumatic_kPa` | ADC / CAN | Rear pneumatic pressure |
| `ignition_status` | CAN from VCU | 1 = HV system active |
| `timestamp_ms` | `HAL_GetTick()` | Current time for timeout checks |

### Output struct (`initial_seq_outputs_t`)

| Field | Applied to | Meaning |
|---|---|---|
| `HW_WDT_Enable` | `t24.HW_WDT_Enable` | 1 = keep toggling WDT pulse |
| `Ignition_Request` | `t24.Ignition_Request` | 1 = ask VCU to activate HV |
| `vehicle_state` | `Vehicle_state_machine` | Set to EMERGENCY on fault |
| `sequence_complete` | Checked by caller | 1 = all checks passed |

---

## Timeout summary

| State | Timeout | Constant |
|---|---|---|
| `Watchdog_check` | 500 ms | `INITIAL_SEQ_SDC_TIMEOUT_MS` |
| `Pressure_check` | None (immediate pass/fail) | — |
| `HV_activation` | 5 000 ms | `INITIAL_SEQ_HV_TIMEOUT_MS` |
| `Pressure_correlation_check` | None (immediate pass/fail) | — |

---

## Emergency path

Any failure in any state routes to `Error_state`, which sets `vehicle_state = EMERGENCY`.  
On the next `Handle_state()` tick, `Handle_Emergency()` runs:

```c
t24.HW_WDT_Enable    = 0;           // stop WDT — SDC will open, cutting power
t24.Ignition_Request = 0;           // cut HV request
HAL_GPIO_WritePin(Solenoid2_GPIO_Port, Solenoid2_Pin, GPIO_PIN_RESET);  // release brake
HAL_GPIO_WritePin(Solenoid1_GPIO_Port, Solenoid1_Pin, GPIO_PIN_RESET);  // release brake
```

Stopping the WDT causes the external watchdog circuit to open the SDC, which removes power from the drive system — this is the hardware-level failsafe.

---

## Source files

| File | Role |
|---|---|
| `Core/Src/Autonomous_functions.c` | State machine logic (hardware-free) |
| `Core/Inc/Autonomous_functions.h` | Public API declaration |
| `Core/Inc/app_types.h` | `initial_seq_ctx_t`, `initial_seq_inputs_t`, `initial_seq_outputs_t`, thresholds |
| `Core/Inc/main.h` | All application enums (`startup_sequence_state_t`, etc.) and GPIO aliases |
| `Core/Src/main.c` | `Handle_autonomous_state()` — hardware glue, calls `initial_sequence()` |
| `Tests/test_initial_sequence.c` | 18 unit tests covering every state and boundary |
