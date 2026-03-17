# ACU V3.0 — Autonomous Control Unit

Firmware for the **T24 Formula Student** Autonomous Control Unit (ACU), running on an **STM32F412RE** microcontroller.

---

## What this firmware does

The ACU is the safety-critical controller responsible for:

1. **Verifying system integrity** before the autonomous mission starts (Initial Sequence).
2. **Monitoring** sensors and system health continuously during a mission (continuous monitoring — in progress).
3. **Triggering an emergency stop** (EBS — Emergency Braking System) whenever a fault is detected.

---

## Repository layout

```
ACU_V3.0/
├── Core/
│   ├── Inc/
│   │   ├── main.h                  # Application types, GPIO pin aliases
│   │   ├── definitions.h           # Initial-sequence I/O types and thresholds
│   │   └── Autonomous_functions.h  # Public API for the state machines
│   └── Src/
│       ├── main.c                  # Main loop, state dispatchers, HAL glue
│       └── Autonomous_functions.c  # Hardware-free state machine logic
├── Tests/
│   ├── test_initial_sequence.c     # Host-side unit tests (no hardware needed)
│   ├── Makefile
│   └── README.md                   # How to build and run the tests
├── docs/
│   └── initial-sequence.md         # Detailed state machine documentation ← start here
└── README.md                       # This file
```

---

## State machines at a glance

### Main vehicle state (`Main_state_machine_t`)

```
Start → IDLE → AS_ON → (runs Autonomous sub-machine)
                ↓ on any fault
             EMERGENCY
```

### Autonomous sub-state (`Autonomous_System_states_t`)

```
OFF → Initial_Sequence → Monitor_sequence → Finish
              ↓ on any fault
          AS_Emergency → EMERGENCY
```

### Initial sequence (`startup_sequence_state_t`)

See **[docs/initial-sequence.md](docs/initial-sequence.md)** for the full description.

```
Watchdog_check → Pressure_check → HV_activation → Pressure_correlation_check
      ↓ (any failure)                                         ↓ pass
  Error_state ──────────────────────────────────────────── sequence_complete
```

---

## Key hardware signals

| Signal | MCU pin | Direction | Description |
|---|---|---|---|
| `WDT_PULSE` | PC0 | Output | Hardware watchdog pulse (toggled ≤ 10 ms) |
| `SDC_FEEDBACK` | PC1 | Input | High when Shutdown Circuit is closed |
| `ASMS` | PD2 | Input | Autonomous System Master Switch |
| `Solenoid1` | PB13 | Output | EBS solenoid 1 (LOW = open = brake released) |
| `Solenoid2` | PB12 | Output | EBS solenoid 2 (LOW = open = brake released) |
| `HB` | PC5 | Output | Heartbeat LED |
| `CAN_INDICATOR` | PB0 | Output | CAN activity LED |

---

## Building and testing

The firmware is built with **STM32CubeIDE** targeting the STM32F412RE.

Unit tests run entirely on a host Linux/macOS machine (no hardware required):

```bash
cd Tests && make
```

See [Tests/README.md](Tests/README.md) for details.

---

## Flowcharts

General state machine:

<img width="678" height="639" alt="image" src="https://github.com/user-attachments/assets/8c39159d-23c4-44f0-86d1-648b6a5f236f" />

Autonomous system:

<img width="1010" height="577" alt="image" src="https://github.com/user-attachments/assets/de70d125-3d66-4332-8db7-6795870800f2" />

