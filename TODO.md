# ACU V3.0 — State Machine Completeness TODO

## Architecture: 5 State Machine Layers

| Layer | Type | Variable | Location |
|-------|------|----------|----------|
| **Vehicle SM** | `Main_state_machine_t` | `Vehicle_state_machine` | `Core/Src/state_machine.c:57` |
| **Auto System SM** | `Autonomous_System_states_t` | `Autonomous_state` | `Core/Src/state_machine.c:4` |
| **Startup Sequence** | `startup_sequence_state_t` | `startup_sequence_state` | `Core/Src/Autonomous_functions.c:29` |
| **BLE Config SM** | `BLE_STATE_MACHINE_t` | `ble_state` (local static) | `Core/Src/ble_handler.c:28` |
| **AS_STATE** (Jetson CAN) | `AS_STATE_t` | `t24.Autonomous_State` | `Core/Src/APP.c:199` |

---

## 🚨 CRITICAL — Priority 0: Safety

| # | Item | File:Line | Problem | Status |
|---|------|-----------|---------|--------|
| C1 | `continuous_monitoring()` wired into `Monitor_sequence` | `state_machine.c:12-14` | SDC monitoring, CAN timeouts, pressure range, and hydraulic correlation now run on every tick during autonomous driving. | ✅ FIXED |
| C2 | Solenoid requests don't reach GPIO | Multiple | Now unified: `initial_sequence()`, `APP.c`, `main.c` CAN tx all write `front_solenoid`/`rear_solenoid`. Duplicate fields removed. | ✅ FIXED |
| C3 | `ASSI_control()` return value discarded | `APP.c:114` | **Now assigned:** `ASSI_leds_control_signal = ASSI_control(...)`. | ✅ FIXED |
| C4 | Variable definition in header | `state_machine.h:18` | `BLE_STATE_MACHINE_t ble_state = BLE_IDLE;` is a **definition** in a header. Causes linker error if included by >1 translation unit. Change to `extern` and define in one `.c`. | ❌ **NOT FIXED** |
| C5 | `Start` state unreachable | `APP.c:71` | **Now fixed:** `app_init()` initializes `Vehicle_state_machine = Start`. First `Handle_state()` call executes `Start` case (enables WDT), then transitions to `IDLE`. | ✅ **FIXED** |

---

## 🔴 HIGH Priority

| # | Item | File:Line | Problem | Status |
|---|------|-----------|---------|--------|
| H1 | `continuous_monitoring()` called in `Monitor_sequence` | `state_machine.c:12-14` | Real-time safety checks (SDC, CAN timeout, pressure, correlation) now run during autonomous driving. | ✅ FIXED |
| H2 | Solenoid actuation path unified | Multiple | All code now uses `front_solenoid`/`rear_solenoid`. Duplicate fields removed. | ✅ FIXED |
| H3 | Wire `ASSI_control()` return to caller | `APP.c:114` | `ASSI_leds_control_signal = ASSI_control(ASSI_leds_control_signal, t24.ASSI_state);` | ✅ FIXED |
| H4 | Fix `ASSI_control()` pass-by-value | `Autonomous_functions.c:207` | Function now returns the LED state mask, caller assigns it. | ✅ FIXED |
| H5 | Fix `ASSI_control()` bitwise logic | `hardware_abstraction.c:60-61` | **Still broken:** `ASSI_leds_control_signal && 0b00000010` uses logical `&&` not bitwise `&`. Result: LEDs never illuminate properly. Fix: change `&&` to `&`. | ❌ **NOT FIXED** |
| H6 | Add solenoid state to acquisition | `hardware_abstraction.c:36` | `Peripheral_aquisition()` never reads back actual solenoid GPIO state. `front_solenoid`/`rear_solenoid` are write-only. | ❌ **NOT FIXED** |

---

## 🟡 MEDIUM Priority

| # | Item | File:Line | Problem | Status |
|---|------|-----------|---------|--------|
| M1 | `DIR_ACTUATOR_LAST_TX` never updated | `APP.c:182-210` | **Now handled:** Frame `AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID` updates `DIR_ACTUATOR_LAST_TX`. | ✅ FIXED |
| M2 | Same timeout for all CAN sources | `Autonomous_functions.h:24` | `MAX_TIMEOUT 1000ms` for VCU, Jetson, pressure. Should use per-signal cycle times from DBC. | ❌ **NOT FIXED** |
| M3 | `continuous_monitoring()` declaration-definition mismatch | `.h:32` vs `.c:152` vs `state_machine.c:14` | **Now fixed:** Removed stale `struct can_timeouts *last_message_from` param from `.c` definition and `NULL` arg from caller. All three sites now agree on 5-parameter signature. | ✅ **FIXED** |
| M4 | `emergency_blame()` no return value | `Autonomous_functions.c:283-285` | Returns `uint32_t` but function body is **empty** — no `return` statement. UB. | ❌ **NOT FIXED** |
| M5 | `Start` state logic vestigial | `state_machine.c:58` | **Now fixed by C5:** `app_init()` sets `Vehicle_state_machine = Start`, so the `Start` case now executes (enables WDT) before transitioning to `IDLE`. | ✅ **FIXED** |
| M6 | CAN TX queue bounds | `hardware_abstraction.c:84` | `add_can_message()` does `can_queue_index++` without checking `>=64`. Buffer overrun risk. | ❌ **NOT FIXED** |
| M7 | `Handle_Emergency()` now uses unified fields | Multiple | `initial_sequence()` now also writes `front_solenoid`/`rear_solenoid`. Duplicate fields removed. | ✅ FIXED |
| M8 | `AS_ON` re-entry deadlock | `state_machine.c:70` | `as_on_first_time` prevents re-running startup. After EMERGENCY→IDLE→AS_ON, `Autonomous_state` is still `OFF` → default case → EMERGENCY again. May be intentional but undocumented. | ❌ **NOT FIXED** |
| M9 | No debounce on mission mismatch | `state_machine.c:17` | Single-sample `t24.Current_Mission != t24.Jetson_mission` immediately triggers EMERGENCY without hysteresis. | ❌ **NOT FIXED** |
| M10 | `Autonomous_functions.c` includes `hardware_abstraction.h` | `Autonomous_functions.c:9` | Include IS used — provides `millis()`, `check_timeout()`, etc. No issue. | ✅ FIXED (false alarm) |
| S1 | Mission selection blocked before HV activation | `main.c:349` + `hardware_abstraction.c:46` | `mission_selector_enable = !t24.ASMS` already blocks mission changes once ASMS=1 (required to enter AS_ON). The EXTI callback checks `mission_selector_enable == 1`. No additional guard needed. | ✅ FIXED (design is correct) |

---

## 🟢 LOW Priority — Enhancements

| # | Item | File:Line | Problem | Status |
|---|------|-----------|---------|--------|
| L1 | BLE EEPROM flush is placeholder | `ble_handler.c:151-177` | `flush_one_record()` still prints "not yet implemented" | ❌ **NOT FIXED** |
| L2 | Missing CAN frame decoders | `APP.c:182-210` | `0x710` (AQT1), `0x720` (AQT2), `0x730` (AQT3), `0x740` (AQT4), `0x500` (DV_dynamics_1), `0x502` (DV_status), `0x81` (VCU_HV), `0x505` (SLAM_STATS) defined in DBC but not decoded | ❌ **NOT FIXED** |
| L3 | `module_timeout()` ignores RES timeout | `Autonomous_functions.c:279` | **Now checks** `t24.RES_LAST_TX` — returns `RES_TIMEOUT` if stale. | ✅ FIXED |
| L4 | TX ring-buffer pop does actual CAN send | `ring_buffer.c:42` | Unusual design — popping from TX buffer transmits CAN. Not used (uses `handle_can_tx` instead). Dual code paths for TX. | ❌ **NOT FIXED** |
| L5 | `handle_uart_logs()` commented out | `APP.c:112` | UART logging disabled | ❌ **NOT FIXED** |
| L6 | EEPROM logger never initialized | `APP.c` | `eeprom_log_init()` exists in logger.c but never called from `app_init()` | ❌ **NOT FIXED** |
| L7 | No tests for Monitor/Handle/Emergency | `tests/` | `test_initial_sequence.c` covers startup sequence only. Missing: `Monitor_sequence`, `Handle_state`, `Handle_Emergency`, `continuous_monitoring`, BLE handler, CAN decoding tests | ❌ **NOT FIXED** |
| L8 | `ble_telemetry_packet_t` endianness | `main.h:40-51` | `__attribute__((packed))` with LE multibyte fields — receiver must know endianness | ❌ **NOT FIXED** |
| L9 | `Peripheral_actuation` writes LEDs as side-effect | `hardware_abstraction.c:49-62` | Mixes solenoid control and LED control in one function. Consider separating. | ❌ **NOT FIXED** |
| L10 | `Autonomous_functions.c` uses `#if SKIP_*` macros | `Autonomous_functions.h:16-22` | Debug skip flags compiled in — potential safety issue if accidentally set to 1 in production | ❌ **NOT FIXED** |

---

## State Machine Transition Table (current implementation)

```
Vehicle SM                Autonomous SM               Startup Sequence
───────────               ─────────────               ─────────────────
Start ──→ IDLE            OFF                         WDT_TOGGLE_CHECK
         │                                            └─ SDC=0 → WDT_STP_TOGGLE_CHECK
         │ (ASMS rising                                └─ SDC=1 → PNEUMATIC_CHECK
         │  & ign_pin=0)                               └─ timeout 5s → SEQUENCE_ERROR
         ↓
       AS_ON ──→ Initial_Sequence                     WDT_STP_TOGGLE_CHECK
         │         │                                   └─ pneum 6-10bar → PRESSURE_CHECK1
         │         │ (AS_STATE_READY)                  └─ fail → SEQUENCE_ERROR
         │         ↓
         │    Monitor_sequence                         PRESSURE_CHECK1
         │         │                                   └─ hyd correlated → HV_ACTIVATION
         │    ┌────┴──────┐                            └─ fail → SEQUENCE_ERROR
         │    │          │
         │  mission   AS_STATE==                        HV_ACTIVATION
         │  mismatch  Finish                            └─ ign_auto==1 → PRESSURE_CHECK_FRONT
         │    │          │
         │    ↓          ↓                             PRESSURE_CHECK_FRONT
         │  EMERGENCY  Finish                           └─ front_solenoid=1, rear_solenoid=0
         │               │                              └─ front corr & rear unloaded
         │               │ (rpm<=10                     │  & >1s → PRESSURE_CHECK_REAR
         │               │  & ASMS=0)                   └─ timeout 5s → SEQUENCE_ERROR
         │               ↓
         │             IDLE                             PRESSURE_CHECK_REAR
         │                                              └─ front_solenoid=0, rear_solenoid=1
         ↓                                              └─ rear corr & front unloaded
       EMERGENCY                                        │  & >1s → PRESSURE_CHECK2
         │                                              └─ timeout 5s → SEQUENCE_ERROR
         │ (ASMS=0 & rpm<10)
         └─→ IDLE                                      PRESSURE_CHECK2
                                                         └─ front_solenoid=0, rear_solenoid=0
                                                         └─ both correlated → AS_STATE_READY
                                                         └─ timeout 5s → SEQUENCE_ERROR
                                                        
                                                         SEQUENCE_ERROR → Vehicle = EMERGENCY
```

---

## State of tests

| Test | File | Status | Notes |
|------|------|--------|-------|
| `test_normal_startup` | `tests/test_initial_sequence.c` | Passing | Walks all 8 stages WDT→READY |
| `test_wdt_timeout` | same | Passing | SDC open >5s → EMERGENCY |
| `test_pneumatic_out_of_range` | same | Passing | Pressure >10bar → EMERGENCY |
| `test_solenoid_front_timeout` | same | Passing | Rear not unloaded >5s → EMERGENCY |
| `test_handle_state_transition` | same | Passing | Initial_Sequence → Monitor_sequence |

**Missing tests:** Monitor_sequence, Handle_state, Handle_Emergency, continuous_monitoring, BLE handler, CAN decode, EMERGENCY→IDLE recovery.
