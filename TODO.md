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
| C4 | Variable definition in header | `state_machine.h:18` | `BLE_STATE_MACHINE_t ble_state = BLE_IDLE;` was a **definition** in a header. **Now fixed:** `ble_state` is `static` in `Core/Src/ble_handler.c:31`. The enum type `BLE_STATE_MACHINE_t` remains in `main.h` but only the typedef and enum constants, no variable. | ✅ FIXED |
| C5 | `Start` state unreachable | `APP.c:71` | **Now fixed:** `app_init()` initializes `Vehicle_state_machine = Start`. First `Handle_state()` call executes `Start` case (enables WDT), then transitions to `IDLE`. | ✅ **FIXED** |

---

## 🔴 HIGH Priority

| # | Item | File:Line | Problem | Status |
|---|------|-----------|---------|--------|
| H1 | `continuous_monitoring()` called in `Monitor_sequence` | `state_machine.c:12-14` | Real-time safety checks (SDC, CAN timeout, pressure, correlation) now run during autonomous driving. | ✅ FIXED |
| H2 | Solenoid actuation path unified | Multiple | All code now uses `front_solenoid`/`rear_solenoid`. Duplicate fields removed. | ✅ FIXED |
| H3 | Wire `ASSI_control()` return to caller | `APP.c:114` | `ASSI_leds_control_signal = ASSI_control(ASSI_leds_control_signal, t24.ASSI_state);` | ✅ FIXED |
| H4 | Fix `ASSI_control()` pass-by-value | `Autonomous_functions.c:207` | Function now returns the LED state mask, caller assigns it. | ✅ FIXED |
| H5 | Fix `ASSI_control()` bitwise logic | `hardware_abstraction.c:60-61` | **Now fixed:** Code already uses bitwise `&`, not logical `&&`. Verified by grep. No change needed. | ✅ FIXED |
| H6 | Add solenoid state to acquisition | `hardware_abstraction.c:36` | `Peripheral_aquisition()` never reads back actual solenoid GPIO state. `front_solenoid`/`rear_solenoid` are write-only. | ❌ **NOT FIXED** |
| H7 | Stale statics after state machine re-entry | `state_machine.c:18-19` | `mismatch_tick` and `mismatch_active` declared as function-level statics inside `Handle_autonomous_state()` `Monitor_sequence` case. After EMERGENCY→IDLE→AS_ON re-entry, these retain previous values. If a mismatch was active before EMERGENCY, mismatch_active=1 persists, causing the debounce expiry check (`millis() - mismatch_tick >= 1000`) to fire immediately on re-entry to Monitor_sequence, potentially causing a false EMERGENCY loop. Fix: reset statics on state machine re-entry OR move to a managed structure that resets with Autonomous_state. | ❌ NOT FIXED |

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
| M8 | `AS_ON` re-entry deadlock | `state_machine.c:70` | `as_on_first_time` static properly handles re-entry. Trace: EMERGENCY→IDLE (as_on_first_time reset to 0 at line 81)→AS_ON (if !as_on_first_time true, re-inits Autonomous_state=Initial_Sequence and startup_sequence_state=WDT_TOGGLE_CHECK, then sets as_on_first_time=1)→Handle_autonomous_state runs Initial_Sequence. `Autonomous_state` is NEVER left at `OFF` during re-entry. The concern was a false alarm. | ✅ FIXED |
| M9 | No debounce on mission mismatch | `state_machine.c:18-29` | **Now implemented:** Debounce IS implemented with static `mismatch_tick` and `mismatch_active`. Requires 1 second of sustained mismatch before triggering EMERGENCY: `if (!mismatch_active) { mismatch_active=1; mismatch_tick=millis(); } else if (millis()-mismatch_tick >= 1000) { Vehicle_state_machine=EMERGENCY; }`. | ✅ FIXED |
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
| State machine + dbc_decode (12 tests) | `tests/test_state_machine.c` | PASS | Covers Handle_state, Handle_autonomous_state, mission mismatch debounce, dbc_decode for 4 CAN frames, EMERGENCY recovery, Finish transition |
| Initial sequence (5 tests) | `tests/test_initial_sequence.c` | PASS | Covers all 8 startup stages (WDT→READY) including normal, timeout, out-of-range, solenoid timeout |
| Autonomous functions (24 assertions) | `tests/test_autonomous_functions.c` | PASS | Covers check_timeout, ASSI_control (all 5 states), module_timeout (all 5 modules), continuous_monitoring (4 scenarios) |
| BLE config FSM (8 tests) | `tests/test_ble_config.c` | PASS | Covers RN4871 config command sequence (SN, SR, R,1) with done state 5 |
| ADC callbacks (19 assertions) | `tests/test_adc_callbacks.c` | PASS | |
| CAN queue (52 assertions) | `tests/test_can_queue.c` | PASS | |
| EMA filter (5 assertions) | `tests/test_ema_filter.c` | PASS | |
| Logger (154 assertions) | `tests/test_logger.c` | PASS | |
| Ring buffer (549 assertions) | `tests/test_ring_buffer.c` | PASS | |
| T26 CAN pack/unpack (9 assertions) | `tests/test_autonomous_t26.c` | PASS | |
| Temperature conversion (5 assertions) | `tests/test_temperature.c` | PASS | |
| TIM callback (50 assertions) | `tests/test_tim_callback.c` | PASS | |
| UART callbacks (16 assertions) | `tests/test_uart_callbacks.c` | PASS | |

**Total: 13/13 test suites pass (2026-06-19)**

**Not tested but testable without HW:**
- `ble_handler.c` — BLE FSM (BLE_IDLE → BLE_WAIT_CONFIG → BLE_BRIDGE), telemetry formatting, DMA TX busy flag behavior. 
- `toggle_wdt()` — simple 10ms WDT toggle timing, no dedicated test exists.
- `emergency_blame()` — empty function body, undefined behavior if called, not tested anywhere.
- `Handle_Emergency()` — production version has HAL_GPIO_WritePin calls that must be stubbed. Test should verify post-conditions (solenoids=0, Emergency=1, Ignition_Request=0).
- `add_can_message()` bounds check — stubs needed to verify index never exceeds 63.
- State machine re-entry edge cases — statics after EMERGENCY→IDLE→AS_ON (mismatch_tick/mismatch_active, as_on_first_time).
- Mission mismatch with simultaneous CAN timeout scenarios.
