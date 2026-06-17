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

| # | Item | File:Line | Problem |
|---|------|-----------|---------|
| C1 | ✅ `continuous_monitoring()` wired into `Monitor_sequence` — FIXED | `state_machine.c:12-14` now calls `continuous_monitoring()` at the start of `Monitor_sequence`. SDC monitoring, CAN timeouts, pressure range, and hydraulic correlation now run on every tick during autonomous driving. Also fixed rear-gain bug on `Autonomous_functions.c:199` — was using `EBS_FRONT_HYD_GAIN` instead of `EBS_REAR_HYD_GAIN_FINAL`. |
| C2 | ✅ Solenoid requests don't reach GPIO — FIXED | Now unified: `initial_sequence()`, `APP.c`, `main.c` CAN tx all write `front_solenoid`/`rear_solenoid`. `Solenoid1_Request`/`Solenoid2_Request` removed from `struct car`. |
| C3 | `ASSI_control()` return value discarded | `APP.c:114` | Called as `ASSI_control(ASSI_leds_control_signal, ...)` but return value not assigned. ASSI LEDs never flash during Driving/Emergency. Fix: `ASSI_leds_control_signal = ASSI_control(...)`. |
| C4 | Variable definition in header | `state_machine.h:18` | `BLE_STATE_MACHINE_t ble_state = BLE_IDLE;` is a *definition* in a header. Causes linker error if included by >1 translation unit. Change to `extern` and define in one `.c`. |
| C5 | `Start` state unreachable | `APP.c:71` + `state_machine.c:58` | `app_init()` sets `Vehicle_state_machine = IDLE`, not `Start`. The `Start→IDLE` transition (enables WDT) never executes. Either init to `Start` or move WDT enable to `app_init()`. |

---

## 🔴 HIGH Priority

| # | Item | File:Line | Problem |
|---|------|-----------|---------|
| H1 | ✅ `continuous_monitoring()` called in `Monitor_sequence` — FIXED | `state_machine.c:12-14`. Real-time safety checks (SDC, CAN timeout, pressure, correlation) now run during autonomous driving. |
| H2 | ✅ Solenoid actuation path unified — FIXED | All code now uses `front_solenoid`/`rear_solenoid`. Duplicate `Solenoid1_Request`/`Solenoid2_Request` fields removed. |
| H3 | Wire `ASSI_control()` return to caller | `APP.c:114` | `ASSI_leds_control_signal = ASSI_control(ASSI_leds_control_signal, t24.ASSI_state);` |
| H4 | Fix `ASSI_control()` pass-by-value | `Autonomous_functions.c:205` | Function takes `uint8_t gpio_state` by value, modifies local copy. Pass pointer or use return value (see H3). |
| H5 | Fix `ASSI_control()` bitwise logic | `Autonomous_functions.c:239-248` | `gpio_state ^= 1; gpio_state &= 0b00000001;` strips all bits. The function should return the LED state mask that `Peripheral_actuation` then applies to GPIO. Also `Peripheral_actuation.c:56-57` uses logical `&&` not bitwise `&`: `ASSI_leds_control_signal && 0b01` → should be `&`. |
| H6 | Add solenoid state to acquisition | `hardware_abstraction.c:36` | If using separate fields, need to read back actual solenoid state or ensure consistency |

---

## 🟡 MEDIUM Priority

| # | Item | File:Line | Problem |
|---|------|-----------|---------|
| S1 |✅ **Lock mission selection before HV activation** | `main.c:344-351` | `MS_BTN` EXTI callback changes `t24.Current_Mission` at any time — even during startup sequence or autonomous driving. Mission must only be selectable BEFORE the `HV_ACTIVATION` step. After ignition-on, mission is fixed. **Suggested implementation (minimal change):** In `HAL_GPIO_EXTI_Callback` at `main.c:344`, add a guard: `if (startup_sequence_state >= HV_ACTIVATION && startup_sequence_state <= PRESSURE_CHECK2) return;` to block changes during the startup sequence once ignition is active. Additionally, in `app()` at `APP.c:107`, after `Peripheral_aquisition`, add: `if (Vehicle_state_machine != IDLE && Vehicle_state_machine != AS_ON) return;` before the mission change path. This ensures mission can only be cycled before the car enters autonomous operation. The `HAL_GPIO_EXTI_Callback` approach alone is insufficient because it's an interrupt — better to use a flag set in the ISR and process it in `app()`: `volatile uint8_t mission_btn_pending;` set in ISR, checked in `app()` loop only when `Vehicle_state_machine == IDLE`. |

| # | Item | File:Line | Problem |
|---|------|-----------|---------|
| M1 | `DIR_ACTUATOR_LAST_TX` never updated | `APP.c:182-210` | `module_timeout()` checks it but no CAN ID handler sets it. Need to handle CubeMars feedback frames (`0x2968`). |
| M2 | Same timeout for all CAN sources | `Autonomous_functions.h:24` | `MAX_TIMEOUT 1000ms` for VCU, Jetson, pressure. Should use per-signal cycle times from DBC. |
| M3 | `can_timeouts` struct unused | `main.h:240-244` | Declared but never instantiated or used. `module_timeout()` uses `t24.*_LAST_TX` fields directly. |
| M4 | `emergency_blame()` no return value | `Autonomous_functions.c:280` | Returns `uint32_t` but function body is empty — no `return` statement. |
| M5 | `Start` state logic vestigial | `state_machine.c:58` | Either init to `Start`, or remove `Start` case and enable WDT in `app_init()` directly. |
| M6 | CAN TX queue bounds | `hardware_abstraction.c:80` | `add_can_message()` does `can_queue_index++` without checking `>=64`. |
| M7 | ✅ `Handle_Emergency()` now uses unified fields — FIXED | `initial_sequence()` now also writes `front_solenoid`/`rear_solenoid`, so the existing clears in `Handle_Emergency()` and `Finish` case are correct. Duplicate `Solenoid1_Request`/`Solenoid2_Request` removed. |
| M8 | `AS_ON` re-entry deadlock | `state_machine.c:70` | `as_on_first_time` prevents re-running startup. After EMERGENCY→IDLE→AS_ON, `Autonomous_state` is still `OFF` → default case → EMERGENCY again. May be intentional. |
| M9 | No debounce on mission mismatch | `state_machine.c:12` | Single-sample `t24.Current_Mission != t24.Jetson_mission` immediately triggers EMERGENCY without hysteresis. |
| M10 | `Autonomous_functions.c` includes `hardware_abstraction.h` but doesn't use it | `Autonomous_functions.c:9` | Unnecessary include |

---

## 🟢 LOW Priority — Enhancements

| # | Item | File:Line | Problem |
|---|------|-----------|---------|
| L1 | BLE EEPROM flush is placeholder | `ble_handler.c:151-177` | `flush_one_record()` only prints "not yet implemented" |
| L2 | Missing CAN frame decoders | `APP.c:182-210` | `0x710` (AQT1), `0x720` (AQT2), `0x730` (AQT3), `0x740` (AQT4), `0x500` (DV_dynamics_1), `0x502` (DV_status), `0x81` (VCU_HV), `0x505` (SLAM_STATS) defined in DBC but not decoded |
| L3 | `module_timeout()` ignores RES timeout | `Autonomous_functions.c:269-277` | `can_timeouts` struct has `res` field but `module_timeout()` doesn't check it |
| L4 | TX ring-buffer pop does actual CAN send | `ring_buffer.c:42` | Unusual design — popping from TX buffer transmits CAN. Not used (uses `handle_can_tx` instead). Dual code paths for TX. |
| L5 | `handle_uart_logs()` commented out | `APP.c:112` | UART logging disabled |
| L6 | EEPROM logger never initialized | `APP.c` | `eeprom_log_init()` exists in logger.c but never called from `app_init()` |
| L7 | No tests for Monitor/Handle/Emergency | `tests/` | `test_initial_sequence.c` covers startup sequence only. Missing: `Monitor_sequence`, `Handle_state`, `Handle_Emergency`, `continuous_monitoring`, BLE handler, CAN decoding tests |
| L8 | `ble_telemetry_packet_t` endianness | `main.h:40-51` | `__attribute__((packed))` with LE multibyte fields — receiver must know endianness |
| L9 | `Peripheral_actuation` writes LEDs as side-effect | `hardware_abstraction.c:55-57` | Mixes solenoid control and LED control in one function. Consider separating. |
| L10 | `Autonomous_functions.c` uses `#if SKIP_*` macros | `Autonomous_functions.h:15-22` | Debug skip flags compiled in — potential safety issue if accidentally set to 1 in production |

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
