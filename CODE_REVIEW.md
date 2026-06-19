# ACU V3.0 — Code Review

**Project:** ACU (Autonomous Control Unit) for Formula Student autonomous vehicle  
**MCU:** STM32F412RETx  
**Toolchain:** STM32CubeIDE / GCC  
**Review date:** 2026-06-16  
**Reviewer:** Automated static + manual analysis

## Summary

This is a bare-metal STM32F412 firmware for an vehicle's autonomous control unit / monitoring. The codebase implements a hierarchical state machine (Vehicle → Autonomous → Startup sub-states), CAN bus communication via a 20-frame DBC ("autonomous_t26"), BLE telemetry via an RN4871 module, EEPROM fault logging, ADC-based pressure/temperature sensing, and an external watchdog. The architecture is well-separated into hardware abstraction, application logic, and HAL initialization layers, and the inclusion of a host-based test harness for the startup sequence is a notable positive. A critical solenoid field mismatch (C1) has been fixed — `initial_sequence()`, `APP.c` init, and CAN telemetry now all write `front_solenoid`/`rear_solenoid`, the same fields that `Peripheral_actuation()` reads to drive GPIO. The duplicate `Solenoid1_Request`/`Solenoid2_Request` fields were removed from `struct car`. Remaining issues include ASSI LEDs not flashing, the monitoring state being dead code, and safety concerns around ISR-driven mission changes, duplicate symbol definitions, and missing bounds checks. This review identifies 17+ specific issues with file:line references.

---

## What's Good

- **Hierarchical state machine design:** `Handle_state` (Vehicle SM) → `Handle_autonomous_state` (Autonomous SM) → `initial_sequence` (Startup sub-SM) is a clean layering (`state_machine.c:55-88`, `state_machine.c:3-41`, `Autonomous_functions.c:29-151`). Each level has a clear responsibility.

- **Host-based test harness:** `tests/test_initial_sequence.c` is a self-contained 685-line test that compiles with `gcc` on any host PC, defines its own stubs for HAL types, and validates `initial_sequence()` through all 8 stages plus timeout/error paths. This is excellent engineering practice and was clearly non-trivial to write.

- **CAN DBC code generation via cantools:** `autonomous_t26.h`/`.c` (3655 + 2329 lines) is generated from a formal DBC specification, eliminating manual packing/unpacking errors. Frame IDs, lengths, signal choices, and encode/decode functions are all consistent and type-safe.

- **RN4871 BLE driver completeness:** `rn4871.c` wraps every documented AT command (enter/exit command mode, connect/disconnect, GATT read/write, DIS, GPIO, advertising, etc.) into a clean `snprintf`-based API (`rn4871.c:1-255`).

- **EEPROM logger design:** `logger.c:13-91` implements a ring-buffer fault logger with magic-number validation, head/tail overflow handling, clear, and indexed read. The design is robust and would work well once integrated.

- **EMA filter on pressure sensors:** `EMA_Filter.h:16-23` / `main.c:151-152` applies exponential moving average (`alpha=0.5`) to ADC pressure readings, reducing noise on the pneumatic pressure signals.

- **CAN ring buffer:** `ring_buffer.c:11-69` implements a 1024-entry circular buffer with separate push functions for TX and RX, arrival time stamping, and proper head/tail/counter management. No dynamic allocation.

- **CAN TX at fixed 10 Hz in TIM2 callback:** `main.c:259-332` packs and queues ACU, DV_STATUS, and ASF_SIGNALS frames every 100 ms. Simple, deterministic, and easy to reason about.

- **BLE configuration via $$$ command mode:** `APP.c:125-179` (`ble_module_config_start/tick`) correctly sequences `$$$` entry → send commands → wait for AOK → `---` exit. The 500 ms inter-command delays respect the RN4871's timing requirements.

- **External watchdog with analog RC:** `state_machine.c:90-100` (`toggle_wdt`) toggles a GPIO at 10 ms intervals when enabled. The hardware RC-based watchdog is a good safety design.

- **ASSI state table comment in `ASSI_control()`:** `Autonomous_functions.c:207-225` documents the AS → ASSI mapping (Off→off, Ready→yellow solid, Driving→yellow flash, Emergency→blue flash, Finished→blue solid) with reference to T 14.8.

- **Vrefint-calibrated temperature sensing:** `main.c:233-253` uses factory-calibrated VREFINT and two-point temperature calibration (TS_CAL1/TS_CAL2 at 30°C / 110°C) for accurate chip temperature measurement.

- **CAN auto-retransmission disabled:** Good for deterministic timeout behavior in a safety system (the `can.c` init presumably sets `CAN_MCR_NART`).

---

## Architecture & Structure

The architecture follows a reasonable layered pattern:

```
main.c (entry, HAL callbacks, TIM2 telemetry)
  └── APP.c (app_init, app superloop)
        ├── state_machine.c (Vehicle SM → Autonomous SM)
        │     ├── Autonomous_functions.c (startup sequence, ASSI, timeouts)
        │     └── hardware_abstraction.c (I/O, CAN TX, LEDs)
        ├── ble_handler.c (BLE CTS + bridge mode)
        └── ring_buffer.c → dbc_decode (CAN RX)
```

The separation of HAL init (`can.c`, `adc.c`, `i2c.c`, etc.) from application logic is good. The superloop in `APP.c:105-122` is well-structured: aquire → handle state → toggle WDT → LEDs → actuate → CAN TX → CAN RX → decode.

However, there are structural concerns:

1. **Two competing BLE configuration paths:** `APP.c:125-179` configures the RN4871 via a simple command sequence, while `ble_handler.c:195-325` has a full CTS time-sync FSM. It is unclear which one actually runs — `APP.c:100` calls `ble_module_config_start()` but `ble_handler_init()` is never called from `app_init()`. The `ble_handler.c` code appears to be dead/unreachable unless explicitly invoked.

2. **`hardware_abstaction.c` filename typo:** The file is named `hardware_abstaction.c` (missing 'r' — should be `hardware_abstraction.c`). The header is correctly named `hardware_abstraction.h`. This mismatch is confusing.

3. **Global variable sprawl:** `struct car t24` is declared in `APP.c:12` as a plain global (not `extern` in a header) and accessed across 6+ translation units via `extern struct car t24;` declarations. This works but makes dependency tracking difficult. Consider a proper singleton accessor.

---

## Critical Issues

### C1. ✅ Solenoids never actuate (field mismatch) — FIXED

**What was wrong:** `initial_sequence()` wrote `Solenoid1_Request`/`Solenoid2_Request` but `Peripheral_actuation()` read `front_solenoid`/`rear_solenoid` — different `struct car` fields. The values set during the startup sequence never reached the GPIO pins.

**Fix applied:** Consolidated onto `front_solenoid`/`rear_solenoid`. Changed:
- `Autonomous_functions.c:93-94, 111-112, 129-130` — `Solenoid1_Request`/`Solenoid2_Request` → `front_solenoid`/`rear_solenoid`
- `main.c:323-324` — CAN telemetry reads `t24.front_solenoid`/`t24.rear_solenoid`
- `APP.c:60-61` — initialization uses `t24.front_solenoid`/`t24.rear_solenoid`
- `tests/test_initial_sequence.c` — struct definition and all assertions use `front_solenoid`/`rear_solenoid`
- `main.h:188-189` — removed `Solenoid1_Request`/`Solenoid2_Request` from `struct car`

`Peripheral_actuation()` and `Handle_Emergency()` already used `front_solenoid`/`rear_solenoid` and required no changes. The test harness compiles cleanly and all 5 tests pass.

### C2. ASSI LEDs never flash (return value discarded)

`ASSI_control()` at `Autonomous_functions.c:205-258` takes `gpio_state` as a **value parameter**, modifies the local copy, and returns the new value. At `APP.c:114`, the call is:

```c
ASSI_control(ASSI_leds_control_signal, t24.ASSI_state);
```

The return value is discarded. The `ASSI_leds_control_signal` variable is never updated. `Peripheral_actuation()` at `hardware_abstraction.c:56-57` reads the same unchanged `ASSI_leds_control_signal`. The ASSI LEDs will always show whatever state was read from the GPIO pins in `Peripheral_aquisition` — which for an output pin is undefined/whatever was last written.

**Fix:** Change to `ASSI_leds_control_signal = ASSI_control(ASSI_leds_control_signal, t24.ASSI_state);`

### C3. ✅ `continuous_monitoring()` is dead code — FIXED

**What was wrong:** `continuous_monitoring()` was defined at `Autonomous_functions.c:152-203` but never called. The `Monitor_sequence` case only checked for mission mismatch and AS_STATE_FINISHED. The vehicle had zero runtime safety monitoring during autonomous driving.

**Fix applied:** `state_machine.c:12-14` now calls:
```c
continuous_monitoring(t24.SDC_feedback, NULL,
    t24.Rear_Pressure.Pneumatic, t24.Front_Pressure.Pneumatic,
    t24.Rear_Pressure.Hydraulic, t24.Front_Pressure.Hydraulic);
```
SDC monitoring, CAN timeout checks (VCU, Jetson, Pressure), pneumatic pressure range validation (6-10 bar), and hydraulic/pneumatic correlation now run on every superloop tick during `Monitor_sequence`.

Also fixed a bug on `Autonomous_functions.c:199` where the rear correlation check used `EBS_FRONT_HYD_GAIN` instead of `EBS_REAR_HYD_GAIN_FINAL`.

### C4. Variable defined in header (linker error waiting to happen)

`state_machine.h:18`:
```c
BLE_STATE_MACHINE_t ble_state = BLE_IDLE;
```

This is a **variable definition** (not declaration) in a header file. If `state_machine.h` is included by more than one translation unit, the linker will emit "multiple definition" errors. Currently it may only be included by `state_machine.c` and `APP.h` → `APP.c`, but this is fragile. This should be `extern BLE_STATE_MACHINE_t ble_state;` with the definition in exactly one `.c` file.

### C5. CAN TX queue overflow (no bounds check)

`hardware_abstraction.c:78-84` (`add_can_message`):
```c
void add_can_message(uint32_t mailbox, CAN_TxHeaderTypeDef tx_header, uint8_t tx_data[8]) {
    can_queue_index++;
    can_tx_queue[can_queue_index].TX_MAILBOX = mailbox;
    ...
}
```

The queue is declared as `struct can_queue can_tx_queue[64]` in `main.c:84`. There is no bounds check on `can_queue_index`. If more than 64 messages are queued between `handle_can_tx()` calls, this will overflow the array and corrupt adjacent memory (potentially the `ble_tx_busy` flag and other globals on the stack/heap).

Similarly, `handle_can_tx()` at `hardware_abstraction.c:63-76` has a logic issue: `tx_index` advances unconditionally, but `can_queue_index` is only decremented when the entire queue is drained. Messages can be skipped on partial mailbox availability.

**Fix:** Add `if (can_queue_index >= 63) return;` at the start of `add_can_message`.

### C6. MS_BTN ISR modifies mission without state guard

`main.c:344-351` (`HAL_GPIO_EXTI_Callback`):
```c
if (GPIO_Pin == MS_BTN_Pin) {
    t24.Current_Mission++;
    if (t24.Current_Mission > AUTOCROSS) {
        t24.Current_Mission = MANUAL;
    }
}
```

This ISR modifies `t24.Current_Mission` with **no guard on vehicle state**. The mission can be changed while `Vehicle_state_machine == AS_ON` or `Vehicle_state_machine == EMERGENCY`. In AS_ON, `Handle_autonomous_state()` mission-mismatch check at `state_machine.c:12` could trigger an immediate EMERGENCY if the ISR changes the mission between the aquire and the check. Worse, the ISR runs at any time including mid-superloop, creating a race condition with `dbc_decode()` which also sets `t24.Current_Mission` (indirectly via `t24.Jetson_mission`).

**Fix:** Guard with `if (Vehicle_state_machine != AS_ON)` or read the button in the main loop instead of using an EXTI.

### C7. Mission mismatch has no hysteresis

`state_machine.c:12-13`:
```c
if(t24.Current_Mission != t24.Jetson_mission){
    Vehicle_state_machine = EMERGENCY;
}
```

A single transient mismatch between the local mission selector and the Jetson-reported mission immediately triggers EMERGENCY. There is no debounce, no persistence count, no timing requirement. Given that CAN messages can be delayed or dropped, this will cause nuisance emergencies. A mismatch should persist for N consecutive cycles (or > 100 ms) before declaring EMERGENCY.

---

## Logic & Correctness Issues

### L1. `module_timeout` uses `t24` fields but `can_timeouts` struct is unused

`Autonomous_functions.c:269-278` (`module_timeout`) checks `t24.VCU_LAST_TX`, `t24.REAR_PRESSURE_LAST_TX`, and `t24.JETSON_LAST_TX` directly against `MAX_TIMEOUT` (1000 ms). The `struct can_timeouts` defined at `main.h:240-244` is completely unused. This is dead type cruft — remove it or use it.

### L2. `DIR_ACTUATOR_LAST_TX` never updated

`main.h:200` defines `DIR_ACTUATOR_LAST_TX` in `struct car`, but it is **never written** anywhere in the codebase. `module_timeout()` doesn't check it either (only VCU, REAR_PRESSURE, JETSON). This field should either be removed or properly populated when the corresponding CAN frame (DIR) is received.

### L3. Start state is unreachable

`APP.c:71` initializes `Vehicle_state_machine = IDLE` directly, and `state_machine.c:58-62` (`Start` case) does nothing but transition to IDLE. The `Start` state exists in the enum but cannot be reached and serves no purpose.

### L4. `emergency_blame()` has empty body

`Autonomous_functions.c:280-282`:
```c
uint32_t emergency_blame(){ }
```

This function has an empty body and no return statement. If called, the return value is undefined. Either implement it or remove it.

### L5. Can timeout threshold is uniform (1000 ms for all modules)

`Autonomous_functions.h:24` defines `#define MAX_TIMEOUT 1000` used for all CAN timeout checks in `module_timeout()`. Different CAN senders may have different expected transmission intervals (e.g., VCU might send at 100 Hz → 10 ms, Jetson at 50 Hz → 20 ms). A single 1000 ms timeout is too lenient for high-frequency messages and too strict for low-frequency ones. Each monitored CAN ID should have its own timeout threshold based on its expected cycle time from the DBC.

### L6. `Peripheral_aquisition` reads ASSI LED GPIO pins

`hardware_abstraction.c:41-42`:
```c
*assi_leds = HAL_GPIO_ReadPin(ASSI_YELLOW_GPIO_Port, ASSI_YELLOW_Pin) << 1
    | HAL_GPIO_ReadPin(ASSI_BLUE_GPIO_Port, ASSI_BLUE_Pin);
```

This reads the **output** pins of the ASSI LEDs. On STM32, reading a GPIO output pin returns the last written value (if pin is configured as push-pull output). This means `ASSI_control()` is reading back whatever `Peripheral_actuation()` wrote last, creating a feedback loop rather than tracking a desired state. The ASSI state should be tracked in a dedicated variable, not read from GPIO.

### L7. `Peripheral_actuation` uses logical `&&` instead of bitwise `&`

`hardware_abstraction.c:56-57`:
```c
HAL_GPIO_WritePin(..., ASSI_leds_control_signal && 0b00000010);
HAL_GPIO_WritePin(..., ASSI_leds_control_signal && 0b00000001);
```

`&&` is logical AND — it evaluates to `true` (1) or `false` (0). For bit masking, `&` must be used. Writing `1` to a GPIO pin sets it HIGH; writing `0` sets it LOW. The ASSI yellow LED (bit 0) will only turn on when `ASSI_leds_control_signal` is **any non-zero value** (since `x && 1` = 1 for any `x != 0`), and the blue LED (bit 1) will only turn on when `ASSI_leds_control_signal` is **any non-zero value** (same reason). The two LEDs become indistinguishable — both turn on/off together based on whether the signal is zero or non-zero.

**Fix:** Replace `&&` with `&`.

### L8. `can_rx_ringbuffer` tail arrival time used for VCU timeout

`APP.c:188`:
```c
t24.VCU_LAST_TX = can_rx_ringbuffer.queue[can_rx_ringbuffer.tail].arrival_time;
```

This reads the arrival time of the **current** message being popped (at the tail of the ring buffer) and writes it to `VCU_LAST_TX`. But `can_rx_ringbuffer.tail` is the index of the message currently being processed in `dbc_decode()`, not necessarily the most recent VCU message. If multiple CAN messages are in the buffer, the tail refers to the oldest, not the newest. This will cause the timeout check to use stale timestamps.

**Fix:** Store the arrival time directly from the message that was just decoded (i.e., set `VCU_LAST_TX = HAL_GetTick()` when a VCU frame is received), or read the header's arrival time at the time of push into the ring buffer.

---

## Safety Concerns

### S1. ~~No runtime monitoring during autonomous driving~~ — FIXED

`continuous_monitoring()` is now called at the start of `Monitor_sequence` (`state_machine.c:12-14`). SDC monitoring, CAN timeout detection (via `module_timeout()`), pneumatic pressure range validation, and hydraulic/pneumatic correlation checks all run on every superloop tick during autonomous driving. The vehicle will trigger emergency braking if any check fails.

### S2. ✅ `module_timeout()` now called from Monitor_sequence — FIXED

`continuous_monitoring()` (now invoked in `Monitor_sequence`) calls `module_timeout()` every tick. CAN timeout enforcement (VCU, Jetson, Pressure) is active during autonomous driving.

### S3. EEPROM logger not initialized

`app_init()` at `APP.c:48-103` never calls `eeprom_log_init()`. The EEPROM fault logger (`logger.c`) is fully implemented but completely disconnected from the application. Fault events will not be logged to persistent storage.

### S4. BLE event log flush is placeholder-only

`ble_handler.c:151-177` (`flush_one_record`) is entirely TODO comments and placeholder `snprintf` calls that always output `"SLOT:%u (EEPROM not yet implemented)\r\n"`. The event log retrieval over BLE does not work.

### S5. `handle_uart_logs` commented out

`APP.c:112` (`//handle_uart_logs();`) disables the USART1 diagnostic log output. While not a safety issue per se, this means the primary debug output is unavailable.

### S6. SKIP_* macros could be left enabled

`Autonomous_functions.h:16-22` defines 7 `SKIP_*` macros. If any are accidentally set to `1` in a production build, critical startup safety checks (pneumatic validation, hydraulic correlation, ignition confirmation) are bypassed. These should be controlled by a build configuration flag (e.g., `#ifdef DEBUG_BUILD`) rather than requiring manual editing.

### S7. `Vehicle_state_machine` and `t24` modified from ISR without protection

`main.c:344-351` (EXTI callback) modifies `t24.Current_Mission`, and `t24` can be read/written from any HAL callback (ADC, CAN, TIM, UART). In a bare-metal single-threaded system this is mostly safe, but the superloop at `APP.c:105-122` reads `t24.Current_Mission` in `Handle_state()` which then calls `Handle_autonomous_state()` which checks `t24.Current_Mission != t24.Jetson_mission`. If the ISR fires between these two reads, inconsistent behavior results.

---

## Code Quality

### Q1. Filename typo: `hardware_abstaction.c`

Should be `hardware_abstraction.c`. The include directive uses the correct spelling (`#include "hardware_abstraction.h"`), so the file compiles, but the mismatch is confusing for future maintainers.

### Q2. ✅ Inconsistent field naming in `struct car` — FIXED

The duplicate `Solenoid1_Request`/`Solenoid2_Request` fields were removed from `struct car`. All code now uses `front_solenoid`/`rear_solenoid` consistently.

### Q3. `prev_ASMS` logic is reversed

`APP.c:107`: `prev_ASMS = t24.ASMS` is read **before** `Peripheral_aquisition()` updates `t24.ASMS`. The value used as "previous" in `Handle_state(prev_ASMS)` at `APP.c:110` is actually the **current** value from the previous loop iteration — which is correct for edge detection. However, on the very first call to `app()`, `t24.ASMS` is 0 (initialized) and `prev_ASMS` is 0 (initialized), so the rising edge on ASMS will not be detected until the second loop iteration. This adds one ~100 µs delay — acceptable but worth noting.

### Q4. Magic numbers in ADC conversion

`main.c:220-225`: The pressure sensor conversion uses `3.3 / 4096`, `0.66` gain, `0.5` offset, `0.4` scale. These should be named constants, not inline literals. Similarly, the temperature formula at `main.c:233-253` uses hardcoded calibration addresses `0x1FFF7A2C` and `0x1FFF7A2E` rather than the standard CMSIS macros.

### Q5. Unused variables and includes

- `APP.c:13`: `struct can_queue can_rx_data` — only used as the target of `can_buffer_pop()` at line 117; could be local to `dbc_decode()`.
- `APP.c:19`: `float temporary_temp = 0;` — assigned at line 109 (`temporary_temp = t24.chip_temp`) but never read.
- `APP.c:29-30`: `prev_car_state`, `prev_as_state` — declared but never used.
- `main.c:63`: `uint16_t raw_vref;` — declared at file scope but only used locally in `GetTemperature()`.
- `Autonomous_functions.h:26-28`: `extern struct ring can_rx_ringbuffer;` and `extern Emergency_cause_t Emergency_cause;` — declared here but also in other headers.

### Q6. Inconsistent `#ifdef` inclusion guard

`hardware_abstraction.h:7-8` uses `#ifndef HW_ABS` / `#define HW_ABS` — an unusual name. All other headers use the pattern `#ifndef INC_FILENAME_H_`. This is purely stylistic but inconsistent.

### Q7. Typo in enum value

`main.h:146`: `UNKOWN` should be `UNKNOWN`.

### Q8. Only 4 of 20 CAN frames decoded

`APP.c:182-211` (`dbc_decode`) processes only 4 frame IDs:
- `AUTONOMOUS_T26_AQT7_FRAME_ID` (0x770) — rear brake pressure
- `AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID` (0x600) — ignition + SDC
- `AUTONOMOUS_T26_JETSON_FRAME_ID` (0x61) — AS state + mission
- `AUTONOMOUS_T26_VCU_RPM_FRAME_ID` (0x509) — RPM

The remaining 16 CAN IDs (including AQT1-AQT4 dynamics, DV_dynamics_1/2, VCU_HV, CubeMars feedback, SLAM stats, RES) are received and buffered by the ring buffer but never decoded. The arrival time of these frames updates the ring buffer tail but no action is taken on their content.

---

## Performance

### P1. CAN TX queue polling in superloop

`hardware_abstraction.c:63-76` (`handle_can_tx`) is called every superloop iteration (~100 µs?) but only sends when mailbox is free. At 10 Hz CAN TX (TIM2 callback), most polls will be no-ops. This is acceptable for a bare-metal system but wastes CPU cycles. Consider using the CAN TX mailbox empty interrupt instead.

### P2. `snprintf` in `ble_handler.c:173-176` every superloop tick during flush

`flush_one_record()` calls `snprintf` with format string `"SLOT:%u (EEPROM not yet implemented)\r\n"` then `HAL_UART_Transmit` with 500 ms timeout. If flushing is triggered, the loop blocks for `20 × 500 ms = 10 seconds` worst case. The TODO comments acknowledge this should be non-blocking.

### P3. BLE telemetry allocation every TIM2 tick

`main.c:309`: `static ble_telemetry_packet_t pkt;` inside the TIM2 callback is re-initialized each tick. The `static` qualifier ensures only one instance exists, but the struct is small (15 bytes). This is fine.

### P4. TIM8 base started but unused

`APP.c:78`: `HAL_TIM_Base_Start(&htim8);` starts TIM8 but the codebase never uses it for anything. TIM8 was intended for PWM/capture but is not configured. The timer is running and generating interrupts unnecessarily.

---

## Testing

### T1. Only one test file exists

`tests/test_initial_sequence.c` covers only `initial_sequence()` and `check_timeout()`. There are **no tests** for:
- `ASSI_control()` (ASSI LED logic)
- `dbc_decode()` (CAN message unpacking)
- `Handle_state()` / `Handle_autonomous_state()` / `Handle_Emergency()`
- `module_timeout()` and `continuous_monitoring()`
- `Peripheral_actuation()` (solenoid + ASSI GPIO writes)
- BLE handler state machine
- EEPROM logger I/O
- Ring buffer push/pop ordering

### T2. Test harness stubs are well done

The test file's approach of defining stub types, a fake `millis()`, and globals at file scope is pragmatic and effective. The 5 tests cover normal startup, WDT timeout, pneumatic OOR, solenoid timeout, and SM transition. The `reset_globals()` helper ensures test isolation.

### T3. Test environment is separate from firmware

The test file copies function bodies verbatim from the firmware source. This means the tests and firmware can diverge — if `initial_sequence()` is modified in `Autonomous_functions.c`, the test copy must be manually updated. A better approach would be to extract the core logic into a platform-independent static library, but the copy approach is understandable given the HAL dependency.

### T4. No hardware-in-the-loop test framework

There is no CI/CD pipeline for running the host-based tests. The compilation command is documented in a comment (`tests/test_initial_sequence.c:7-8`) but no `Makefile` or script automates it.

---

## Specific File Reviews

### `Core/Inc/main.h` (260 lines)

- **Good:** Clean structure with enumerations for all state machines, packed BLE telemetry struct, pin defines, and CAN timeout constant.
- **Mixed:** The `struct car` is a "god struct" containing 20+ fields from different subsystems (CAN, ADC, GPIO, state). This tightly couples all modules to a single data structure.
- **Issues:** `CAN_MSG_MAX_TIMEOUT 1000` at line 114 is used by nothing in the actual timeout logic (which uses `MAX_TIMEOUT` from `Autonomous_functions.h:24`). `can_timeouts` struct at lines 240-244 is unused. `DIR_ACTUATOR_LAST_TX` at line 200 is never written.

### `Core/Inc/state_machine.h` (27 lines)

- **Critical:** Line 18 defines `BLE_STATE_MACHINE_t ble_state = BLE_IDLE;` — a variable definition in a header. This will cause multiple-definition linker errors if included from more than one `.c` file.

### `Core/Src/main.c` (404 lines)

- **Good:** TIM2 callback at line 259 handles CAN TX (3 frames) and BLE telemetry efficiently. ADC callback at line 218 updates pressure + temperature. Can RX callback at line 334 pushes to ring buffer.
- **Issues:** EXTI callback at line 344 modifies mission without state guard. `HAL_ADC_ConvCpltCallback` at line 218 does not check `hadc` pointer before use (though safe with single ADC).

### `Core/Src/APP.c` (211 lines)

- **Good:** Clean superloop at line 105-122 with logical ordering. `ble_module_config_start/tick/is_done` is a clean async config FSM.
- **Critical:** Line 114 discards `ASSI_control()` return value. EEPROM logger never initialized.
- **Issues:** `temporary_temp` at line 19/109 is assigned but never read. `prev_car_state`/`prev_as_state` at lines 29-30 are unused.

### `Core/Src/state_machine.c` (101 lines)

- **Good:** Clean 4-state vehicle SM with nested autonomous SM. Emergency handler properly clears solenoids and disables ignition/WDT.
- **Critical:** ✅ `continuous_monitoring()` now called at the start of `Monitor_sequence` (line 12-14) — enables SDC, CAN timeout, pressure, and correlation checks during autonomous driving.
- **Issues:** `as_on_first_time` at line 56 uses a `static` that persists across entries; while functionally correct, explicit state would be cleaner. `Autonomous_state = Finish` at line 15 uses the enum value directly but should check `t24.Autonomous_State == AS_STATE_FINISHED` for clarity (currently works because enum values match).

### `Core/Src/Autonomous_functions.c` (282 lines)

- **Good:** 8-step startup sequence at lines 29-151 is well-structured with clear macros, timing guards, and error propagation. `ASSI_control()` at lines 205-258 has correct 330 ms flash timing (~3 Hz, within the 2-5 Hz requirement).
- **Critical:** ✅ `continuous_monitoring()` at lines 152-203 now called from `Monitor_sequence`. `ASSI_control()` modifies local copy (C2). `emergency_blame()` at line 280 is empty.
- **Issues:** `module_timeout()` at lines 269-278 hardcodes 1000 ms for all three modules. `IN_RANGE` macro at line 23 uses strict inequality — pressure exactly at 6.0 or 10.0 bar is rejected (might be intentional).

### `Core/Src/hardware_abstraction.c` (114 lines)

- **Critical:** Lines 56-57 use `&&` instead of `&` for bit masking (L7). Line 78-84 has no bounds check on `can_queue_index` (C5).
- **Issues:** `handle_can_tx()` at line 63-76 has a logic flaw — `tx_index` advances unconditionally on each call, but `can_queue_index` only resets when all messages are sent. If mailbox is full, messages are skipped.

### `Core/Src/ble_handler.c` (329 lines)

- **Good:** CTS time sync via BLE Current Time Service is well-implemented with proper fallback. DMA circular buffer with `rx_contains()`/`rx_read()` API is clean. Bridge mode with `stop/start/flush` commands is useful.
- **Issues:** `flush_one_record()` at lines 151-177 is entirely placeholder. `ble_handler_init()` is never called. The comment at line 218 notes that `BLE_CONNECT` is repurposed as `WAIT_CMD` — the enum should be extended. `HAL_Delay(300)` at line 237 blocks in the state machine — should use the non-blocking timeout pattern.

### `Core/Src/ring_buffer.c` (69 lines)

- **Good:** Clean circular buffer with separate TX/RX push functions, arrival time stamping, and counter management. 1024 entries is generous for CAN RX.
- **Issues:** `can_buffer_pop()` at lines 42-69 mixes TX and RX logic in one function based on `tx_or_rx` flag. This violates separation of concerns. The TX path has a subtle bug: if `HAL_CAN_AddTxMessage()` fails, the retry on next call will attempt the same message again (`tail` doesn't advance), which is correct, but the function doesn't return any status to indicate success/failure.

### `Core/Src/logger.c` (91 lines)

- **Good:** Clean ring-buffer EEPROM logger with magic validation, head/tail management, and overflow handling. The design correctly wraps head on overflow (ring buffer semantics) rather than overwriting the newest entry.
- **Issues:** Never initialized from `app_init()`. The I2C timeout of 1000 ms for every EEPROM operation is aggressive for a fault logger. No CRC on individual entries — only a magic number in the header.

### `Core/Src/rn4871.c` (255 lines)

- **Good:** Comprehensive, well-organized, consistent API. The `RN4871_FMT` macro at lines 5-10 elegantly handles snprintf + bounds checking + return. Every documented command is wrapped.
- **Issues:** None significant.

### `Core/Inc/autonomous_t26.h` (3655 lines)

- **Good:** Properly generated by cantools 40.5.0. All frame IDs, signal choices, encode/decode functions present.
- **Issues:** None — generated code is clean.

### `tests/test_initial_sequence.c` (685 lines)

- **Good:** Self-contained, well-commented, 5 meaningful tests. Proper use of fake time for timeout testing. Good coverage of the startup sequence.
- **Issues:** Function bodies are duplicated from firmware source (maintenance burden). No Makefile or automation. Only covers `initial_sequence()` — critical functions like `ASSI_control()`, `module_timeout()`, and `continuous_monitoring()` have no tests.

---

## Recommendations

### Top 5 things to fix immediately:

1. **✅ Solenoid actuation (C1) — FIXED.** All code now uses `front_solenoid`/`rear_solenoid` consistently.

2. **✅ `continuous_monitoring()` wired into `Monitor_sequence` (C3) — FIXED.** Runtime safety checks (SDC, CAN timeout, pressure, correlation) now active during autonomous driving.

3. **Fix ASSI LED control** (C2, L7): Two-part fix: (a) Change `APP.c:114` to capture the return value: `ASSI_leds_control_signal = ASSI_control(ASSI_leds_control_signal, t24.ASSI_state);` (b) Change `hardware_abstraction.c:56-57` to use `&` instead of `&&`.

4. **Fix `state_machine.h:18`** (C4): Change `BLE_STATE_MACHINE_t ble_state = BLE_IDLE;` to `extern BLE_STATE_MACHINE_t ble_state;` and move the definition to a `.c` file.

5. **Add bounds check to `add_can_message()`** (C5): Guard against `can_queue_index >= 63` before incrementing. This prevents silent memory corruption from CAN TX queue overflow.

### Additional high-priority fixes:

6. Guard `MS_BTN` ISR with state check (C6).
7. Add hysteresis to mission mismatch detection (C7).
8. Initialize EEPROM logger from `app_init()` (S3).
9. Replace `handle_uart_logs();` comment with actual call or remove it (S5).
10. Implement `flush_one_record()` EEPROM read path in `ble_handler.c` (S4).
11. Remove unused fields: `can_timeouts` struct, `DIR_ACTUATOR_LAST_TX`, `temporary_temp`, `prev_car_state`, `prev_as_state`.
12. Per-module CAN timeout thresholds instead of single 1000 ms (L5).
13. Per-module arrival time tracking in `dbc_decode()` instead of reading from ring buffer tail (L8).
14. Build system integration for the host-based test (T4).
15. Add tests for `ASSI_control()`, `module_timeout()`, `continuous_monitoring()`, and `dbc_decode()` (T1).
