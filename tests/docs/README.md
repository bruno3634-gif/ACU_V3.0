# ACU V3.0 — Project Status & Test Suite

## 1. Project Overview

ACU (Active Control Unit) firmware for a Formula Student autonomous vehicle. Targets the **STM32F412RETX** microcontroller using the **STM32CubeMX HAL** and compiled as **C11**. The firmware handles CAN bus communication with vehicle subsystems (VCU, Jetson, pressure sensors), BLE telemetry, ASSI (Autonomous System Status Indicator) LED control, EEPROM fault logging, mission selection, and the core vehicle/autonomous state machines.

This document serves as both the project status report and the test suite reference. It catalogues known bugs, safety concerns, code quality issues, and provides a hardware verification plan for first power-on.

---

## 2. Source Files Overview

### Core Application (`Core/Src/`)

| File | Lines | Purpose |
|------|-------|---------|
| `autonomous_t26.c` | 2,329 | CAN DBC generated codec (cantools) |
| `main.c` | 408 | Entry point, HAL callbacks, TIM2 telemetry |
| `stm32f4xx_it.c` | 381 | Interrupt handlers |
| `ble_handler.c` | 329 | BLE CTS handler + bridge mode |
| `rn4871.c` | 255 | BLE AT command driver |
| `APP.c` | 219 | Application superloop, init, dbc_decode |
| `Autonomous_functions.c` | 284 | Startup sequence, monitoring, ASSI control |
| `usart.c` | 268 | UART peripheral init |
| `adc.c` | 183 | ADC peripheral init |
| `tim.c` | 175 | TIM peripheral init |
| `system_stm32f4xx.c` | 747 | CMSIS system init |
| `state_machine.c` | 106 | Vehicle + Autonomous state machines |
| `can.c` | 125 | CAN peripheral init |
| `hardware_abstaction.c` | 118 | I/O abstraction, CAN TX, UART logs |
| `i2c.c` | 116 | I2C peripheral init |
| `rtc.c` | 133 | RTC peripheral init |
| `gpio.c` | 122 | GPIO peripheral init |
| `logger.c` | 91 | EEPROM fault logger |
| `stm32f4xx_hal_msp.c` | 82 | HAL MSP init |
| `ring_buffer.c` | 69 | CAN ring buffer |
| `sysmem.c` | 79 | Heap implementation |
| `dma.c` | 65 | DMA peripheral init |
| `EMA_Filter.c` | 19 | Exponential moving average filter |
| `syscalls.c` | 176 | Syscall stubs |

### Test Suite (`tests/`)

| File | Lines | Tests |
|------|-------|-------|
| `test_autonomous_t26.c` | 1,307 | 9 |
| `test_state_machine.c` | 1,031 | 8 |
| `test_tim_callback.c` | 833 | 12 |
| `test_autonomous_functions.c` | 701 | 16 |
| `test_logger.c` | 696 | 8 |
| `test_initial_sequence.c` | 683 | 5 |
| `test_adc_callbacks.c` | 617 | 10 |
| `test_can_queue.c` | 556 | 8 |
| `test_ble_config.c` | 553 | 8 |
| `test_ring_buffer.c` | 482 | 8 |
| `test_uart_callbacks.c` | 380 | 10 |
| `test_ema_filter.c` | 299 | 5 |
| `test_temperature.c` | 263 | 5 |
| **Total** | **8,401** | **101** |

---

## 3. Quick Start

```bash
./tests/run_tests.sh   # compiles and runs all 13 tests
```

Report written to `tests/report/test_report.txt`.

---

## 4. Bugs Fixed

| Bug | File | Fix |
|-----|------|-----|
| Wrong enum comparison | `state_machine.c:19` | `Finish` → `AS_STATE_FINISHED` |
| Missing break in switch | `Autonomous_functions.c:185` | Added `break` after `case RES_TIMEOUT` |

---

## 5. CRITICAL BUGS (still present) — labelled P0

### P0-1: ASSI LEDs never flash

**File**: `hardware_abstaction.c:60-61`

```c
HAL_GPIO_WritePin(ASSI_BLUE_GPIO_Port, ASSI_BLUE_Pin, ASSI_leds_control_signal && 0b00000010);
HAL_GPIO_WritePin(ASSI_YELLOW_GPIO_Port, ASSI_YELLOW_Pin, ASSI_leds_control_signal && 0b00000001);
```

**Bug**: `&&` is logical AND, not bitwise AND. `ASSI_leds_control_signal && 0b00000010` evaluates to `true` (1) for ANY non-zero signal, not just when bit 1 is set. Both LEDs turn on/off together. Should use `&`.

**Impact**: ASSI status LEDs non-functional. AS_STATE_OFF, READY, DRIVING, EMERGENCY, and FINISHED all show the same LED pattern.

---

### P0-2: Variable definition in header — linker error risk

**File**: `state_machine.h:18`

```c
BLE_STATE_MACHINE_t ble_state = BLE_IDLE;
```

**Bug**: This is a **definition** (allocates storage) in a header file. If `state_machine.h` is included by more than one translation unit, the linker emits a "multiple definition" error.

**Fix**: Change to `extern BLE_STATE_MACHINE_t ble_state;` in the header, define in a `.c` file.

---

### P0-3: CAN TX queue overflow — RAM corruption

**File**: `hardware_abstaction.c:84`

```c
void add_can_message(...) {
    can_queue_index++;           // no bounds check!
    can_tx_queue[can_queue_index] = ...;
}
```

**Bug**: `can_tx_queue[64]` but no bounds check. If more than 64 messages are queued between `handle_can_tx()` calls (e.g., TIM2 at 10 Hz queues 3 per tick, CAN bus off means no TX), memory after `can_tx_queue` is silently corrupted.

**Impact**: Stack or adjacent global variables corrupted. Non-deterministic crashes.

**Fix**: Add `if (can_queue_index >= 63) return;` guard.

---

### P0-4: Mission race condition in ISR

**File**: `main.c:348-354`

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == MS_BTN_Pin && mission_selector_enable == 1) {
        t24.Current_Mission++;
```

**Bug**: Interrupt modifies `t24.Current_Mission` which is also read/written by `dbc_decode()` (CAN RX interrupt) and `Handle_state()` (main loop). No atomic access or critical section.

**Impact**: Race condition — mission state corruption if the button is pressed during CAN reception.

---

## 6. HIGH-SEVERITY ISSUES (P1)

### P1-1: EEPROM logger never initialized

`eeprom_log_init()` exists in `logger.c` but is never called from `app_init()` in `APP.c`. All fault events are lost.

### P1-2: BLE handler never initialized

`ble_handler_init()` exists in `ble_handler.c:195` but is never called. The BLE CTS state machine never starts. Two competing BLE config paths coexist.

### P1-3: UART debug logging disabled

`handle_uart_logs()` in `hardware_abstaction.c` is commented out at the call site (`APP.c:112`). Primary UART debug output never runs.

### P1-4: EEPROM read over BLE is placeholder

`flush_one_record()` in `ble_handler.c:151-177` only outputs `"EEPROM not yet implemented"` placeholder text.

### P1-5: Only 4 of 20+ CAN frames decoded

`dbc_decode()` in `APP.c:182-218` only processes: AQT7 (0x770), VCU_IGN_R2_D (0x600), JETSON (0x61), VCU_RPM (0x509). Frames at 0x710, 0x720, 0x730, 0x740, 0x500, 0x502, 0x81, and 0x505 are received but never decoded.

### P1-6: `emergency_blame()` has empty body

`Autonomous_functions.c:282-284`:

```c
uint32_t emergency_blame(){}
```

No return statement — undefined behavior if called.

### P1-7: TIM8 started but never used

`APP.c:78`: `HAL_TIM_Base_Start(&htim8)` — TIM8 runs but no ISR or application code uses it.

### P1-8: `handle_can_tx()` has message-skipping logic flaw

`hardware_abstaction.c:67-80`: `tx_index` advances unconditionally on every call, but `can_queue_index` only resets when the queue is fully drained. If CAN mailboxes become full partway through draining, those messages are lost forever.

### P1-9: AS_ON re-entry deadlock

`state_machine.c:70-74`: `static uint8_t as_on_first_time = 0;` is never reset. After an EMERGENCY → IDLE → AS_ON transition, the startup sequence is skipped because the flag stays 1. The car enters AS_ON with uninitialized actuators.

---

## 7. SAFETY CONCERNS

### S-1: SKIP_* debug macros compiled in production

`Autonomous_functions.h:16-22`: 7 macros (`SKIP_WDT`, `SKIP_PNEUMATIC`, `SKIP_PRESSURE`, `SKIP_HV`, `SKIP_INIT`, etc.) can bypass critical startup safety checks. Not gated by `#ifdef DEBUG`. If accidentally defined, the safety chain is disabled.

### S-2: Mission mismatch lacks hysteresis

`state_machine.c:17-18`: A single transient CAN message with a mismatched mission triggers immediate EMERGENCY. No debounce or persistence count. A single corrupted CAN frame can trigger an emergency shutdown.

### S-3: Single timeout threshold for all CAN modules

`#define MAX_TIMEOUT 1000` in `Autonomous_functions.h:24`. VCU (10 Hz = 100 ms), Jetson (varying rate), and pressure sensors all share one timeout. Wrong for all.

### S-4: ASSI reads its own GPIO output as input

`hardware_abstaction.c:42-43`: `Peripheral_aquisition()` reads back the GPIO pins that were just written by `ASSI_control()`. This creates a feedback loop — if the write fails, the read also fails silently.

### S-5: `REAR_PRESSURE_LAST_TX` uses oldest ring buffer timestamp

`APP.c:188`: Uses `can_rx_ringbuffer.queue[can_rx_ringbuffer.tail].arrival_time` which is the **oldest** entry, not the newest. Pressure timeout fires too late.

---

## 8. CODE QUALITY ISSUES

### Dead / Unused Code

| Variable / Symbol | File | Never |
|-------------------|------|-------|
| `temporary_temp` | `APP.c:19` | Assigned but never read |
| `prev_car_state`, `prev_as_state` | `APP.c:29-30` | Declared, never written |
| `CAN_MSG_MAX_TIMEOUT` | `main.h:114` | Defined, never used |
| `struct can_timeouts` | `main.h:240-244` | Defined, never used |
| `GPIO.cpp`, `GPIO.hpp` | `Core/Inc/` | C++ files in a C project, never compiled |
| `can_buffer_push()` | `ring_buffer.c:17` | TX ring buffer push, never called |

### Typos

- `hardware_abstaction.c` (missing `r`) vs. header `hardware_abstraction.h`
- `UNKOWN` should be `UNKNOWN` (`main.h:146`)
- `.gitgnore` (typo) coexists with `.gitignore`

### Missing Error Handling (HAL return values ignored)

Return values are discarded for all of the following:

- `HAL_RTC_SetTime`
- `HAL_RTC_SetDate`
- `HAL_UART_Transmit` (5 call sites)
- `HAL_UART_Transmit_DMA` (2 call sites)
- `HAL_UART_Receive_DMA`
- `HAL_UART_Abort`
- `HAL_ADC_Start_DMA`
- `HAL_TIM_Base_Start`
- `HAL_CAN_ConfigFilter`
- `HAL_CAN_ActivateNotification`
- `HAL_CAN_Start`
- `HAL_CAN_AddTxMessage`

None of these check their return value. A silent HAL failure will go undetected.

---

## 9. Test Suite

### 13 test files, 101 test cases, all passing

| Test File | Module(s) Tested | Logic Lines | Test Cases | Category |
|-----------|-----------------|-------------|------------|----------|
| `test_adc_callbacks.c` | `HAL_ADC_ConvCpltCallback()`, `HAL_GPIO_EXTI_Callback()` | ~40 | 10 | MCU Callbacks |
| `test_autonomous_functions.c` | `ASSI_control()`, `module_timeout()`, `continuous_monitoring()` | ~130 | 16 | Safety & Monitoring |
| `test_autonomous_t26.c` | CAN DBC pack/unpack (autonomous_t26) | ~300 | 9 | Communication |
| `test_ble_config.c` | `ble_module_config_start()`, `ble_module_config_tick()` | ~55 | 8 | BLE Configuration |
| `test_can_queue.c` | `add_can_message()`, `handle_can_tx()` | ~25 | 8 | Communication |
| `test_ema_filter.c` | `ema_init()`, `ema_update()` | ~19 | 5 | Signal Processing |
| `test_initial_sequence.c` | `initial_sequence()`, `check_timeout()` | ~150 | 5 | State Machine |
| `test_logger.c` | EEPROM circular log buffer | ~90 | 8 | Data Logging |
| `test_ring_buffer.c` | CAN ring buffer push/pop | ~70 | 8 | Data Structures |
| `test_state_machine.c` | `Handle_autonomous_state()`, `Handle_state()`, `dbc_decode()` | ~200 | 8 | State Machine |
| `test_temperature.c` | `GetTemperature()` | ~20 | 5 | Signal Processing |
| `test_tim_callback.c` | `HAL_TIM_PeriodElapsedCallback()` | ~85 | 12 | MCU Callbacks |
| `test_uart_callbacks.c` | `HAL_UARTEx_RxEventCallback()`, `HAL_UART_TxCpltCallback()` | ~25 | 10 | MCU Callbacks |

**Total: 13 test files, ~1,200 lines of logic under test, 101 test cases**

### Test Architecture

Each test file is self-contained and follows this pattern:

```
1. Type stubs         — typedefs and structs matching the firmware's types
2. Macros/constants   — copied from the source headers
3. Fake time          — fake_time_ms variable + millis() stub
4. Global variables   — matching the firmware's globals (t24, etc.)
5. Function bodies    — VERBATIM copy from Core/Src/<module>.c
6. TEST_ASSERT macro  — prints failure with file:line, returns 0 from test
7. Test functions     — one per scenario
8. main()             — runs all tests, counts pass/fail, exits 0/1
```

This avoids any dependency on:

- `stm32f4xx_hal.h` and CMSIS headers
- Cross-compiler toolchains (`arm-none-eabi-gcc`)
- Linker scripts or hardware memory maps
- Physical peripherals (GPIO, ADC, CAN, UART)

### What Gets Tested

**Pure Logic (no stubs beyond standard C)**

- **EMA Filter**: Initialization, first-sample passthrough, convergence behavior, alpha=1 passthrough, alpha=0 hold
- **CAN DBC Codec**: Pack/unpack round-trips for 5 message types (ACU, AQT7, VCU_IGN_R2_D, VCU_RPM, ASF_SIGNALS), signal encode/decode, buffer overflow detection, init functions
- **Ring Buffer**: Init state, push/pop correctness, multi-push, full-buffer overflow protection, empty-pop safety, wrap-around, RX arrival time stamping
- **GetTemperature()**: VREF=0 guard, known temperature at calibration points, VREF supply voltage compensation, division-by-zero handling
- **CAN TX Queue**: Message enqueue (`add_can_message`), dequeue via HAL stub (`handle_can_tx`), FIFO ordering, mailbox-full backpressure, 64-entry array limits

**State Machine Logic**

- **BLE Configuration**: Full 6-state BLE module setup sequence: idle → enter → wait_enter → send_cmd → wait_cmd → done. 5 configuration commands transmitted with 500 ms inter-command delays, no UART interaction before `start()`
- **`initial_sequence()`**: Full 8-stage startup walkthrough, WDT timeout, pneumatic out-of-range, solenoid timeout, `Handle_autonomous_state` transition
- **`Handle_state()`**: Start → IDLE, IDLE → AS_ON (ASMS rising edge), EMERGENCY hold and recovery to IDLE
- **`Handle_autonomous_state()`**: Initial_Sequence → Monitor_sequence transition after successful startup

**Safety & Monitoring with Time Dependencies**

- **`ASSI_control()`**: All 5 AS states (OFF/READY/DRIVING/EMERGENCY/FINISHED), flashing frequency verification for DRIVING and EMERGENCY
- **`module_timeout()`**: All 5 timeout types (VCU, JETSON, PRESSURE, DIR, RES) plus normal (no timeout) case
- **`continuous_monitoring()`**: SDC open, pneumatic out-of-range, hydraulic correlation failure, normal pass-through

**MCU Callbacks (tested by calling directly with arranged state)**

- **`HAL_ADC_ConvCpltCallback()`**: ADC raw → voltage → pressure conversion math, temperature computation via `GetTemperature`, correct updates to `t24.Front_Pressure` and `t24.Rear_Pressure`
- **`HAL_GPIO_EXTI_Callback()`**: Mission selector button press (increment with wrap), suppress when `mission_selector_enable=0`, ignore non-MS_BTN pins
- **`HAL_UARTEx_RxEventCallback()`**: UART command parsing — "log" sets dump flag, "resume" clears it, whitespace stripping, case insensitivity, unknown commands ignored, USART2 instance filtering
- **`HAL_UART_TxCpltCallback()`**: BLE TX busy flag cleared on USART2 transmit complete
- **`HAL_TIM_PeriodElapsedCallback()`**: CAN frame packing (ACU at 0x51, ASF_SIGNALS at 0x511), pressure ×10 scaling, BLE telemetry transmission via DMA stub, value clamping (uint16 saturation at 65535, int16 saturation at ±32767, negative→0), DMA timeout recovery (abort on stuck TX > 500 ms, automatic re-send)

**CAN Message Routing**

- **`dbc_decode()`**: Routing of 4 CAN frame types (AQT7, VCU_IGN_R2_D, JETSON, VCU_RPM) and correct field updates on the `t24` struct

**Data Persistence**

- **EEPROM Logger**: Fresh init, existing-log detection, write/read, multiple entries, circular overwrite (19-entry capacity), out-of-range read, count function, clear+rewrite

### What Is NOT Tested on Host

The following modules depend on STM32 HAL registers, interrupt hardware, or external peripherals and are **not compiled on the host**:

| Module | Reason |
|--------|--------|
| `can.c`, `adc.c`, `dma.c`, `gpio.c`, `i2c.c`, `tim.c`, `usart.c` | HAL peripheral configuration |
| `stm32f4xx_hal_msp.c`, `stm32f4xx_it.c` | Interrupt vector table and HAL MSP init |
| `syscalls.c`, `sysmem.c`, `system_stm32f4xx.c` | Platform startup code |
| `rn4871.c` | BLE UART driver — full duplex protocol, not pure logic |
| `hardware_abstaction.c` | `Peripheral_aquisition()`, `Peripheral_actuation()`, `LED_indicator_controller()`, `handle_uart_logs()` |

**Note on callback testing**: STM32 HAL callbacks (`HAL_ADC_ConvCpltCallback`, `HAL_TIM_PeriodElapsedCallback`, etc.) are tested on host by calling them directly with pre-arranged global state. This validates the **logic inside the callback** but does NOT test:

- Interrupt timing or nesting
- DMA descriptor chain integrity
- Real ADC sampling or conversion timing
- Real CAN bus arbitration or ACK/NACK

These aspects require hardware-in-the-loop (HIL) testing.

### Known Gap: Test Makefile Outdated

`tests/Makefile` only lists 6 of the 13 targets. Use `run_tests.sh` instead of `make` for running the full suite.

---

## 10. Hardware Verification Plan

### Priority 1 — Critical for first power-on

1. **CAN bus traffic**: Connect a CAN analyzer. Verify the ACU transmits frames at 0x51, 0x502, and 0x511 on TIM2 (10 Hz). Verify CAN RX receives frames from VCU, Jetson, and pressure sensors.

2. **Main loop sequence**: Debugger step-through of `app()`. Verify the order: `Peripheral_aquisition` → `continuous_monitoring` → `Handle_state` → `Peripheral_actuation` → `handle_can_tx`.

3. **Startup sequence**: Verify all 8 stages of `initial_sequence()` complete before `Monitor_sequence` begins.

### Priority 2 — Functional verification

4. **ASSI LEDs**: Verify the correct LED pattern for each AS state:
   - Blue = AS_READY
   - Yellow = AS_DRIVING (500 ms flash)
   - Yellow + Blue = AS_EMERGENCY (250 ms flash)

5. **Mission selector**: Press MS_BTN. Verify CAN output changes the mission field. Test wrap from AUTOCROSS to MANUAL.

6. **BLE telemetry**: Receive a 15-byte telemetry packet over BLE. Verify all fields (pressures, temperature, state).

### Priority 3 — Fault injection

7. **SDC open**: Open the shutdown circuit. Verify EMERGENCY state within one monitoring cycle.

8. **CAN timeout**: Disconnect the VCU. Verify `VCU_TIMEOUT` emergency after 1000 ms.

9. **EEPROM logger**: Trigger a fault, power cycle, read back the log via the UART `"log"` command.

---

## 11. CI/CD

The test suite runs entirely on the host with no hardware dependencies. Add this GitHub Actions workflow:

```yaml
name: ACU Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: sudo apt-get install -y gcc
      - run: ./tests/run_tests.sh
```
