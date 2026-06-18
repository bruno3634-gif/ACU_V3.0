# ACU V3.0 — Host-Based Test Suite

## Overview

This directory contains standalone host-compilable unit tests for the ACU V3.0
firmware (STM32F412RETX). They are designed to run on a **PC without any STM32
hardware** using the host's GCC compiler. Each test file is **self-contained** —
it includes its own type stubs and verbatim copies of the function bodies under
test, avoiding dependencies on the STM32 HAL and CMSIS headers.

## Prerequisites

- **GCC** (any version with C11 support) — tested with GCC 15.2.0 on Ubuntu (WSL)
- **GNU Make**
- **Bash** (for `run_tests.sh`)

## Quick Start

```bash
./tests/run_tests.sh
```

This compiles all tests, runs them, and writes a detailed report to
`tests/report/test_report.txt`.

## Test Modules

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

**Total: 13 test files, ~1200 lines of logic under test, 101 test cases**

## How to Run

### Using the shell script (recommended):
```bash
./tests/run_tests.sh
```

### Using Make:
```bash
cd tests
make        # Compile all tests
make run    # Compile and run all tests (output to console)
make report # Compile, run, and generate report file
```

### Run a single test manually:
```bash
gcc -o tests/test_ema_filter tests/test_ema_filter.c -Wall -Wextra -std=c11 -lm
./tests/test_ema_filter
```

## What Gets Tested

### Pure Logic (no stubs beyond standard C)
- **EMA Filter**: Initialization, first-sample passthrough, convergence behavior,
  alpha=1 passthrough, alpha=0 hold
- **CAN DBC Codec**: Pack/unpack round-trips for 5 message types (ACU, AQT7,
  VCU_IGN_R2_D, VCU_RPM, ASF_SIGNALS), signal encode/decode, buffer overflow
  detection, init functions
- **Ring Buffer**: Init state, push/pop correctness, multi-push, full-buffer
  overflow protection, empty-pop safety, wrap-around, RX arrival time stamping
- **GetTemperature()**: VREF=0 guard, known temperature at calibration points,
  VREF supply voltage compensation, division-by-zero handling
- **CAN TX Queue**: Message enqueue (add_can_message), dequeue via HAL stub
  (handle_can_tx), FIFO ordering, mailbox-full backpressure, 64-entry array
  limits

### State Machine Logic
- **BLE Configuration**: Full 6-state BLE module setup sequence: idle→enter→
  wait_enter→send_cmd→wait_cmd→done. 5 configuration commands transmitted
  with 500ms inter-command delays, no-UART interaction before start()
- **initial_sequence()**: Full 8-stage startup walkthrough, WDT timeout,
  pneumatic out-of-range, solenoid timeout, Handle_autonomous_state transition
- **Handle_state()**: Start→IDLE, IDLE→AS_ON (ASMS rising edge), EMERGENCY
  hold and recovery to IDLE
- **Handle_autonomous_state()**: Initial_Sequence→Monitor_sequence transition
  after successful startup

### Safety & Monitoring with Time Dependencies
- **ASSI_control()**: All 5 AS states (OFF/READY/DRIVING/EMERGENCY/FINISHED),
  flashing frequency verification for DRIVING and EMERGENCY
- **module_timeout()**: All 5 timeout types (VCU, JETSON, PRESSURE, DIR, RES)
  plus normal (no timeout) case
- **continuous_monitoring()**: SDC open, pneumatic out-of-range, hydraulic
  correlation failure, normal pass-through

### MCU Callbacks (tested by calling directly with arranged state)
- **HAL_ADC_ConvCpltCallback()**: ADC raw→voltage→pressure conversion math,
  temperature computation via GetTemperature, correct updates to t24.Front_Pressure
  and t24.Rear_Pressure
- **HAL_GPIO_EXTI_Callback()**: Mission selector button press (increment with wrap),
  suppress when mission_selector_enable=0, ignore non-MS_BTN pins
- **HAL_UARTEx_RxEventCallback()**: UART command parsing — "log" sets dump flag,
  "resume" clears it, whitespace stripping, case insensitivity, unknown commands
  ignored, USART2 instance filtering
- **HAL_UART_TxCpltCallback()**: BLE TX busy flag cleared on USART2 transmit
  complete
- **HAL_TIM_PeriodElapsedCallback()**: CAN frame packing (ACU at 0x51, ASF_SIGNALS
  at 0x511), pressure ×10 scaling, BLE telemetry transmission via DMA stub, value
  clamping (uint16 saturation at 65535, int16 saturation at ±32767, negative→0),
  DMA timeout recovery (abort on stuck TX >500ms, automatic re-send)

### CAN Message Routing
- **dbc_decode()**: Routing of 4 CAN frame types (AQT7, VCU_IGN_R2_D, JETSON,
  VCU_RPM) and correct field updates on the `t24` struct

### Data Persistence
- **EEPROM Logger**: Fresh init, existing-log detection, write/read, multiple
  entries, circular overwrite (19-entry capacity), out-of-range read, count
  function, clear+rewrite

## What Is NOT Tested on Host

The following modules depend on STM32 HAL registers, interrupt handlers, or
external hardware and are **not compiled on the host**:

| Module | Reason |
|--------|--------|
| `can.c`, `adc.c`, `dma.c`, `gpio.c`, `i2c.c`, `tim.c`, `usart.c` | HAL peripheral configuration |
| `stm32f4xx_hal_msp.c`, `stm32f4xx_it.c` | Interrupt vector table and HAL callbacks |
| `syscalls.c`, `sysmem.c`, `system_stm32f4xx.c` | Platform startup code |
| `rn4871.c` | BLE UART driver (depends on HAL UART) |
| `hardware_abstaction.c` | `Peripheral_aquisition()`, `Peripheral_actuation()`, `handle_can_tx()`, etc. |

These should be tested on target using STM32CubeIDE debugger, logic analyzer,
or hardware-in-the-loop (HIL) testing.

## Architecture

Each test file follows this pattern:

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

This approach avoids any dependency on:
- `stm32f4xx_hal.h` and CMSIS headers
- Cross-compiler toolchains (`arm-none-eabi-gcc`)
- Linker scripts or hardware memory maps
- Physical peripherals (GPIO, ADC, CAN, UART)

## Adding New Tests

1. Create `tests/test_<module>.c` following the pattern above
2. Copy only the function bodies you need (not the entire file)
3. Add the test target to `tests/Makefile` `TARGETS` list
4. Update this documentation in `tests/docs/README.md`
5. Verify with: `gcc -o tests/test_<module> tests/test_<module>.c -Wall -Wextra -std=c11 -lm`

## Test Reports

Running `make report` or `./tests/run_tests.sh` generates a detailed report at
`tests/report/test_report.txt`. The report includes:

- Per-test section with test name
- Full console output showing test conditions, steps, and individual assertions
- Overall PASS/FAIL verdict per test with exit code
- Summary line with pass/total count

Each test prints its conditions inline, for example:

```
=== TEST 1: Normal successful startup ===
  PASS: Normal startup completed all 8 stages correctly
=== TEST 2: WDT timeout ===
  PASS: WDT timeout correctly triggers EMERGENCY
=== Results: 5 passed, 0 failed ===
```

## CI Integration

The test suite runs entirely on the host with no hardware dependencies, making
it suitable for CI pipelines (GitHub Actions, GitLab CI, Jenkins, etc.).

```yaml
# Example GitHub Actions step
- name: Run ACU tests
  run: |
    sudo apt-get install -y gcc make
    cd tests && make report
```
