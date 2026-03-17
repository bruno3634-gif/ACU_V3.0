# Tests — ACU V3.0 Unit Tests

Host-side unit tests for the ACU firmware. They compile and run on any Linux or macOS machine with `gcc` — **no STM32 hardware, no STM32CubeIDE, and no cross-compiler required**.

---

## How it works

The real firmware includes `stm32f4xx_hal.h` which is only available in the STM32 toolchain. The test build satisfies this dependency with a **minimal stub** (`Tests/stubs/stm32f4xx_hal.h`) that contains only `#include <stdint.h>`. The include path puts the stub directory first so the host compiler finds it instead of the real HAL.

The full include chain for the test build:

```
test_initial_sequence.c
  └─ Autonomous_functions.h
       └─ main.h
            ├─ stm32f4xx_hal.h   ← stub (from Tests/stubs/)
            ├─ application types (enums, structs)
            └─ app_types.h       ← initial_seq_* types and thresholds
```

---

## Running the tests

```bash
cd Tests
make          # compiles and runs automatically
make clean    # removes build artefacts
```

Expected output (all passing):

```
=== initial_sequence() unit tests ===

  test_watchdog_activates_wdt_and_advances_when_sdc_closed  OK
  test_watchdog_activates_wdt_while_sdc_open                OK
  test_watchdog_sdc_open_at_timeout_goes_to_error           OK
  test_watchdog_sdc_open_just_before_timeout_stays          OK
  test_pressure_ok_advances_to_hv_activation                OK
  test_pressure_front_low_goes_to_error                     OK
  test_pressure_rear_low_goes_to_error                      OK
  test_pressure_both_above_threshold_pass                   OK
  test_hv_activation_sets_ignition_request                  OK
  test_hv_activation_ignition_confirmed_advances            OK
  test_hv_activation_timeout_goes_to_error                  OK
  test_hv_activation_just_before_timeout_waits              OK
  test_pressure_correlation_within_tolerance_completes      OK
  test_pressure_correlation_zero_difference_completes       OK
  test_pressure_correlation_excess_difference_goes_to_error OK
  test_pressure_correlation_reverse_direction_also_checked  OK
  test_error_state_sets_vehicle_emergency                   OK
  test_full_happy_path                                      OK

=== Results: 41/41 passed ===
```

---

## Test coverage

| State tested | Tests | What is covered |
|---|---|---|
| `Watchdog_check` | 4 | WDT activated every tick; advance on SDC=1; stay within timeout; error at timeout |
| `Pressure_check` | 4 | Both above threshold; front low; rear low; both above with margin |
| `HV_activation` | 4 | Ignition request asserted; advance on confirmation; error at timeout; wait before timeout |
| `Pressure_correlation_check` | 4 | Within tolerance (exact); zero diff; exceeds tolerance; reverse direction |
| `Error_state` | 1 | vehicle_state set to EMERGENCY |
| Full happy path | 1 | All 4 states in sequence, inputs/outputs wired correctly across ticks |

---

## Adding a new test

Tests use a minimal inline framework (`ASSERT` + `TEST` macros). No external libraries are needed.

```c
TEST(test_my_new_case)
{
    initial_seq_ctx_t     ctx = make_ctx();   // starts at Watchdog_check, t=0
    initial_seq_inputs_t  in  = make_inputs(); // all zeroes / defaults
    initial_seq_outputs_t out = make_outputs(); // HW_WDT_Enable=1, vehicle_state=AS_ON

    /* Set up the scenario */
    ctx.state = Pressure_check;
    in.front_pneumatic_kPa = 600.0f;
    in.rear_pneumatic_kPa  = 600.0f;

    initial_sequence(&ctx, &in, &out);

    ASSERT(ctx.state == HV_activation);
}
```

Then add `test_my_new_case_run();` in `main()`.

---

## Files

| File | Description |
|---|---|
| `test_initial_sequence.c` | All tests for `initial_sequence()` |
| `Makefile` | Build recipe |
| `stubs/stm32f4xx_hal.h` | Minimal HAL stub for host compilation |
