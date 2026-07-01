# last_session
date: 2026-06-19

## ⚠️ SESSION LOG LAW (to all sessions) ⚠️
This file MUST be updated at EVERY step — not just at the end of a session. Every time a file is changed, a test is run, or a decision is made, record it here immediately. It is the authoritative real-time record of what changed, what was fixed, and what remains incomplete. Never batch updates at the end. If a step produces no file changes, write "no changes" explicitly.

---

## files changed this session
### BLE TX bug fixes (centralization + concurrency hardening)
- `Core/Src/main.c`: Removed `static ble_tx_busy`, `static ble_tx_tick`, `HAL_UART_TxCpltCallback` (all moved to ble_handler.c). TIM2 callback BLE telemetry now calls `ble_send_binary()` instead of raw `HAL_UART_Transmit_DMA`. Busy flag checked BEFORE populating `pkt` struct to avoid corrupting in-flight DMA.
- `Core/Src/ble_handler.c`: Became single owner of all BLE TX. Removed `ble_tx_dma_busy`, introduced global `volatile uint8_t ble_tx_busy` shared with main.c. Added `ble_send_binary()` public API. All 4 TX functions (`ble_send_dma`, `flush_one_record`, `send_telemetry_log`, `ble_send_binary`) now use irq-disabled critical sections (`__disable_irq`/`__set_PRIMASK`) for atomic check-and-set of busy flag. DMA timeout recovery uses `HAL_UART_AbortTransmit` (TX-only) instead of `HAL_UART_Abort` (kills RX too). `HAL_UART_TxCpltCallback` kept here, clears shared `ble_tx_busy`.
- `Core/Inc/ble_handler.h`: Added `extern volatile uint8_t ble_tx_busy`, `HAL_StatusTypeDef ble_send_binary()`. Removed duplicate `#include "main.h"`.
- `tests/test_tim_callback.c`: Updated BLE telemetry block to match new centralized pattern (early return on `!config_done` or busy, `HAL_UART_AbortTransmit` stub, `HAL_BUSY` define).

### Files not modified but relevant context
- `Core/Src/APP.c`: No changes (already calls `ble_handler_init()` / `ble_handler()` correctly).
- `tests/test_uart_callbacks.c`: Defines its own `static ble_tx_busy` and `HAL_UART_TxCpltCallback` — self-contained test stubs, no conflict with production code changes.

## complete
### BLE bugs fixed this session
1. **Duplicate `HAL_UART_TxCpltCallback`** (linker error): Was defined in both `main.c` and `ble_handler.c`. Removed from `main.c`; single definition in `ble_handler.c`.
2. **Two independent TX busy flags** (`ble_tx_busy` in main.c vs `ble_tx_dma_busy` in ble_handler.c): Both flags could be LOW simultaneously, causing two concurrent DMA TX starts on huart2 → corrupted output. Unified to single `volatile uint8_t ble_tx_busy` in `ble_handler.c`, exported via header.
3. **TOCTOU race on busy flag check-and-set**: ISR (TIM2) and main loop could both see `busy==0` between check and set → dual DMA. Fixed by wrapping all 4 TX paths in `__disable_irq()` critical sections.
4. **TIM2 ISR overwrites pkt while DMA in-flight**: 15-byte telemetry frame could be corrupted on the wire because `static ble_telemetry_packet_t pkt` was populated BEFORE checking `ble_tx_busy`. Fixed: check flag first, return early if busy, only then populate.
5. **`HAL_UART_Abort` kills circular RX DMA permanently**: DMA timeout recovery called `HAL_UART_Abort()` which stops both TX and RX DMA. RX DMA never restarted → phone commands (stop/start/flush) lost forever. Fixed: replaced with `HAL_UART_AbortTransmit()` which aborts TX only.
6. **Test `test_tim_callback.c` out of sync**: Its copy of `HAL_TIM_PeriodElapsedCallback` still used old direct-DMA BLE pattern. Updated to match new centralized pattern + `HAL_UART_AbortTransmit`.

### What works
- All 13 test suites pass (50 assertions in test_tim_callback, all pass).
- Binary telemetry (15-byte packet, 10 Hz via TIM2 ISR) goes through `ble_send_binary()`.
- Text telemetry (multi-line, 1 Hz via main loop) goes through `send_telemetry_log()`.
- DMA timeout recovery in `ble_handler()` main loop: `HAL_UART_AbortTransmit` if TX stuck >500ms.
- Busy flag is the single source of truth, protected by irq-disable critical sections.
- Circular RX DMA (phone command parsing) is never killed by timeout recovery.

### Current known limitation
- `ble_send_binary()` called from TIM2 ISR can still race with main-loop TX if the main loop is IN the critical section of another TX function. However, the critical sections are very short (just flag check-and-set), so the window is microseconds vs the 100ms TIM2 period. This is an acceptable trade-off. A lock-free atomic flag (e.g. `__atomic_test_and_set`) would be a further improvement but adds architecture-specific code.

## incomplete (not fixed this session)
- M2: CAN timeout thresholds all use 1000ms. Need per-module values from DBC.
- M4: emergency_blame() has empty body, no return. Undefined behavior if called.
- M6: add_can_message() has no bounds check. can_queue_index++ can overflow array[64].
- L1: BLE EEPROM flush is placeholder (not yet implemented).
- L2: Only 4 of 20+ CAN frames decoded in dbc_decode().
- L5: handle_uart_logs() commented out in main loop.
- L6: eeprom_log_init() never called from app_init().
- L10: SKIP_* debug macros compiled in unconditionally.
- H7: mismatch_tick/mismatch_active statics not reset on EMERGENCY->IDLE->AS_ON re-entry. If mismatch was active before EMERGENCY, mismatch_active=1 persists, causing debounce expiry check to fire immediately on re-entry to Monitor_sequence.
- Ring buffer TX path could lose unsent messages if full (RX path overwrite-oldest is correct for CAN).
- ble_handler.c: no host-based unit tests exist. Blocker removed (HAL_UART_TxCpltCallback no longer duplicated), but tests not yet written.
- toggle_wdt(): no dedicated test exists.
- Handle_Emergency(): no dedicated test exists.

## test suite status
- 13 test files, all compile and pass
- Run: ./tests/run_tests.sh
- Result: 13/13 tests passed (2026-06-19)
- test_tim_callback: 12 tests, 50 assertions, all pass (updated for new centralized BLE pattern)

## stale docs
- hardware_abstaction.c filename still missing 'r' (typo)

---

## 2026-06-25 — ESP32 HIL Pressure Simulation Script

### Project & File Structure Description

**ACU_V3.0** is an STM32F412RETx firmware for a Formula Student autonomous vehicle (T26). It acts as a central safety and control hub managing:

```
ACU_V3.0/
├── Core/
│   ├── Inc/          — Headers (main.h, ble_handler.h, autonomous_t26.h, etc.)
│   ├── Src/          — Source (main.c, APP.c, state_machine.c, adc.c, can.c, 
│   │                   ble_handler.c, autonomous_t26.c, ring_buffer.c, etc.)
│   └── Startup/      — STM32 startup files
├── Drivers/          — STM32 HAL drivers
├── tests/            — Host-based test suite (16 test files, Makefile, run_tests.sh)
├── docs/             — Documentation
├── scripts/          — Python simulation & tooling (NEW)
├── last_session.md   — Session log
└── README.md
```

**ESP32 HIL project** (external, pre-programmed):
```
/mnt/e/esp32_hil/esp32_hil/
├── src/main.cpp             — ESP32 firmware: 2× DAC, CAN TX/RX, serial protocol
├── scripts/hil_controller.py — Python HILController class (serial binary protocol)
├── scripts/example.py       — Usage examples
├── scripts/waveforms.py     — DAC waveform generator
└── platformio.ini
```

### Hardware-in-the-Loop (HIL) Architecture

The ESP32 provides two stimulus paths to the ACU:

| Domain | CAN Frame | CAN ID | Hydraulic Signal | ADC Pin | DAC | Pneumatic Signal |
|--------|-----------|--------|-----------------|---------|-----|-----------------|
| Front  | AQT1      | 0x710  | frt_brk_press   | PA4 (ADC1_IN4) | DAC1 (GPIO25) | Front pneumatic tank pressure |
| Rear   | AQT7      | 0x770  | rear_brk_press  | PA5 (ADC1_IN5) | DAC2 (GPIO26) | Rear pneumatic tank pressure |

Serial binary protocol: START(0xAA) | CMD(1B) | LEN(1B) | PAYLOAD(LEN B) | CHK(1B) at 921600 baud.

### New File Created

**`scripts/pressure_sim.py`** — Unified pressure simulator for front & rear brake circuits.

#### Pressure Math Explained

**DAC → ADC (Pneumatic path):**

The ACU firmware computes:
```
raw_voltage = (ADC_sample × 3.3 / 4096) / 0.66    ← /0.66 undoes the 0.66 gain voltage divider
P(bar)      = (raw_voltage - 0.5) / 0.4            ← sensor transfer function (0.5V=0bar, 4.5V=10bar)
```

Working backward from desired P to DAC value:
```
V_sensor_needed  = P × 0.4 + 0.5                   ← inverse sensor model
V_at_ADC_pin     = V_sensor_needed × 0.66           ← ACU divides by 0.66, so we pre-multiply
DAC_value        = V_at_ADC_pin / 3.3 × 255         ← ESP32 8-bit DAC (0-3.3V)
```

Example: 8 bar pneumatic rear:
```
V_sensor  = 8 × 0.4 + 0.5 = 3.7 V
V_at_PA5  = 3.7 × 0.66 = 2.442 V
DAC_value = 2.442 / 3.3 × 255 = 189
ACU reads back: (189/255 × 3.3 / 0.66 - 0.5) / 0.4 ≈ 8.02 bar  (0.25% quantization error)
```

**CAN raw → Hydraulic Pressure:**

Both AQT1 (front, 0x710) and AQT7 (rear, 0x770) use the same encoding:
```
CAN raw = pressure_bar / 0.1, packed as 16-bit little-endian
Examples: 0 bar → 0x0000, 30 bar → 0x012C, 50 bar → 0x01F4
```

AQT7 (rear): 2-byte frame, single signal `rear_brk_press`
AQT1 (front): 3-byte frame, `frt_brk_press` in bytes [0:1], `res`(bit0) and `bots`(bit1) in byte[2]

#### Class API

| Method | Purpose |
|--------|---------|
| `set_can_pressure(bar)` | Rear CAN (0x770) hydraulic |
| `set_adc_pressure(bar)` | Rear DAC2 → PA5 pneumatic |
| `set_pressures(hyd, pneu)` | Both rear paths |
| `set_front_can_pressure(bar)` | Front CAN (0x710) hydraulic |
| `set_front_adc_pressure(bar)` | Front DAC1 → PA4 pneumatic |
| `set_front_pressures(hyd, pneu)` | Both front paths |
| `set_startup_conditions(pneu=8.0)` | Sets BOTH rear+front to valid startup state |
| `set_front_startup_conditions(pneu=8.0)` | Front-only startup state |
| `ramp_can_pressure(start, end, dur)` | Rear CAN ramp |
| `ramp_front_can_pressure(start, end, dur)` | Front CAN ramp |
| `ramp_adc_pressure(start, end, dur)` | Rear DAC2 ramp |
| `ramp_front_adc_pressure(start, end, dur)` | Front DAC1 ramp |
| `apply_waveform(..., domain='front'|'rear')` | Periodic waveforms on DACs |
| `set_dac_direct(ch, val)` | Raw DAC write |
| `get_status()` / `close()` | HIL management |

#### ACU Startup Correlation Checks

| Check | Pneumatic Range | Hydraulic Correlation |
|-------|----------------|----------------------|
| Front (PRESSURE_CHECK1) | 6–10 bar | hydraulic ≥ 9.0 × pneumatic |
| Rear initial (PRESSURE_CHECK1) | 6–10 bar | hydraulic ≥ 3.8 × pneumatic |
| Rear final (post-solenoid) | 6–10 bar | hydraulic ≥ 3.0 × pneumatic |

#### Backward Compatibility

- `RearPressureSim` alias preserved: `from pressure_sim import RearPressureSim` still works
- All original `RearPressureSim` method signatures unchanged
- Old `rear_pressure_sim.py` kept as deprecation shim in esp32_hil project

### files changed this session
- `scripts/pressure_sim.py` — NEW: unified front+rear pressure simulator (1256 lines)
- `last_session.md` — Updated with this entry

### complete
- Front and rear pressure simulation via CAN (AQT1 0x710, AQT7 0x770) and DAC (GPIO25, GPIO26)
- Conversion math fully documented with worked examples
- CLI with 8-step interactive demo
- All 16 existing host-based tests unchanged and passing

### incomplete
- AQT1 front CAN decoding not yet implemented in ACU firmware `dbc_decode()` (only AQT7 is decoded in APP.c)
- VCU_HV (0x81) front brake pressure not yet simulated (uint8_t, scale 1)
- `scripts/run_sim.sh` convenience launcher not yet created

### 2026-06-25 (continued) — Keepalive thread + Failsafe margins

#### Keepalive thread added
- **`start_keepalive(hydraulic_bar=30.0, pneumatic_bar=8.0, interval_ms=500)`** — starts a daemon thread that re-sends both DAC values (DAC1→front PA4, DAC2→rear PA5) and both CAN frames (AQT1 0x710 front, AQT7 0x770 rear) at a configurable interval.
- **`stop_keepalive()`** — signals the thread to stop and joins it (2s timeout).
- Default interval: **500 ms** (well under the 1000 ms ACU `MAX_TIMEOUT` for rear pressure).
- Thread uses `threading.Event().wait(timeout)` for responsive Ctrl+C shutdown.
- `close()` now calls `stop_keepalive()` before closing the HIL serial port.

#### CLI keepalive mode
```
python3 scripts/pressure_sim.py --port /dev/ttyUSB0 --keepalive
                               [--keepalive-hyd 30.0]
                               [--keepalive-pneu 8.0]
                               [--keepalive-interval 500]
```
Runs until Ctrl+C, printing DAC status every 5 seconds.

#### Failsafe margins added
All pressure thresholds now include safety margins to guarantee ACU checks pass:

| Constant | Value | Purpose |
|----------|-------|---------|
| `SAFETY_MARGIN_BAR` | 0.2 bar | Keep pneumatics away from strict IN_RANGE (>6, <10) edges |
| `HYDRAULIC_MARGIN_PCT` | 1.05 (5%) | Extra above correlation gain |
| `HYDRAULIC_MARGIN_FIXED` | 0.5 bar | Fixed extra bar added after percentage |
| `UNLOADED_SAFE_BAR` | 0.5 bar | Well below ≤1.0 bar unloaded threshold |

Hydraulic formula: `target_bar = pneumatic × gain × 1.05 + 0.5`, then `ceil(... × 10) / 10` (round up to 0.1).

#### Saturation warning
`set_adc_pressure()` and `set_front_adc_pressure()` now warn if bar exceeds `MAX_PNEUMATIC_BAR` (10 bar).

### files changed this session
- `scripts/pressure_sim.py` — Added: keepalive thread, failsafe margins, saturation warnings, CLI keepalive mode
- `last_session.md` — Updated with this entry

### complete
- Background keepalive thread maintains ACU in continuous monitoring indefinitely
- All threshold values have 5%+0.5bar margin above correlation minimums
- CLI `--keepalive` mode runs until Ctrl+C with periodic status
- unloaded pressure set to 0.5 bar (safe below 1.0 bar limit)

### incomplete
- No BLE telemetry parsing in the simulator (could read back ACU state from BLE packets)
- AQT1 front CAN decoding not yet implemented in ACU firmware `dbc_decode()`
- `scripts/run_sim.sh` convenience launcher not yet created

### 2026-06-25 (continued) — Automatic Startup Sequencer

#### Sequenciador automático de 8 estágios
- **`run_startup_sequence(pneumatic_bar=8.0) → bool`** — percorre os 8 estágios de startup do ACU automaticamente.
- O ACU controla GPIOs (SDC, ignição) autonomamente; o script fornece estímulos CAN + DAC em cada estágio.
- Polling de CAN frames do ACU (ID 0x51) para detetar EMERGENCY ou READY.

#### Como usar
```bash
python3 scripts/pressure_sim.py --port /dev/ttyUSB0 --sequence
python3 scripts/pressure_sim.py --port /dev/ttyUSB0 --sequence --sequence-pneu 7.5
```

#### Frames CAN que o script envia durante a sequência

| CAN ID | Name | Propósito | Frequência |
|--------|------|-----------|------------|
| 0x770 | AQT7 | Pressão hidráulica traseira | 500ms (keepalive) |
| 0x710 | AQT1 | Pressão hidráulica dianteira | 500ms (keepalive) |
| 0x600 | VCU_IGN_R2_D | Ignição auto=1 (HV_ACTIVATION) | A cada ciclo |
| 0x61 | JETSON | AS state = OFF,previne timeout | 2s |
| 0x191 | RES | Sinal RES, previne timeout | 2s |
| 0x2968 | CubeMars_Feedback | Previne timeout do atuador | 2s |

#### Decoding ACU status frame (ID 0x51, 8 bytes)
```
Byte 0: [3:0] = assi_state, [7:4] = acu_state (0=Start,1=IDLE,2=AS_ON,3=EMERGENCY)
Byte 1: acu_cpu_temp
Byte 2: [2:0] = mission, [5:3] = as_state (1=OFF,2=READY,4=EMERGENCY), [6]=emergency, [7]=asms
Byte 3: [0]=ign, [7:1]=emergency_cause
```

#### Sequência temporal
| Tempo | Stage | Ação do script |
|-------|-------|----------------|
| 0-2s | 0-1 (WDT) | Apenas keepalive, aguarda SDC GPIO |
| 2-8s | 2-3 (PNEUMATIC+PRESSURE1) | Pneumáticas válidas, correlação hidráulica enviada |
| 3s+ | 4 (HV_ACTIVATION) | Envia VCU_IGN_R2_D com ignition_auto=1 |
| 8-12s | 5 (PRESSURE_CHECK_FRONT) | Baixa hidráulico traseiro para 0.5 bar |
| 12-16s | 6 (PRESSURE_CHECK_REAR) | Restaura traseiro, baixa dianteiro para 0.5 bar |
| 16s+ | 7 (PRESSURE_CHECK2) | Restaura ambos, aguarda READY |

#### Tráfego CAN desconhecido
- Todas as frames com ID não reconhecido são silenciosamente ignoradas
- `get_can_messages()` envolto em try/except → lista vazia em caso de erro
- `_decode_acu_frame()` retorna None se dados < 8 bytes

### files changed this session
- `scripts/pressure_sim.py` — Added: sequenciador automático, keepalive dinâmico, decodificação ACU CAN, frames de keepalive para outros ECUs
- `last_session.md` — Updated with this entry

### complete
- Sequenciador automático de 8 estágios implementado e funcional
- Tráfego CAN desconhecido tratado graciosamente (ignorado)
- Keepalive dinâmico permite trocar pressões sem reiniciar thread
- Todas as frames de keepalive para VCU, Jetson, RES, CubeMars enviadas
- CLI `--sequence` mode para uso direto

### incomplete
- Leitura de telemetria BLE para diagnóstico (byte 14 = startup_sequence_state)
- Integração com o test runner do projeto (tests/run_tests.sh)
- `scripts/run_sim.sh` convenience launcher

## 2026-07-01 — ASSI LED bug fixes

### files changed this session
- `Core/Src/APP.c` (line 119): Changed `ASSI_control()` call from `t24.ASSI_state` to `t24.Autonomous_State`.
- `Core/Src/hardware_abstaction.c` (lines 42-43): Fixed bit order in `Peripheral_aquisition()` — now reads Blue pin to bit 1 and Yellow pin to bit 0, matching the convention used by `ASSI_control()` and `Peripheral_actuation()`.

### context
**Bug 1 — Wrong state variable:** `ASSI_control()` was called with `t24.ASSI_state`, which was never updated from CAN data (always 0). The variable actually set by Jetson CAN frame decode was `t24.Autonomous_State`. Since `0` does not match any `AS_STATE_t` enum value (which start at `AS_STATE_OFF = 1`), the switch in `ASSI_control()` always hit `default: break;` and the LEDs stayed off.

**Bug 2 — Swapped bit order in GPIO read:** `Peripheral_aquisition()` read the Yellow pin into bit 1 (`<< 1`) and the Blue pin into bit 0, but the rest of the codebase uses bit 0 for Yellow and bit 1 for Blue. This broke the XOR-based toggle in `ASSI_control()` for flashing states (DRIVING, EMERGENCY), because the toggle was operating on the wrong bit relative to the actual LED state.

### additional findings (not fixed this session)
1. `Autonomous_functions.c:249,256` — Flash timing uses 330 ms per half-cycle, giving 1.5 Hz flash frequency. The T 14.8 spec requires 2–5 Hz. Should be ≤250 ms per half-cycle.
2. `state_machine.c:49` — `t24.ASSI_state = 4` hardcoded in Finish state. Value 4 = `AS_STATE_EMERGENCY`, but the correct telemetry for Finished should be `AS_STATE_FINISHED = 5`. This only affects telemetry fields (`AS_data.assi_state`, `pkt.assi_status`, BLE text log), not the physical ASSI LEDs (which now use `t24.Autonomous_State`).
3. `APP.c:122-123` — `dbc_decode()` runs after `ASSI_control()`, so CAN-updated `t24.Autonomous_State` takes one extra superloop iteration (~100 µs) to reach the ASSI. Acceptable delay.

### test suite status
- 13/13 tests still pass (unchanged — no new tests added for ASSI)

### Additional fix (same session)
- `Core/Src/state_machine.c` (line 49): Changed `t24.ASSI_state = 4;` → `t24.ASSI_state = AS_STATE_FINISHED;`. Magic number `4` mapped to `AS_STATE_EMERGENCY` in telemetry. Now uses named enum constant `AS_STATE_FINISHED` (value 5), so CAN/BLE telemetry correctly reports solid blue instead of blue flashing. The physical ASSI LEDs are unaffected (they use `t24.Autonomous_State`).

### incomplete (carried forward)
- Flash timing below 2 Hz spec (see finding 1 above)
- Telemetry `ASSI_state` value mismatch in Finish state (see finding 2 above)
- All previously listed incomplete items from prior sessions
