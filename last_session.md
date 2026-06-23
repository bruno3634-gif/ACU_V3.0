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
