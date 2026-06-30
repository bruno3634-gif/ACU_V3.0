#!/usr/bin/env python3
"""
Pressure Simulator for ACU V3.0 — Front & Rear
===============================================
Simulates brake pressure for both front and rear circuits using the ESP32 HIL
hardware.  Replaces the earlier ``rear_pressure_sim.py`` with a unified class.

Includes an **automatic startup sequencer** that walks the ACU through the
8-stage startup sequence by sending the right CAN frames and DAC voltages
at each stage.

Architecture
------------
The ACU reads pressure from two independent paths per channel:

  **CAN bus** — Hydraulic pressure (high-range, up to ~200 bar)
  **ADC**     — Pneumatic pressure (low-range, 0–10 bar sensor)

+-----------+-------------+---------------+------------------+
| Domain    | CAN Frame   | CAN ID        | ADC Input / DAC   |
+-----------+-------------+---------------+------------------+
| Front     | AQT1        | 0x710         | ADC1_IN4 / DAC1   |
| Rear      | AQT7        | 0x770         | ADC1_IN5 / DAC2   |
+-----------+-------------+---------------+------------------+

How DAC output voltage maps to ACU pressure reading
----------------------------------------------------
The ACU firmware computes (for *both* front on PA4 and rear on PA5):

    raw_voltage = (ADC_sample x 3.3 / 4096) / 0.66   ← /0.66 undoes hardware divider
    P(bar)      = (raw_voltage - 0.5) / 0.4           ← sensor transfer function

Working backward from desired P to DAC value:

    V_sensor_needed  = P x 0.4 + 0.5                  ← inverse sensor model (0.5V=0bar, 4.5V=10bar)
    V_at_PA5         = V_sensor_needed x 0.66          ← ACU divides by 0.66, so we pre-multiply
    DAC_value        = V_at_PA5 / 3.3 x 255            ← ESP32 8-bit DAC (0-3.3V)

Example: 8 bar pneumatic rear

    V_sensor  = 8 x 0.4 + 0.5 = 3.7 V
    V_at_PA5  = 3.7 x 0.66 = 2.442 V
    DAC_value = 2.442 / 3.3 x 255 = 189

ACU reads back: (189/255 x 3.3 / 0.66 - 0.5) / 0.4 = 8.02 bar  (0.25 % quantization error)

How CAN raw value maps to pressure
-----------------------------------
**AQT7 (0x770) — Rear Hydraulic:**

    CAN raw = pressure_bar / 0.1, packed as 16-bit LE
    Examples: 0 bar -> 0x0000, 30 bar -> 0x012C, 50 bar -> 0x01F4

**AQT1 (0x710) — Front Hydraulic:**

    CAN raw = pressure_bar / 0.1, packed as 16-bit LE in bytes [0:1]
    Byte [2] carries res(bit0) and bots(bit1) — set to 0 for simulation

Hardware mapping
----------------
    ESP32 DAC1 (GPIO25) -> ACU ADC1_IN4 (PA4)   — front pneumatic
    ESP32 DAC2 (GPIO26) -> ACU ADC1_IN5 (PA5)   — rear pneumatic
    ESP32 CAN           -> ACU CAN bus (1 Mbps)

Usage as CLI tool
-----------------
    python3 pressure_sim.py --port /dev/ttyUSB0
    python3 pressure_sim.py --port /dev/ttyUSB0 --table   (conversion table only)
    python3 pressure_sim.py --port /dev/ttyUSB0 --sequence  (startup sequencer)
    python3 pressure_sim.py --port /dev/ttyUSB0 --sequence --sequence-pneu 7.5

Usage as library
----------------
    from pressure_sim import PressureSim

    with PressureSim('/dev/ttyUSB0') as sim:
        # Rear (same API as old RearPressureSim)
        sim.set_pressures(hydraulic_bar=30.0, pneumatic_bar=8.0)
        sim.set_startup_conditions(pneumatic_bar=8.0)

        # Front (new)
        sim.set_front_pressures(hydraulic_bar=80.0, pneumatic_bar=8.0)
        sim.set_front_startup_conditions(pneumatic_bar=8.0)

        # Automatic startup sequencer
        sim.run_startup_sequence(pneumatic_bar=8.0)

    # Backward-compatible import still works:
    from pressure_sim import RearPressureSim

Safety margins
--------------
All pressure values use a 5% margin above correlation thresholds
and a 0.5 bar fixed extra to avoid floating-point edge cases with
the ACU's strict IN_RANGE (> min, < max) checks.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

# --- HIL Controller import ---
# Look for hil_controller.py in several locations:
# 1. Environment variable ESP32_HIL_SCRIPTS
# 2. Relative to this script: ../../esp32_hil/esp32_hil/scripts/
# 3. In the current directory
_hil_paths = [
    os.environ.get('ESP32_HIL_SCRIPTS', ''),
    os.path.abspath(os.path.join(os.path.dirname(__file__),
                                 '..', '..', '..', '..', 'esp32_hil', 'esp32_hil', 'scripts')),
    os.path.abspath(os.path.join(os.path.dirname(__file__),
                                 '..', '..', '..', 'esp32_hil', 'esp32_hil', 'scripts')),
    os.path.abspath(os.path.join(os.path.dirname(__file__),
                                 '..', '..', 'esp32_hil', 'esp32_hil', 'scripts')),
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'esp32_hil', 'esp32_hil', 'scripts'),
    os.getcwd(),
]
for _p in _hil_paths:
    if _p and os.path.isfile(os.path.join(_p, 'hil_controller.py')):
        if _p not in sys.path:
            sys.path.insert(0, _p)
        break
else:
    print("WARNING: Could not find hil_controller.py. "
          "Set ESP32_HIL_SCRIPTS env var or ensure the esp32_hil project is nearby.")

from hil_controller import HILController  # noqa: E402


# ---------------------------------------------------------------------------
# Constants — Rear (also kept for backward compat)
# ---------------------------------------------------------------------------

# AQT7 CAN frame ID for rear brake pressure (standard 11-bit)
CAN_AQT7_ID = 0x770

# ACU startup condition thresholds (from state_machine.c / test_initial_sequence.c)
EBS_MIN_BAR = 6.0       # Minimum pneumatic pressure for startup
EBS_MAX_BAR = 10.0      # Maximum pneumatic pressure for startup

# Rear hydraulic correlation gain used in startup sequence stage 3:
#   "Hyd correlation pre-ignition (Front >= 9xPneu, Rear >= 3.8xPneu)"
# See main.h PRESSURE_CHECK1 enum comment.
EBS_REAR_HYD_GAIN_INITIAL = 3.8   # Used during startup (PRESSURE_CHECK1)
EBS_REAR_HYD_GAIN_FINAL   = 3.0   # Used after solenoid engagement

# Kept for backward compat — users who relied on HYDRAULIC_CORRELATION = 3.0
# get the final-stage gain.  New code should use EBS_REAR_HYD_GAIN_INITIAL or
# EBS_REAR_HYD_GAIN_FINAL explicitly.
HYDRAULIC_CORRELATION = EBS_REAR_HYD_GAIN_FINAL

# Safety margins — always keep values away from strict thresholds
# ACU uses: IN_RANGE = (val > min && val < max)  ← strict!
#           IS_CORRELATED = (hyd >= gain * pneu)  ← floating-point risk
SAFETY_MARGIN_BAR = 0.2           # bar above min / below max for pneumatics
HYDRAULIC_MARGIN_PCT = 1.05       # 5% above correlation gain (multiply by 1.05)
HYDRAULIC_MARGIN_FIXED = 0.5      # extra 0.5 bar added after percentage
UNLOADED_SAFE_BAR = 0.5           # well below EBS_HYD_UNLOADED_BAR (1.0)

# DAC / ADC limits (shared by front and rear)
DAC_MAX = 255
DAC_VREF = 3.3           # ESP32 DAC reference voltage
ADC_VREF = 3.3           # STM32 ADC reference voltage
ADC_RESOLUTION = 4096    # 12-bit ADC
ADC_GAIN = 0.66          # Voltage divider gain on PA4/PA5 inputs
SENSOR_OFFSET_V = 0.5    # Pressure sensor: 0.5 V at 0 bar
SENSOR_SLOPE = 0.4       # Pressure sensor: 0.4 V/bar

# Maximum reasonable pressures
MAX_PNEUMATIC_BAR = 10.0   # Sensor range
MAX_HYDRAULIC_BAR = 200.0  # Practical limit for CAN encoding


# ---------------------------------------------------------------------------
# Constants — Front
# ---------------------------------------------------------------------------

# AQT1 CAN frame ID for front brake pressure (standard 11-bit)
CAN_AQT1_ID = 0x710

# Front hydraulic correlation gain (PRESSURE_CHECK1):
#   hydraulic >= EBS_FRONT_HYD_GAIN * pneumatic
EBS_FRONT_HYD_GAIN = 9.0


# ---------------------------------------------------------------------------
# Conversion Helpers — Rear / Generic
# ---------------------------------------------------------------------------

def pressure_to_can_raw(bar: float) -> int:
    """
    Encode pressure in bar to the 16-bit CAN raw value (little-endian).

    From autonomous_t26.c::

        uint16_t autonomous_t26_aqt7_rear_brk_press_encode(double value)
        {
            return (uint16_t)(value / 0.1);
        }

    Examples:
        0   bar ->      0 (0x0000)
        30  bar ->    300 (0x012C)
        50  bar ->    500 (0x01F4)
        123.4 bar -> 1234 (0x04D2)
    """
    raw = int(round(bar / 0.1))
    return max(0, min(0xFFFF, raw))


def can_raw_to_pressure(raw: int) -> float:
    """
    Decode a 16-bit CAN raw value back to bar.

    From autonomous_t26.c::

        double autonomous_t26_aqt7_rear_brk_press_decode(uint16_t value)
        {
            return ((double)value * 0.1);
        }
    """
    return float(raw & 0xFFFF) * 0.1


def pressure_to_dac_value(bar: float) -> int:
    """
    Convert pneumatic pressure (bar) to DAC2 register value (0-255).

    The ACU firmware computes::

        raw_voltage = (ADC_sample * 3.3 / 4096) / 0.66
        P(bar)      = (raw_voltage - 0.5) / 0.4

    Working backward from desired P to required DAC voltage at PA5::

        1. raw_voltage_needed = P * 0.4 + 0.5           (inverse sensor model)
        2. V_at_PA5           = raw_voltage_needed       (voltage at ADC pin)
        3. V_at_PA5           = V_dac * 0.66             (divider: DAC_out -> PA5)
           The /0.66 in the ACU code *undoes* the divider.
           To make the ACU read P bar we pre-multiply:
               V_dac = raw_voltage_needed * 0.66
        4. DAC_value = V_dac / 3.3 * 255

    Combined formula::

        DAC_value = ((P * 0.4 + 0.5) * 0.66) / 3.3 * 255

    Example: 8 bar -> ((3.2 + 0.5) * 0.66) / 3.3 * 255
                    = 2.442 / 3.3 * 255
                    = 188.7 -> 189

    The same formula applies to front pressure (DAC1 -> PA4).
    Use :func:`pressure_to_dac1_value` for an alias.

    Args:
        bar: Pneumatic pressure in bar (typically 0-10)

    Returns:
        DAC register value (0-255)

    Raises:
        ValueError: If bar is negative
    """
    if bar < 0.0:
        raise ValueError(f"Pressure cannot be negative: {bar}")

    # Sensor output voltage for this pressure
    sensor_voltage = bar * SENSOR_SLOPE + SENSOR_OFFSET_V

    # Voltage at ADC pin after the 0.66 gain divider.
    # The DAC drives the ADC pin directly, so V_pin = V_dac.
    # But the ACU firmware divides the measured pin voltage by 0.66 to
    # reconstruct the sensor voltage.  To make the ACU read the correct
    # pressure, we must pre-multiply by 0.66 so that after the ACU divides,
    # it gets back the original sensor voltage.
    v_pin = sensor_voltage * ADC_GAIN

    # Clamp to DAC output range
    v_pin = max(0.0, min(DAC_VREF, v_pin))

    # 8-bit DAC register value
    dac = int(round(v_pin / DAC_VREF * float(DAC_MAX)))
    return max(0, min(DAC_MAX, dac))


def dac_value_to_pressure(dac_value: int) -> float:
    """
    Inverse: given a DAC register value, compute what pressure the ACU reads.

    Useful for logging / verification.  Works for both DAC1 (front) and
    DAC2 (rear) since the sensor model is identical.

    Args:
        dac_value: DAC register value (0-255)

    Returns:
        Equivalent pneumatic pressure in bar
    """
    dac_value = max(0, min(DAC_MAX, int(dac_value)))
    v_dac = dac_value / DAC_MAX * DAC_VREF
    # ACU computes: raw_voltage = V_pin / 0.66, pressure = (raw_voltage - 0.5) / 0.4
    raw_voltage = v_dac / ADC_GAIN
    pressure = (raw_voltage - SENSOR_OFFSET_V) / SENSOR_SLOPE
    return max(0.0, pressure)


# ---------------------------------------------------------------------------
# Conversion Helpers — Front
# ---------------------------------------------------------------------------

def pressure_to_aqt1_raw(bar: float) -> int:
    """
    Encode pressure in bar to the 16-bit CAN raw value for AQT1 (front).

    Identical math to :func:`pressure_to_can_raw` but semantically tied to
    the front CAN frame.  From autonomous_t26_aqt1_pack()::

        dst_p[0] |= pack_left_shift_u16(src_p->frt_brk_press, 0u, 0xffu);
        dst_p[1] |= pack_right_shift_u16(src_p->frt_brk_press, 8u, 0xffu);

    Encoding::

        uint16_t autonomous_t26_aqt1_frt_brk_press_encode(double value)
        {
            return (uint16_t)(value / 0.1);
        }

    Args:
        bar: Hydraulic pressure in bar (0-6553.5, typically <= 200)

    Returns:
        16-bit CAN raw value
    """
    return pressure_to_can_raw(bar)


def pressure_to_dac1_value(bar: float) -> int:
    """
    Convert pneumatic pressure (bar) to DAC1 register value (0-255).

    Uses the identical sensor model as DAC2 (same sensor, same divider,
    same ADC formula).  This is simply an alias for :func:`pressure_to_dac_value`
    but named to make the DAC1 -> front-pneumatic mapping explicit.

    Args:
        bar: Pneumatic pressure in bar (typically 0-10)

    Returns:
        DAC register value (0-255)
    """
    return pressure_to_dac_value(bar)


# ---------------------------------------------------------------------------
# Waveform Generators (return lists of pressure values in bar)
# ---------------------------------------------------------------------------

def _generate_sine_wave(num_samples: int, amplitude: float,
                        offset: float) -> list[float]:
    """Return ``num_samples`` values of a sine wave."""
    return [
        offset + amplitude * math.sin(2.0 * math.pi * i / num_samples)
        for i in range(num_samples)
    ]


def _generate_sawtooth_wave(num_samples: int, amplitude: float,
                            offset: float) -> list[float]:
    """Return ``num_samples`` values of a sawtooth wave."""
    return [
        offset + amplitude * (2.0 * i / (num_samples - 1) - 1.0)
        for i in range(num_samples)
    ]


def _generate_square_wave(num_samples: int, amplitude: float,
                          offset: float) -> list[float]:
    """Return ``num_samples`` values of a square wave."""
    half = num_samples // 2
    samples: list[float] = []
    for i in range(num_samples):
        samples.append(offset + amplitude if i < half else offset - amplitude)
    return samples


# ---------------------------------------------------------------------------
# ACU Frame Decoder
# ---------------------------------------------------------------------------

def _decode_acu_frame(data: bytes) -> dict | None:
    """
    Decode ACU status frame (ID 0x51, 8 bytes).

    The ACU transmits its state machine status on CAN ID 0x51 at 10 Hz.
    Byte layout::

        Byte 0: [3:0] = assi_state, [7:4] = acu_state
        Byte 1: acu_cpu_temp
        Byte 2: [2:0] = mission_select, [5:3] = as_state, [6] = emergency, [7] = asms
        Byte 3: [0] = ign, [7:1] = emergency_cause

    Args:
        data: 8-byte CAN payload

    Returns:
        Decoded dict, or None if data is too short
    """
    if len(data) < 8:
        return None
    return {
        'assi_state': data[0] & 0x0F,
        'acu_state': (data[0] >> 4) & 0x0F,
        'acu_cpu_temp': data[1],
        'mission_select': data[2] & 0x07,
        'as_state': (data[2] >> 3) & 0x07,
        'emergency': (data[2] >> 6) & 0x01,
        'asms': (data[2] >> 7) & 0x01,
        'ign': data[3] & 0x01,
        'emergency_cause': (data[3] >> 1) & 0x7F,
    }


# ---------------------------------------------------------------------------
# CAN Frame Builders (for VCU, Jetson, RES, CubeMars keepalive)
# ---------------------------------------------------------------------------

def _build_vcu_ign_r2_d(ignition_auto: int = 1,
                         shutdown_signal: int = 1) -> bytes:
    """
    Build VCU_IGN_R2_D frame (ID 0x600, 8 bytes).

    Byte 2 = ignition_auto, Byte 4 = shutdown_signal.

    Args:
        ignition_auto:   Ignition auto state (0 or 1, default 1)
        shutdown_signal: Shutdown signal (0 or 1, default 1)

    Returns:
        8-byte payload
    """
    return bytes([0, 0, ignition_auto & 0xFF, 0,
                  shutdown_signal & 0xFF, 0, 0, 0])


def _build_jetson_frame(as_state: int = 1) -> bytes:
    """
    Build JETSON frame (ID 0x61, 5 bytes).

    Args:
        as_state: AS state (1=OFF, 2=READY, 3=DRIVING, 4=EMERGENCY, default 1)

    Returns:
        5-byte payload
    """
    return bytes([as_state & 0xFF, 0, 25, 25, 0])


def _build_res_frame() -> bytes:
    """
    Build RES frame (ID 0x191, 8 bytes) — just keepalive.

    Returns:
        8-byte payload
    """
    return bytes([1, 0, 0, 0, 0, 0, 0, 0])


def _build_cubemars_feedback() -> bytes:
    """
    Build CubeMars_Feedback frame (ID 0x2968 extended, 8 bytes) — keepalive.

    Returns:
        8-byte payload
    """
    return bytes([0, 0, 0, 0, 0, 0, 0, 0])


def _build_can_keepalive_payloads() -> list[tuple[int, bytes, bool]]:
    """
    Return list of (can_id, data, extd) tuples for keepalive frames.

    Returns:
        List of (CAN ID, payload, extended-flag) tuples
    """
    return [
        (0x600, _build_vcu_ign_r2_d(), False),
        (0x61,  _build_jetson_frame(), False),
        (0x191, _build_res_frame(), False),
        (0x2968, _build_cubemars_feedback(), True),
    ]


# ---------------------------------------------------------------------------
# PressureSim Class
# ---------------------------------------------------------------------------

class PressureSim:
    """
    Simulate brake pressure for ACU V3.0 via ESP32 HIL.

    Provides two independent output paths per domain (front / rear):

        ====== ===================== ======================
        Path   Front                 Rear
        ====== ===================== ======================
        CAN    AQT1 (0x710)          AQT7 (0x770)
        ADC    DAC1 (GPIO25 -> PA4)  DAC2 (GPIO26 -> PA5)
        ====== ===================== ======================

    Context-manager aware::

        with PressureSim('/dev/ttyUSB0') as sim:
            sim.set_pressures(hydraulic_bar=30.0, pneumatic_bar=8.0)   # rear
            sim.set_front_pressures(hydraulic_bar=80.0, pneumatic_bar=8.0)

    Args:
        port: Serial port for ESP32 HIL (e.g. '/dev/ttyUSB0' or 'COM3')
        baud: Serial baud rate (default 921600)
    """

    def __init__(self, port: str, baud: int = 921600):
        self._hil = HILController(port, baud=baud)
        self._keepalive_thread: Optional[threading.Thread] = None
        self._keepalive_stop = threading.Event()

    # ------------------------------------------------------------------
    # Public API — Rear (legacy, all original signatures preserved)
    # ------------------------------------------------------------------

    def set_can_pressure(self, bar: float) -> bool:
        """
        Send rear brake hydraulic pressure over CAN (AQT7, ID 0x770).

        Encodes bar as a 16-bit little-endian value::

            raw = bar / 0.1

        The ACU ``dbc_decode()`` in APP.c unpacks this frame and stores the
        decoded value in ``t24.Rear_Pressure.Hydraulic``.

        Args:
            bar: Hydraulic pressure in bar (0-6553.5, typically <= 200)

        Returns:
            True if the CAN frame was sent successfully
        """
        if bar < 0.0:
            raise ValueError(f"Cannot send negative pressure: {bar}")
        raw = pressure_to_can_raw(bar)
        # 2-byte little-endian payload: [low_byte, high_byte]
        data = struct.pack('<H', raw)
        ok = self._hil.can_send(CAN_AQT7_ID, data, extd=False)
        if not ok:
            print(f"WARNING: CAN send failed for pressure {bar:.1f} bar "
                  f"(raw=0x{raw:04X})")
        return ok

    def set_adc_pressure(self, bar: float) -> bool:
        """
        Set DAC2 (GPIO26) to simulate rear pneumatic pressure on ADC1_IN5 (PA5).

        The ACU firmware reads PA5 via ADC and computes::

            P(bar) = ((sample * 3.3 / 4096) / 0.66 - 0.5) / 0.4

        This function outputs the DAC voltage that produces the requested
        pressure reading on the ACU side.

        Args:
            bar: Pneumatic pressure in bar (typically 0-10)

        Returns:
            True if the DAC was written successfully
        """
        if bar < 0.0:
            raise ValueError(f"Cannot simulate negative pressure: {bar}")
        if bar > MAX_PNEUMATIC_BAR:
            print(f"WARNING: {bar:.1f} bar exceeds sensor max "
                  f"({MAX_PNEUMATIC_BAR} bar). "
                  f"ACU will read a saturated value.")
        dac_val = pressure_to_dac_value(bar)
        ok = self._hil.set_dac2(dac_val)
        if not ok:
            print(f"WARNING: DAC2 set failed for pressure {bar:.1f} bar "
                  f"(DAC={dac_val})")
        return ok

    def set_pressures(self, hydraulic_bar: float,
                      pneumatic_bar: float) -> Tuple[bool, bool]:
        """
        Set both CAN (hydraulic) and ADC (pneumatic) pressures for **rear**.

        DAC2 is set first (fire-and-forget), then the CAN frame is sent.

        Args:
            hydraulic_bar: Hydraulic pressure for CAN AQT7 frame
            pneumatic_bar: Pneumatic pressure for DAC2 -> ADC simulation

        Returns:
            Tuple ``(can_ok, adc_ok)`` where each is True on success
        """
        adc_ok = self.set_adc_pressure(pneumatic_bar)
        can_ok = self.set_can_pressure(hydraulic_bar)
        return can_ok, adc_ok

    def set_startup_conditions(self, pneumatic_bar: float = 8.0
                               ) -> Tuple[bool, bool, bool, bool]:
        """
        Set pressures that satisfy ACU startup conditions for **both** domains.

        The ACU initial sequence checks (PRESSURE_CHECK1, stage 3):

            - Rear pneumatic  in [EBS_MIN_BAR, EBS_MAX_BAR]   = [6, 10] bar
            - Rear hydraulic >= EBS_REAR_HYD_GAIN_INITIAL (3.8) x pneumatic
            - Front pneumatic in [EBS_MIN_BAR, EBS_MAX_BAR]   = [6, 10] bar
            - Front hydraulic >= EBS_FRONT_HYD_GAIN (9.0) x pneumatic

        This helper sets **both** rear and front to valid correlated values.

        Args:
            pneumatic_bar: Desired pneumatic pressure (default 8.0 bar),
                           must be in [6.0, 10.0]

        Returns:
            Tuple ``(rear_can_ok, rear_adc_ok, front_can_ok, front_adc_ok)``.
            All four values are boolean; True = OK.

        Raises:
            ValueError: If ``pneumatic_bar`` is outside the startup range
        """
        if not (EBS_MIN_BAR <= pneumatic_bar <= EBS_MAX_BAR):
            raise ValueError(
                f"Pneumatic pressure {pneumatic_bar:.1f} bar is outside "
                f"the startup range [{EBS_MIN_BAR}, {EBS_MAX_BAR}] bar"
            )

        # ---- Rear (backward-compat path) ----
        # Use the INITIAL gain (3.8) since this is for pre-ignition startup check
        # Add 5% margin + 0.5 bar fixed extra to guarantee IS_CORRELATED passes
        # even with floating-point jitter on the ACU side.
        raw_rear = (
            pneumatic_bar * EBS_REAR_HYD_GAIN_INITIAL
            * HYDRAULIC_MARGIN_PCT + HYDRAULIC_MARGIN_FIXED
        )
        rear_hydraulic_bar = math.ceil(raw_rear * 10.0) / 10.0
        can_ok, adc_ok = self.set_pressures(
            hydraulic_bar=rear_hydraulic_bar,
            pneumatic_bar=pneumatic_bar,
        )

        # ---- Front (always set alongside rear) ----
        raw_front = (
            pneumatic_bar * EBS_FRONT_HYD_GAIN
            * HYDRAULIC_MARGIN_PCT + HYDRAULIC_MARGIN_FIXED
        )
        front_hydraulic_bar = math.ceil(raw_front * 10.0) / 10.0
        f_can_ok = self.set_front_can_pressure(front_hydraulic_bar)
        f_adc_ok = self.set_front_adc_pressure(pneumatic_bar)
        if not f_can_ok:
            print(f"WARNING: Front CAN write failed for {front_hydraulic_bar:.1f} bar")
        if not f_adc_ok:
            print(f"WARNING: Front DAC write failed for {pneumatic_bar:.1f} bar")

        # Return tuple extended with front status (backward-compat: rear is first two)
        return can_ok, adc_ok, f_can_ok, f_adc_ok

    def set_dac_direct(self, channel: int, value: int) -> bool:
        """
        Set a DAC channel directly to a raw 8-bit value.

        Bypasses pressure-to-voltage conversion.  Useful for edge-case
        testing (e.g. output 0 V or full-scale).

        Args:
            channel: 1 = DAC1 (GPIO25 -> front pneumatic), 2 = DAC2 (GPIO26 -> rear)
            value:   8-bit register value (0-255)

        Returns:
            True if the DAC was written successfully

        Raises:
            ValueError: If channel is not 1 or 2
        """
        value = max(0, min(DAC_MAX, int(value)))
        if channel == 1:
            return self._hil.set_dac1(value)
        elif channel == 2:
            return self._hil.set_dac2(value)
        else:
            raise ValueError(f"Invalid DAC channel: {channel} "
                             f"(must be 1 or 2)")

    def ramp_can_pressure(self, start_bar: float, end_bar: float,
                          duration_s: float, steps: int = 50) -> None:
        """
        Smoothly ramp CAN (hydraulic) pressure over time.

        Sends on **AQT7 (0x770) — rear hydraulic**.  For front hydraulic,
        use :meth:`ramp_front_can_pressure`.

        Performs a linear interpolation from ``start_bar`` to ``end_bar``
        over ``duration_s`` seconds in ``steps`` discrete steps.

        Args:
            start_bar:  Starting hydraulic pressure (bar)
            end_bar:    Ending hydraulic pressure (bar)
            duration_s: Ramp duration (seconds)
            steps:      Number of intermediate steps (default 50)
        """
        if steps < 2:
            steps = 2
        delay = duration_s / steps
        for i in range(steps + 1):
            fraction = i / steps
            bar = start_bar + (end_bar - start_bar) * fraction
            self.set_can_pressure(bar)
            time.sleep(delay)

    def ramp_adc_pressure(self, start_bar: float, end_bar: float,
                          duration_s: float, steps: int = 50) -> None:
        """
        Smoothly ramp ADC (pneumatic) pressure over time on **DAC2 (rear)**.

        For front pneumatic, use :meth:`ramp_front_adc_pressure`.

        Performs a linear interpolation from ``start_bar`` to ``end_bar``
        over ``duration_s`` seconds in ``steps`` discrete steps.

        Args:
            start_bar:  Starting pneumatic pressure (bar)
            end_bar:    Ending pneumatic pressure (bar)
            duration_s: Ramp duration (seconds)
            steps:      Number of intermediate steps (default 50)
        """
        if steps < 2:
            steps = 2
        delay = duration_s / steps
        for i in range(steps + 1):
            fraction = i / steps
            bar = start_bar + (end_bar - start_bar) * fraction
            self.set_adc_pressure(bar)
            time.sleep(delay)

    def apply_waveform(self, dac_ch: int = 2,
                       waveform_type: str = 'sine',
                       freq_hz: float = 1.0,
                       amplitude_bar: float = 4.0,
                       offset_bar: float = 5.0,
                       duration_s: float = 4.0,
                       domain: Optional[str] = None) -> None:
        """
        Apply a periodic waveform to a DAC channel for ADC simulation.

        The waveform is defined in pressure units (bar).  The conversion
        to DAC voltage uses the pressure-sensor model so the ACU reads
        back the correct pressure.

        .. versionchanged:: 2.0
           Added ``domain`` parameter for explicit front/rear selection.
           When ``domain`` is ``None`` (default), the legacy behaviour is
           preserved (``dac_ch=1`` -> linear mapping, ``dac_ch=2`` ->
           pressure-sensor model).

        Args:
            dac_ch:        DAC channel (1 or 2).  Used when ``domain`` is None.
            waveform_type: ``'sine'``, ``'sawtooth'``, or ``'square'``
            freq_hz:       Fundamental frequency (Hz)
            amplitude_bar: Amplitude of the waveform (bar, peak deviation
                           from offset)
            offset_bar:    DC offset (bar)
            duration_s:    How long to play the waveform (seconds)
            domain:        Pressure domain for mapping.
                           - ``'front'`` -> DAC1 with pressure-sensor model
                           - ``'rear'``  -> DAC2 with pressure-sensor model
                           - ``None``    -> legacy ``dac_ch``-based behaviour

        Raises:
            ValueError: If ``dac_ch``, ``waveform_type``, or ``domain`` is
                        invalid, or if any bar value produces an invalid DAC
                        setting
        """
        # Validate domain
        valid_domains = (None, 'front', 'rear')
        if domain not in valid_domains:
            raise ValueError(
                f"Invalid domain: '{domain}'. "
                f"Use 'front', 'rear', or None (legacy)."
            )

        # Validate dac_ch when domain is not given
        if domain is None and dac_ch not in (1, 2):
            raise ValueError(f"Invalid DAC channel: {dac_ch} "
                             f"(must be 1 or 2)")

        if waveform_type not in ('sine', 'sawtooth', 'square'):
            raise ValueError(
                f"Unknown waveform type: '{waveform_type}'. "
                f"Use 'sine', 'sawtooth', or 'square'."
            )

        # Guard: ensure the waveform won't drive pressure negative
        if offset_bar - amplitude_bar < 0.0:
            amplitude_bar = min(amplitude_bar, offset_bar)
            print(f"NOTE: Clamped amplitude to {amplitude_bar:.2f} bar "
                  f"to keep pressure non-negative")

        # Parameters for sample generation
        samples_per_cycle = 100
        num_cycles = max(1, int(round(freq_hz * duration_s)))
        total_samples = num_cycles * samples_per_cycle
        sample_interval = 1.0 / (freq_hz * samples_per_cycle)

        # Generate one cycle of the waveform in bar units
        if waveform_type == 'sine':
            cycle_bar = _generate_sine_wave(
                samples_per_cycle, amplitude_bar, offset_bar)
        elif waveform_type == 'sawtooth':
            cycle_bar = _generate_sawtooth_wave(
                samples_per_cycle, amplitude_bar, offset_bar)
        else:  # square
            cycle_bar = _generate_square_wave(
                samples_per_cycle, amplitude_bar, offset_bar)

        # ------------------------------------------------------------------
        # Determine the DAC set function and conversion method
        # ------------------------------------------------------------------
        if domain == 'front':
            # DAC1 with proper pressure-sensor model for front pneumatic on PA4
            cycle_dac = [
                max(0, min(DAC_MAX, pressure_to_dac1_value(bar)))
                for bar in cycle_bar
            ]
            _set_fn = self._hil.set_dac1
            _is_pressure_model = True
        elif domain == 'rear':
            # DAC2 with proper pressure-sensor model for rear pneumatic on PA5
            cycle_dac = [
                max(0, min(DAC_MAX, pressure_to_dac_value(bar)))
                for bar in cycle_bar
            ]
            _set_fn = self._hil.set_dac2
            _is_pressure_model = True
        elif dac_ch == 1:
            # Legacy: DAC1, map 0-10 bar -> 0-255 (10 bar = full scale)
            cycle_dac = [
                max(0, min(DAC_MAX,
                    int(round(bar / MAX_PNEUMATIC_BAR * DAC_MAX))))
                for bar in cycle_bar
            ]
            _set_fn = self._hil.set_dac1
            _is_pressure_model = False
        else:
            # Legacy: DAC2, use the pressure-sensor voltage model
            cycle_dac = [
                max(0, min(DAC_MAX, pressure_to_dac_value(bar)))
                for bar in cycle_bar
            ]
            _set_fn = self._hil.set_dac2
            _is_pressure_model = True

        # Play the waveform with precise timing
        next_sample = time.perf_counter()
        for i in range(total_samples):
            idx = i % samples_per_cycle
            _set_fn(cycle_dac[idx])
            next_sample += sample_interval
            now = time.perf_counter()
            sleep_time = next_sample - now
            if sleep_time > 0.0:
                time.sleep(sleep_time)

        # Return to steady state at the offset pressure
        if _is_pressure_model:
            if _set_fn == self._hil.set_dac2:
                self.set_adc_pressure(offset_bar)
            else:
                self.set_front_adc_pressure(offset_bar)
        else:
            self._hil.set_dac1(0)

    # ------------------------------------------------------------------
    # Public API — Front (new)
    # ------------------------------------------------------------------

    def set_front_can_pressure(self, bar: float,
                               res: int = 0, bots: int = 0) -> bool:
        """
        Send front brake hydraulic pressure over CAN (AQT1, ID 0x710).

        Encodes bar as a 16-bit little-endian value in bytes [0:1]::

            raw = bar / 0.1

        Byte [2] carries ``res`` (bit 0) and ``bots`` (bit 1); both default
        to 0 for normal simulation.

        Encoding follows ``autonomous_t26_aqt1_pack()``::

            dst_p[0] |= pack_left_shift_u16(src_p->frt_brk_press, 0u, 0xffu);
            dst_p[1] |= pack_right_shift_u16(src_p->frt_brk_press, 8u, 0xffu);
            dst_p[2] |= pack_left_shift_u8(src_p->res, 0u, 0x01u);
            dst_p[2] |= pack_left_shift_u8(src_p->bots, 1u, 0x02u);

        Args:
            bar:  Hydraulic pressure in bar (0-6553.5, typically <= 200)
            res:  Reserve bit (bit 0 of byte 2), default 0
            bots: BOTS bit (bit 1 of byte 2), default 0

        Returns:
            True if the CAN frame was sent successfully
        """
        if bar < 0.0:
            raise ValueError(f"Cannot send negative pressure: {bar}")
        raw = pressure_to_aqt1_raw(bar)
        # Byte [0:1]: 16-bit LE pressure, Byte [2]: flags
        byte2 = (res & 0x01) | ((bots & 0x01) << 1)
        data = struct.pack('<HB', raw, byte2)
        ok = self._hil.can_send(CAN_AQT1_ID, data, extd=False)
        if not ok:
            print(f"WARNING: CAN AQT1 send failed for pressure {bar:.1f} bar "
                  f"(raw=0x{raw:04X})")
        return ok

    def set_front_adc_pressure(self, bar: float) -> bool:
        """
        Set DAC1 (GPIO25) to simulate front pneumatic pressure on ADC1_IN4 (PA4).

        Uses the same sensor model as the rear channel (same sensor type,
        same voltage divider, same ADC formula).  The ACU firmware reads
        PA4 via ADC and computes::

            P(bar) = ((sample * 3.3 / 4096) / 0.66 - 0.5) / 0.4

        Args:
            bar: Pneumatic pressure in bar (typically 0-10)

        Returns:
            True if the DAC was written successfully
        """
        if bar < 0.0:
            raise ValueError(f"Cannot simulate negative pressure: {bar}")
        if bar > MAX_PNEUMATIC_BAR:
            print(f"WARNING: {bar:.1f} bar exceeds sensor max "
                  f"({MAX_PNEUMATIC_BAR} bar). "
                  f"ACU will read a saturated value.")
        dac_val = pressure_to_dac1_value(bar)
        ok = self._hil.set_dac1(dac_val)
        if not ok:
            print(f"WARNING: DAC1 set failed for pressure {bar:.1f} bar "
                  f"(DAC={dac_val})")
        return ok

    def set_front_pressures(self, hydraulic_bar: float,
                            pneumatic_bar: float) -> Tuple[bool, bool]:
        """
        Set both CAN (hydraulic) and ADC (pneumatic) pressures for **front**.

        DAC1 is set first (fire-and-forget), then the CAN frame is sent on
        AQT1 (0x710).

        Args:
            hydraulic_bar: Hydraulic pressure for CAN AQT1 frame
            pneumatic_bar: Pneumatic pressure for DAC1 -> ADC simulation

        Returns:
            Tuple ``(can_ok, adc_ok)`` where each is True on success
        """
        adc_ok = self.set_front_adc_pressure(pneumatic_bar)
        can_ok = self.set_front_can_pressure(hydraulic_bar)
        return can_ok, adc_ok

    def set_front_startup_conditions(self, pneumatic_bar: float = 8.0
                                     ) -> Tuple[bool, bool]:
        """
        Set **front** pressures that satisfy ACU startup conditions.

        The ACU front correlation check (PRESSURE_CHECK1):

            - Front pneumatic in [EBS_MIN_BAR, EBS_MAX_BAR] = [6, 10] bar
            - Front hydraulic >= EBS_FRONT_HYD_GAIN (9.0) x pneumatic

        Args:
            pneumatic_bar: Desired pneumatic pressure (default 8.0 bar),
                           must be in [6.0, 10.0]

        Returns:
            Tuple ``(can_ok, adc_ok)``

        Raises:
            ValueError: If ``pneumatic_bar`` is outside the startup range
        """
        if not (EBS_MIN_BAR <= pneumatic_bar <= EBS_MAX_BAR):
            raise ValueError(
                f"Pneumatic pressure {pneumatic_bar:.1f} bar is outside "
                f"the startup range [{EBS_MIN_BAR}, {EBS_MAX_BAR}] bar"
            )

        # Set hydraulic well above the correlation threshold so floating-point
        # rounding cannot trip the >= comparison on the ACU side.
        # 5% margin + 0.5 bar fixed extra guarantees IS_CORRELATED passes.
        raw_hyd = (
            pneumatic_bar * EBS_FRONT_HYD_GAIN
            * HYDRAULIC_MARGIN_PCT + HYDRAULIC_MARGIN_FIXED
        )
        hydraulic_bar = math.ceil(raw_hyd * 10.0) / 10.0

        return self.set_front_pressures(hydraulic_bar=hydraulic_bar,
                                        pneumatic_bar=pneumatic_bar)

    # ------------------------------------------------------------------
    # Ramp helpers for front
    # ------------------------------------------------------------------

    def ramp_front_can_pressure(self, start_bar: float, end_bar: float,
                                duration_s: float, steps: int = 50) -> None:
        """
        Smoothly ramp front CAN (hydraulic) pressure over time on AQT1 (0x710).

        Performs a linear interpolation from ``start_bar`` to ``end_bar``
        over ``duration_s`` seconds in ``steps`` discrete steps.

        Args:
            start_bar:  Starting hydraulic pressure (bar)
            end_bar:    Ending hydraulic pressure (bar)
            duration_s: Ramp duration (seconds)
            steps:      Number of intermediate steps (default 50)
        """
        if steps < 2:
            steps = 2
        delay = duration_s / steps
        for i in range(steps + 1):
            fraction = i / steps
            bar = start_bar + (end_bar - start_bar) * fraction
            self.set_front_can_pressure(bar)
            time.sleep(delay)

    def ramp_front_adc_pressure(self, start_bar: float, end_bar: float,
                                duration_s: float, steps: int = 50) -> None:
        """
        Smoothly ramp front ADC (pneumatic) pressure over time on DAC1.

        Performs a linear interpolation from ``start_bar`` to ``end_bar``
        over ``duration_s`` seconds in ``steps`` discrete steps.

        Args:
            start_bar:  Starting pneumatic pressure (bar)
            end_bar:    Ending pneumatic pressure (bar)
            duration_s: Ramp duration (seconds)
            steps:      Number of intermediate steps (default 50)
        """
        if steps < 2:
            steps = 2
        delay = duration_s / steps
        for i in range(steps + 1):
            fraction = i / steps
            bar = start_bar + (end_bar - start_bar) * fraction
            self.set_front_adc_pressure(bar)
            time.sleep(delay)

    # ------------------------------------------------------------------
    # Status / diagnostics (shared)
    # ------------------------------------------------------------------

    def get_status(self) -> Optional[Dict[str, Any]]:
        """
        Get HIL device status.

        Returns a dict with keys ``dac1``, ``dac2``, ``can_ok``,
        ``rx_pending``, or ``None`` if the device did not respond.
        """
        return self._hil.get_status()

    def get_can_messages(self) -> List[Dict[str, Any]]:
        """
        Return any CAN frames received since the last poll.

        Each dict has keys: ``id``, ``extd``, ``rtr``, ``dlc``, ``data``.
        """
        return self._hil.get_can_messages()

    # ------------------------------------------------------------------
    # Keepalive thread — continuous monitoring
    # ------------------------------------------------------------------

    def start_keepalive(self, hydraulic_bar: float = 30.0,
                        pneumatic_bar: float = 8.0,
                        interval_ms: float = 500) -> None:
        """
        Start a background daemon thread that continuously re-applies
        both DAC values and both CAN frames every ``interval_ms``
        milliseconds.

        The thread maintains:

          - Rear hydraulic (AQT7)  = ``hydraulic_bar``
          - Front hydraulic (AQT1) = ``pneumatic_bar * EBS_FRONT_HYD_GAIN``
            (correlated: front hydraulic >= 9x pneumatic)
          - Rear pneumatic (DAC2)  = ``pneumatic_bar``
          - Front pneumatic (DAC1) = ``pneumatic_bar``

        Uses the public HIL API.  CAN sends are command/response so the
        thread may occasionally block for up to the response timeout; the
        default 500 ms interval keeps the loop well under the 1000 ms ACU
        timeout even if a CAN response is delayed.

        Args:
            hydraulic_bar: Rear hydraulic pressure in bar (default 30.0)
            pneumatic_bar: Pneumatic pressure for both front and rear
                           (default 8.0)
            interval_ms:   Loop interval in milliseconds (default 500)
        """
        if self._keepalive_thread is not None:
            print("WARNING: Keepalive thread already running — stopping first.")
            self.stop_keepalive()

        # Calculate front hydraulic from pneumatic correlation:
        #   EBS_FRONT_HYD_GAIN = 9.0  → front hydraulic >= 9 x pneumatic
        front_hydraulic_bar = pneumatic_bar * EBS_FRONT_HYD_GAIN

        # Warn if pneumatic_bar is dangerously close to the strict IN_RANGE limits
        if not (EBS_MIN_BAR + SAFETY_MARGIN_BAR <= pneumatic_bar <= EBS_MAX_BAR - SAFETY_MARGIN_BAR):
            print(f"WARNING: pneumatic_bar={pneumatic_bar:.1f} is close to the "
                  f"threshold limits [{EBS_MIN_BAR}, {EBS_MAX_BAR}] bar. "
                  f"Values within \u00b1{SAFETY_MARGIN_BAR} of the limits "
                  f"may fail the ACU's strict IN_RANGE check.")

        # Pre-compute DAC values (these don't change between iterations)
        rear_dac = pressure_to_dac_value(pneumatic_bar)
        front_dac = pressure_to_dac1_value(pneumatic_bar)

        # Clear the stop event before starting
        self._keepalive_stop.clear()

        def _keepalive_loop() -> None:
            """Inner loop: re-apply all four outputs at each tick."""
            interval_s = interval_ms / 1000.0
            while not self._keepalive_stop.is_set():
                tick_start = time.perf_counter()

                # --- DAC outputs (fire-and-forget, fast) ---
                self._hil.set_dac2(rear_dac)
                self._hil.set_dac1(front_dac)

                # --- CAN sends (command/response, may block briefly) ---
                rear_raw = pressure_to_can_raw(hydraulic_bar)
                rear_data = struct.pack('<H', rear_raw)
                if not self._hil.can_send(CAN_AQT7_ID, rear_data, extd=False):
                    print("WARNING: Keepalive CAN send failed for rear "
                          f"hydraulic {hydraulic_bar:.1f} bar")

                front_raw = pressure_to_aqt1_raw(front_hydraulic_bar)
                front_data = struct.pack('<HB', front_raw, 0)
                if not self._hil.can_send(CAN_AQT1_ID, front_data, extd=False):
                    print("WARNING: Keepalive CAN send failed for front "
                          f"hydraulic {front_hydraulic_bar:.1f} bar")

                # Sleep for the remainder of the interval
                elapsed = time.perf_counter() - tick_start
                remaining = interval_s - elapsed
                if remaining > 0.0:
                    # Use event wait so we can be interrupted immediately
                    self._keepalive_stop.wait(timeout=remaining)

        self._keepalive_thread = threading.Thread(
            target=_keepalive_loop,
            daemon=True,
            name='pressure-sim-keepalive',
        )
        self._keepalive_thread.start()
        print(f"Keepalive thread started: rear hyd={hydraulic_bar:.1f} bar, "
              f"front hyd={front_hydraulic_bar:.1f} bar, "
              f"pneu={pneumatic_bar:.1f} bar, "
              f"interval={interval_ms:.0f} ms")

    def stop_keepalive(self) -> None:
        """
        Stop the keepalive thread.

        Sets the stop event, joins the thread (with a 2-second timeout),
        and resets ``_keepalive_thread`` to ``None``.
        """
        if self._keepalive_thread is None:
            return

        self._keepalive_stop.set()
        self._keepalive_thread.join(timeout=2.0)
        if self._keepalive_thread.is_alive():
            print("WARNING: Keepalive thread did not stop within 2 seconds")
        self._keepalive_thread = None
        print("Keepalive thread stopped.")

    # ------------------------------------------------------------------
    # Internal keepalive — supports dynamic pressure updates for sequencer
    # ------------------------------------------------------------------

    def _print_stage_progress(self, stage: int, elapsed: float,
                              message: str) -> None:
        """Print a stage progress message (helper for sequencer)."""
        print(f"  [Stage {stage}] [{elapsed:.1f}s] {message}")

    def _start_keepalive_internal(self,
                                  hydraulic_bar: float,
                                  pneumatic_bar: float,
                                  front_hydraulic_bar: float | None = None,
                                  interval_ms: float = 500) -> None:
        """
        Internal: start keepalive with separate front/rear hydraulic values.

        The keepalive thread reads target pressures from ``_keepalive_cfg``,
        which can be updated at runtime via :meth:`_update_keepalive`.

        Args:
            hydraulic_bar:      Rear hydraulic pressure in bar
            pneumatic_bar:      Pneumatic pressure (front and rear) in bar
            front_hydraulic_bar: Front hydraulic pressure in bar.
                                 If None, defaults to ``pneumatic_bar * EBS_FRONT_HYD_GAIN``
            interval_ms:        Loop interval in milliseconds (default 500)
        """
        if self._keepalive_thread is not None:
            print("WARNING: Keepalive thread already running — stopping first.")
            self.stop_keepalive()

        if front_hydraulic_bar is None:
            front_hydraulic_bar = pneumatic_bar * EBS_FRONT_HYD_GAIN

        self._keepalive_cfg: dict[str, float] = {
            'hydraulic_bar': hydraulic_bar,
            'pneumatic_bar': pneumatic_bar,
            'front_hydraulic_bar': front_hydraulic_bar,
            'interval': interval_ms / 1000.0,
        }

        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop_internal,
            daemon=True,
            name='pressure-sim-keepalive',
        )
        self._keepalive_thread.start()

    def _keepalive_loop_internal(self) -> None:
        """
        Internal keepalive loop that reads target pressures from ``_keepalive_cfg``.

        Updates DACs and sends CAN frames each iteration.  The config dict
        is updated atomically by :meth:`_update_keepalive` (dict field swaps
        are thread-safe in CPython).
        """
        while not self._keepalive_stop.is_set():
            cfg = self._keepalive_cfg
            h = cfg['hydraulic_bar']
            p = cfg['pneumatic_bar']
            f = cfg['front_hydraulic_bar']

            # --- DAC outputs (fire-and-forget) ---
            self._hil.set_dac2(pressure_to_dac_value(p))   # rear pneumatic
            self._hil.set_dac1(pressure_to_dac1_value(p))  # front pneumatic

            # --- CAN sends ---
            rear_raw = pressure_to_can_raw(h)
            rear_data = struct.pack('<H', rear_raw)
            self._hil.can_send(CAN_AQT7_ID, rear_data, extd=False)

            front_raw = pressure_to_aqt1_raw(f)
            front_data = struct.pack('<HB', front_raw, 0)
            self._hil.can_send(CAN_AQT1_ID, front_data, extd=False)

            # Sleep for remainder of interval (interruptible by stop event)
            self._keepalive_stop.wait(timeout=cfg['interval'])

    def _update_keepalive(self,
                          hydraulic_bar: float | None = None,
                          pneumatic_bar: float | None = None,
                          front_hydraulic_bar: float | None = None) -> None:
        """
        Update keepalive target pressures dynamically (thread-safe).

        Only the provided fields are updated; ``None`` fields keep their
        current value.

        Args:
            hydraulic_bar:      New rear hydraulic pressure, or None
            pneumatic_bar:      New pneumatic pressure (front and rear), or None
            front_hydraulic_bar: New front hydraulic pressure, or None
        """
        if hydraulic_bar is not None:
            self._keepalive_cfg['hydraulic_bar'] = hydraulic_bar
        if pneumatic_bar is not None:
            self._keepalive_cfg['pneumatic_bar'] = pneumatic_bar
        if front_hydraulic_bar is not None:
            self._keepalive_cfg['front_hydraulic_bar'] = front_hydraulic_bar

    def _send_ignition_frame(self) -> None:
        """
        Send VCU_IGN_R2_D with ignition_auto=1 (needed for HV_ACTIVATION).

        This is called periodically during the startup sequence to signal
        the ACU that the ignition is active.
        """
        try:
            self._hil.can_send(
                0x600,
                _build_vcu_ign_r2_d(ignition_auto=1, shutdown_signal=1),
                extd=False,
            )
        except Exception:
            pass  # Gracefully handle CAN send failures

    def _send_can_keepalives(self) -> None:
        """
        Send keepalive frames for VCU, Jetson, RES, CubeMars.

        Prevents timeouts on the ACU for missing peer CAN nodes during
        the startup sequence.
        """
        for can_id, data, extd in _build_can_keepalive_payloads():
            try:
                self._hil.can_send(can_id, data, extd=extd)
            except Exception:
                pass  # Gracefully handle CAN send failures

    # ------------------------------------------------------------------
    # Automatic startup sequencer
    # ------------------------------------------------------------------

    def run_startup_sequence(self, pneumatic_bar: float = 8.0,
                             stage_timeout_s: float = 10.0,
                             poll_interval_s: float = 0.1) -> bool:
        """
        Run the full ACU startup sequence automatically.

        The ACU controls GPIO (SDC, ignition switch) autonomously.
        This method provides the CAN and DAC stimuli needed at each stage::

            Stage 0: WDT_TOGGLE_CHECK   — ACU handles SDC open (GPIO)
            Stage 1: WDT_STP_TOGGLE_CHECK — ACU handles SDC closed (GPIO)
            Stage 2: PNEUMATIC_CHECK    — simulator provides valid pneumatics
            Stage 3: PRESSURE_CHECK1    — simulator sends hydraulic correlation
            Stage 4: HV_ACTIVATION      — simulator sends ignition CAN frame
            Stage 5: PRESSURE_CHECK_FRONT — front solenoid ON, rear unloaded
            Stage 6: PRESSURE_CHECK_REAR  — rear solenoid ON, front unloaded
            Stage 7: PRESSURE_CHECK2    — both off, final correlation → READY

        Args:
            pneumatic_bar:  Pneumatic pressure to simulate (6-10 bar, default 8.0)
            stage_timeout_s: Max time to wait per stage (default 10s)
            poll_interval_s: CAN poll interval in seconds (default 0.1)

        Returns:
            True if ACU reached READY, False if EMERGENCY or timeout

        Raises:
            ValueError: If ``pneumatic_bar`` is outside valid range
        """
        # Validate pneumatic range with safety margins
        if not (EBS_MIN_BAR + SAFETY_MARGIN_BAR <= pneumatic_bar
                <= EBS_MAX_BAR - SAFETY_MARGIN_BAR):
            raise ValueError(
                f"Pneumatic pressure {pneumatic_bar:.1f} bar is too close to "
                f"threshold limits [{EBS_MIN_BAR}, {EBS_MAX_BAR}] bar. "
                f"Use a value between {EBS_MIN_BAR + SAFETY_MARGIN_BAR:.1f} "
                f"and {EBS_MAX_BAR - SAFETY_MARGIN_BAR:.1f}."
            )

        # Pre-compute all pressure values with failsafe margins
        front_pneu = pneumatic_bar
        rear_pneu = pneumatic_bar

        # Stage 3 correlation: front >= 9xPneu, rear >= 3.8xPneu
        raw_rear_init = (
            pneumatic_bar * EBS_REAR_HYD_GAIN_INITIAL
            * HYDRAULIC_MARGIN_PCT + HYDRAULIC_MARGIN_FIXED
        )
        rear_hyd_initial = math.ceil(raw_rear_init * 10.0) / 10.0

        raw_front = (
            pneumatic_bar * EBS_FRONT_HYD_GAIN
            * HYDRAULIC_MARGIN_PCT + HYDRAULIC_MARGIN_FIXED
        )
        front_hyd = math.ceil(raw_front * 10.0) / 10.0

        # Stage 6-7 correlation: rear >= 3.0xPneu
        raw_rear_final = (
            pneumatic_bar * EBS_REAR_HYD_GAIN_FINAL
            * HYDRAULIC_MARGIN_PCT + HYDRAULIC_MARGIN_FIXED
        )
        rear_hyd_final = math.ceil(raw_rear_final * 10.0) / 10.0

        unloaded = UNLOADED_SAFE_BAR  # 0.5 bar

        print(f"\n{'=' * 60}")
        print(f"  ACU Startup Sequence Automático")
        print(f"{'=' * 60}")
        print(f"  Pneumático: {pneumatic_bar:.1f} bar")
        print(f"  Hidráulico frente: {front_hyd:.1f} bar  "
              f"(\u2265{EBS_FRONT_HYD_GAIN}\u00d7{pneumatic_bar:.1f})")
        print(f"  Hidráulico traseiro inicial: {rear_hyd_initial:.1f} bar  "
              f"(\u2265{EBS_REAR_HYD_GAIN_INITIAL}\u00d7{pneumatic_bar:.1f})")
        print(f"  Hidráulico traseiro final: {rear_hyd_final:.1f} bar  "
              f"(\u2265{EBS_REAR_HYD_GAIN_FINAL}\u00d7{pneumatic_bar:.1f})")
        print(f"  Descarregado: {unloaded:.1f} bar")
        print(f"{'=' * 60}\n")

        print("NOTA: O teu hardware ACU tem de estar ligado e na rede CAN.")
        print("      O ACU controla GPIOs (SDC, ignição) autonomamente.\n")

        # Start keepalive with stage-appropriate pressures
        self._start_keepalive_internal(
            hydraulic_bar=rear_hyd_initial,
            pneumatic_bar=pneumatic_bar,
            front_hydraulic_bar=front_hyd,
            interval_ms=500,
        )

        # Also send keepalive for other CAN nodes (VCU, Jetson, RES, CubeMars)
        self._send_can_keepalives()

        start_time = time.time()
        last_acu_decode: dict = {}

        try:
            # ── Main sequence loop ──
            stage = 0   # Our tracking of what stimuli we're providing
            stage_start = time.time()

            while True:
                elapsed = time.time() - start_time

                # --- Poll received CAN messages ---
                try:
                    msgs = self.get_can_messages()
                except Exception:
                    msgs = []   # Graceful: if CAN read fails, skip this cycle

                for msg in msgs:
                    # Skip unknown / unexpected frames silently
                    if msg['id'] == 0x51 and len(msg['data']) >= 8:
                        decoded = _decode_acu_frame(msg['data'])
                        if decoded:
                            last_acu_decode = decoded

                            # Check for EMERGENCY
                            if decoded['acu_state'] == 3 or decoded['emergency']:
                                cause = decoded['emergency_cause']
                                print(f"\n  [{(time.time() - start_time):.1f}s] "
                                      f"EMERGENCY! acu_state={decoded['acu_state']} "
                                      f"as_state={decoded['as_state']} cause={cause}")
                                return False

                            # Check for READY
                            if decoded['as_state'] == 2:   # AS_STATE_READY
                                duration = time.time() - start_time
                                print(f"\n  [{(time.time() - start_time):.1f}s] "
                                      f"ACU atingiu READY! ({duration:.1f}s)")
                                return True

                # ── Stage-based stimuli logic ──
                # Stages 0-1: ACU handles SDC GPIO autonomously.
                # We keep pressures valid and wait for the ACU to progress.

                if elapsed < 2.0:
                    if stage < 2:
                        self._print_stage_progress(
                            stage, elapsed, "A aguardar SDC (GPIO)...")
                        stage = 2
                        stage_start = time.time()

                # Stage 2: PNEUMATIC_CHECK — handled by keepalive DAC values
                # Stage 3: PRESSURE_CHECK1 — handled by keepalive CAN values

                # Stage 4: HV_ACTIVATION — send ignition CAN
                if elapsed > 3.0 and stage == 2:
                    print(f"  [{(time.time() - start_time):.1f}s] "
                          "Stage 3: A enviar correlação hidráulica...")
                    stage = 3
                    stage_start = time.time()

                # Send ignition CAN frame repeatedly during stages 3-4
                self._send_ignition_frame()

                # Stage 5: PRESSURE_CHECK_FRONT (drop rear hydraulic)
                if elapsed > 8.0 and stage == 3:
                    print(f"\n  [{(time.time() - start_time):.1f}s] "
                          "Stage 5: A baixar pressão traseira para "
                          f"{unloaded:.1f} bar...")
                    self._update_keepalive(hydraulic_bar=unloaded,
                                           front_hydraulic_bar=front_hyd)
                    print("  (simula solenoide da frente ligado, "
                          "traseira descarregada)")
                    stage = 4
                    stage_start = time.time()

                # Stage 6: PRESSURE_CHECK_REAR (drop front, restore rear)
                if elapsed > 12.0 and stage == 4:
                    print(f"\n  [{(time.time() - start_time):.1f}s] "
                          "Stage 6: A baixar pressão dianteira, "
                          "restaurar traseira...")
                    self._update_keepalive(hydraulic_bar=rear_hyd_final,
                                           front_hydraulic_bar=unloaded)
                    print(f"  Traseira: {rear_hyd_final:.1f} bar  "
                          f"Frente: {unloaded:.1f} bar (descarregada)")
                    stage = 5
                    stage_start = time.time()

                # Stage 7: PRESSURE_CHECK2 (restore both)
                if elapsed > 16.0 and stage == 5:
                    print(f"\n  [{(time.time() - start_time):.1f}s] "
                          "Stage 7: A restaurar ambas as pressões...")
                    self._update_keepalive(hydraulic_bar=rear_hyd_final,
                                           front_hydraulic_bar=front_hyd)
                    print(f"  Traseira: {rear_hyd_final:.1f} bar  "
                          f"Frente: {front_hyd:.1f} bar")
                    print("  A aguardar correlação final...")
                    stage = 6
                    stage_start = time.time()

                # Periodic CAN keepalive for other nodes (every 2 seconds)
                elapsed_int = int(elapsed)
                prev_int = int(elapsed - poll_interval_s)
                if elapsed_int % 2 == 0 and elapsed_int != prev_int:
                    self._send_can_keepalives()
                    if elapsed_int % 10 == 0:
                        acu = last_acu_decode
                        if acu:
                            print(f"  [{elapsed:.0f}s] ACU state: "
                                  f"acu={acu['acu_state']} "
                                  f"as={acu['as_state']} "
                                  f"emerg={acu['emergency']}")
                        else:
                            print(f"  [{elapsed:.0f}s] "
                                  "A aguardar frames CAN do ACU...")

                # Check overall timeout
                if elapsed > 30.0:
                    print(f"\n  [{(time.time() - start_time):.1f}s] "
                          "TIMEOUT: ACU não atingiu READY após 30s")
                    return False

                time.sleep(poll_interval_s)

        except KeyboardInterrupt:
            print("\n\nSequência interrompida pelo utilizador.")
            return False
        finally:
            self.stop_keepalive()

    def close(self) -> None:
        """Stop keepalive (if running) and close the HIL serial connection."""
        self.stop_keepalive()
        self._hil.close()

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> 'PressureSim':
        return self

    def __exit__(self, *args) -> None:
        self.close()


# ---------------------------------------------------------------------------
# Backward-compatible alias — allows ``from pressure_sim import RearPressureSim``
# ---------------------------------------------------------------------------

RearPressureSim = PressureSim

__all__ = [
    # Class
    'PressureSim',
    'RearPressureSim',
    # Rear / generic conversion helpers
    'pressure_to_can_raw',
    'can_raw_to_pressure',
    'pressure_to_dac_value',
    'dac_value_to_pressure',
    # Front conversion helpers
    'pressure_to_aqt1_raw',
    'pressure_to_dac1_value',
    # ACU frame decoder / CAN builders
    '_decode_acu_frame',
    '_build_vcu_ign_r2_d',
    '_build_jetson_frame',
    '_build_res_frame',
    '_build_cubemars_feedback',
    '_build_can_keepalive_payloads',
    # Constants
    'CAN_AQT7_ID',
    'CAN_AQT1_ID',
    'EBS_MIN_BAR',
    'EBS_MAX_BAR',
    'EBS_REAR_HYD_GAIN_INITIAL',
    'EBS_REAR_HYD_GAIN_FINAL',
    'EBS_FRONT_HYD_GAIN',
    'HYDRAULIC_CORRELATION',
    'DAC_MAX',
    'DAC_VREF',
    'ADC_VREF',
    'ADC_RESOLUTION',
    'ADC_GAIN',
    'SENSOR_OFFSET_V',
    'SENSOR_SLOPE',
    'MAX_PNEUMATIC_BAR',
    'MAX_HYDRAULIC_BAR',
    # Safety margins
    'SAFETY_MARGIN_BAR',
    'HYDRAULIC_MARGIN_PCT',
    'HYDRAULIC_MARGIN_FIXED',
    'UNLOADED_SAFE_BAR',
]


# ---------------------------------------------------------------------------
# Diagnostic verification
# ---------------------------------------------------------------------------

def _print_conversion_table() -> None:
    """Print a table of pressure -> DAC2 values for verification (rear)."""
    print("Rear pressure-to-DAC2 conversion table:")
    print(f"  {'bar':>6s}  {'V_sensor':>8s}  {'V_PA5':>8s}  {'DAC2':>4s}  "
          f"{'readback':>8s}")
    print("  " + "-" * 42)
    for bar in [0.0, 1.0, 2.0, 5.0, 8.0, 10.0]:
        dac = pressure_to_dac_value(bar)
        v_sens = bar * SENSOR_SLOPE + SENSOR_OFFSET_V
        v_pa5 = dac / DAC_MAX * DAC_VREF
        readback = dac_value_to_pressure(dac)
        print(f"  {bar:6.1f}  {v_sens:8.3f}  {v_pa5:8.3f}  "
              f"{dac:4d}  {readback:8.3f}")
    print()


def _print_full_conversion_table() -> None:
    """
    Print a combined conversion table for both DAC1 (front) and DAC2 (rear).

    Since both channels use the same sensor model and ADC gain, the DAC
    values are identical for the same pressure.  This table makes the
    mapping explicit and allows easy cross-checking.
    """
    header = (
        f"  {'bar':>6s}  "
        f"{'V_sensor':>8s}  {'V_pin':>8s}  "
        f"{'DAC1':>4s}  {'DAC2':>4s}  "
        f"{'readback':>8s}  {'err%':>6s}"
    )
    sep = "  " + "-" * 58
    print("Full pressure-to-DAC conversion table (front DAC1 / rear DAC2):")
    print(f"  Both channels: V_sensor = P x 0.4 + 0.5,  "
          f"V_pin = V_sensor x 0.66,  DAC = V_pin / 3.3 x 255")
    print(header)
    print(sep)
    for bar in [0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 6.0, 8.0, 10.0]:
        dac = pressure_to_dac_value(bar)
        v_sens = bar * SENSOR_SLOPE + SENSOR_OFFSET_V
        v_pin = dac / DAC_MAX * DAC_VREF
        readback = dac_value_to_pressure(dac)
        err_pct = ((readback - bar) / bar * 100.0) if bar > 0.0 else 0.0
        print(f"  {bar:6.1f}  "
              f"{v_sens:8.3f}  {v_pin:8.3f}  "
              f"{dac:4d}  {dac:4d}  "
              f"{readback:8.3f}  {err_pct:6.3f}")
    print(sep)
    print("  Note: DAC1 (front) and DAC2 (rear) use the identical sensor model.")
    print("  The ACU reads back pressure = ((DAC/255*3.3/0.66)-0.5)/0.4")
    print()


# ---------------------------------------------------------------------------
# Keepalive mode
# ---------------------------------------------------------------------------

def _run_keepalive_mode(port: str, baud: int,
                        hydraulic_bar: float,
                        pneumatic_bar: float,
                        interval_ms: float) -> None:
    """
    Run in keepalive mode: connect, start keepalive, print status every
    5 seconds, clean up on Ctrl+C.

    Args:
        port:          Serial port for ESP32 HIL
        baud:          Serial baud rate
        hydraulic_bar: Rear hydraulic pressure (bar)
        pneumatic_bar: Pneumatic pressure (bar, front and rear)
        interval_ms:   Keepalive loop interval (ms)
    """
    print("=" * 60)
    print("  ACU V3.0 — Pressure Simulator — Keepalive Mode")
    print("=" * 60)
    print(f"  Port:     {port} @ {baud} baud")
    print()

    # Connect
    try:
        sim = PressureSim(port, baud=baud)
    except Exception as e:
        print(f"ERROR: Failed to connect to ESP32 HIL on {port}: {e}")
        sys.exit(1)

    front_hydraulic_bar = pneumatic_bar * EBS_FRONT_HYD_GAIN

    print("[1/2] Starting keepalive mode...")
    print(f"  Hydraulic: {hydraulic_bar:.1f} bar  "
          f"(rear AQT7) / {front_hydraulic_bar:.1f} bar (front AQT1)")
    print(f"  Pneumatic: {pneumatic_bar:.1f} bar   "
          f"(DAC1→front PA4, DAC2→rear PA5)")
    print(f"  Interval:  {interval_ms:.0f} ms")
    print(f"  Press Ctrl+C to stop")
    print()

    start_time = time.perf_counter()

    try:
        sim.start_keepalive(
            hydraulic_bar=hydraulic_bar,
            pneumatic_bar=pneumatic_bar,
            interval_ms=interval_ms,
        )

        next_report = 5.0  # first status report at 5 seconds

        while True:
            now = time.perf_counter()
            elapsed = now - start_time

            # Wait until next report or stop signal
            wait_time = next_report - elapsed
            if wait_time > 0:
                # Use a short sleep so we remain responsive to Ctrl+C
                try:
                    time.sleep(min(wait_time, 0.5))
                except KeyboardInterrupt:
                    break
                continue

            # Report status every 5 seconds
            st = sim.get_status()
            dac1 = st['dac1'] if st else '?'
            dac2 = st['dac2'] if st else '?'
            can_ok = st['can_ok'] if st else '?'
            print(f"  --- Keepalive running for {elapsed:.1f}s ---")
            print(f"  DAC1={dac1}  DAC2={dac2}  CAN_OK={can_ok}")
            next_report += 5.0

    except KeyboardInterrupt:
        print()
        print("[2/2] Stop signal received. Stopping keepalive...")
    finally:
        elapsed_total = time.perf_counter() - start_time
        sim.stop_keepalive()
        sim.close()
        print(f"Keepalive stopped after {elapsed_total:.1f} seconds.")


# ---------------------------------------------------------------------------
# CLI main()
# ---------------------------------------------------------------------------

def main() -> None:
    """Command-line interface for the pressure simulator (front & rear)."""

    parser = argparse.ArgumentParser(
        description='Pressure Simulator for ACU V3.0 — Front & Rear',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  %(prog)s --port /dev/ttyUSB0\n"
            "  %(prog)s --port COM3\n"
            "  %(prog)s --port /dev/ttyUSB0 --table\n"
            "  %(prog)s --port /dev/ttyUSB0 --keepalive\n"
            "  %(prog)s --port /dev/ttyUSB0 --keepalive "
            "--keepalive-hyd 50 --keepalive-pneu 7\n"
            "  %(prog)s --port /dev/ttyUSB0 --sequence\n"
            "  %(prog)s --port /dev/ttyUSB0 --sequence "
            "--sequence-pneu 7.5\n"
        ),
    )
    parser.add_argument(
        '--port', type=str, default='/dev/ttyUSB0',
        help='Serial port for ESP32 HIL (default: /dev/ttyUSB0)',
    )
    parser.add_argument(
        '--baud', type=int, default=921600,
        help='Serial baud rate (default: 921600)',
    )
    parser.add_argument(
        '--table', action='store_true',
        help='Print pressure-to-DAC conversion table and exit',
    )
    parser.add_argument(
        '--keepalive', action='store_true',
        help='Run in keepalive mode (continuous monitoring)',
    )
    parser.add_argument(
        '--keepalive-hyd', type=float, default=30.0,
        help='Rear hydraulic pressure for keepalive (bar, default 30.0)',
    )
    parser.add_argument(
        '--keepalive-pneu', type=float, default=8.0,
        help='Pneumatic pressure for keepalive (bar, default 8.0)',
    )
    parser.add_argument(
        '--keepalive-interval', type=float, default=500,
        help='Keepalive loop interval in ms (default 500)',
    )

    # Automatic startup sequence
    parser.add_argument(
        '--sequence', action='store_true',
        help='Run automatic startup sequence',
    )
    parser.add_argument(
        '--sequence-pneu', type=float, default=8.0,
        help='Pneumatic pressure for sequence (bar, default 8.0)',
    )

    args = parser.parse_args()

    # Conversion table mode
    if args.table:
        _print_full_conversion_table()
        return

    # ---- Keepalive mode ----
    if args.keepalive:
        _run_keepalive_mode(
            port=args.port,
            baud=args.baud,
            hydraulic_bar=args.keepalive_hyd,
            pneumatic_bar=args.keepalive_pneu,
            interval_ms=args.keepalive_interval,
        )
        return

    # ---- Automatic startup sequence ----
    if args.sequence:
        sim = PressureSim(args.port, baud=args.baud)
        try:
            success = sim.run_startup_sequence(
                pneumatic_bar=args.sequence_pneu)
            sys.exit(0 if success else 1)
        finally:
            sim.close()
        return

    # ---- Connect ----
    print("=" * 60)
    print("  ACU V3.0 — Pressure Simulator (Front & Rear)")
    print("=" * 60)
    print(f"  Port: {args.port} @ {args.baud} baud")
    print()

    try:
        sim = PressureSim(args.port, baud=args.baud)
    except Exception as e:
        print(f"ERROR: Failed to connect to ESP32 HIL on {args.port}: {e}")
        sys.exit(1)

    try:
        # ---- Step 1: Status check ----
        print("[1/8] Checking HIL status...")
        st = sim.get_status()
        if st is None:
            print("ERROR: No response from HIL device "
                  "(check port, power, and baud rate)")
            sys.exit(1)
        print(f"  DAC1={st['dac1']:3d}  DAC2={st['dac2']:3d}  "
              f"CAN_OK={st['can_ok']}  "
              f"RX_PENDING={st['rx_pending']}")
        print()

        # ---- Step 2: Fixed pressure values — Rear ----
        print("[2/8] REAR fixed pressure values on CAN (hydraulic)...")
        test_pressures = [0.0, 5.0, 10.0, 30.0, 50.0]
        for p in test_pressures:
            can_raw = pressure_to_can_raw(p)
            can_ok = sim.set_can_pressure(p)
            status = "OK" if can_ok else "FAIL"
            print(f"  CAN {p:5.1f} bar -> raw=0x{can_raw:04X}  [{status}]")
            time.sleep(0.2)

        print()
        print("  DAC2 fixed pressures (rear pneumatic)...")
        for p in [0.0, 2.0, 5.0, 8.0, 10.0]:
            dac_val = pressure_to_dac_value(p)
            adc_ok = sim.set_adc_pressure(p)
            readback = dac_value_to_pressure(dac_val)
            status = "OK" if adc_ok else "FAIL"
            print(f"  ADC {p:5.1f} bar -> DAC2={dac_val:3d}  "
                  f"(simulates {readback:.2f} bar)  [{status}]")
            time.sleep(0.2)
        print()

        # ---- Step 3: Fixed pressure values — Front ----
        print("[3/8] FRONT fixed pressure values on CAN AQT1 (0x710)...")
        for p in [0.0, 5.0, 10.0, 30.0, 50.0]:
            aqt1_raw = pressure_to_aqt1_raw(p)
            ok = sim.set_front_can_pressure(p)
            status = "OK" if ok else "FAIL"
            print(f"  AQT1 {p:5.1f} bar -> raw=0x{aqt1_raw:04X}  [{status}]")
            time.sleep(0.2)

        print()
        print("  DAC1 fixed pressures (front pneumatic)...")
        for p in [0.0, 2.0, 5.0, 8.0, 10.0]:
            dac_val = pressure_to_dac1_value(p)
            adc_ok = sim.set_front_adc_pressure(p)
            readback = dac_value_to_pressure(dac_val)
            status = "OK" if adc_ok else "FAIL"
            print(f"  ADC {p:5.1f} bar -> DAC1={dac_val:3d}  "
                  f"(simulates {readback:.2f} bar)  [{status}]")
            time.sleep(0.2)
        print()

        # ---- Step 4: Startup conditions (both domains) ----
        print("[4/8] Setting ACU startup conditions "
              "(pneumatic = 8.0 bar, both domains)...")
        can_ok, adc_ok, f_can_ok, f_adc_ok = sim.set_startup_conditions(pneumatic_bar=8.0)
        st = sim.get_status()
        print(f"  REAR:  CAN={'OK' if can_ok else 'FAIL'}  "
              f"ADC={'OK' if adc_ok else 'FAIL'}")
        print(f"  FRONT CAN={'OK' if f_can_ok else 'FAIL'}  "
              f"ADC={'OK' if f_adc_ok else 'FAIL'}")
        if st:
            readback_bar = dac_value_to_pressure(st['dac2'])
            print(f"         DAC2 = {st['dac2']:3d}  "
                  f"(ACU reads {readback_bar:.2f} bar pneumatic)")
            front_readback = dac_value_to_pressure(st['dac1'])
            print(f"         DAC1 = {st['dac1']:3d}  "
                  f"(front ADC reads {front_readback:.2f} bar pneumatic)")
        print(f"  REAR  Hydraulic >= {8.0 * EBS_REAR_HYD_GAIN_INITIAL:.1f} bar")
        print(f"  FRONT Hydraulic >= {8.0 * EBS_FRONT_HYD_GAIN:.1f} bar")
        print()

        # ---- Step 5: Front-specific startup ----
        print("[5/8] FRONT startup conditions "
              "(pneumatic = 7.0 bar, explicit)...")
        f_can_ok, f_adc_ok = sim.set_front_startup_conditions(pneumatic_bar=7.0)
        st = sim.get_status()
        front_readback = dac_value_to_pressure(st['dac1'] if st else 0)
        print(f"  FRONT CAN={'OK' if f_can_ok else 'FAIL'}  "
              f"ADC={'OK' if f_adc_ok else 'FAIL'}")
        if st:
            print(f"         DAC1 = {st['dac1']:3d}  "
                  f"(ACU reads {front_readback:.2f} bar pneumatic)")
        print()

        # ---- Step 6: CAN ramp demo (rear) ----
        print("[6/8] REAR CAN ramp:  0 -> 50 bar over 5 seconds...")
        sim.ramp_can_pressure(0.0, 50.0, duration_s=5.0, steps=50)
        print("  Ramp complete (held at 50 bar)")
        time.sleep(0.5)

        print("  REAR CAN ramp: 50 -> 0 bar over 3 seconds...")
        sim.ramp_can_pressure(50.0, 0.0, duration_s=3.0, steps=30)
        print("  Ramp complete")
        print()

        # ---- Step 7: ADC ramp demo (front) ----
        print("[7/8] FRONT ADC ramp:  0 -> 10 bar over 5 seconds on DAC1...")
        sim.ramp_front_adc_pressure(0.0, 10.0, duration_s=5.0, steps=50)
        print("  Ramp complete (held at 10 bar)")
        time.sleep(0.5)

        print("  FRONT ADC ramp: 10 -> 0 bar over 3 seconds...")
        sim.ramp_front_adc_pressure(10.0, 0.0, duration_s=3.0, steps=30)
        print("  Ramp complete")
        print()

        # ---- Step 8: Waveform demos ----
        print("[8/8] Waveform demos...")
        print()

        # 8a: Rear sine on DAC2 (legacy style)
        print("  [8a] REAR sine:  1 Hz, amp=4 bar, offset=5 bar, DAC2, 3 s...")
        sim.apply_waveform(
            dac_ch=2,
            waveform_type='sine',
            freq_hz=1.0,
            amplitude_bar=4.0,
            offset_bar=5.0,
            duration_s=3.0,
        )
        print("  Complete")

        # 8b: Front sine on DAC1 (new domain parameter)
        print("  [8b] FRONT sine:  1 Hz, amp=4 bar, offset=5 bar, DAC1, 3 s...")
        sim.apply_waveform(
            dac_ch=1,
            waveform_type='sine',
            freq_hz=1.0,
            amplitude_bar=4.0,
            offset_bar=5.0,
            duration_s=3.0,
            domain='front',
        )
        print("  Complete")

        # 8c: Combined both-domain sawtooth (alternating)
        print("  [8c] COMBINED: front square on DAC1, rear square on DAC2, 3 s...")
        sim.apply_waveform(
            domain='front',
            waveform_type='square',
            freq_hz=0.8,
            amplitude_bar=3.0,
            offset_bar=5.0,
            duration_s=3.0,
        )
        time.sleep(0.2)
        sim.apply_waveform(
            domain='rear',
            waveform_type='square',
            freq_hz=0.8,
            amplitude_bar=3.0,
            offset_bar=5.0,
            duration_s=3.0,
        )
        print("  Complete")
        print()

        # ---- Final status ----
        print("=" * 60)
        print("Final HIL status:")
        st = sim.get_status()
        if st:
            print(f"  DAC1 = {st['dac1']:3d}  "
                  f"(front pneumatic: {dac_value_to_pressure(st['dac1']):.2f} bar)")
            print(f"  DAC2 = {st['dac2']:3d}  "
                  f"(rear pneumatic: {dac_value_to_pressure(st['dac2']):.2f} bar)")
            print(f"  CAN  = {'OK' if st['can_ok'] else 'ERROR'}")
            print(f"  RX pending = {st['rx_pending']}")
        print()
        print("Pressure Simulator completed successfully.")

    except KeyboardInterrupt:
        print("\nInterrupted by user -- shutting down.")
    except ValueError as e:
        print(f"ERROR: Invalid parameter: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {type(e).__name__}: {e}")
        sys.exit(1)
    finally:
        sim.close()


if __name__ == '__main__':
    main()
