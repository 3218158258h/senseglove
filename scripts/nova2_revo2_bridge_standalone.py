#!/usr/bin/env python3
"""
nova2_revo2_bridge_standalone.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Standalone bridge: SenseGlove Nova2 → Revo2 Dexterous Hand (dual hand).

Does NOT require ROS2.  Reads normalised finger data directly from the
SenseGlove C++ SDK via a thin ctypes wrapper (nova2_sensor_api.so).

Prerequisites
─────────────
1. SenseCom must be running before this script starts.
2. Build the C wrapper once:
       cd scripts && make
3. The Revo2 Python SDK (revo2_utils, utils) must be on PYTHONPATH.

Sensor layout  (nova2_sensor_api returns 6 floats, 0.0 – 1.0):
   [0]  Thumb   Abduction        (内收/外展)
   [1]  Thumb   FlexionProximal  (屈伸)
   [2]  Index   FlexionProximal
   [3]  Index   FlexionDistal    ← used for index finger
   [4]  Middle  Flexion
   [5]  Ring    Flexion          ← shared for ring AND pinky

Revo2 6-DOF positions  (0 = fist, 1000 = fully open):
   [0]  大拇指Flex  ← nova2[1] direct
   [1]  大拇指Aux   ← nova2[0] direct
   [2]  食指        ← nova2[3] direct
   [3]  中指        ← nova2[4] direct
   [4]  无名指      ← nova2[5] direct
   [5]  小拇指      ← nova2[5] direct
"""

import asyncio
import ctypes
import os
import sys

from revo2_utils import logger, libstark
from utils import setup_shutdown_event

# ── Configuration ─────────────────────────────────────────────────────────────
RIGHT_PORT = "/dev/ttyUSB0"
LEFT_PORT  = "/dev/ttyUSB1"
RIGHT_ID   = 0x7F
LEFT_ID    = 0x7E
BAUDRATE   = libstark.Baudrate.Baud460800

CONTROL_HZ = 20          # Hz – how often to send positions to Revo2 hands
SPEEDS     = [1000] * 6  # Revo2 motor speeds (max = 1000)

# Path to the compiled C wrapper (built via `make` in scripts/)
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SENSOR_LIB = os.path.join(_THIS_DIR, "nova2_sensor_api.so")


# ── ctypes wrapper ─────────────────────────────────────────────────────────────
class Nova2SensorReader:
    """Thin Python wrapper around nova2_sensor_api.so."""

    def __init__(self, lib_path: str = SENSOR_LIB) -> None:
        if not os.path.exists(lib_path):
            raise FileNotFoundError(
                f"Sensor library not found: {lib_path}\n"
                "Run `cd scripts && make` to build it first."
            )
        try:
            self._lib = ctypes.CDLL(lib_path)
        except OSError as exc:
            msg = str(exc)
            if "undefined symbol" in msg:
                raise OSError(
                    f"\n"
                    f"  Cannot load {lib_path}: {msg}\n"
                    f"\n"
                    f"  C++ ABI mismatch: the SenseGlove-API-master v22 libs were compiled\n"
                    f"  with Clang/libc++ (std::__1 ABI). The wrapper must be compiled with\n"
                    f"  the same ABI.  Rebuild using clang++ + libc++:\n"
                    f"\n"
                    f"      # Ubuntu: apt install clang libc++-dev libc++abi-dev\n"
                    f"      cd <repo_root>/scripts && make clean && make\n"
                    f"\n"
                    f"  Do NOT compile with plain g++ – it uses libstdc++ (different ABI).\n"
                ) from exc
            if "GLIBC_" in msg:
                import re
                versions = re.findall(r"GLIBC_[\d.]+", msg)
                required = versions[0] if versions else "GLIBC_2.38+"
                raise OSError(
                    f"\n"
                    f"  Cannot load {lib_path}\n"
                    f"  Required: {required}\n"
                    f"  Your system glibc is too old.\n"
                    f"\n"
                    f"  The Makefile defaults to SenseGlove-API-master/lib/linux/v22 libs\n"
                    f"  (requires only GLIBC_2.17, compatible with Ubuntu 22.04+).\n"
                    f"  Rebuild the wrapper:\n"
                    f"      cd <repo_root>/scripts && make\n"
                    f"\n"
                    f"  If you built with the old senseglove_api Ubuntu libs (requires\n"
                    f"  GLIBC_2.38/2.39), run inside Docker instead:\n"
                    f"      bash scripts/run_docker.sh\n"
                ) from exc
            raise

        # int nova2_sensecom_running()
        self._lib.nova2_sensecom_running.restype  = ctypes.c_int
        self._lib.nova2_sensecom_running.argtypes = []

        # int nova2_send_normalized_data(int right_hand)
        self._lib.nova2_send_normalized_data.restype  = ctypes.c_int
        self._lib.nova2_send_normalized_data.argtypes = [ctypes.c_int]

        # int nova2_get_normalized_input(int right_hand, float* out, int* out_len)
        self._lib.nova2_get_normalized_input.restype  = ctypes.c_int
        self._lib.nova2_get_normalized_input.argtypes = [
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_int),
        ]

        # int nova2_get_normalization_state(int right_hand)
        self._lib.nova2_get_normalization_state.restype  = ctypes.c_int
        self._lib.nova2_get_normalization_state.argtypes = [ctypes.c_int]

        # int nova2_get_raw_sensor_data(int right_hand, float* out, int* out_len)
        self._lib.nova2_get_raw_sensor_data.restype  = ctypes.c_int
        self._lib.nova2_get_raw_sensor_data.argtypes = [
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_int),
        ]

    def sensecom_running(self) -> bool:
        return bool(self._lib.nova2_sensecom_running())

    def send_normalized_data(self, right_hand: bool) -> bool:
        """Command the glove to start streaming normalised data. Call once at startup."""
        return bool(self._lib.nova2_send_normalized_data(1 if right_hand else 0))

    def get_normalized(self, right_hand: bool) -> list[float]:
        """Return a list of 6 floats (0-1), or [0]*6 if the glove is unavailable."""
        buf = (ctypes.c_float * 6)()
        length = ctypes.c_int(0)
        ok = self._lib.nova2_get_normalized_input(
            1 if right_hand else 0,
            buf,
            ctypes.byref(length),
        )
        if not ok or length.value == 0:
            return [0.0] * 6
        return list(buf[: length.value])

    # Normalization state codes (ENormalizationState in NormalizationState.hpp):
    _NORM_STATE_NAMES = {
        -1: "GLOVE_NOT_FOUND",
        -2: "SENSOR_DATA_UNAVAILABLE",
        1:  "Unknown",
        2:  "SendingRawData",
        3:  "Normalizing_MoveFingers",
        4:  "Normalizing_AwaitConfirm",
        5:  "NormalizationFinished",
    }

    def get_normalization_state(self, right_hand: bool) -> tuple[int, str]:
        """Return (code, name) for the current normalization state of the glove."""
        code = self._lib.nova2_get_normalization_state(1 if right_hand else 0)
        name = self._NORM_STATE_NAMES.get(code, f"state_{code}")
        return code, name

    def get_raw_sensor_data(self, right_hand: bool) -> list[float]:
        """Return 6 raw sensor values from GetSensorData (bypasses normalization).

        Values are NOT guaranteed to be in [0, 1] when the glove has not finished
        normalization.  Use to verify the glove hardware is live.
        """
        buf = (ctypes.c_float * 6)()
        length = ctypes.c_int(0)
        ok = self._lib.nova2_get_raw_sensor_data(
            1 if right_hand else 0,
            buf,
            ctypes.byref(length),
        )
        if not ok or length.value == 0:
            return [0.0] * 6
        return list(buf[: length.value])


# ── Sensor → Revo2 conversion ─────────────────────────────────────────────────
def nova2_to_revo2(values: list[float]) -> list[int]:
    """Map 6 normalised Nova2 values (0-1) → Revo2 positions (0-1000).

    Nova2 convention: 0 = finger fully extended (open), 1 = fully flexed (fist).
    Revo2 convention: 0 = fist (closed),            1000 = fully open.
    Therefore the mapping must be INVERTED: revo2 = (1 - nova2) * 1000.
    """

    def inv(v: float) -> int:
        return int((1.0 - max(0.0, min(1.0, v))) * 1000)

    def fwd(v: float) -> int:
        return int(max(0.0, min(1.0, v)) * 1000)

    if len(values) < 6:
        values = list(values) + [0.0] * (6 - len(values))

    return [
        inv(values[1]),  # [0] 大拇指Flex  ← Thumb FlexionProximal (inverted)
        fwd(values[0]),  # [1] 大拇指Aux   ← Thumb Abduction       (direct: 0=adducted, 1=spread)
        inv(values[3]),  # [2] 食指        ← Index FlexionDistal   (inverted)
        inv(values[4]),  # [3] 中指        ← Middle Flexion        (inverted)
        inv(values[5]),  # [4] 无名指      ← Ring Flexion          (inverted)
        inv(values[5]),  # [5] 小拇指      ← Ring Flexion          (inverted)
    ]


# ── Revo2 async control loop ──────────────────────────────────────────────────
async def control_loop(
    client_left,
    client_right,
    reader: Nova2SensorReader,
    shutdown_event: asyncio.Event,
) -> None:
    interval = 1.0 / CONTROL_HZ
    logger.info("Control loop started at %d Hz", CONTROL_HZ)
    loop = asyncio.get_running_loop()

    while not shutdown_event.is_set():
        # Run blocking ctypes calls in a thread pool so the asyncio event loop
        # remains free to process Revo2 serial I/O.  Without this, GIL-holding
        # ctypes calls would starve the Modbus transport layer, causing every
        # set_finger_positions_and_speeds call after the first to time out.
        rh_raw, lh_raw, rh_raw_hw, lh_raw_hw, rh_norm_state, lh_norm_state = (
            await asyncio.gather(
                loop.run_in_executor(None, reader.get_normalized, True),
                loop.run_in_executor(None, reader.get_normalized, False),
                loop.run_in_executor(None, reader.get_raw_sensor_data, True),
                loop.run_in_executor(None, reader.get_raw_sensor_data, False),
                loop.run_in_executor(None, reader.get_normalization_state, True),
                loop.run_in_executor(None, reader.get_normalization_state, False),
            )
        )
        rh_pos = nova2_to_revo2(rh_raw)
        lh_pos = nova2_to_revo2(lh_raw)

        # Print raw glove sensor values so you can verify the glove is streaming.
        # Use print() directly – libstark.init_logging() may raise the logger
        # threshold above INFO, silencing logger.info() calls.
        rh_fmt     = " ".join(f"{v:.3f}" for v in rh_raw)
        lh_fmt     = " ".join(f"{v:.3f}" for v in lh_raw)
        rh_hw_fmt  = " ".join(f"{v:.3f}" for v in rh_raw_hw)
        lh_hw_fmt  = " ".join(f"{v:.3f}" for v in lh_raw_hw)
        print(f"GLOVE norm  RH[{rh_fmt}]  LH[{lh_fmt}]  |  state RH={rh_norm_state[1]} LH={lh_norm_state[1]}", flush=True)
        print(f"GLOVE raw   RH[{rh_hw_fmt}]  LH[{lh_hw_fmt}]", flush=True)
        print(f"REVO2 pos   RH{rh_pos}  LH{lh_pos}", flush=True)

        # ENormalizationState: 5 = NormalizationFinished.
        # In any other state (especially 2 = SendingRawData) the SDK returns
        # raw-ADC values clamped to [0, 1] – these are NOT real finger angles.
        # Sending those bogus values to Revo2 causes a fist on startup.
        # Wait until both gloves have finished normalization before commanding.
        _NORM_FINISHED = 5
        rh_ready = (rh_norm_state[0] == _NORM_FINISHED)
        lh_ready = (lh_norm_state[0] == _NORM_FINISHED)
        if not rh_ready or not lh_ready:
            waiting = []
            if not rh_ready:
                waiting.append(f"RH({rh_norm_state[1]})")
            if not lh_ready:
                waiting.append(f"LH({lh_norm_state[1]})")
            print(
                f"[WAITING] Glove(s) not yet normalized: {', '.join(waiting)}. "
                "Open SenseCom → calibrate glove → confirm, then re-run.",
                flush=True,
            )
            await asyncio.sleep(interval)
            continue

        try:
            await asyncio.gather(
                client_right.set_finger_positions_and_speeds(RIGHT_ID, rh_pos, SPEEDS),
                client_left.set_finger_positions_and_speeds(LEFT_ID,  lh_pos, SPEEDS),
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("Error sending positions: %s", exc)

        await asyncio.sleep(interval)


async def main_async(reader: Nova2SensorReader) -> None:
    libstark.init_logging()
    shutdown_event = setup_shutdown_event(logger)

    if not reader.sensecom_running():
        logger.critical("SenseCom is not running.  Please start it first.")
        sys.exit(1)

    client_left  = await libstark.modbus_open(LEFT_PORT,  BAUDRATE)
    client_right = await libstark.modbus_open(RIGHT_PORT, BAUDRATE)
    if not client_left or not client_right:
        logger.critical("Failed to open one or both Revo2 serial ports.")
        sys.exit(1)

    left_info  = await client_left.get_device_info(LEFT_ID)
    right_info = await client_right.get_device_info(RIGHT_ID)
    if not left_info or not right_info:
        logger.critical("Failed to get Revo2 device info.")
        sys.exit(1)

    logger.info("Left  hand: %s", left_info.description)
    logger.info("Right hand: %s", right_info.description)

    # Command both gloves to start streaming normalised sensor data.
    # Without this the SDK may return stale/default values instead of live readings.
    for hand_name, is_right in (("right", True), ("left", False)):
        ok = reader.send_normalized_data(is_right)
        logger.info("SendNormalizedData %s glove: %s", hand_name, "OK" if ok else "FAILED (glove not found?)")

    task = asyncio.create_task(
        control_loop(client_left, client_right, reader, shutdown_event)
    )
    await shutdown_event.wait()

    task.cancel()
    await asyncio.gather(task, return_exceptions=True)
    await client_left.modbus_close()
    await client_right.modbus_close()
    logger.info("Done.")


# ── Entry point ───────────────────────────────────────────────────────────────
def main() -> None:
    try:
        reader = Nova2SensorReader()
    except (FileNotFoundError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

    try:
        asyncio.run(main_async(reader))
    except KeyboardInterrupt:
        logger.info("User interrupted")
    sys.exit(0)


if __name__ == "__main__":
    main()
