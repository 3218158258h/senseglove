#!/usr/bin/env python3
"""
nova2_revo2_bridge_standalone.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
独立桥接脚本：SenseGlove Nova2 → Revo2 灵巧手（双手）。

不依赖 ROS2。通过轻量 ctypes 封装（nova2_sensor_api.so）
直接从 SenseGlove C++ SDK 读取归一化手指数据。

前置条件
────────
1. 启动脚本前必须先运行 SenseCom。
2. 先编译一次 C 封装：
       cd scripts && make
3. Revo2 Python SDK（revo2_utils, utils）需在 PYTHONPATH 中。

传感器布局（nova2_sensor_api 返回 6 个 float，范围 0.0 – 1.0）：
   [0]  Thumb   Abduction        (内收/外展)
   [1]  Thumb   FlexionProximal  (屈伸)
   [2]  Index   FlexionProximal
   [3]  Index   FlexionDistal    ← used for index finger
   [4]  Middle  Flexion
   [5]  Ring    Flexion          ← shared for ring AND pinky

Revo2 6-DOF 位置（0 = 握拳，1000 = 完全张开）：
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

# ── 配置 ───────────────────────────────────────────────────────────────────────
RIGHT_PORT = "/dev/ttyUSB0"
LEFT_PORT  = "/dev/ttyUSB1"
RIGHT_ID   = 0x7F
LEFT_ID    = 0x7E
BAUDRATE   = libstark.Baudrate.Baud460800

CONTROL_HZ = 20          # Hz：下发到 Revo2 的频率
SPEEDS     = [1000] * 6  # Revo2 电机速度（最大 1000）

# 输入/输出稳定性参数
EMA_ALPHA = 0.35              # 0..1，越大响应越快，越小越平滑
OUTPUT_DEADBAND = 6           # Revo2 指令死区，抑制微小来回抖动

# 正向映射握拳增强（保持“正向映射”，但让慢速握拳更容易达到满指令）
FIST_ASSIST_CHANNELS = {0, 2, 3, 4, 5}  # 拇指屈伸 + 四指（不包含拇指辅助通道）
FIST_ASSIST_GAMMA = 0.65                 # <1 会增强中高段，向“握拳”方向推进
FIST_SNAP_THRESHOLD = 0.90               # 高于该值直接吸附到完全握拳（1000）
OPEN_SNAP_THRESHOLD = 0.01               # 低于该值直接吸附到完全张开（0）

# 已编译 C 封装路径（在 scripts/ 下执行 `make` 生成）
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SENSOR_LIB = os.path.join(_THIS_DIR, "nova2_sensor_api.so")


# ── ctypes 封装 ────────────────────────────────────────────────────────────────
class Nova2SensorReader:
    """nova2_sensor_api.so 的轻量 Python 封装。"""

    def __init__(self, lib_path: str = SENSOR_LIB) -> None:
        if not os.path.exists(lib_path):
            raise FileNotFoundError(
                f"Sensor library not found: {lib_path}\n"
                "请先运行 `cd scripts && make` 进行编译。"
            )
        try:
            self._lib = ctypes.CDLL(lib_path)
        except OSError as exc:
            msg = str(exc)
            if "undefined symbol" in msg:
                raise OSError(
                    f"\n"
                     f"  无法加载 {lib_path}: {msg}\n"
                    f"\n"
                     f"  C++ ABI 不匹配：SenseGlove-API-master v22 库使用 Clang/libc++\n"
                     f"  （std::__1 ABI）编译。封装也必须使用同 ABI。请用 clang++ + libc++ 重编译：\n"
                    f"\n"
                     f"      # Ubuntu: apt install clang libc++-dev libc++abi-dev\n"
                    f"      cd <repo_root>/scripts && make clean && make\n"
                    f"\n"
                     f"  不要使用普通 g++ 编译——它使用 libstdc++（ABI 不同）。\n"
                ) from exc
            if "GLIBC_" in msg:
                import re
                versions = re.findall(r"GLIBC_[\d.]+", msg)
                required = versions[0] if versions else "GLIBC_2.38+"
                raise OSError(
                    f"\n"
                     f"  无法加载 {lib_path}\n"
                     f"  需要版本：{required}\n"
                     f"  当前系统 glibc 版本过低。\n"
                    f"\n"
                     f"  Makefile 默认链接 SenseGlove-API-master/lib/linux/v22 库\n"
                     f"  （仅需 GLIBC_2.17，兼容 Ubuntu 22.04+）。\n"
                     f"  请重编译封装：\n"
                    f"      cd <repo_root>/scripts && make\n"
                    f"\n"
                     f"  如果你链接了旧版 senseglove_api Ubuntu 库（需要\n"
                     f"  GLIBC_2.38/2.39），请改为在 Docker 中运行：\n"
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
        """命令手套开始输出归一化数据（启动时调用一次）。"""
        return bool(self._lib.nova2_send_normalized_data(1 if right_hand else 0))

    def get_normalized(self, right_hand: bool) -> list[float]:
        """返回 6 个浮点数（0-1）；手套不可用时返回 [0]*6。"""
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

    # 归一化状态码（见 NormalizationState.hpp 的 ENormalizationState）
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
        """返回当前手套归一化状态 (code, name)。"""
        code = self._lib.nova2_get_normalization_state(1 if right_hand else 0)
        name = self._NORM_STATE_NAMES.get(code, f"state_{code}")
        return code, name

    def get_raw_sensor_data(self, right_hand: bool) -> list[float]:
        """返回 GetSensorData 的 6 路原始值（不经过归一化）。

        当手套尚未完成归一化时，返回值不保证在 [0, 1] 范围内。
        可用于确认手套硬件是否在线。
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


# ── 传感器 → Revo2 转换 ────────────────────────────────────────────────────────
def nova2_to_revo2(values: list[float]) -> list[int]:
    """将 6 路 Nova2 归一化值（0-1）映射到 Revo2 位置（0-1000）。

    使用正向映射：0~1 Nova2 值 → 0~1000 Revo2 指令值。
    """

    def _clip_0_1000(x: int) -> int:
        return max(0, min(1000, x))

    def fwd(v: float, channel: int) -> int:
        x = max(0.0, min(1.0, v))
        if channel in FIST_ASSIST_CHANNELS:
            x = x ** FIST_ASSIST_GAMMA
            if x >= FIST_SNAP_THRESHOLD:
                x = 1.0
        if x <= OPEN_SNAP_THRESHOLD:
            x = 0.0
        return _clip_0_1000(int(round(x * 1000)))

    if len(values) < 6:
        values = list(values) + [0.0] * (6 - len(values))

    return [
        fwd(values[1], 0),  # [0] 大拇指Flex  ← Thumb FlexionProximal (forward)
        fwd(values[0], 1),  # [1] 大拇指Aux   ← Thumb Abduction       (direct: 0=adducted, 1=spread)
        fwd(values[3], 2),  # [2] 食指        ← Index FlexionDistal   (forward)
        fwd(values[4], 3),  # [3] 中指        ← Middle Flexion        (forward)
        fwd(values[5], 4),  # [4] 无名指      ← Ring Flexion          (forward)
        fwd(values[5], 5),  # [5] 小拇指      ← Ring Flexion          (forward)
    ]


def ema_step(current: list[float], previous: list[float] | None) -> list[float]:
    if previous is None:
        return list(current)
    n = min(len(current), len(previous))
    out = [EMA_ALPHA * current[i] + (1.0 - EMA_ALPHA) * previous[i] for i in range(n)]
    if len(current) > n:
        out.extend(current[n:])
    return out


def apply_output_deadband(current: list[int], previous: list[int] | None) -> list[int]:
    if previous is None:
        return list(current)
    n = min(len(current), len(previous))
    out = []
    for i in range(n):
        if abs(current[i] - previous[i]) < OUTPUT_DEADBAND:
            out.append(previous[i])
        else:
            out.append(current[i])
    if len(current) > n:
        out.extend(current[n:])
    return out


# ── Revo2 异步控制循环 ─────────────────────────────────────────────────────────
async def control_loop(
    client_left,
    client_right,
    reader: Nova2SensorReader,
    shutdown_event: asyncio.Event,
) -> None:
    interval = 1.0 / CONTROL_HZ
    logger.info("控制循环已启动，频率：%d Hz", CONTROL_HZ)
    loop = asyncio.get_running_loop()
    rh_ema: list[float] | None = None
    lh_ema: list[float] | None = None
    rh_last_pos: list[int] | None = None
    lh_last_pos: list[int] | None = None

    while not shutdown_event.is_set():
        # 将阻塞 ctypes 调用放入线程池，避免占用 asyncio 事件循环。
        # 否则持有 GIL 的 ctypes 调用会阻塞 Modbus 传输层，导致除首次外
        # 的 set_finger_positions_and_speeds 调用频繁超时。
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
        rh_ema = ema_step(rh_raw, rh_ema)
        lh_ema = ema_step(lh_raw, lh_ema)

        rh_pos = nova2_to_revo2(rh_ema)
        lh_pos = nova2_to_revo2(lh_ema)
        rh_pos = apply_output_deadband(rh_pos, rh_last_pos)
        lh_pos = apply_output_deadband(lh_pos, lh_last_pos)
        rh_last_pos = list(rh_pos)
        lh_last_pos = list(lh_pos)

        # 打印手套数据，确认手套在持续输出。
        # 这里直接用 print()，因为 libstark.init_logging() 可能把日志级别
        # 提升到 INFO 以上，导致 logger.info() 不可见。
        rh_fmt     = " ".join(f"{v:.3f}" for v in rh_raw)
        lh_fmt     = " ".join(f"{v:.3f}" for v in lh_raw)
        rh_hw_fmt  = " ".join(f"{v:.3f}" for v in rh_raw_hw)
        lh_hw_fmt  = " ".join(f"{v:.3f}" for v in lh_raw_hw)
        print(f"手套归一化  右[{rh_fmt}]  左[{lh_fmt}]  |  状态 右={rh_norm_state[1]} 左={lh_norm_state[1]}", flush=True)
        print(f"手套原始值  右[{rh_hw_fmt}]  左[{lh_hw_fmt}]", flush=True)
        print(f"REVO2 目标  右{rh_pos}  左{lh_pos}", flush=True)

        try:
            # 顺序下发并检查返回值，便于定位“目标值正确但执行不对”的问题。
            right_ret = await client_right.set_finger_positions_and_speeds(
                RIGHT_ID, list(rh_pos), SPEEDS
            )
            left_ret = await client_left.set_finger_positions_and_speeds(
                LEFT_ID, list(lh_pos), SPEEDS
            )
            if not right_ret or not left_ret:
                print(
                    f"下发返回异常：右手={right_ret!r} 左手={left_ret!r}  |  "
                    f"右目标={rh_pos} 左目标={lh_pos}",
                    flush=True,
                )
        except Exception as exc:  # noqa: BLE001
            logger.error("下发位置指令失败：%s", exc)

        await asyncio.sleep(interval)


async def main_async(reader: Nova2SensorReader) -> None:
    libstark.init_logging()
    shutdown_event = setup_shutdown_event(logger)

    if not reader.sensecom_running():
        logger.critical("SenseCom 未运行，请先启动。")
        sys.exit(1)

    client_left  = await libstark.modbus_open(LEFT_PORT,  BAUDRATE)
    client_right = await libstark.modbus_open(RIGHT_PORT, BAUDRATE)
    if not client_left or not client_right:
        logger.critical("打开 Revo2 串口失败（单侧或双侧）。")
        sys.exit(1)

    left_info  = await client_left.get_device_info(LEFT_ID)
    right_info = await client_right.get_device_info(RIGHT_ID)
    if not left_info or not right_info:
        logger.critical("读取 Revo2 设备信息失败。")
        sys.exit(1)

    logger.info("左手设备：%s", left_info.description)
    logger.info("右手设备：%s", right_info.description)

    # 命令双手开始输出归一化数据。
    # 否则 SDK 可能返回默认值/旧值，而不是实时数据。
    for hand_name, is_right in (("右手", True), ("左手", False)):
        ok = reader.send_normalized_data(is_right)
        logger.info("SendNormalizedData %s：%s", hand_name, "成功" if ok else "失败（可能未找到手套）")

    task = asyncio.create_task(
        control_loop(client_left, client_right, reader, shutdown_event)
    )
    await shutdown_event.wait()

    task.cancel()
    await asyncio.gather(task, return_exceptions=True)
    await client_left.modbus_close()
    await client_right.modbus_close()
    logger.info("已退出。")


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
        logger.info("用户中断")
    sys.exit(0)


if __name__ == "__main__":
    main()
