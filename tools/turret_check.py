# -*- coding: utf-8 -*-
"""轉塔步進馬達檢查：把目前位置當 0°，轉到指定角度再轉回來（接上真的 Nucleo）。

用來確認方向、每度幾步（config.h 的 YAW_STEPS_PER_DEG / PITCH_STEPS_PER_DEG）、來回重複精度。
正值 = 水平往右（從上面看順時針）/ 俯仰往上。

用法：
    python -m tools.turret_check --port COM5 --axis yaw --deg 5             # 第一次：小角度，看方向
    python -m tools.turret_check --port COM5 --axis yaw --deg 90 --hold 10  # 量每度幾步：停 10 秒量角度
    python -m tools.turret_check --port COM5 --axis yaw --deg 30 --cycles 20  # 來回 20 次，看雷射點有沒有跑掉
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rotbot.actuation.link import SerialMCU

CONFIG_H = Path(__file__).resolve().parent.parent / "firmware" / "rotbot_mcu" / "config.h"


def steps_per_deg(axis: str) -> float | None:
    """repo 裡 config.h 的每度步數（燒進去的韌體要是同一份）。"""
    m = re.search(rf"#define {axis.upper()}_STEPS_PER_DEG\s+([\d.]+)", CONFIG_H.read_text(encoding="utf-8"))
    return float(m.group(1)) if m else None


def wait_still(mcu, timeout: float):
    t0 = time.monotonic()
    st = mcu.get_status()
    while st.moving and time.monotonic() - t0 < timeout:
        time.sleep(0.05)
        st = mcu.get_status()
    return st, time.monotonic() - t0


def main():
    ap = argparse.ArgumentParser(description="轉塔步進馬達檢查")
    ap.add_argument("--port", required=True, help="COM5 / /dev/ttyACM0")
    ap.add_argument("--axis", required=True, choices=["yaw", "pitch"], help="yaw = 水平、pitch = 俯仰")
    ap.add_argument("--deg", type=float, default=5.0, help="要轉的角度（預設 5°；負 = 往左 / 往下）")
    ap.add_argument("--hold", type=float, default=3.0, help="到位後停幾秒（給你量角度）")
    ap.add_argument("--cycles", type=int, default=1, help="來回幾次")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    name = "水平" if args.axis == "yaw" else "俯仰"
    pose = (lambda d: (0.0, d)) if args.axis == "yaw" else (lambda d: (d, 0.0))
    print(f"{name}：目前位置當 0°，轉到 {args.deg:+.1f}° 再回 0°，共 {args.cycles} 次；Ctrl+C 立刻停")
    with SerialMCU(args.port) as mcu:
        if mcu.get_status().estop:
            print("MCU 在急停狀態，馬達不會動")
            return
        mcu.zero(0.0, 0.0)
        try:
            for i in range(args.cycles):
                for target in (args.deg, 0.0):
                    mcu.aim(*pose(target))
                    st, dt = wait_still(mcu, timeout=10.0)
                    got = st.yaw_deg if args.axis == "yaw" else st.pitch_deg
                    note = "（10 s 內沒停，檢查接線）" if st.moving else ""
                    print(f"  第 {i + 1} 次  → {target:+7.2f}°  MCU 回報 {got:+7.2f}°  用時 {dt:4.1f} s{note}",
                          flush=True)
                    time.sleep(args.hold if target else 0.5)
        except KeyboardInterrupt:
            print("（Ctrl+C 停止）")
        finally:
            mcu.stop()
    k = steps_per_deg(args.axis)
    if k and args.deg:
        print(f"\n量到實際轉了 θ 度 → config.h 的 {args.axis.upper()}_STEPS_PER_DEG 改成 "
              f"{k:g} × {abs(args.deg):g} ÷ θ（目前 {k:g}）")
    print("方向相反 → hal_arduino.h 的 " + args.axis.upper() + "_DIR_INVERT 改成 1")


if __name__ == "__main__":
    main()
