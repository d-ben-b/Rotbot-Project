# -*- coding: utf-8 -*-
"""摩擦輪檢查：用指定轉速轉一段時間（接上真的 Nucleo）。

兩顆摩擦輪由 D5（左）、D6（右）同一個 PWM 驅動。沒有轉速計時韌體是開迴路：
PWM 比例 ≈ 目標 rpm ÷ FLY_MAX_RPM（config.h，預設 3000），例如 600 rpm ≈ 20%。

用法：
    python -m tools.fly_check --port COM5 --rpm 600 --seconds 5           # 第一次：低速
    python -m tools.fly_check --port COM5 --rpm 2400 --seconds 5 --ramp 2  # 設計轉速，2 秒內慢慢加上去
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rotbot.actuation.link import SerialMCU


def main():
    ap = argparse.ArgumentParser(description="摩擦輪檢查")
    ap.add_argument("--port", required=True, help="COM5 / /dev/ttyACM0")
    ap.add_argument("--rpm", type=int, default=600, help="目標轉速（預設 600 ≈ PWM 20%%）")
    ap.add_argument("--seconds", type=float, default=5.0, help="持續時間（含加速）")
    ap.add_argument("--ramp", type=float, default=1.0, help="從 0 加到目標轉速的秒數")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    print(f"摩擦輪 {args.rpm} rpm、{args.seconds:.0f} 秒（{args.ramp:.1f} 秒內加速）；Ctrl+C 立刻停")
    with SerialMCU(args.port) as mcu:
        if mcu.get_status().estop:
            print("MCU 在急停狀態，馬達不會動")
        t0 = time.monotonic()
        try:
            while (el := time.monotonic() - t0) < args.seconds:
                k = min(1.0, el / args.ramp) if args.ramp > 0 else 1.0
                mcu.flywheel(round(args.rpm * k))
                st = mcu.get_status()
                meas = "無轉速計" if st.rpm < 0 else f"{st.rpm:5d} rpm"
                print(f"  {el:4.1f} s  目標 {st.rpm_target:5d} rpm  量測 {meas}  到速 {'是' if st.fly_ok else '否'}",
                      flush=True)
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("（Ctrl+C 停止）")
        finally:
            mcu.flywheel(0)
            print("已送出停止（目標 0 rpm）")


if __name__ == "__main__":
    main()
