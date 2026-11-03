# -*- coding: utf-8 -*-
"""車輪檢查：一次看一顆輪子的編碼器與驅動（接上真的 Nucleo）。

用法：
    # 只讀編碼器、不送馬達指令：用手轉輪子，看對應的數字有沒有變（往前轉應該是正的）
    python -m tools.wheel_check --port COM5
    # 只轉一顆輪子（車要架高）：看輪子有沒有轉、數字有沒有跟著變
    python -m tools.wheel_check --port COM5 --spin M1 --duty 0.4 --seconds 3
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rotbot.actuation.link import SerialMCU

NAMES = ("M1 左前", "M2 左後", "M3 右前", "M4 右後")


def main():
    ap = argparse.ArgumentParser(description="車輪檢查（編碼器 / 單輪驅動）")
    ap.add_argument("--port", required=True, help="COM5 / /dev/ttyACM0")
    ap.add_argument("--spin", choices=["M1", "M2", "M3", "M4"], help="只轉這一顆（不給 = 只讀編碼器）")
    ap.add_argument("--duty", type=float, default=0.4, help="輸出比例 -1..1（負 = 往後）")
    ap.add_argument("--seconds", type=float, default=20.0, help="持續時間；--spin 時建議 3")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    w = [0.0] * 4
    if args.spin:
        w[int(args.spin[1]) - 1] = max(-1.0, min(1.0, args.duty))
        print(f"轉 {NAMES[int(args.spin[1]) - 1]}，輸出 {w[int(args.spin[1]) - 1]:+.0%}，{args.seconds:.0f} 秒")
    else:
        print(f"只讀編碼器 {args.seconds:.0f} 秒：用手轉輪子，看數字（輸出軸 rpm）")
    print("    " + "  ".join(f"{n:>8}" for n in NAMES))
    with SerialMCU(args.port) as mcu:
        st = mcu.get_status()
        if st.estop:
            print("MCU 在急停狀態，馬達不會動")
        end = time.monotonic() + args.seconds
        try:
            while time.monotonic() < end:
                if args.spin:
                    mcu.drive(*w)                      # 要持續送，MCU 0.5 s 沒收到就停
                rpm = mcu.wheels()
                print("    " + "  ".join(f"{r:8.1f}" for r in rpm), flush=True)
                time.sleep(0.2)
        finally:
            mcu.drive(0, 0, 0, 0)


if __name__ == "__main__":
    main()
