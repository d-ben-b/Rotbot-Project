# -*- coding: utf-8 -*-
"""推彈檢查：送 n 次 FIRE，每次等 EV FIRED，印出每一發用了幾秒（接上真的 Nucleo）。

推彈是 28BYJ-48（config.h 的 FEED_*），FIRE 一次轉一圈。韌體要「已歸零 + 摩擦輪到速 + 轉塔停穩」才接受 FIRE：
  * MCU 還沒歸零（剛開機或急停過，俯仰落在靜止角度）：把目前姿態設成 (field.yaml turret.pitch_rest_deg, 0°)。
    水平沒擺正前方的話，之後要用轉塔，擺好姿態再到 teleop 重新歸零。
  * --rpm 0（預設）：送 FLY 1，換算成 PWM 是 0，摩擦輪不轉；沒有轉速計時 1.5 s 後就算到速。
    shortcut: 只在 FLY_HAS_TACH 0（目前實機）有效；裝了轉速計後要給真的轉速。
  * Ctrl+C 不會中斷正在轉的那一圈（韌體會轉完）；要立刻停就按急停。

用法（先關掉 teleop_web：COM 埠一次只能一個程式用）：
    python -m tools.feed_check --port COM5                         # 推 1 次，摩擦輪不轉
    python -m tools.feed_check --port COM5 --shots 10              # 連推 10 次，看輸出軸記號有沒有越停越偏
    python -m tools.feed_check --port COM5 --shots 10 --rpm 960    # 放球、摩擦輪 960
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rotbot.actuation import protocol
from rotbot.actuation.link import LinkError, SerialMCU
from rotbot.config import load_config


def run(mcu, shots: int, rpm: int = 0, gap: float = 1.0, log=print) -> list[float]:
    """回傳每一發 FIRE → EV FIRED 的秒數。"""
    st = mcu.get_status()
    if st.estop:
        raise LinkError("MCU 在急停狀態：放開急停再試")
    if not st.homed:
        p = mcu.pitch_rest_deg or 0.0
        mcu.zero(p, 0.0)
        log(f"MCU 還沒歸零：已把目前姿態設成 ({p:g}°, 0°)")
    mcu.flywheel(max(1, rpm))
    log("等 READY（摩擦輪到速、轉塔停穩）…")
    mcu.wait_ready(timeout=6.0)
    times = []
    for i in range(1, shots + 1):
        mark = mcu.event_mark()
        t0 = time.perf_counter()
        mcu.fire()
        ev = mcu.wait_event("FIRED", timeout=protocol.FIRE_TIMEOUT_S, since=mark)
        times.append(time.perf_counter() - t0)
        log(f"  第 {i} 發：{times[-1]:.2f} s（MCU 累計 {ev.arg} 發）")
        if i < shots:
            time.sleep(gap)
            mcu.wait_ready(timeout=6.0)
    return times


def main():
    ap = argparse.ArgumentParser(description="推彈檢查")
    ap.add_argument("--port", required=True, help="COM5 / /dev/ttyACM0")
    ap.add_argument("--shots", type=int, default=1, help="推幾次")
    ap.add_argument("--rpm", type=int, default=0, help="摩擦輪轉速；0 = 不轉（預設）")
    ap.add_argument("--gap", type=float, default=1.0, help="兩發之間停幾秒（看記號用）")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    try:
        mcu = SerialMCU(args.port, pitch_rest_deg=load_config().turret.pitch_rest_deg).open()
    except Exception as e:
        sys.exit(f"開不了 {args.port}：{e}\n（teleop_web 還開著的話先關掉）")
    print(f"推彈 {args.shots} 次、摩擦輪 {args.rpm} rpm；Ctrl+C 停止（正在轉的那一圈會轉完）")
    try:
        times = run(mcu, args.shots, args.rpm, args.gap)
        print(f"完成 {len(times)} 發，平均 {statistics.mean(times):.2f} s／發")
    except KeyboardInterrupt:
        print("（Ctrl+C 停止）")
    except LinkError as e:
        print(f"失敗：{e}")
    finally:
        mcu.flywheel(0)
        mcu.close()
        print("摩擦輪已停")


if __name__ == "__main__":
    main()
