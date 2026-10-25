# -*- coding: utf-8 -*-
"""MCU 連線驗證（協定 v1）—— 同一套測試打：韌體模擬器 / 電腦上的 Arduino / Orin 上的 Arduino。

測試分三類：
  通訊（不會動任何東西）：BOOT、PING/版本、往返延遲、壞 CRC、垃圾重同步、未知指令、格式錯、
                          未 ZERO 就 AIM、超出範圍、未就緒就 FIRE、吞吐量
  動作（--allow-motion 或 --sil 才跑）：轉塔到位、飛輪到速、推球、重送去重、JOG/STOP、看門狗
  模擬器專屬（--sil）：急停、電池電壓下降時的轉速閉迴路、推球卡彈

用法：
    python -m tools.mcu_link_test --sil                       # 自動建置+啟動模擬器，全部測試
    python -m tools.mcu_link_test --sil --variant basic       # 測開迴路飛輪/固定時間推球的韌體變體
    python -m tools.mcu_link_test --port COM5                 # Arduino 接電腦：只跑通訊類
    python -m tools.mcu_link_test --port /dev/ttyACM0 --allow-motion   # Orin + Arduino + 馬達
結果另存 runs/mcu_link/<時間>_<目標>.json。
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rotbot.actuation import protocol
from rotbot.actuation.link import LinkError, NakError, SerialMCU

REPO_ROOT = Path(__file__).resolve().parent.parent


class Suite:
    def __init__(self, url: str, sil: bool, allow_motion: bool, variant: str | None,
                 n_ping: int = 200, verbose: bool = True):
        self.url, self.sil, self.motion = url, sil, allow_motion or sil
        self.variant, self.n_ping, self.verbose = variant, n_ping, verbose
        self.results: list[dict] = []
        self.link: SerialMCU | None = None

    # ── 小工具 ──────────────────────────────────────
    def record(self, name, status, detail="", t0=None):
        r = dict(name=name, status=status, detail=detail,
                 ms=None if t0 is None else round((time.perf_counter() - t0) * 1000, 1))
        self.results.append(r)
        if self.verbose:
            print(f"  [{status:4s}] {name:28s} {detail}")
        return r

    def expect_nak(self, code, cmd, *args):
        try:
            self.link.request(cmd, *args)
        except NakError as e:
            return e.code == code, f"NAK {protocol.ERROR_NAMES.get(e.code, e.code)}"
        return False, "竟然 ACK"

    def run_test(self, name, fn, needs_motion=False, sil_only=False, variant=None):
        if needs_motion and not self.motion:
            return self.record(name, "SKIP", "需要 --allow-motion")
        if sil_only and not self.sil:
            return self.record(name, "SKIP", "僅模擬器")
        if variant and self.variant != variant:
            return self.record(name, "SKIP", f"僅 {variant} 變體")
        t0 = time.perf_counter()
        try:
            status, detail = fn()
        except (LinkError, protocol.ProtocolError) as e:
            status, detail = "FAIL", f"{type(e).__name__}: {e}"
        return self.record(name, status, detail, t0)

    def wait_status(self, pred, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            st = self.link.get_status()
            if pred(st):
                return st
            time.sleep(0.02)
        return None

    # ── 測試 ────────────────────────────────────────
    def t_boot(self):
        if self.link.booted:
            return "PASS", "收到 EV BOOT（開埠重置 → 開機）"
        return "WARN", "沒收到 EV BOOT（板子開埠不重置；不影響通訊）"

    def t_ping(self):
        proto, fw, up = self.link.ping()
        return "PASS", f"協定 v{proto}、韌體 {fw}、MCU 已開機 {up} ms"

    def t_rtt(self):
        rtts = []
        for _ in range(self.n_ping):
            t0 = time.perf_counter()
            self.link.ping()
            rtts.append((time.perf_counter() - t0) * 1000)
        rtts.sort()
        p95 = rtts[int(0.95 * (len(rtts) - 1))]
        lim = 10.0 if self.sil else 20.0
        detail = (f"n={len(rtts)} 平均 {statistics.mean(rtts):.2f} ms、p95 {p95:.2f} ms、"
                  f"最大 {rtts[-1]:.2f} ms（門檻 p95<{lim:.0f}）")
        return ("PASS" if p95 < lim else "FAIL"), detail

    def t_bad_crc(self):
        before = self.link.get_status().crc_errors
        self.link.write_raw(b"PING,99*00\n")
        time.sleep(0.15)
        after = self.link.get_status().crc_errors
        ok = after == before + 1
        return ("PASS" if ok else "FAIL"), f"MCU CRC 錯誤計數 {before} → {after}（壞幀被丟棄、不回覆）"

    def t_garbage(self):
        self.link.write_raw(bytes(range(33, 127)) * 3 + b"\n" + b"\x00\xff\xfe" * 20 + b"\n")
        time.sleep(0.1)
        self.link.ping()
        return "PASS", "垃圾位元組 + 超長行後 PING 正常（換行重新同步）"

    def t_unknown(self):
        ok, d = self.expect_nak(protocol.E_UNKNOWN, "FOO")
        return ("PASS" if ok else "FAIL"), d

    def t_format(self):
        ok, d = self.expect_nak(protocol.E_FORMAT, "AIM", 100)
        return ("PASS" if ok else "FAIL"), d

    def t_not_homed(self):
        st = self.link.get_status()
        if st.homed:
            return "SKIP", "MCU 已 ZERO 過（非剛開機）"
        ok, d = self.expect_nak(protocol.E_NOT_HOMED, "AIM", 1000, 0)
        return ("PASS" if ok else "FAIL"), d + "（未 ZERO 不准動，防止開機姿態未知時亂轉）"

    def t_zero(self):
        self.link.zero(0.0, 0.0)
        st = self.link.get_status()
        return ("PASS" if st.homed and st.pitch_deg == 0 and st.yaw_deg == 0 else "FAIL"), \
            f"homed={st.homed} pitch={st.pitch_deg} yaw={st.yaw_deg}"

    def t_range(self):
        ok, d = self.expect_nak(protocol.E_RANGE, "AIM", 9000, 0)
        return ("PASS" if ok else "FAIL"), d + "（俯仰 90° 超出上限）"

    def t_fire_not_ready(self):
        self.link.flywheel(0)
        ok, d = self.expect_nak(protocol.E_NOT_READY, "FIRE")
        return ("PASS" if ok else "FAIL"), d + "（飛輪沒轉不准推球）"

    def t_throughput(self):
        n = 100
        t0 = time.perf_counter()
        for _ in range(n):
            self.link.get_status()
        dt = time.perf_counter() - t0
        return "PASS", f"{n} 次 STAT 一問一答 {dt * 1000:.0f} ms（{n / dt:.0f} 次/秒）"

    def t_aim(self):
        t0 = time.perf_counter()
        self.link.aim(20.0, -5.0)
        st = self.wait_status(lambda s: not s.moving and abs(s.pitch_deg - 20) < 0.05
                              and abs(s.yaw_deg + 5) < 0.05)
        if st is None:
            return "FAIL", "4 s 內未到位"
        return "PASS", f"轉到 (20°, -5°) 用時 {(time.perf_counter() - t0) * 1000:.0f} ms"

    def t_flywheel(self):
        t0 = time.perf_counter()
        self.link.flywheel(2400)
        st = self.link.wait_ready(timeout=6.0)
        detail = f"2400 rpm 到速 + 轉塔穩定 → READY 用時 {(time.perf_counter() - t0) * 1000:.0f} ms"
        if st.rpm >= 0:
            detail += f"（量測 {st.rpm} rpm）"
        return "PASS", detail

    def t_fire(self):
        shots0 = self.link.get_status().shots
        self.link.wait_ready(timeout=6.0)
        mark = self.link.event_mark()
        t0 = time.perf_counter()
        self.link.fire()
        ev = self.link.wait_event("FIRED", timeout=3.0, since=mark)
        shots1 = self.link.get_status().shots
        ok = shots1 == shots0 + 1 and ev.arg == shots1
        return ("PASS" if ok else "FAIL"), \
            f"FIRE → EV FIRED #{ev.arg} 用時 {(time.perf_counter() - t0) * 1000:.0f} ms"

    def t_dedupe(self):
        self.link.wait_ready(timeout=6.0)
        shots0 = self.link.get_status().shots
        seq = (self.link._seq + 100) % 255 + 1        # 刻意選一個最近沒用過的 SEQ
        line = protocol.encode_line("FIRE", seq)
        self.link.write_raw(line + line)              # 模擬「ACK 遺失 → 同 SEQ 重送」
        time.sleep(1.6)
        n_new = self.link.get_status().shots - shots0
        return ("PASS" if n_new == 1 else "FAIL"), f"同 SEQ 的 FIRE 送兩次 → 實際推球 {n_new} 次（應為 1）"

    def t_jog(self):
        y0 = self.link.get_status().yaw_deg
        t_end = time.monotonic() + 0.5
        while time.monotonic() < t_end:               # 手動模式：持續送 JOG（deadman）
            self.link.jog(0.0, 20.0)
            time.sleep(0.05)
        self.link.stop()
        st = self.wait_status(lambda s: not s.moving)
        if st is None:
            return "FAIL", "STOP 後仍在動"
        moved = st.yaw_deg - y0
        time.sleep(0.2)
        hold = self.link.get_status().yaw_deg
        ok = 6.0 < moved < 14.0 and abs(hold - st.yaw_deg) < 0.01 and st.rpm_target == 0
        return ("PASS" if ok else "FAIL"), \
            f"JOG +20°/s × 0.5 s → 轉了 {moved:.2f}°、STOP 後停住、飛輪目標 {st.rpm_target}"

    def t_watchdog(self):
        self.link.flywheel(1500)
        time.sleep(0.2)
        mark = self.link.event_mark()
        ev = self.link.wait_event("WDOG", timeout=2.5, since=mark, raise_on_timeout=False)
        st = self.link.get_status()
        ok = ev is not None and st.rpm_target == 0 and not st.wdog
        return ("PASS" if ok else "FAIL"), \
            (f"沉默 {ev.arg if ev else '?'} ms → EV WDOG、飛輪目標歸 0；收到新指令後解除"
             if ev else "沒有觸發看門狗")

    def t_estop(self):
        mark = self.link.event_mark()
        self.link.write_raw(b"!ESTOP 1\n")
        ev = self.link.wait_event("ESTOP", timeout=1.0, since=mark)
        ok1, d1 = self.expect_nak(protocol.E_ESTOP, "AIM", 1000, 0)
        st = self.link.get_status()
        mark = self.link.event_mark()
        self.link.write_raw(b"!ESTOP 0\n")
        ev2 = self.link.wait_event("ESTOP", timeout=1.0, since=mark)
        st2 = self.link.get_status()
        ok = ev.arg == 1 and ok1 and st.estop and ev2.arg == 0 and not st2.homed
        self.link.zero(0.0, 0.0)
        return ("PASS" if ok else "FAIL"), f"按下 → EV ESTOP、AIM {d1}；放開後需重新 ZERO（homed={st2.homed}）"

    def t_battery(self):
        self.link.write_raw(b"!BATT 0.85\n")
        try:
            self.link.flywheel(2400)
            t0 = time.perf_counter()
            st = self.link.wait_ready(timeout=8.0)
            dt = time.perf_counter() - t0
            err = abs(st.rpm - 2400) / 2400 * 100
        finally:
            self.link.write_raw(b"!BATT 1.0\n")
        return ("PASS" if err < 3 else "FAIL"), \
            (f"電池掉到 {st.vbat} V（85%）時飛輪 {dt:.1f} s 到速、穩在 {st.rpm} rpm（誤差 {err:.1f}%）"
             f"；電壓前饋 + PI 補償，未補償會掉到 ~2040 rpm")

    def t_jam(self):
        self.link.flywheel(2400)
        self.link.wait_ready(timeout=6.0)
        self.link.write_raw(b"!JAM 1\n")
        try:
            mark = self.link.event_mark()
            self.link.fire()
            ev = self.link.wait_event("FAULT", timeout=3.0, since=mark)
            st = self.link.get_status()
        finally:
            self.link.write_raw(b"!JAM 0\n")
        ok = ev.arg == 1 and not st.firing
        return ("PASS" if ok else "FAIL"), "推球卡住 → 逾時 EV FAULT、馬達停（不計球數）"

    # ── 主流程 ──────────────────────────────────────
    def run(self) -> list[dict]:
        if self.verbose:
            print(f"MCU 連線驗證：{self.url}（{'模擬器' if self.sil else '實機'}"
                  f"{'、含動作' if self.motion else '、只測通訊'}）")
        self.link = SerialMCU(self.url, poll_hz=0)   # 不背景輪詢：測試自己控制流量（看門狗測試要沉默）
        t0 = time.perf_counter()
        try:
            self.link.open()
        except LinkError as e:
            self.record("連線 + PING", "FAIL", str(e), t0)
            return self.results
        try:
            T = self.run_test
            T("開機事件 BOOT", self.t_boot)
            T("PING / 協定版本", self.t_ping)
            T("往返延遲 RTT", self.t_rtt)
            T("壞 CRC 被丟棄", self.t_bad_crc)
            T("垃圾資料後重新同步", self.t_garbage)
            T("未知指令 → NAK", self.t_unknown)
            T("參數格式錯 → NAK", self.t_format)
            T("未 ZERO 就 AIM → NAK", self.t_not_homed)
            T("ZERO 設定零點", self.t_zero)
            T("超出角度範圍 → NAK", self.t_range)
            T("未就緒 FIRE → NAK", self.t_fire_not_ready)
            T("吞吐量", self.t_throughput)
            T("轉塔 AIM 到位", self.t_aim, needs_motion=True)
            T("飛輪到速 → READY", self.t_flywheel, needs_motion=True)
            T("推球 FIRE → FIRED", self.t_fire, needs_motion=True)
            T("重送去重（不連發）", self.t_dedupe, needs_motion=True)
            T("手動 JOG / STOP", self.t_jog, needs_motion=True)
            T("看門狗（Orin 失聯）", self.t_watchdog, needs_motion=True)
            T("急停", self.t_estop, sil_only=True)
            T("電池下降時轉速閉迴路", self.t_battery, sil_only=True, variant="full")
            T("推球卡彈偵測", self.t_jam, sil_only=True, variant="full")
        finally:
            try:
                self.link.request("STOP")
            except LinkError:
                pass
            self.link.close()
        return self.results


def save(results: list[dict], target: str, meta: dict) -> Path:
    out = REPO_ROOT / "runs" / "mcu_link"
    out.mkdir(parents=True, exist_ok=True)
    safe = "".join(c if c.isalnum() else "_" for c in target)[:40]
    p = out / f"{time.strftime('%Y%m%d_%H%M%S')}_{safe}.json"
    p.write_text(json.dumps(dict(meta, results=results), ensure_ascii=False, indent=1),
                 encoding="utf-8")
    return p


def main():
    ap = argparse.ArgumentParser(description="MCU 連線驗證（協定 v1）")
    ap.add_argument("--port", help="COM5 / /dev/ttyACM0 / socket://127.0.0.1:5760")
    ap.add_argument("--sil", action="store_true", help="自動建置並啟動韌體模擬器")
    ap.add_argument("--variant", default="full", choices=["full", "basic"])
    ap.add_argument("--allow-motion", action="store_true", help="實機也跑會動馬達的測試")
    ap.add_argument("--n-ping", type=int, default=200)
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if not args.sil and not args.port:
        ap.error("請給 --port 或 --sil")

    if args.sil:
        from tools.mcu_sil import SilProcess
        with SilProcess(args.variant) as sil:
            print(sil.banner)
            suite = Suite(sil.url, sil=True, allow_motion=True, variant=args.variant,
                          n_ping=args.n_ping)
            results = suite.run()
        target = f"sil_{args.variant}"
    else:
        suite = Suite(args.port, sil=False, allow_motion=args.allow_motion, variant=None,
                      n_ping=args.n_ping)
        results = suite.run()
        target = args.port
    n = {s: sum(r["status"] == s for r in results) for s in ("PASS", "FAIL", "WARN", "SKIP")}
    print(f"\n結果：PASS {n['PASS']}  FAIL {n['FAIL']}  WARN {n['WARN']}  SKIP {n['SKIP']}")
    p = save(results, target, dict(target=target, host=os.environ.get("COMPUTERNAME", ""),
                                   time=time.strftime("%Y-%m-%d %H:%M:%S")))
    print(f"→ {p}")
    sys.exit(1 if n["FAIL"] else 0)


if __name__ == "__main__":
    main()
