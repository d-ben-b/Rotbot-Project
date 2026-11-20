# -*- coding: utf-8 -*-
"""MCU 韌體模擬器（Software-in-the-Loop）—— 建置並啟動「在電腦上跑的同一份韌體」。

firmware/rotbot_mcu 的 app.cpp / proto.cpp（Arduino 上跑的那份）+ firmware/host/sil_main.cpp
（模擬硬體 + TCP）→ build/mcu_sil_<variant>。Orin 端程式用 SerialMCU("socket://127.0.0.1:5760")
連線，與接真 Arduino 走同一條程式路徑。

變體：full  = 選配感測全上：飛輪轉速計（PI 穩速）+ 電池電壓前饋補償
              （預設，6m 建議的硬體配置）
      basic = 開迴路飛輪（config.h 出廠值）

編譯器：依序找 g++ / clang++ / c++；都沒有就用 pip 套件 ziglang（`pip install ziglang`，免安裝 VS）。

用法：
    python -m tools.mcu_sil                    # 建置 + 在 127.0.0.1:5760 執行（Ctrl+C 結束）
    python -m tools.mcu_sil --variant basic --port 5761
    python -m tools.mcu_sil --build            # 只建置
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
FW_DIR = REPO_ROOT / "firmware" / "rotbot_mcu"
HOST_DIR = REPO_ROOT / "firmware" / "host"
BUILD_DIR = REPO_ROOT / "build"
SOURCES = [FW_DIR / "app.cpp", FW_DIR / "proto.cpp", HOST_DIR / "sil_main.cpp"]
# shortcut: 模擬器的推彈約 0.35 s 一圈（實機 28BYJ-48 約 5 s，見 config.h），測試不用每發等 5 秒；要模擬比賽節奏時拿掉這兩項
SIL_FEED = ["-DFEED_STEP_HZ=6000", "-DFEED_ACCEL=200000", "-DMAG_TIMEOUT_MS=500"]   # 模擬器上彈 0.2 s
VARIANTS = {"full": ["-DFLY_HAS_TACH=1", "-DVBAT_ENABLED=1", *SIL_FEED],
            "basic": ["-DFLY_HAS_TACH=0", "-DVBAT_ENABLED=0", *SIL_FEED]}


class CompilerNotFound(RuntimeError):
    pass


def find_compiler() -> list[str]:
    """回傳 C++ 編譯器指令（list）；找不到丟 CompilerNotFound。"""
    for name in ("g++", "clang++", "c++"):
        exe = shutil.which(name)
        if exe:
            return [exe]
    try:
        import ziglang  # noqa: F401
        return [sys.executable, "-m", "ziglang", "c++"]
    except ImportError:
        pass
    raise CompilerNotFound("找不到 C++ 編譯器：請安裝 g++/clang++，或 `pip install ziglang`")


def exe_path(variant: str) -> Path:
    return BUILD_DIR / (f"mcu_sil_{variant}" + (".exe" if os.name == "nt" else ""))


def build(variant: str = "full", force: bool = False, verbose: bool = False) -> Path:
    """建置模擬器（原始碼沒變就略過）；回傳執行檔路徑。"""
    out = exe_path(variant)
    deps = SOURCES + list(FW_DIR.glob("*.h"))
    if not force and out.exists() and out.stat().st_mtime >= max(p.stat().st_mtime for p in deps):
        return out
    BUILD_DIR.mkdir(exist_ok=True)
    cmd = find_compiler() + ["-std=c++17", "-O2", f"-I{FW_DIR}", *VARIANTS[variant],
                             *map(str, SOURCES), "-o", str(out)]
    if os.name == "nt":
        cmd += ["-lws2_32", "-lwinmm"]
    if verbose:
        print(" ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"SIL 建置失敗：\n{r.stdout}\n{r.stderr}")
    return out


class SilProcess:
    """啟動模擬器子行程（context manager）；.url 給 SerialMCU 用。"""

    def __init__(self, variant: str = "full", port: int = 0, host: str = "127.0.0.1"):
        self.variant, self.port, self.host = variant, port, host
        self.proc: subprocess.Popen | None = None

    def start(self) -> "SilProcess":
        exe = build(self.variant)
        self.proc = subprocess.Popen([str(exe), "--port", str(self.port), "--host", self.host],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        line = self.proc.stdout.readline()               # "SIL listening on 127.0.0.1:NNNN ..."
        if "listening" not in line:
            self.stop()
            raise RuntimeError(f"SIL 啟動失敗：{line!r}")
        self.port = int(line.split()[3].rsplit(":", 1)[1])
        self.banner = line.strip()
        return self

    @property
    def url(self) -> str:
        return f"socket://{self.host}:{self.port}"

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


def main():
    ap = argparse.ArgumentParser(description="建置/執行 MCU 韌體模擬器（SIL）")
    ap.add_argument("--variant", choices=sorted(VARIANTS), default="full")
    ap.add_argument("--port", type=int, default=5760)
    ap.add_argument("--host", default="127.0.0.1",
                    help="0.0.0.0 = 讓區網其他電腦（例如 Orin）也連得到")
    ap.add_argument("--build", action="store_true", help="只建置不執行")
    ap.add_argument("--force", action="store_true", help="強制重新建置")
    args = ap.parse_args()
    exe = build(args.variant, force=args.force, verbose=True)
    print(f"建置完成：{exe}")
    if args.build:
        return
    try:
        subprocess.run([str(exe), "--port", str(args.port), "--host", args.host])
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
