# -*- coding: utf-8 -*-
"""自主主迴圈 —— 掃視 → 看 → 算 → 轉 → 等 READY → 發射 →（有落點回授就）校正。

一發的流程（step）：
  1. 轉塔到「掃視姿態」（讓 12 孔都入鏡）並等停穩
  2. 拍照 → detector.detect → localize_holes（整板 PnP）→ 每孔量測（相機系）
  3. 用轉塔「實際」角度算相機外參（frames.camera_extrinsic_at）→ plan_next_shot 選孔、解仰角/水平角
  4. 疊加線上校正 → AIM + 飛輪 → 等 MCU READY（到位穩定 + 到速）→ FIRE → 等 EV FIRED
  5. 若有落點回授 observe_impact（實機：落點偵測或操作手標記；模擬：sim/twin.py）→ 校正更新

actuator 需要 SerialMCU 的介面（aim/flywheel/get_status/wait_ready/fire/event_mark/wait_event）；
camera/detector 走 HAL（真相機 + YOLO，或 sim/twin.py 的數位孿生）。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from ..config import Config
from ..decision import frames
from ..decision.calibration import AdaptiveCalibrator
from ..decision.strategy import ShotPlanner, plan_next_shot
from ..hal import Camera
from ..perception.detector import Detector
from ..perception.localize import localize_holes
from .state_machine import GameState


def hole_index(group: str, idx: int) -> int:
    """孔 → 校正器索引（main 0~8、bonus 9~11）。"""
    return idx if group == "main" else 9 + idx


@dataclass
class ShotLog:
    ball: int
    target: str                 # "main#4" / "bonus#1"
    is_lit: bool
    pitch_deg: float            # 實際下達的指令（含校正）
    yaw_deg: float
    corr_deg: tuple[float, float]
    n_dets: int
    perceive_ms: float          # 轉到掃視姿態 + 拍照 + 偵測 + 定位
    aim_ms: float               # 轉到射擊姿態 + 等 READY
    fire_ms: float              # FIRE → EV FIRED
    total_ms: float
    error_cm: float | None = None
    hit: bool | None = None
    points: int | None = None
    source: str = "camera"      # "camera" = 這一球拍到網板；"last_seen" = 沒看到，沿用上一次看到的網板


class ControlLoop:
    """感知→決策→致動 的自主主迴圈。"""

    STALE_S = 1.0               # 相機這麼久沒有新畫面（SharedCamera.age）就不拍了，直接沿用上一次看到的網板

    def __init__(self, cfg: Config, camera: Camera, detector: Detector, actuator, *,
                 rpm: float, calib: AdaptiveCalibrator | None = None,
                 planner: ShotPlanner | None = None, sm=None, survey: tuple | None = None,
                 observe_impact=None, ready_timeout: float = 4.0, perceive_retries: int = 2,
                 is_active=None, reuse_board: bool = True):
        self.cfg, self.camera, self.detector, self.mcu = cfg, camera, detector, actuator
        # 每個會動到機構的步驟前都問一次；操作手切手動/急停時立刻放手（不會在交接後還發射）
        self.is_active = is_active or (lambda: True)
        self.rpm = rpm
        self.calib = calib
        self.planner = planner or ShotPlanner(cfg)
        self.sm = sm
        self.survey = survey or self.default_survey_pose(cfg)
        self.observe_impact = observe_impact
        self.ready_timeout, self.perceive_retries = ready_timeout, perceive_retries
        self.K = camera.intrinsics()
        self.logs: list[ShotLog] = []
        self.skipped = 0              # 連續幾次看不到板（相機停了或 PnP 失敗）；看到一次就歸零
        self.skipped_total = 0        # 這一場累計看不到板的次數（紀錄用）
        self.no_target = False        # 沒有可投目標（球數/孔上限用完、射程外）
        self.reuse_board = reuse_board
        self.last_seen = None         # (SceneEstimate, 外參, 時間, MCU 重開機次數)：上一次解出整板位姿
        self.from_last_seen = False   # 這一次 perceive 是不是沿用 last_seen
        self.t0 = time.monotonic()

    @staticmethod
    def default_survey_pose(cfg: Config) -> tuple[float, float]:
        """掃視姿態：相機對準孔陣列的垂直中心（含加分排），水平對準網板中心。"""
        f = cfg.field
        top = f.grid_cy + f.row_pitch + f.bonus_dy + f.bonus_d / 2
        bottom = f.grid_cy - f.row_pitch - f.frame_d / 2
        pitch = math.degrees(math.atan2(0.5 * (top + bottom) - f.launch_h, f.dist_x))
        yaw = math.degrees(math.atan2(f.grid_cx, f.dist_x))
        return pitch, yaw

    def elapsed(self) -> float:
        return self.sm.elapsed if self.sm is not None and self.sm.t_start else time.monotonic() - self.t0

    def _move_and_settle(self, pitch: float, yaw: float, timeout: float = 4.0):
        """轉到指定角度並等停穩（不要求飛輪到速）；回傳 MCU 狀態。"""
        self.mcu.aim(pitch, yaw)
        deadline = time.monotonic() + timeout
        while True:
            st = self.mcu.get_status()
            if (not st.moving and abs(st.pitch_deg - pitch) < 0.05
                    and abs(st.yaw_deg - yaw) < 0.05):
                return st
            if time.monotonic() > deadline:
                raise TimeoutError(f"轉塔 {timeout:.1f} s 內未到 ({pitch:.2f}, {yaw:.2f})")
            time.sleep(0.01)

    def _reboots(self) -> int:
        return getattr(self.mcu, "reboots", 0)

    def perceive(self):
        """掃視 + 拍照 + 偵測 + 定位；回傳 (SceneEstimate, 外參, 偵測數)。失敗回傳 (None, None, n)。

        看不到板（相機停了、PnP 解不出來）時沿用上一次看到的網板，self.from_last_seen = True。
        自主模式中底盤不動、網板固定，上一次的「估計 + 當時的外參」換算到發射點系仍然有效。
        作廢的時機：MCU 重開機（補回的歸零可能有誤差）；手動時開過車（AutonomousRunner.note_chassis_moved）。
        """
        self.from_last_seen = False
        if not self.is_active():
            return None, None, 0
        st = self._move_and_settle(*self.survey)
        extr = frames.camera_extrinsic_at(self.cfg, st.pitch_deg, st.yaw_deg)  # 用實際角度
        n = 0
        age = getattr(self.camera, "age", None)
        if age is None or age() <= self.STALE_S:         # 相機已經停了就不必等 read() 逾時
            for _ in range(self.perceive_retries + 1):
                try:
                    img = self.camera.read()
                except Exception:                        # 相機逾時：不再重拍
                    break
                dets = self.detector.detect(img)
                n = len(dets)
                est = localize_holes(dets, self.cfg, self.K)
                if est.board_pose is not None:
                    self.last_seen = (est, extr, time.monotonic(), self._reboots())
                    return est, extr, n
        if self.reuse_board and self.last_seen is not None and self.last_seen[3] == self._reboots():
            self.from_last_seen = True
            return self.last_seen[0], self.last_seen[1], n
        return None, None, n

    def step(self) -> ShotLog | None:
        """一發；沒有可投目標（球投完 / 看不到板）回傳 None。"""
        t_start = time.perf_counter()
        if not self.is_active():
            return None
        est, extr, n_dets = self.perceive()
        t_seen = time.perf_counter()
        if est is None or self.from_last_seen:
            if self.is_active():                         # 真的看不到板（不是被接手）才算跳過
                self.skipped += 1
                self.skipped_total += 1
            if est is None:
                return None
        else:
            self.skipped = 0
        plan = plan_next_shot(est.aims, self.planner, self.cfg, extrinsic=extr,
                              elapsed_s=self.elapsed())
        if plan is None:
            self.no_target = True
            return None
        t = plan.target
        hidx = hole_index(t.group, t.idx)
        dp = dy = 0.0
        if self.calib is not None:
            dp, dy = (math.degrees(v) for v in self.calib.correction(hidx))
        pitch, yaw = plan.pitch_deg + dp, plan.yaw_deg + dy

        if not self.is_active():
            return None
        self.mcu.aim(pitch, yaw)
        self.mcu.flywheel(self.rpm)
        self.mcu.wait_ready(timeout=self.ready_timeout)
        t_ready = time.perf_counter()
        if not self.is_active():                         # 等 READY 期間被接手 → 不發射
            return None
        st = self.mcu.get_status()                       # 發射當下的實際角度（給落點回授用）
        mark = self.mcu.event_mark()
        self.mcu.fire()
        self.mcu.wait_event("FIRED", timeout=3.0, since=mark)
        t_fired = time.perf_counter()
        self.planner.record(t)

        log = ShotLog(ball=len(self.logs) + 1, target=f"{t.group}#{t.idx}", is_lit=t.is_lit,
                      pitch_deg=pitch, yaw_deg=yaw, corr_deg=(dp, dy), n_dets=n_dets,
                      perceive_ms=(t_seen - t_start) * 1000, aim_ms=(t_ready - t_seen) * 1000,
                      fire_ms=(t_fired - t_ready) * 1000, total_ms=(t_fired - t_start) * 1000,
                      source="last_seen" if self.from_last_seen else "camera")
        if self.observe_impact is not None:
            obs = self.observe_impact(t, st.pitch_deg, st.yaw_deg)
            if obs is not None:
                (e_z, e_y), hit, pts = obs
                log.error_cm = float(np.hypot(e_z, e_y) * 100)
                log.hit, log.points = hit, pts
                if self.calib is not None:
                    self.calib.update(hidx, (e_z, e_y))
        self.logs.append(log)
        return log

    def run(self, max_shots: int | None = None) -> list[ShotLog]:
        """連續投到球用完 / 達上限 / 狀態機離開 AUTONOMOUS / 連續 6 次看不到板。"""
        while self.planner.remaining > 0 and (max_shots is None or len(self.logs) < max_shots):
            if self.sm is not None and self.sm.update() != GameState.AUTONOMOUS:
                break
            if self.step() is None and (self.no_target or self.skipped > 5):
                break
        return self.logs


class AutonomousRunner:
    """背景執行緒：比賽狀態為 AUTONOMOUS 時持續呼叫 ControlLoop.step()。

    與手動操控網頁同一個程式：操作手按「開始」→ 前 60 秒 AI 投球（網頁手動輸入被拒）→
    第 61 秒起操作手「切手動」時，下一個會動機構的步驟前就放手；「交還 AI」則接著投（同一份球數排程）。
    make_loop()：每場（每次按開始）建立新的 ControlLoop（新的排程與計時）。
    看不到板時不放棄，一直重試到切手動或比賽結束（以前累計 6 次就整場停掉）；warning() 給網頁顯示原因。
    """

    WARN_SKIPS = 3          # 連續幾次看不到板就在網頁顯示
    WARN_ERROR_S = 3.0      # 例外（相機逾時、通訊）發生後顯示幾秒

    def __init__(self, sm, make_loop, on_log=None):
        import threading
        self.sm, self.make_loop, self.on_log = sm, make_loop, on_log
        self.loop: ControlLoop | None = None
        self.errors: list[str] = []
        self._error_t = -1e9
        self._match_t0 = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="autonomous", daemon=True)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=5.0)

    def _run(self):
        while not self._stop.is_set():
            if self.sm.update() != GameState.AUTONOMOUS:
                self._stop.wait(0.05)
                continue
            if self.loop is None or self._match_t0 != self.sm.t_start:      # 新的一場
                self.loop, self._match_t0 = self.make_loop(), self.sm.t_start
            if self.loop.planner.remaining <= 0 or self.loop.no_target:
                self._stop.wait(0.2)
                continue
            try:
                log = self.loop.step()
            except Exception as e:                    # 通訊逾時/相機逾時：記錄後繼續，不讓執行緒死掉
                self.errors.append(f"{type(e).__name__}: {e}")
                self._error_t = time.monotonic()
                self._stop.wait(0.2)
                continue
            if log is None and self.loop.skipped:     # 看不到板：稍等再重試
                self._stop.wait(0.1)
            if log is not None and self.on_log:
                self.on_log(log)

    def note_chassis_moved(self) -> None:
        """手動時開過車：上一次看到的網板位置作廢（TeleopServer 送出非零的 DRIVE 時呼叫）。

        只轉轉塔不算：網板相對底盤的位置沒變，last_seen 換算到發射點系仍然有效。
        """
        if self.loop is not None:
            self.loop.last_seen = None

    def resume_blocker(self) -> str:
        """「交還 AI」前的檢查；回傳不能交還的原因，空字串 = 可以。

        相機停了，又沒有可沿用的網板位置（手動時開過車、或從來沒看到過網板）時，
        AI 接手後只會在掃視姿態一直重試、不會投球，不如留在手動。
        """
        loop = self.loop
        if loop is None:
            return ""
        age = getattr(loop.camera, "age", None)
        if age is not None and age() > loop.STALE_S and loop.last_seen is None:
            return "相機沒有畫面，AI 也沒有可沿用的網板位置（手動時開過車）"
        return ""

    def warning(self) -> str:
        """給網頁顯示的 AI 狀態警告；沒事時是空字串。"""
        loop = self.loop
        if loop is not None and self.sm.state == GameState.MANUAL:
            why = self.resume_blocker()
            return f"現在不能交還 AI：{why}" if why else ""
        if loop is None or self.sm.state != GameState.AUTONOMOUS:
            return ""
        if time.monotonic() - self._error_t < self.WARN_ERROR_S and self.errors:
            return f"AI 發生錯誤，持續重試中：{self.errors[-1]}"
        seen = loop.last_seen
        if loop.from_last_seen and seen is not None:
            return f"AI 看不到網板，沿用 {time.monotonic() - seen[2]:.0f} s 前看到的網板位置投球"
        if loop.skipped >= self.WARN_SKIPS:
            return f"AI 連續 {loop.skipped} 次看不到網板（PnP 失敗），持續重試中"
        return ""
