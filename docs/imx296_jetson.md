# IMX296 相機（Raspberry Pi Global Shutter Camera）@ Jetson Orin Nano

機上相機的硬體與驅動說明。軟體介面是 `rotbot/perception/imx296.py` 的 `IMX296Camera`，
設定在 `configs/field.yaml` 的 `camera_hw:` 區段。

> 以下數字都是 2026-10-07 在實機上量的（Jetson Orin Nano Super devkit、L4T R36.4.4、25W 模式）。

## 硬體

| 項目 | 值 |
|---|---|
| Sensor | Sony IMX296LQR（彩色、global shutter），1456×1088，像素 3.45 µm，感光區 5.02×3.75 mm |
| 模式 | 1456×1088 @ 60 fps；1280×720 @ 90 fps（中央裁切，視角變窄） |
| 鏡頭座 | C / CS-mount。CS 鏡頭要拆掉 C-CS 轉接環，C 鏡頭要留著 |
| 接線 | 15-pin → 22-pin 轉接排線，接 devkit 的 **CAM1（J21）** |
| 曝光 / 增益 | 15–15700 µs；0–48 dB（類比） |

這顆取代了原本規劃的 e-CAM25_CUONX（AR0234，1920×1200）。鏡頭怎麼選（像圈、焦距對應看得到幾個孔、
光圈）見 `Parts purchasing/Camera/IMX296_Lens_Requirements.md`。目前裝的是 6 mm。

視角換算（f 為鏡頭焦距 mm）：水平 FOV = 2·atan(2.51/f)，垂直 FOV = 2·atan(1.88/f)。
6 mm → 45°×35°；4 mm → 64°×50°；16 mm → 18°×13°。

## 驅動（不在本 repo）

NVIDIA 沒有內建這顆 sensor 的驅動，要自己編 kernel module 和 device-tree overlay：

- 原始碼：<https://github.com/peclatj/jetson-orin-nano-devkit-imx296-rpi-global-shutter>
- 編譯需要 NVIDIA 的 `public_sources.tbz2`，**版本必須跟機器的 L4T 完全一致**（`cat /etc/nv_tegra_release`）。
  目前用 **R36.4.4**（JetPack 6.2.1 SD 卡映像檔）。R36.4.5–4.7 的原始碼 NVIDIA 沒有公開，
  所以裝好系統後要先凍結套件，**不要 `apt upgrade`**：
  `sudo apt-mark hold $(dpkg-query -W -f='${Package}\n' 'nvidia-l4t-*')`
- 安裝後 `/boot/extlinux/extlinux.conf` 多一個 `imx296` 開機項目（套用 overlay），目前設為預設；
  原本的 `primary` 項目保留，開機選單按 `1` 可選回。
- overlay 生效時模組會自動載入。確認方式：`sudo dmesg | grep imx296` 應看到
  `IMX296LQ (color) detected`，且 `/dev/video0` 存在。

這台 Jetson 上完整的建置紀錄與踩過的坑在 `~/Desktop/camera-imx296/CLAUDE.md`（repo 外）。

## 使用

```bash
python -m tools.camera_check                 # 拍一張、量幀率、檢查亮度
python -m tools.camera_check --auto-wb       # 對著白紙量白平衡，把印出的值寫進 field.yaml
```

```python
from rotbot.config import load_config
from rotbot.perception.camera import make_camera

cam = make_camera(load_config())
frame = cam.read()          # 最新一張，BGR uint8 (1088, 1456, 3)，已去畸變
K = cam.intrinsics()        # 這張影像的針孔內參（PnP 用這個）
cam.set_exposure(250)       # µs，執行中可改
cam.release()
```

實測：`read()` 47–50 fps（每張約 20 ms：ISP 約 11 ms + 去畸變約 3 ms + 取像複製）；
未標定（不去畸變）時 59 fps；`camera_hw.ccm: false` 時 ISP 約 3 ms。
`read()` 一律回傳最新一張，處理比 60 fps 慢時舊的會被丟掉，不會累積延遲。

## 內參標定

內參是把「孔在畫面上的位置與大小」換算成真實距離與角度所需的數字（像素單位的焦距、主點、鏡頭畸變）。
**鏡頭的光圈與對焦先調好並鎖緊再做**；換鏡頭或重新對焦後要重做。

目前的結果（2026-10-07，6 mm 鏡頭，存在 `configs/field.yaml` 的 `camera_calib:`）：
原始 fx 1922.6 / fy 1924.0、主點 (777.7, 578.9)、k1 −0.508、k2 +0.269；RMS 0.71 px（39 張）。
鏡頭實際視角 44.6°×32.9°，**桶狀畸變很明顯：畫面四角位移約 129 px**。

### 相機類別怎麼用這份標定

`IMX296Camera.read()` 回傳的是**去畸變後**的影像，`intrinsics()` 回傳該影像的針孔內參
（fx = fy = 1794.1，主點同上）。所以 YOLO 與 PnP 都當它是理想針孔相機即可，
**但要用 `camera.intrinsics()`，不要用 `cfg.camera.matrix()`** —— `field.yaml` 的 `camera:` 是模擬與合成資料用的相機模型
（1920×1080、視角 69°），跟實機刻意分開存（原因見下方「6 mm 鏡頭與定位」）。

去畸變後畫面上下緣中央各有一小條黑邊（約佔 1%），是為了保留完整的水平視角。
`camera_hw.undistort_interp`：`nearest`（預設，約 3 ms）或 `linear`（約 18 ms，邊緣較平滑）。
要拿鏡頭原始影像用 `read_distorted()`。

### 重新標定的步驟

1. 印出 [checkerboard_A4.pdf](checkerboard_A4.pdf)（A4 橫式、「實際大小 / 100%」），**整張平貼在硬板或紙箱上**。
   直接手拿紙會彎，結果不能用（實測 RMS 1.22 px、主點偏 200 px）。格子實際印成幾 mm 不影響內參。
2. `python3 -m tools.calibrate_camera`，用瀏覽器開 `http://localhost:8081`
   （別台電腦看：加 `--lan`，開 `http://<Jetson IP>:8081`）。
3. 把棋盤格拿到相機前 **0.4–0.8 m**，整張都要在畫面內。每換一個位置停住約半秒，畫面邊框閃綠色＝收到一張。
   要把綠色區塊塗滿（尤其四個角落），並且有幾張把板子往左右、上下傾斜 20–40 度。
   拿太遠（板子不到畫面寬度的 1/5）工具會拒收：遠的照片誤差數字漂亮，但焦距與主點定不準。
4. 收滿 20 張後自動計算，結果印在終端機並存到 `runs/calibration/<時間>/`。RMS < 0.5 px 良好，< 1 px 可用。
5. 把印出的 `camera_calib:` 區塊覆蓋到 `configs/field.yaml`。
6. 想更穩：多拍一輪，然後合併計算
   `python3 -m tools.calibrate_camera --resolve runs/calibration/<第一次> runs/calibration/<第二次>`。

### 6 mm 鏡頭與定位

把模擬用的相機換成這顆 6 mm 的實際視角後，現有的「孔編號指派」在只看到部分板面時會認錯孔。
合成測試（完美偵測、隨機視角 2.0–3.2 m、各 120 個場景）中，出現「某個孔定位偏差 > 10 cm」的場景比例：
3.5 mm 8%、4 mm 23%、5 mm 62%、**6 mm 91%**。這是待解的問題（換短焦鏡頭，或讓孔編號指派能處理部分視野），
在解決之前 `camera:` 不改成實機內參，以免模擬與既有測試失效。

> **2026-10-25 更新**：投球距離改為 6 m 後，整面網框都在畫面內（掃視姿態四邊留 ≥ 218 px），比賽距離不會遇到這個問題，
> 所以 `camera:` 已改成實機去畸變後的針孔內參（1456×1088、f 1794.1）。用實機內參重跑 6 m 評估的結果見
> [results/2026-10-25_reassess_6m_imx296.md](results/2026-10-25_reassess_6m_imx296.md)。2.5 m 的數字仍然成立，但已不是比賽距離。

## 要知道的幾件事

- **不走 Argus / `nvarguscamerasrc`**。Argus 沒有這顆 sensor 的 tuning，自動曝光會持續震盪，也不能固定短曝光。
- **曝光與增益一定要在串流進行中設定**。驅動每次開串流都會把 sensor 重設成預設值，而 V4L2 仍記著上次的值；
  新值跟記住的值相同時核心不會真的寫入。`IMX296Camera` 已處理（開串流後先寫相鄰值再寫目標值）。
  用 `v4l2-ctl` 手動「先設定、再另一行抓圖」的話，設的值不會生效。
- **不要改 `frame_rate` 控制項**。驅動內部記住的 frame length 不會在重新上電時還原，之後算曝光會出錯。
- **同一時間只能有一個程式開相機**。第二個會得到 `Device or resource busy`。
- **8333 µs 是 60 Hz 市電燈光下不閃的曝光**。用更短的曝光時，畫面亮度可能逐張變動（看燈具而定）。
- **時間戳**（`cam.timestamp`）來自 sensor 時序，相鄰兩張固定差 16.56 ms，適合算球速；
  但它跟 `time.monotonic()` 不是同一個時鐘，只能拿來算差值。
- **標定只對目前的鏡頭狀態有效**。動過對焦環、光圈以外的任何東西（換鏡頭、調後焦環）都要重新標定。
