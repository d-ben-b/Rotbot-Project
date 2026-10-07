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
frame = cam.read()          # 最新一張，BGR uint8 (1088, 1456, 3)
cam.set_exposure(250)       # µs，執行中可改
cam.release()
```

實測：`read()` 59 fps（每張 17 ms，其中 ISP 約 11 ms）；`camera_hw.ccm: false` 時 ISP 約 3 ms。
`read()` 一律回傳最新一張，處理比 60 fps 慢時舊的會被丟掉，不會累積延遲。

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
- **內參還沒標定**。`field.yaml` 的 `camera:` 仍是 1920×1080 的範例值，`IMX296Camera.intrinsics()` 會拒絕回傳；
  標定後把 `camera:` 改成 1456×1088 的真實內參。
