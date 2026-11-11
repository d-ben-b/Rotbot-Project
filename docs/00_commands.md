# 指令集

每一段都標明在哪台機器執行；程式碼框裡的內容可以整段複製貼上。

- **筆電**：Windows PowerShell，先 `cd` 到 `Rotbot-Project` 資料夾、啟用 `.venv`，指令用 `python`。
- **Orin**：SSH 登入後先 `cd ~/Desktop/Rotbot-Project`，指令用 `python3`。
- 第 3 節的測試工具在 Orin 上也能跑：把 `COM5` 換成 `/dev/ttyACM0`、`python` 換成 `python3`。

## 0. 固定資訊

| 項目 | 值 |
| --- | --- |
| Orin 帳號與 IP（Wi-Fi） | `ben-orin@192.168.50.10` |
| Orin 帳號與 IP（USB-C 線，不需要路由器） | `ben-orin@192.168.55.1` |
| Orin 上的專案資料夾 | `~/Desktop/Rotbot-Project` |
| Nucleo 序列埠（接筆電） | `COM5` |
| Nucleo 序列埠（接 Orin） | `/dev/ttyACM0` |
| 操控網頁 | `http://localhost:8080` |

筆電啟用 `.venv`（每開一個新的 PowerShell 視窗做一次）：

```powershell
cd C:\Users\Ben\Documents\GitHub\Rotbot-Project
.venv\Scripts\Activate.ps1
```

## 1. 連線與開關機

**筆電：登入 Orin（Wi-Fi）**

```powershell
ssh ben-orin@192.168.50.10
```

**筆電：登入 Orin（USB-C 線）**

```powershell
ssh ben-orin@192.168.55.1
```

**筆電：手把用的 SSH 通道**（另開一個視窗，開著不要關；之後瀏覽器開 `http://localhost:8080`）

```powershell
ssh -N -L 8080:127.0.0.1:8080 ben-orin@192.168.50.10
```

**Orin：關機**（拔 Orin 的電池前一定要做，等燈全部熄滅再拔）

```bash
sudo shutdown -h now
```

**Orin：找不到 `/dev/ttyACM0` 時，看 Nucleo 有沒有被認到**

```bash
ls /dev/ttyACM* /dev/ttyUSB*
lsusb | grep -i 0483
sudo dmesg | tail -20
```

## 2. 操控網頁（手把開車、瞄準）

**Orin：實機，Nucleo 接 Orin，開相機**

```bash
cd ~/Desktop/Rotbot-Project
pkill -f tools.teleop_web
python3 -m tools.teleop_web --mcu /dev/ttyACM0 --camera cfg --auto-s 5
```

**筆電：實機，Nucleo 接筆電**（筆電本機開網頁，不用 SSH 通道）

```powershell
python -m tools.teleop_web --mcu COM5 --camera none --auto-s 5
```

**筆電：只用模擬器，不接任何硬體**

```powershell
python -m tools.teleop_web --mcu sil --camera sim --auto-s 5
```

網頁上依序按：「歸零」、「開始」，等 5 秒後按「切手動」。

| 參數 | 值 | 意思 |
| --- | --- | --- |
| `--mcu` | `COM5`、`/dev/ttyACM0` | 真的 Nucleo |
| `--mcu` | `sil` | 電腦上的韌體模擬器，**實機不會動** |
| `--mcu` | `none` | 不接 MCU |
| `--camera` | `cfg` | 機上的 IMX296（只有 Orin 有） |
| `--camera` | `0` | USB 相機或筆電 webcam |
| `--camera` | `sim` | 合成畫面 |
| `--camera` | `none` | 沒有畫面 |
| `--auto-s` | `5` | 按「開始」後 AI 自主的秒數；比賽是 60 |

## 3. 硬體測試（筆電接 Nucleo）

同一個序列埠一次只能給一個程式用：跑這些工具前，先停掉 `teleop_web`，並關掉 Arduino IDE 的序列埠監控視窗。

### 通訊

```powershell
python -m tools.mcu_link_test --port COM5
```

✅ 最後一行 `FAIL 0`。

### 車輪（車要架高、裝上保險絲）

**只讀編碼器**（用手轉輪子，往前轉是正的）

```powershell
python -m tools.wheel_check --port COM5
```

**編碼器除錯**（顯示 A、B 腳位電位和原始計數）

```powershell
python -m tools.wheel_check --port COM5 --raw
```

**四顆各往前轉 3 秒**（✅ 往箭頭方向轉、rpm 是正的）

```powershell
python -m tools.wheel_check --port COM5 --spin M1 --duty 0.4 --seconds 3
python -m tools.wheel_check --port COM5 --spin M2 --duty 0.4 --seconds 3
python -m tools.wheel_check --port COM5 --spin M3 --duty 0.4 --seconds 3
python -m tools.wheel_check --port COM5 --spin M4 --duty 0.4 --seconds 3
```

**四顆各往後轉 3 秒**（✅ rpm 是負的）

```powershell
python -m tools.wheel_check --port COM5 --spin M1 --duty -0.4 --seconds 3
python -m tools.wheel_check --port COM5 --spin M2 --duty -0.4 --seconds 3
python -m tools.wheel_check --port COM5 --spin M3 --duty -0.4 --seconds 3
python -m tools.wheel_check --port COM5 --spin M4 --duty -0.4 --seconds 3
```

M1 左前、M2 左後、M3 右前、M4 右後，前方是底盤上綠色箭頭的方向。

**四顆自動測一輪**（逐輪轉、編碼器方向、停止送指令後自動停車）

```powershell
python -m tools.mcu_link_test --port COM5 --allow-drive
```

### 轉塔

**水平：第一次，小角度看方向**

```powershell
python -m tools.turret_check --port COM5 --axis yaw --deg 5
```

**水平：轉 90° 停 10 秒，量每度幾步**

```powershell
python -m tools.turret_check --port COM5 --axis yaw --deg 90 --hold 10
```

**水平：來回 20 次，看重複精度**

```powershell
python -m tools.turret_check --port COM5 --axis yaw --deg 30 --cycles 20
```

**俯仰**

```powershell
python -m tools.turret_check --port COM5 --axis pitch --deg 60
```

### 摩擦輪

**第一次：低速**

```powershell
python -m tools.fly_check --port COM5 --rpm 600 --seconds 5
```

**設計轉速，2 秒內慢慢加上去**

```powershell
python -m tools.fly_check --port COM5 --rpm 2400 --seconds 5 --ramp 2
```

## 4. 相機與推論（Orin）

**相機：拍一張、量幀率**

```bash
cd ~/Desktop/Rotbot-Project
python3 -m tools.camera_check
```

**模型推論速度與功耗（5 分鐘）**：TensorRT engine 檔 `yolo_fp16.engine` 放在 `~`；檔名裡的 `test` 每次換一個名字，避免蓋掉舊結果。

```bash
cd ~
sudo tegrastats --interval 1000 --logfile ~/tegra_test.log --start
/usr/src/tensorrt/bin/trtexec --loadEngine=yolo_fp16.engine --duration=300 2>&1 | tee ~/trt_test.txt | grep -E "Throughput|Latency|Compute Time|PASSED|FAILED|\[E\]"
sudo tegrastats --stop
```

✅ 最後一行是 `&&&& PASSED`。

**筆電：把結果拿回來**

```powershell
mkdir -Force runs/orin_power
scp "ben-orin@192.168.50.10:~/tegra_test.log" runs/orin_power/
scp "ben-orin@192.168.50.10:~/trt_test.txt" runs/orin_power/
```

## 5. 更新 Orin 上的程式（Orin 沒有外網）

**筆電：打包、傳過去**

```powershell
git bundle create runs/to_orin/rotbot.bundle main
scp runs/to_orin/rotbot.bundle ben-orin@192.168.50.10:~/to_orin/
```

**Orin：從傳過去的檔案更新**

```bash
cd ~/Desktop/Rotbot-Project
git pull --no-edit ~/to_orin/rotbot.bundle main
```

**Orin：`requirements-jetson.txt` 有改時才需要**（離線安裝；安裝檔在筆電的 `runs/to_orin/jetson_wheels/`，要先整個資料夾傳到 Orin 的 `~/to_orin/`）

```bash
pip3 install --no-index --find-links ~/to_orin/jetson_wheels -r requirements-jetson.txt
```

## 6. 燒錄韌體（筆電）

用 Arduino IDE 打開 `firmware/rotbot_mcu/rotbot_mcu.ino`，按「→」上傳。設定見 [00_todo.md](00_todo.md) 第 1 步。✅ 程式大小約 35 KB（6%），沒有紅字。

## 7. 電腦上的模擬與測試（筆電，不需要硬體）

**全部的自動測試**

```powershell
python -m pytest -q
```

**韌體模擬器 + 全部通訊測試**

```powershell
python -m tools.mcu_link_test --sil
```

**整台機器人模擬**（網頁操控 + AI 自主）

```powershell
python -m tools.robot_main --sim
```
