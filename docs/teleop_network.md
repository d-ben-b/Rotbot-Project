# Orin 手動操控（第 61 秒起）：路由器區網 + 網頁 + 鍵盤 / 搖桿

> 2026-10-25。回答「怎麼用路由器做到網頁操控、怎麼證明可行」。程式：`rotbot/teleop/`、`tools/teleop_web.py`。

## 1. 結論

- **架構可行，已在電腦上完整驗證**：網頁 → WebSocket → 伺服器 → `SerialMCU` → 韌體（模擬器，同一份 Arduino 原始碼）。
  經本機區網 IP（`192.168.1.102`，走家用路由器那張網卡）實測：瀏覽器↔伺服器往返 **0.41 ms**、瀏覽器指令→MCU ACK **0.61 ms**、
  前 60 秒手動輸入一律被拒、切手動後轉塔跟著轉、READY 後發射、急停飛輪歸 0；相機畫面（數位孿生）隨轉塔即時改變。
- **「localhost」的意思**：伺服器跑在 Orin 上、只在自備路由器的區網內，**不經網際網路**（規則第 10 條禁止場館外遙控、第 14 條場館不提供 Wi‑Fi）。
- **搖桿要「安全連線」**：瀏覽器只在 `https://` 或 `http://localhost` 開放 Gamepad API（Firefox 81+、Chrome 86+）；
  用 `http://<Orin IP>` 開時搖桿看起來就像沒插（鍵盤不受影響）。解法：**筆電用 SSH 通道把 Orin 的 8080 轉到自己的 localhost**（§4）。
- **最大風險是無線干擾，不是軟體**：規則第 13 條：場館攝影機使用 **5.1–5.9 GHz、6.5 W** → 5 GHz Wi‑Fi 可能被壓；2.4 GHz 又被各隊擠滿。
  練習日一定要實測兩個頻段（§6），並保留備案。**通訊硬體在成果報告書（11/9）繳交後不得變更**，要在那之前定案。

## 2. 架構

```text
操作手筆電（≤30 吋螢幕、一個搖桿）          自備路由器（不接外網）              機器人
  瀏覽器 http://localhost:8080  ──SSH通道──▶  Wi‑Fi  ──────────────────▶  Orin：python -m tools.teleop_web
  （或 http://192.168.50.10:8080，只能鍵盤）                                    ├ GET /            控制頁
                                                                               ├ GET /video.mjpg  相機串流 + HUD
                                                                               ├ WS  /ws          20 Hz 輸入 / 10 Hz 狀態
                                                                               └ SerialMCU ─USB─▶ Arduino ─▶ 馬達
```

規則對應：只能兩台電腦（操作手 1、機器人 1）、只能一個遙控器一人操作（伺服器同時間只認一個「駕駛」連線，其他只能看）、
前 60 秒不得遙控（`MatchStateMachine` 鎖定，伺服器端拒絕，不靠前端自律）。

## 3. 路由器與 Orin 設定

1. 路由器：WPA2/WPA3、自訂 SSID、**WAN 不接**；LAN 例如 `192.168.50.0/24`。
2. 給 Orin 固定 IP（路由器 DHCP 保留，或在 Orin 上）：
   `sudo nmcli con mod "<連線名>" ipv4.method manual ipv4.addresses 192.168.50.10/24 ipv4.gateway 192.168.50.1`
3. Orin 防火牆：`sudo ufw allow 8080/tcp`（有開 ufw 時）；SSH 已開（`sudo systemctl enable --now ssh`）。
4. 啟動（之後可做成 systemd 開機自動跑）：
   `python -m tools.teleop_web --mcu /dev/ttyACM0 --camera 0`（CSI 相機給 GStreamer 管線字串，見工具說明）。
5. 備案：不帶路由器時可讓 Orin 自己當熱點：`nmcli dev wifi hotspot ifname wlan0 ssid rotbot password <密碼>`。

## 4. 操作手筆電

| 輸入 | 怎麼開 | 說明 |
|---|---|---|
| 鍵盤 | `http://192.168.50.10:8080` | 任何瀏覽器 |
| 搖桿（建議） | `ssh -N -L 8080:127.0.0.1:8080 <user>@192.168.50.10`，再開 `http://localhost:8080` | localhost 算安全連線 → Gamepad API 可用；流量照樣走路由器 |
| 搖桿（替代） | `--tls-cert/--tls-key` 自簽憑證，開 `https://192.168.50.10:8080`，第一次接受警告 | `openssl req -x509 -newkey rsa:2048 -nodes -keyout key.pem -out cert.pem -days 365 -subj "/CN=rotbot"` |

頁面在非安全連線時會顯示黃色提示。按鍵：W/S 俯仰、A/D 水平（Shift 微調）、R 飛輪開關、Q/E 轉速 ±100、Space 發射、X 停止、Esc 軟體急停；
搖桿：左搖桿瞄準、RT 飛輪、A 發射、B 停止、LB 微調。⚠️ 比賽只能用「一個遙控器」→ 正式比賽用搖桿，鍵盤只當測試。

## 5. 安全設計

| 機制 | 行為 | 驗證 |
|---|---|---|
| 自主鎖定 | 開始後 60 秒內所有手動輸入被伺服器拒絕；60 秒後由操作手按「切手動」 | `tests/test_teleop.py`、實測 |
| 單一駕駛 | 第一個連線是駕駛；其他連線只能看（急停除外）；「取得控制權」可接手 | 測試 |
| Deadman | 駕駛 300 ms 沒送輸入 → JOG 歸零（瀏覽器每 50 ms 送一次；切到背景分頁送 0） | 測試（韌體模擬器上轉塔停住） |
| 斷線 | 駕駛斷線 → STOP（JOG 停 + 飛輪停） | 測試 |
| MCU 看門狗 | Orin 當機 / USB 斷 1 s → 飛輪停、JOG 停 | `mcu_link_test` |
| 急停 | 網頁「軟體急停」= STOP + 狀態 ESTOP；**實體紅色按鈕才斷電**（規則第 6 條） | 測試 |

## 6. 怎麼證明（照做即可）

**A. 今天在家（不需要機器人）**
1. 電腦：`python -m tools.teleop_web --mcu sil --camera sim --auto-s 5`（韌體模擬器 + 數位孿生相機；自主鎖定縮成 5 秒方便試）。
2. 手機或另一台筆電連同一台家用路由器，開工具印出的區網網址（例如 `http://192.168.1.102:8080`）。
   Windows 防火牆第一次會問是否允許 Python 接受連線 → 允許「私人網路」。
3. 按「開始」→ 5 秒內按 W/A/S/D 會被拒絕 → 按「切手動」→ 轉塔與畫面一起動 → 按 R 起轉、等 READY 燈 → Space 發射。
4. 頁首會顯示瀏覽器↔Orin、Orin↔MCU 與指令延遲。

**B. Orin + 路由器（板子到之前也能做）**：Orin 上 `python -m tools.teleop_web --mcu sil --camera sim`（Orin 內建 g++，可建置韌體模擬器），
筆電經路由器連線，量 Wi‑Fi 延遲；再接上真 Arduino 換成 `--mcu /dev/ttyACM0`。

**C. 練習日（場館）**：
- 2.4 GHz 與 5 GHz 各測一次：在人員作業區開頁面，看延遲數字與畫面是否卡頓；可用手機 Wi‑Fi 分析 App 找最空的頻道並在路由器固定頻道。
- 路由器盡量靠近人員作業區與機器人（場地 4 × 5 m，距離不是問題，干擾才是）。
- 若 Wi‑Fi 不穩：備案是 MCU 直接接商用 RC 接收機（規則提到「使用商用飛航遙控器干擾較少」），Wi‑Fi 只看畫面。
  要用這個備案，**必須在 11/9 成果報告書前列入通訊硬體**。
