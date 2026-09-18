# e-CAM25_CUONX 鏡頭需求規格

## 1. 使用情境

- 相機模組：e-CAM25_CUONX
- 感光元件：AR0234
- 解析度：1920 × 1200
- Pixel pitch：約 3 μm
- 快門：Global Shutter
- 運算平台：Jetson Orin Nano Super
- 用途：
  - 高速球體追蹤
  - 九宮格表面辨識
  - 判斷球是否進入指定區域
- 球直徑：約 7 cm
- 球速：約 60 ft/s（約 18.3 m/s）
- 球距離範圍：約 0.51–7 m
- 九宮格尺寸：約 5 m × 3 m
- 九宮格距離：約 7 m
- 環境條件：
  - 高瓦數照明
  - 可能有反光表面
  - 可能存在其他攝影機或 IR 光源

---

## 2. 鏡頭核心規格

| 項目 | 需求 |
|---|---|
| Mount | M12 × 0.5 / S-Mount |
| 焦距 | 約 6 mm，建議評估範圍 6–7 mm |
| Sensor 支援尺寸 | 至少支援 1/2.6\"，最好支援 1/2.5\" 或 1/2.3\" |
| Image Circle | 必須完整覆蓋 AR0234 感光區 |
| 解析力 | 至少 2.3 MP，建議 3 MP 以上，5 MP 更佳 |
| 光圈 | 建議 F/1.8–F/2.4 |
| Minimum Object Distance | ≤ 0.51 m，最好 ≤ 0.3 m |
| Focus | 可手動調整後固定，或原廠固定焦點但需覆蓋工作距離 |
| Distortion | Low distortion |
| IR-cut | 建議有，約 650 nm cutoff |
| 邊緣畫質 | 中央與邊緣皆需有良好解析度 |
| 鎖焦能力 | 最好可 lock ring 或點膠固定 |
| 工作波段 | 可見光為主，抑制近紅外 |
| 機械相容性 | 需確認 back focus、鏡筒長度與 M12 holder 相容 |

---

## 3. FoV 要求

因為需要在約 7 m 距離完整拍到 5 m × 3 m 九宮格：

- 水平 FoV 至少約 39°
- 垂直 FoV 至少約 24°

但考慮：

- 安裝誤差
- 機器人震動
- 鏡頭畸變
- 相機角度偏移
- 九宮格位置誤差

實際建議：

- 水平 FoV ≥ 45°
- 垂直 FoV ≥ 28–30°

因此 8 mm 雖然理論上接近可用，但安全餘裕太小；6–7 mm 較合理。

---

## 4. 焦距選擇結論

### 6 mm

優點：

- 可完整涵蓋 7 m 處 5 × 3 m 九宮格
- 安裝與震動容錯較大
- 0.51 m 處仍可完整看到 7 cm 球
- 最穩健

缺點：

- 7 m 處球直徑約只有 20 px

### 7 mm

優點：

- 遠距球約可增加到 23 px
- 九宮格仍可完整涵蓋
- 解析度與 FoV 之間折衷較好

缺點：

- 安全邊界較 6 mm 小
- 7 mm M12 鏡頭選擇可能較少

### 8 mm

- 理論上幾乎剛好涵蓋 5 m 寬九宮格
- 實際安全餘裕太小
- 不建議作為第一選擇

目前優先順序：

1. 6 mm
2. 7 mm
3. 不優先考慮 8 mm 以上

---

## 5. 高速球追蹤相關需求

球速約：

$$
18.3\ \mathrm{m/s}
$$

在 7 m、6 mm 鏡頭下，球直徑約 20 px。

為降低 motion blur，建議：

- 曝光時間：約 1/3000–1/5000 s
- 初始建議：約 1/4000 s
- motion blur 目標：≤ 1–2 px

曝光控制策略：

- 優先固定短曝光
- Gain 從最低值開始增加
- 優先使用 analog gain
- 不建議完全依賴 Auto Exposure
- 若可設定，建議限制最大曝光時間
- 可考慮 Fixed Exposure + Limited Auto Gain

---

## 6. 為什麼需要大光圈

因為曝光時間可能需要縮短至約：

$$
1/4000\ \mathrm{s}
$$

若光圈太小，例如 F/4：

- 進光不足
- 需要提高 gain
- Noise 增加

因此建議：

- F/1.8–F/2.4

但也不宜只追求超大光圈，仍需兼顧：

- 景深
- 邊緣解析度
- 像差

---

## 7. IR-cut 需求

建議鏡頭或相機端具備 IR-cut。

原因：

- AR0234 對近紅外仍有感度
- 場地可能有：
  - IR 補光
  - ToF / depth camera
  - 其他攝影機 IR 光源
  - 高 IR 成分燈源
- IR 可能造成：
  - RGB 色彩偏移
  - saturation 降低
  - Auto Exposure 判斷錯誤
  - Auto White Balance 漂移
  - 反射面額外亮斑

建議：

- 約 650 nm cutoff
- 以可見光成像為主

注意：
IR-cut 無法解決可見光鏡面反射，若九宮格表面反光嚴重，仍需考慮：

- 偏光鏡
- 調整照明方向
- 使用霧面材質
- 固定曝光

---

## 8. 對焦需求

此相機為 Fixed Focus 類型，因此：

- 不依靠軟體自動對焦
- 焦點主要由鏡頭與 sensor 的距離決定
- 若更換 6 mm / 7 mm M12 鏡頭，需重新確認最佳焦點

需求：

- 0.51–7 m 內皆需維持足夠清晰
- 最好鏡頭可手動旋轉對焦後鎖定
- 避免比賽時因震動導致跑焦

---

## 9. 採購時可直接使用的搜尋條件

建議搜尋：

> 6 mm M12 × 0.5 S-Mount low distortion machine vision lens, support 1/2.6 inch or larger sensor, 3 MP or higher, F/1.8–F/2.4, MOD ≤ 0.5 m, IR-cut compatible

理想規格可再縮成：

> 6 mm M12, 1/2.5 inch or 1/2.3 inch, 5 MP, low distortion, F/1.8–F/2.0, manual focus, MOD ≤ 0.3 m, IR-cut

---

## 10. 最終優先檢查項目

採購前至少確認：

1. 是否為 M12 × 0.5 / S-Mount
2. 是否支援 1/2.6\" 或更大 sensor
3. 焦距是否約 6–7 mm
4. 7 m 處 FoV 是否足夠拍完整 5 × 3 m 九宮格
5. MOD 是否 ≤ 0.51 m
6. 光圈是否約 F/1.8–F/2.4
7. 是否 ≥ 3 MP
8. 是否 low distortion
9. 是否有 IR-cut
10. 是否能對焦後鎖定
11. 是否確認與 e-CAM25_CUONX 的 back focus / lens holder 相容
12. 是否有完整 datasheet 或 MTF / FoV 資料可確認
