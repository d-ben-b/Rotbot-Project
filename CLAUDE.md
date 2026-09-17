# CLAUDE.md — 專案工作準則

AI 自主棒球投手機器人（2026 TEL Robot Combat，隊伍：嘻諧們）。使用者負責軟體（感知・決策・模擬）。溝通用繁體中文。

## 最重要：每次變更都要更新 PROGRESS.md（不要再忘記）

**任何對程式碼、設定、模型或實驗做出實質變更後，必須在同一輪內更新 [PROGRESS.md](PROGRESS.md)：**
- 在 **§9 變更紀錄（Changelog）以 append-only 方式**新增條目：日期、做了什麼、為什麼、關鍵驗證數字（測試/評估）。**不改舊條目**，保留歷史。
- 若影響「最新快照」（§0–§8，如成果數字、待確認風險），一併更新該區。
- 提交前若有跑測試/實驗，把數字寫進條目。

這條規則優先於「先做完再說」——寧可當下補一行 changelog，也不要事後補漏。

## 其他慣例
- **設定集中**在 `configs/field.yaml`（型別化於 `rotbot/config.py`）；硬體到位只改 YAML，不動程式碼。
- 變更後**跑 `pytest`**（目前 37 tests），把結果寫進 changelog。
- **權重 `.pt`、`datasets/`、`runs/`、`rotbot/sim/assets/`** 皆 gitignore，不入庫。
- 分層：`perception → decision → actuation`，模擬替身在 `sim/`，硬體介面在 `hal.py`。
- 執行工具用 `python -m tools.xxx` 或直接 `python tools/xxx.py`（皆支援）。
- 數字誠信：模擬/合成得到的指標要標明「模擬/合成評估」，勿講成實機成績。
- 未 commit 的新檔在工作階段間可能遺失 → 要保存的產出（報告等）記得 commit。
