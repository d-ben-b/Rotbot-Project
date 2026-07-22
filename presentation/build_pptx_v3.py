# -*- coding: utf-8 -*-
"""嘻諧們 簡報日 pptx v3 —— 四大難題結構，每題「軟體 + 硬體」雙解方。
口號暫不放。逐字稿在備註欄。重跑：python build_pptx_v3.py"""
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

BG    = RGBColor(0x0B, 0x12, 0x20)
PANEL = RGBColor(0x15, 0x20, 0x38)
CYAN  = RGBColor(0x22, 0xD3, 0xEE)   # 軟體
GREEN = RGBColor(0x34, 0xD3, 0x99)   # 硬體
BLUE  = RGBColor(0x38, 0xBD, 0xF8)
WHITE = RGBColor(0xF1, 0xF5, 0xF9)
GREY  = RGBColor(0x94, 0xA3, 0xB8)
AMBER = RGBColor(0xFB, 0xBF, 0x24)
RED   = RGBColor(0xF8, 0x71, 0x71)
FONT  = "Microsoft JhengHei"

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
SW, SH = prs.slide_width, prs.slide_height
BLANK = prs.slide_layouts[6]
TOTAL = 9


def slide():
    s = prs.slides.add_slide(BLANK)
    r = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, SW, SH)
    r.fill.solid(); r.fill.fore_color.rgb = BG; r.line.fill.background()
    r.shadow.inherit = False
    s.shapes._spTree.remove(r._element); s.shapes._spTree.insert(2, r._element)
    return s


def box(s, l, t, w, h, fill=None, line=None, lw=1.0, round_=True):
    shp = s.shapes.add_shape(
        MSO_SHAPE.ROUNDED_RECTANGLE if round_ else MSO_SHAPE.RECTANGLE, l, t, w, h)
    if fill is None: shp.fill.background()
    else: shp.fill.solid(); shp.fill.fore_color.rgb = fill
    if line is None: shp.line.fill.background()
    else: shp.line.color.rgb = line; shp.line.width = Pt(lw)
    shp.shadow.inherit = False
    return shp


def txt(s, l, t, w, h, runs, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, sa=5, ls=1.05):
    tb = s.shapes.add_textbox(l, t, w, h)
    tf = tb.text_frame; tf.word_wrap = True; tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Inches(0.05)
    tf.margin_top = tf.margin_bottom = Inches(0.02)
    if isinstance(runs[0], tuple): runs = [runs]
    for i, para in enumerate(runs):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align; p.space_after = Pt(sa); p.line_spacing = ls
        for (text, size, color, bold) in para:
            r = p.add_run(); r.text = text
            r.font.name = FONT; r.font.size = Pt(size)
            r.font.color.rgb = color; r.font.bold = bold
    return tb


def kicker(s, text, badge=None):
    box(s, Inches(0.6), Inches(0.5), Inches(0.16), Inches(0.62), fill=CYAN, round_=False)
    txt(s, Inches(0.9), Inches(0.44), Inches(9.3), Inches(0.75),
        [[(text, 30, WHITE, True)]], anchor=MSO_ANCHOR.MIDDLE)
    if badge:
        box(s, Inches(10.6), Inches(0.55), Inches(2.1), Inches(0.55), fill=AMBER)
        txt(s, Inches(10.6), Inches(0.55), Inches(2.1), Inches(0.55),
            [[(badge, 17, BG, True)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)


def footer(s, n):
    txt(s, Inches(0.6), Inches(7.02), Inches(6), Inches(0.4),
        [[("嘻諧們 ｜ 2026 TEL Robot Combat", 11, GREY, False)]])
    txt(s, Inches(11.2), Inches(7.02), Inches(1.5), Inches(0.4),
        [[(f"{n}/{TOTAL}", 11, GREY, False)]], align=PP_ALIGN.RIGHT)


def notes(s, t): s.notes_slide.notes_text_frame.text = t


def img_ph(s, l, t, w, h, label):
    box(s, l, t, w, h, fill=PANEL, line=BLUE, lw=1.25)
    txt(s, l, t, w, h, [[("🖼 圖片區", 13, BLUE, True)], [(label, 11, GREY, False)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=4)


def challenge(n, title, sub, why, soft, hard, note, page):
    """難題頁：上=難題，下=軟體 / 硬體 雙解方"""
    s = slide()
    kicker(s, f"難題 {n}　｜　{title}", badge=None)
    # 難題描述帶
    box(s, Inches(0.85), Inches(1.3), Inches(11.6), Inches(1.55),
        fill=RGBColor(0x2A, 0x14, 0x18), line=RED, lw=1.5)
    body = [[("⚠ " + sub, 19, RED, True)]]
    body += [[("· " + w, 15, WHITE, False)] for w in why]
    txt(s, Inches(1.1), Inches(1.42), Inches(11.1), Inches(1.35), body, sa=3)
    # 軟體
    box(s, Inches(0.85), Inches(3.1), Inches(5.65), Inches(3.55), fill=PANEL, line=CYAN, lw=2)
    txt(s, Inches(1.05), Inches(3.22), Inches(5.25), Inches(0.5),
        [[("💻 軟體解方", 20, CYAN, True)]])
    sbody = []
    for head, detail in soft:
        sbody.append([(head, 16, WHITE, True)])
        if detail: sbody.append([("   " + detail, 13, GREY, False)])
    txt(s, Inches(1.05), Inches(3.8), Inches(5.25), Inches(2.75), sbody, sa=4)
    # 硬體
    box(s, Inches(6.8), Inches(3.1), Inches(5.65), Inches(3.55), fill=PANEL, line=GREEN, lw=2)
    txt(s, Inches(7.0), Inches(3.22), Inches(5.25), Inches(0.5),
        [[("🔧 硬體解方", 20, GREEN, True)]])
    hbody = []
    for head, detail in hard:
        hbody.append([(head, 16, WHITE, True)])
        if detail: hbody.append([("   " + detail, 13, GREY, False)])
    txt(s, Inches(7.0), Inches(3.8), Inches(5.25), Inches(2.75), hbody, sa=4)
    footer(s, page)
    notes(s, note)
    return s


# ============================================ 1 封面
s = slide()
box(s, 0, Inches(3.0), SW, Inches(0.06), fill=CYAN, round_=False)
txt(s, Inches(0.9), Inches(1.15), Inches(11.5), Inches(0.6),
    [[("2026 TEL Robot Combat ｜ 東京威力科創機器人大賽", 18, CYAN, True)]])
txt(s, Inches(0.9), Inches(1.75), Inches(11.5), Inches(1.4),
    [[("嘻諧們", 62, WHITE, True)], [("AI 自主棒球投手機器人", 30, BLUE, True)]], sa=2)
txt(s, Inches(0.9), Inches(3.4), Inches(11.5), Inches(1.0),
    [[("四大難題　×　軟硬體整合解方", 30, CYAN, True)]])
txt(s, Inches(0.9), Inches(5.75), Inches(11.5), Inches(1.0),
    [[("NCKU 機械系・電機所　｜　林敬堯 · 阮紹銘 · 何詺威　｜　指導 張怡玲 教授", 16, GREY, False)]])
img_ph(s, Inches(9.4), Inches(5.2), Inches(3.3), Inches(1.5), "機器人整機圖")
notes(s, "（10s）大家好，我們是嘻諧們。這次我們做的是一台 AI 自主棒球投手機器人。"
          "接下來六分鐘，我們用「四大難題」帶大家看——每一個難題，我們都同時從軟體和硬體兩邊解決。")

# ============================================ 2 四大難題總覽
s = slide(); kicker(s, "這場比賽，難在哪？")
items = [("①", "60 秒自主控制", "不能碰遙控，全靠自己", RED),
         ("②", "距離方位未知", "還要精準入框", AMBER),
         ("③", "3 分鐘投 27 顆", "平均不到 7 秒一顆", BLUE),
         ("④", "眩光干擾", "過曝、特徵遺失", GREEN)]
x0, w, g, y, h = Inches(0.85), Inches(2.85), Inches(0.15), Inches(2.3), Inches(2.6)
for i, (num, big, small, c) in enumerate(items):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=2)
    txt(s, l, Inches(2.5), w, Inches(0.6), [[(num, 30, c, True)]], align=PP_ALIGN.CENTER)
    txt(s, l, Inches(3.2), w, Inches(1.0), [[(big, 21, WHITE, True)]], align=PP_ALIGN.CENTER)
    txt(s, l, Inches(4.15), w, Inches(0.6), [[(small, 14, GREY, False)]], align=PP_ALIGN.CENTER)
box(s, Inches(0.85), Inches(5.35), Inches(11.6), Inches(1.0),
    fill=RGBColor(0x10, 0x1A, 0x30), line=CYAN, lw=1.5)
txt(s, Inches(1.1), Inches(5.35), Inches(11.1), Inches(1.0),
    [[("每一個難題，我們都用 ", 18, WHITE, True), ("軟體 + 硬體", 22, CYAN, True),
      (" 兩邊一起解", 18, WHITE, True)]], anchor=MSO_ANCHOR.MIDDLE)
footer(s, 2)
notes(s, "（30s）這場比賽真正難的有四件事：第一，前 60 秒不能碰遙控，機器人得自己判斷；"
          "第二，目標的距離和方位事先都不知道，而且還得精準投進網框；"
          "第三，3 分鐘要投完 27 顆，平均不到 7 秒一顆；第四，現場眩光會讓影像過曝、特徵遺失。"
          "接下來每一題，我們都會講軟體怎麼解、硬體怎麼解。")

# ============================================ 3 難題① 60秒自主
challenge(
    "①", "60 秒自主控制", "前 60 秒禁止遙控，機器人必須自己完成感知 → 決策 → 發射",
    ["無人介入，全程閉迴路自動運行", "不能依賴無線通訊（現場干擾嚴重）"],
    [("YOLO26 即時辨識", "九宮格網框、亮燈得分格、對手機器人"),
     ("原生 NMS-free 架構", "後處理精簡，推論延遲極低"),
     ("感知→決策→致動 閉迴路", "自動觸發發射，全程不需人介入")],
    [("Jetson Orin Nano Super", "8GB / 67 TOPS 機載邊緣運算，TensorRT 加速"),
     ("USB Webcam 機上視覺", "與發射器同軸，視角穩定"),
     ("Orin —UART/序列→ Arduino", "自訂封包協定；前 60 秒不靠無線 = 天生抗干擾")],
    "（50s）第一題：前 60 秒不能碰遙控。軟體上，我們用 YOLO26 即時辨識九宮格、亮起的得分格和對手，"
    "它原生 NMS-free、後處理少所以延遲極低，形成感知、決策、致動的自動閉迴路。"
    "硬體上，運算跑在機上的 Jetson Orin Nano Super，8GB、67 TOPS，用 TensorRT 加速；"
    "算完的指令用 UART 序列傳給 Arduino 驅動馬達。最關鍵的是——前 60 秒我們完全不靠無線通訊，"
    "純機上視覺，所以天生抗干擾。", 3)

# ============================================ 4 難題② 距離方位
challenge(
    "②", "距離方位未知・精準入框", "目標距離與角度事先未知，且必須把球精準送進網框",
    ["機器人可在作業區移動，距離隨時在變", "網框被打到會位移、變形，座標不能寫死"],
    [("單目幾何測距", "網框 40cm / 加分格 20cm 尺寸已知 → 反推距離與方位角"),
     ("解析拋體模型", "由距離與高度求解所需俯仰角，物理可解釋"),
     ("每發重新定位", "不依賴固定座標，網框移位也能追蹤")],
    [("水平調整", "迴轉支撐滾珠軸承系統承載機台，保持水平旋轉自由度"),
     ("鉛直調整", "Y 字型支架側面齒輪，配合機體側面 90° 扇形齒輪"),
     ("步進馬達驅動", "俯仰 + 旋轉雙軸，定位精度高、可重複")],
    "（50s）第二題：距離和方位事先不知道，而且還要精準入框。軟體上，因為網框尺寸是已知的——"
    "一般格 40 公分、加分格 20 公分——我們用單目幾何從影像中框的大小直接反推距離跟方位角，"
    "不需要額外的測距感測器；再用拋體方程解出需要的俯仰角。而且我們每一發都重新定位，"
    "所以就算網框被打歪也追得到。硬體上，水平方向用迴轉支撐滾珠軸承承載機台保持旋轉自由度，"
    "鉛直方向用 Y 字支架側面的齒輪配合 90 度扇形齒輪，都由步進馬達驅動，定位精準又可重複。", 4)

# ============================================ 5 難題③ 3分鐘27顆
challenge(
    "③", "3 分鐘投 27 顆", "180 秒 ÷ 27 顆 ≈ 每 6.7 秒就要完成一次完整投擲",
    ["7 秒內要完成 辨識 → 測距 → 瞄準 → 發射", "補彈需用救援卡，全場只有 3 張"],
    [("單階段偵測", "一次前向運算同時輸出所有目標，不跑多條流程"),
     ("低延遲推論", "NMS-free + TensorRT，把感知時間壓到最短"),
     ("自動連續發射", "閉迴路自動觸發，不等人下指令")],
    [("彈倉轉輪供彈", "每倉 4 顆；齒輪 + 齒條推進裝置推球滾入發射軌道"),
     ("自動換倉", "彈倉清空後，前方齒輪旋轉至下一彈倉"),
     ("雙高速飛輪投射", "1700 rpm（理論 1592 rpm，補償球體擠壓摩擦）→ 球速約 10 m/s")],
    "（50s）第三題：3 分鐘要投完 27 顆，平均不到 7 秒一顆，這 7 秒裡要完成辨識、測距、瞄準、發射，"
    "而且補彈得用救援卡、全場只有 3 張。軟體上，YOLO 是單階段偵測，一次前向運算就同時輸出所有目標，"
    "不用跑多條流程，再加上 NMS-free 和 TensorRT 把延遲壓到最短，全程自動連續發射。"
    "硬體上，供彈用彈倉轉輪，每倉 4 顆，用齒輪加齒條的推進裝置把球推進發射軌道；"
    "彈倉清空後由前方齒輪自動轉到下一倉。發射靠雙高速飛輪，轉速設在 1700 rpm——"
    "理論值是 1592，我們考量球體擠壓摩擦所以設略高——換算球速大約每秒 10 公尺。", 5)

# ============================================ 6 難題④ 眩光
challenge(
    "④", "眩光干擾", "賽場高亮度聚光燈與反射光，會讓影像過曝、特徵遺失",
    ["鏡頭直射光源 → 局部飽和，網框特徵消失", "辨識與測距精度直接崩壞，傳統固定閾值必失效"],
    [("Unity 數位孿生 + 域隨機化", "隨機變化光照、視角、背景與亮燈狀態產生合成資料"),
     ("生成式影像增強", "重繪光影、加入眩光，縮小合成與真實的外觀差距"),
     ("YOLO 學整體特徵", "非單一顏色或閾值，對光照變化具韌性")],
    [("相機同軸固定安裝", "視角穩定，避免自動對焦/晃動造成的額外變異"),
     ("鎖定曝光 / 白平衡", "避免自動曝光在強光下劇烈震盪"),
     ("⚠ 遮光罩設計 — 待你們確認", "如已有實體遮光或濾鏡方案，請補上規格")],
    "（45s）第四題：現場眩光。強光直射鏡頭會造成局部過曝、網框特徵直接消失，"
    "傳統用固定顏色閾值的方法在這種環境一定失效。軟體上，我們用 Unity 數位孿生做域隨機化，"
    "隨機變化光照、視角、背景和亮燈狀態產生合成資料，再用生成式影像增強重繪光影、加入眩光，"
    "縮小合成影像跟真實賽場的差距；而 YOLO 學的是整體特徵而不是單一顏色，本身對光照就有韌性。"
    "硬體上，相機與發射器同軸固定確保視角穩定，並鎖定曝光與白平衡避免自動曝光震盪。", 6)

# ============================================ 7 貫穿全場：線上自適應
s = slide(); kicker(s, "貫穿四題的核心　｜　線上自適應校正")
txt(s, Inches(0.85), Inches(1.3), Inches(11.6), Inches(0.6),
    [[("比賽中每投一顆，就依落點誤差自動微調 —— 命中率越投越準", 22, CYAN, True)]])
flow = ["拋體公式\n算理論值", "相機回授\n實際落點", "計算誤差", "小幅修正\n下一發"]
x0, w, g, y, h = Inches(0.85), Inches(2.6), Inches(0.4), Inches(2.15), Inches(1.4)
for i, st in enumerate(flow):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=GREEN, lw=2)
    txt(s, l, y, w, h, [[(st, 16, WHITE, True)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    if i < 3:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW,
            Emu(int(l) + int(w) + int(Inches(0.04))), Inches(2.65), Inches(0.32), Inches(0.4))
        ar.fill.solid(); ar.fill.fore_color.rgb = GREEN
        ar.line.fill.background(); ar.shadow.inherit = False
box(s, Inches(0.85), Inches(3.9), Inches(5.65), Inches(2.5), fill=PANEL, line=CYAN, lw=1.5)
txt(s, Inches(1.05), Inches(4.05), Inches(5.25), Inches(2.2),
    [[("這不是訓練模型", 18, CYAN, True)],
     [("沒有梯度、沒有權重更新", 14, WHITE, False)],
     [("只是瞄準參數的誤差回授修正", 14, WHITE, False)],
     [("→ 即時、可解釋、現場可除錯", 15, GREEN, True)]], sa=6)
box(s, Inches(6.8), Inches(3.9), Inches(5.65), Inches(2.5), fill=PANEL, line=AMBER, lw=1.5)
txt(s, Inches(7.0), Inches(4.05), Inches(5.25), Inches(2.2),
    [[("自動補償現場差異", 18, AMBER, True)],
     [("場地、球體、電量造成的偏移", 14, WHITE, False)],
     [("網框被打歪後的位置變化", 14, WHITE, False)],
     [("→ 賽前建表，賽中只做小幅修正", 15, GREEN, True)]], sa=6)
footer(s, 7)
notes(s, "（40s）這四題背後有一個貫穿的核心：線上自適應校正。我們先用拋體公式算理論值，"
          "發射後由相機回授實際落點，算出誤差，再對下一發做小幅修正，命中率越投越準。"
          "要特別說明——這不是在訓練模型，沒有梯度也沒有權重更新，只是瞄準參數的誤差回授，"
          "所以它即時、可解釋、現場可以除錯。它會自動補償當天場地、球體、電量的差異，"
          "還有網框被打歪後的位置變化。賽前練習我們就把基礎對照表建好，比賽中只做小幅修正。")

# ============================================ 8 可行性 + 預算
s = slide(); kicker(s, "執行可行性 + 預算")
stg = [("設計", GREEN, "✔ 完成"), ("加工", GREEN, "✔ 完成"),
       ("模型訓練", GREEN, "✔ 完成"), ("整合 / 校正", AMBER, "▶ 進行中")]
x0, w, g, y, h = Inches(0.85), Inches(2.4), Inches(0.35), Inches(1.5), Inches(1.15)
for i, (name, c, mark) in enumerate(stg):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=2)
    txt(s, l, y, w, h, [[(name, 17, WHITE, True)], [(mark, 13, c, True)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=3)
    if i < 3:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW,
            Emu(int(l) + int(w) + int(Inches(0.02))), Inches(1.88), Inches(0.3), Inches(0.38))
        ar.fill.solid(); ar.fill.fore_color.rgb = WHITE
        ar.line.fill.background(); ar.shadow.inherit = False
txt(s, Inches(0.85), Inches(2.95), Inches(11.6), Inches(0.6),
    [[("校正走輕量方法 → ", 17, WHITE, False), ("現場可除錯、結果可解釋，不賭黑箱", 18, GREEN, True)]])
txt(s, Inches(0.85), Inches(3.8), Inches(11.6), Inches(0.7),
    [[("總預算　", 22, WHITE, True), ("40,000 元", 38, CYAN, True)]])
cards = [("AI 運算 · 感測", "~1.9 萬", "Orin / Webcam / Arduino", CYAN),
         ("結構 · 電源 · 耗材", "~1.5 萬", "鋁擠 / 電池 / 球", BLUE),
         ("發射 · 驅動", "~0.6 萬", "步進+直流馬達 / 驅動器", GREEN)]
x0, w, g, y, h = Inches(0.85), Inches(3.75), Inches(0.2), Inches(4.65), Inches(1.65)
for i, (name, amt, desc, c) in enumerate(cards):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=1.5)
    txt(s, l, y, w, h, [[(name, 15, c, True)], [(amt, 26, WHITE, True)], [(desc, 11, GREY, False)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=3)
box(s, Inches(0.85), Inches(6.45), Inches(11.6), Inches(0.45),
    fill=RGBColor(0x2A, 0x1F, 0x10), line=AMBER, lw=1)
txt(s, Inches(1.0), Inches(6.45), Inches(11.4), Inches(0.45),
    [[("⚠ 預算數字請你們再核對一次（原檔標示不確定）", 13, AMBER, True)]], anchor=MSO_ANCHOR.MIDDLE)
footer(s, 8)
notes(s, "（30s）可行性上，設計、加工、模型訓練都已完成，目前在整合與校正階段。"
          "校正刻意走輕量方法，現場可除錯、結果可解釋，不賭黑箱。"
          "預算四萬元，最大兩塊是 AI 運算與結構電源，錢花在真正決定成績的地方。")

# ============================================ 9 結尾
s = slide()
box(s, 0, Inches(3.05), SW, Inches(0.06), fill=CYAN, round_=False)
txt(s, Inches(0.9), Inches(1.5), Inches(11.5), Inches(1.4),
    [[("四大難題，軟硬體一起解", 40, CYAN, True)]])
pairs = "自主控制 · 精準入框 · 投擲節奏 · 抗眩光"
txt(s, Inches(0.9), Inches(3.35), Inches(11.5), Inches(0.9),
    [[(pairs, 22, WHITE, True)]])
txt(s, Inches(0.9), Inches(4.9), Inches(11.5), Inches(1.4),
    [[("Thank you ｜ Q & A", 40, WHITE, True)], [("嘻諧們 ｜ NCKU", 18, GREY, False)]], sa=6)
box(s, Inches(0.9), Inches(6.35), Inches(4.6), Inches(0.5),
    fill=RGBColor(0x1A, 0x1A, 0x2E), line=GREY, lw=1)
txt(s, Inches(0.9), Inches(6.35), Inches(4.6), Inches(0.5),
    [[("（口號位置保留，之後再定）", 13, GREY, False)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
img_ph(s, Inches(9.4), Inches(4.9), Inches(3.3), Inches(1.5), "機器人主視覺")
notes(s, "（15s）總結一下：四大難題，我們每一題都從軟體和硬體兩邊一起解決——"
          "自主控制、精準入框、投擲節奏、抗眩光。以上是嘻諧們的簡報，謝謝，歡迎提問！")

import os
base = os.path.dirname(__file__)
out = os.path.join(base, "嘻諧們_簡報日_v3.pptx")
try:
    prs.save(out)
except PermissionError:
    out = os.path.join(base, "嘻諧們_簡報日_v3_new.pptx")
    prs.save(out)
    print("!! 原檔被 PowerPoint 鎖住，已改存 _new.pptx")
print("saved:", out.encode("utf-8", "replace").decode("utf-8", "replace"),
      "slides:", len(prs.slides._sldIdLst))
