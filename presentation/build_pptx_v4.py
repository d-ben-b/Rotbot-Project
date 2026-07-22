# -*- coding: utf-8 -*-
"""嘻諧們 簡報日 pptx v4 —— 大圖 + 關鍵字（字少圖多），四大難題 × 軟硬體雙解方。
圖片來源：assets/（自 after_discussion.pptx 抽出的真實 CAD 與實拍）
重跑：python build_pptx_v4.py"""
import os
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from PIL import Image

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
A     = os.path.join(os.path.dirname(__file__), "assets")

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
SW, SH = prs.slide_width, prs.slide_height
BLANK = prs.slide_layouts[6]
TOTAL = 10


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


def txt(s, l, t, w, h, runs, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, sa=6, ls=1.05):
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


def pic(s, fname, bl, bt, bw, bh, caption=None, frame=True):
    """等比例塞進 (bl,bt,bw,bh) 並置中；可加外框與圖說"""
    path = os.path.join(A, fname)
    iw, ih = Image.open(path).size
    ar, box_ar = iw / ih, bw / bh
    if ar > box_ar: w, h = bw, int(bw / ar)
    else:           h, w = bh, int(bh * ar)
    l, t = bl + (bw - w) // 2, bt + (bh - h) // 2
    if frame:
        box(s, l - Inches(0.06), t - Inches(0.06), w + Inches(0.12), h + Inches(0.12),
            fill=None, line=BLUE, lw=1.25)
    s.shapes.add_picture(path, l, t, width=w, height=h)
    if caption:
        txt(s, bl, bt + bh + Inches(0.04), bw, Inches(0.32),
            [[(caption, 12, GREY, False)]], align=PP_ALIGN.CENTER)


def kicker(s, text):
    box(s, Inches(0.6), Inches(0.42), Inches(0.16), Inches(0.6), fill=CYAN, round_=False)
    txt(s, Inches(0.9), Inches(0.36), Inches(11.5), Inches(0.72),
        [[(text, 30, WHITE, True)]], anchor=MSO_ANCHOR.MIDDLE)


def hurt(s, line):
    """難題一句話（紅帶）"""
    box(s, Inches(0.85), Inches(1.2), Inches(11.6), Inches(0.72),
        fill=RGBColor(0x2A, 0x14, 0x18), line=RED, lw=1.5)
    txt(s, Inches(1.1), Inches(1.2), Inches(11.1), Inches(0.72),
        [[("⚠ " + line, 19, RED, True)]], anchor=MSO_ANCHOR.MIDDLE)


def solbox(s, l, t, w, h, kind, lines):
    c = CYAN if kind == "軟體" else GREEN
    icon = "💻" if kind == "軟體" else "🔧"
    box(s, l, t, w, h, fill=PANEL, line=c, lw=2)
    txt(s, l + Inches(0.18), t + Inches(0.1), w - Inches(0.36), Inches(0.45),
        [[(f"{icon} {kind}", 19, c, True)]])
    body = [[("· " + x, 16, WHITE, True)] for x in lines]
    txt(s, l + Inches(0.18), t + Inches(0.62), w - Inches(0.36), h - Inches(0.75), body, sa=7)


def footer(s, n):
    txt(s, Inches(0.6), Inches(7.02), Inches(6), Inches(0.4),
        [[("嘻諧們 ｜ 2026 TEL Robot Combat", 11, GREY, False)]])
    txt(s, Inches(11.2), Inches(7.02), Inches(1.5), Inches(0.4),
        [[(f"{n}/{TOTAL}", 11, GREY, False)]], align=PP_ALIGN.RIGHT)


def notes(s, t): s.notes_slide.notes_text_frame.text = t


# =============================================== 1 封面
s = slide()
box(s, 0, Inches(3.0), Inches(7.6), Inches(0.06), fill=CYAN, round_=False)
txt(s, Inches(0.9), Inches(1.2), Inches(7.0), Inches(0.5),
    [[("2026 TEL Robot Combat", 17, CYAN, True)]])
txt(s, Inches(0.9), Inches(1.75), Inches(7.0), Inches(1.3),
    [[("嘻諧們", 60, WHITE, True)], [("AI 自主棒球投手機器人", 27, BLUE, True)]], sa=2)
txt(s, Inches(0.9), Inches(3.35), Inches(7.0), Inches(0.9),
    [[("四大難題 × 軟硬體整合", 27, CYAN, True)]])
txt(s, Inches(0.9), Inches(5.9), Inches(7.2), Inches(1.0),
    [[("NCKU 機械系・電機所", 15, WHITE, True)],
     [("林敬堯 · 阮紹銘 · 何詺威 ｜ 指導 張怡玲 教授", 13, GREY, False)]], sa=3)
pic(s, "image4.jpg", Inches(8.0), Inches(1.15), Inches(4.6), Inches(5.4), frame=False)
notes(s, "（10s）大家好，我們是嘻諧們，這是一台 AI 自主棒球投手機器人。"
          "接下來六分鐘，我們用四大難題帶大家看——每一題我們都從軟體和硬體兩邊一起解。")

# =============================================== 2 四大難題
s = slide(); kicker(s, "這場比賽，難在哪？")
items = [("①", "60 秒自主", "不能碰遙控", RED),
         ("②", "距離未知", "還要精準入框", AMBER),
         ("③", "3 分鐘 27 顆", "7 秒一顆", BLUE),
         ("④", "眩光干擾", "影像過曝", GREEN)]
x0, w, g, y, h = Inches(0.85), Inches(2.85), Inches(0.15), Inches(1.9), Inches(3.0)
for i, (num, big, small, c) in enumerate(items):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=2.5)
    txt(s, l, Inches(2.15), w, Inches(0.9), [[(num, 44, c, True)]], align=PP_ALIGN.CENTER)
    txt(s, l, Inches(3.15), w, Inches(0.9), [[(big, 24, WHITE, True)]], align=PP_ALIGN.CENTER)
    txt(s, l, Inches(4.05), w, Inches(0.6), [[(small, 15, GREY, False)]], align=PP_ALIGN.CENTER)
box(s, Inches(0.85), Inches(5.35), Inches(11.6), Inches(1.05), fill=RGBColor(0x10, 0x1A, 0x30),
    line=CYAN, lw=1.5)
txt(s, Inches(1.1), Inches(5.35), Inches(11.1), Inches(1.05),
    [[("每一題都用 ", 19, WHITE, True), ("軟體 ＋ 硬體", 26, CYAN, True), (" 一起解", 19, WHITE, True)]],
    align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
footer(s, 2)
notes(s, "（30s）這場比賽真正難的有四件事：前 60 秒不能碰遙控；目標距離方位未知還要精準入框；"
          "3 分鐘要投完 27 顆，平均不到 7 秒一顆；還有現場眩光會讓影像過曝。"
          "接下來每一題，我們都會講軟體怎麼解、硬體怎麼解。")

# =============================================== 3 難題① 60秒自主
s = slide(); kicker(s, "難題 ①　｜　60 秒自主控制")
hurt(s, "前 60 秒禁止遙控 — 感知、決策、發射全都得自己來")
pic(s, "image3.png", Inches(0.85), Inches(2.2), Inches(7.5), Inches(3.9),
    caption="YOLO26 在同等精度下延遲最低 — 這是我們選它的理由")
solbox(s, Inches(8.6), Inches(2.2), Inches(3.85), Inches(1.85), "軟體",
       ["YOLO26 即時辨識", "NMS-free 極低延遲", "閉迴路自動發射"])
solbox(s, Inches(8.6), Inches(4.25), Inches(3.85), Inches(1.85), "硬體",
       ["Jetson Orin 機載運算", "同軸 USB 相機", "UART→Arduino"])
footer(s, 3)
notes(s, "（50s）第一題，前 60 秒不能碰遙控。軟體上我們用 YOLO26 即時辨識九宮格、亮燈得分格和對手，"
          "從這張官方比較圖可以看到，YOLO26 在同樣精度下延遲是最低的——這就是我們選它的理由；"
          "它原生 NMS-free，後處理少所以更快，形成自動閉迴路。"
          "硬體上運算跑在機上的 Jetson Orin，相機與發射器同軸，算完用 UART 傳給 Arduino。"
          "最關鍵的是，前 60 秒完全不靠無線通訊，所以天生抗干擾。")

# =============================================== 4 難題② 精準入框
s = slide(); kicker(s, "難題 ②　｜　距離方位未知・精準入框")
hurt(s, "距離角度事先未知，網框還會被打歪 — 座標不能寫死")
solbox(s, Inches(0.85), Inches(2.2), Inches(3.7), Inches(3.9), "軟體",
       ["單目幾何測距", "網框 40cm 為已知尺寸", "解析拋體模型算角度", "每發重新定位"])
solbox(s, Inches(4.75), Inches(2.2), Inches(3.7), Inches(3.9), "硬體",
       ["迴轉支撐滾珠軸承", "（水平旋轉自由度）", "90° 扇形齒輪 + Y 字支架", "（鉛直俯仰）", "步進馬達雙軸驅動"])
pic(s, "image6.png", Inches(8.7), Inches(2.2), Inches(3.75), Inches(3.9),
    caption="扇形齒輪俯仰機構")
footer(s, 4)
notes(s, "（50s）第二題，距離方位未知，而且網框被打到還會位移，所以座標不能寫死。"
          "軟體上，因為網框尺寸已知——一般格 40 公分——我們用單目幾何從影像中框的大小反推距離和方位角，"
          "不需要額外測距感測器，再用拋體方程解出俯仰角，而且每一發都重新定位。"
          "硬體上，水平方向用迴轉支撐滾珠軸承保持旋轉自由度，鉛直方向就是右邊這個"
          "90 度扇形齒輪配合 Y 字支架，由步進馬達驅動，定位精準又可重複。")

# =============================================== 5 難題③ 3分鐘27顆
s = slide(); kicker(s, "難題 ③　｜　3 分鐘投 27 顆")
hurt(s, "180 秒 ÷ 27 顆 ≈ 每 6.7 秒完成一次 辨識→瞄準→發射")
pic(s, "image9.png", Inches(0.85), Inches(2.2), Inches(6.3), Inches(3.7),
    caption="彈倉轉輪：齒輪＋齒條推進，清空後自動換倉")
solbox(s, Inches(7.5), Inches(2.2), Inches(4.95), Inches(1.75), "軟體",
       ["單階段一次輸出所有目標", "TensorRT 加速推論", "自動連續發射"])
solbox(s, Inches(7.5), Inches(4.15), Inches(4.95), Inches(1.75), "硬體",
       ["彈倉每倉 4 顆", "齒輪＋齒條推球入軌", "清空後前方齒輪自動換倉"])
footer(s, 5)
notes(s, "（50s）第三題，3 分鐘要投完 27 顆，平均不到 7 秒一顆，而且補彈要用救援卡、全場只有 3 張。"
          "軟體上，YOLO 是單階段偵測，一次前向運算就輸出所有目標，再加 TensorRT 加速，全程自動連發。"
          "硬體上就是左邊這個彈倉轉輪，每倉裝 4 顆，用齒輪加齒條的推進裝置把球推進發射軌道，"
          "彈倉清空之後由前方齒輪自動旋轉到下一倉，不用人介入。")

# =============================================== 6 球速（必講）
s = slide(); kicker(s, "預計球速　｜　飛輪轉速怎麼定")
pic(s, "image7.png", Inches(0.85), Inches(1.35), Inches(5.5), Inches(4.6),
    caption="雙高速飛輪對夾摩擦投射")
box(s, Inches(6.8), Inches(1.35), Inches(5.65), Inches(2.0), fill=RGBColor(0x10, 0x1A, 0x30),
    line=CYAN, lw=2)
txt(s, Inches(6.8), Inches(1.5), Inches(5.65), Inches(1.7),
    [[("預計球速", 18, WHITE, True)], [("約 10 m/s", 46, CYAN, True)],
     [("≈ 36 km/h", 16, GREY, False)]], align=PP_ALIGN.CENTER, sa=3)
box(s, Inches(6.8), Inches(3.55), Inches(5.65), Inches(1.35), fill=PANEL, line=BLUE, lw=1.5)
txt(s, Inches(7.0), Inches(3.55), Inches(5.25), Inches(1.35),
    [[("v = r · ω = r · (2π × rpm / 60)", 19, BLUE, True)],
     [("半徑 r = 6 cm（飛輪直徑 12 cm）", 15, WHITE, False)]],
    anchor=MSO_ANCHOR.MIDDLE, sa=5)
cards = [("理論轉速", "1592", "rpm", GREY), ("實際設定", "1700", "rpm", AMBER)]
for i, (name, val, unit, c) in enumerate(cards):
    l = Emu(int(Inches(6.8)) + i * int(Inches(2.95)))
    box(s, l, Inches(5.1), Inches(2.7), Inches(1.3), fill=PANEL, line=c, lw=2)
    txt(s, l, Inches(5.1), Inches(2.7), Inches(1.3),
        [[(name, 14, c, True)], [(val + " ", 30, WHITE, True), (unit, 14, GREY, False)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=2)
txt(s, Inches(6.8), Inches(6.45), Inches(5.65), Inches(0.45),
    [[("設定略高於理論值，補償球體擠壓與打滑", 14, GREEN, True)]], align=PP_ALIGN.CENTER)
footer(s, 6)
notes(s, "（40s）這裡說明預計球速。發射靠雙高速飛輪對夾摩擦，球的出射速度等於飛輪的切線速度，"
          "也就是 v 等於半徑乘上角速度。我們的飛輪半徑是 6 公分，要達到每秒 10 公尺，"
          "換算理論轉速是 1592 rpm；實際我們設在 1700 rpm，略高於理論值，"
          "用來補償球體擠壓變形和打滑造成的損失。所以預計球速大約每秒 10 公尺、時速 36 公里。")

# =============================================== 7 難題④ 眩光
s = slide(); kicker(s, "難題 ④　｜　眩光干擾")
hurt(s, "強光直射鏡頭 → 影像過曝、網框特徵消失，固定閾值必失效")
pic(s, "image2.jpg", Inches(0.85), Inches(2.2), Inches(6.3), Inches(3.9),
    caption="賽場聚光燈與反射光造成的過曝與眩光")
solbox(s, Inches(7.5), Inches(2.2), Inches(4.95), Inches(1.85), "軟體",
       ["Unity 數位孿生合成資料", "域隨機化：光照/視角/背景", "YOLO 學整體特徵不靠閾值"])
solbox(s, Inches(7.5), Inches(4.25), Inches(4.95), Inches(1.85), "硬體",
       ["相機同軸固定安裝", "鎖定曝光 / 白平衡", "⚠ 遮光罩方案待補"])
footer(s, 7)
notes(s, "（45s）第四題，現場眩光。強光直射鏡頭會造成局部過曝、網框特徵直接消失，"
          "傳統用固定顏色閾值的方法在這種環境一定失效。"
          "軟體上我們用 Unity 數位孿生做域隨機化，隨機變化光照、視角、背景產生合成資料訓練，"
          "而 YOLO 學的是整體特徵不是單一顏色，本身就對光照有韌性。"
          "硬體上相機與發射器同軸固定確保視角穩定，並鎖定曝光與白平衡避免自動曝光震盪。")

# =============================================== 8 線上自適應
s = slide(); kicker(s, "貫穿四題的核心　｜　線上自適應校正")
txt(s, Inches(0.85), Inches(1.25), Inches(11.6), Inches(0.6),
    [[("每投一顆，就依落點誤差自動微調 —— 越投越準", 24, CYAN, True)]])
flow = ["拋體公式\n算理論值", "相機回授\n實際落點", "計算誤差", "修正下一發"]
x0, w, g, y, h = Inches(0.85), Inches(2.6), Inches(0.4), Inches(2.15), Inches(1.5)
for i, st in enumerate(flow):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=GREEN, lw=2)
    txt(s, l, y, w, h, [[(st, 17, WHITE, True)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    if i < 3:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW,
            Emu(int(l) + int(w) + int(Inches(0.04))), Inches(2.7), Inches(0.32), Inches(0.4))
        ar.fill.solid(); ar.fill.fore_color.rgb = GREEN
        ar.line.fill.background(); ar.shadow.inherit = False
box(s, Inches(0.85), Inches(4.1), Inches(5.65), Inches(2.3), fill=PANEL, line=CYAN, lw=2)
txt(s, Inches(1.05), Inches(4.25), Inches(5.25), Inches(2.0),
    [[("這不是訓練模型", 20, CYAN, True)],
     [("沒有梯度、沒有權重更新", 15, WHITE, False)],
     [("→ 即時、可解釋、現場可除錯", 16, GREEN, True)]], sa=8)
box(s, Inches(6.8), Inches(4.1), Inches(5.65), Inches(2.3), fill=PANEL, line=AMBER, lw=2)
txt(s, Inches(7.0), Inches(4.25), Inches(5.25), Inches(2.0),
    [[("自動補償現場差異", 20, AMBER, True)],
     [("場地、球體、電量、網框位移", 15, WHITE, False)],
     [("→ 賽前建表，賽中小幅修正", 16, GREEN, True)]], sa=8)
footer(s, 8)
notes(s, "（40s）這四題背後有一個貫穿的核心：線上自適應校正。先用拋體公式算理論值，"
          "發射後由相機回授實際落點，算出誤差，再對下一發做小幅修正，命中率越投越準。"
          "要特別說明，這不是在訓練模型，沒有梯度也沒有權重更新，只是瞄準參數的誤差回授，"
          "所以即時、可解釋、現場能除錯。賽前練習就把基礎對照表建好，比賽中只做小幅修正。")

# =============================================== 9 可行性 + 預算
s = slide(); kicker(s, "執行可行性 + 預算")
stg = [("設計", GREEN, "✔"), ("加工", GREEN, "✔"), ("模型訓練", GREEN, "✔"), ("整合 / 校正", AMBER, "▶")]
x0, w, g, y, h = Inches(0.85), Inches(2.4), Inches(0.35), Inches(1.5), Inches(1.2)
for i, (name, c, mark) in enumerate(stg):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=2)
    txt(s, l, y, w, h, [[(mark + " " + name, 18, WHITE, True)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    if i < 3:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW,
            Emu(int(l) + int(w) + int(Inches(0.02))), Inches(1.9), Inches(0.3), Inches(0.4))
        ar.fill.solid(); ar.fill.fore_color.rgb = WHITE
        ar.line.fill.background(); ar.shadow.inherit = False
txt(s, Inches(0.85), Inches(3.0), Inches(11.6), Inches(0.6),
    [[("校正走輕量方法 → ", 17, WHITE, False), ("現場可除錯、不賭黑箱", 19, GREEN, True)]])
txt(s, Inches(0.85), Inches(3.85), Inches(11.6), Inches(0.7),
    [[("總預算　", 22, WHITE, True), ("40,000 元", 40, CYAN, True)]])
cards = [("AI 運算 · 感測", "~1.9 萬", CYAN), ("結構 · 電源 · 耗材", "~1.5 萬", BLUE),
         ("發射 · 驅動", "~0.6 萬", GREEN)]
x0, w, g, y, h = Inches(0.85), Inches(3.75), Inches(0.2), Inches(4.7), Inches(1.6)
for i, (name, amt, c) in enumerate(cards):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=2)
    txt(s, l, y, w, h, [[(name, 15, c, True)], [(amt, 28, WHITE, True)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=3)
box(s, Inches(0.85), Inches(6.45), Inches(11.6), Inches(0.45),
    fill=RGBColor(0x2A, 0x1F, 0x10), line=AMBER, lw=1)
txt(s, Inches(1.0), Inches(6.45), Inches(11.4), Inches(0.45),
    [[("⚠ 預算數字請再核對（原檔標示不確定）", 13, AMBER, True)]], anchor=MSO_ANCHOR.MIDDLE)
footer(s, 9)
notes(s, "（30s）可行性上，設計、加工、模型訓練都已完成，目前在整合與校正。"
          "校正刻意走輕量方法，現場可除錯、不賭黑箱。預算四萬元，"
          "最大兩塊是 AI 運算與結構電源，錢花在真正決定成績的地方。")

# =============================================== 10 結尾
s = slide()
box(s, 0, Inches(3.05), Inches(7.6), Inches(0.06), fill=CYAN, round_=False)
txt(s, Inches(0.9), Inches(1.6), Inches(7.0), Inches(1.3),
    [[("四大難題", 44, CYAN, True)], [("軟硬體一起解", 34, WHITE, True)]], sa=4)
txt(s, Inches(0.9), Inches(3.4), Inches(7.0), Inches(0.8),
    [[("自主控制 · 精準入框 · 投擲節奏 · 抗眩光", 18, GREY, False)]])
txt(s, Inches(0.9), Inches(4.6), Inches(7.0), Inches(1.3),
    [[("Thank you ｜ Q & A", 38, WHITE, True)], [("嘻諧們 ｜ NCKU", 16, GREY, False)]], sa=6)
box(s, Inches(0.9), Inches(6.2), Inches(4.4), Inches(0.5), fill=RGBColor(0x1A, 0x1A, 0x2E),
    line=GREY, lw=1)
txt(s, Inches(0.9), Inches(6.2), Inches(4.4), Inches(0.5),
    [[("（口號位置保留，之後再定）", 13, GREY, False)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
pic(s, "image4.jpg", Inches(8.0), Inches(1.5), Inches(4.6), Inches(4.8), frame=False)
notes(s, "（15s）總結：四大難題，我們每一題都從軟體和硬體兩邊一起解決——"
          "自主控制、精準入框、投擲節奏、抗眩光。以上是嘻諧們，謝謝，歡迎提問！")

base = os.path.dirname(__file__)
out = os.path.join(base, "嘻諧們_簡報日_v4.pptx")
try:
    prs.save(out)
except PermissionError:
    out = os.path.join(base, "嘻諧們_簡報日_v4_new.pptx")
    prs.save(out)
    print("!! 原檔被 PowerPoint 鎖住，已改存 _new.pptx")
print("saved:", out.encode("utf-8", "replace").decode("utf-8", "replace"),
      "slides:", len(prs.slides._sldIdLst))
