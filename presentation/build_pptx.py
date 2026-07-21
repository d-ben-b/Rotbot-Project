# -*- coding: utf-8 -*-
"""嘻諧們 隊伍簡報日 pptx（字少版・8 頁・故事線：動機→問題→看/算/投→可行）。
逐字稿在每頁備註欄。重跑：python build_pptx.py"""
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

BG    = RGBColor(0x0B, 0x12, 0x20)
PANEL = RGBColor(0x15, 0x20, 0x38)
CYAN  = RGBColor(0x22, 0xD3, 0xEE)
BLUE  = RGBColor(0x38, 0xBD, 0xF8)
WHITE = RGBColor(0xF1, 0xF5, 0xF9)
GREY  = RGBColor(0x94, 0xA3, 0xB8)
AMBER = RGBColor(0xFB, 0xBF, 0x24)
GREEN = RGBColor(0x34, 0xD3, 0x99)
RED   = RGBColor(0xF8, 0x71, 0x71)
FONT  = "Microsoft JhengHei"

prs = Presentation()
prs.slide_width  = Inches(13.333)
prs.slide_height = Inches(7.5)
SW, SH = prs.slide_width, prs.slide_height
BLANK = prs.slide_layouts[6]


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


def kicker(s, text, must=False):
    box(s, Inches(0.6), Inches(0.55), Inches(0.16), Inches(0.6), fill=CYAN, round_=False)
    txt(s, Inches(0.9), Inches(0.48), Inches(9.5), Inches(0.75),
        [[(text, 32, WHITE, True)]], anchor=MSO_ANCHOR.MIDDLE)
    if must:
        b = box(s, Inches(10.7), Inches(0.6), Inches(2.0), Inches(0.55), fill=AMBER, round_=True)
        txt(s, Inches(10.7), Inches(0.6), Inches(2.0), Inches(0.55),
            [[("★ 必講", 18, BG, True)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)


def chip(s, l, t, w, h, big, small, c):
    box(s, l, t, w, h, fill=PANEL, line=c, lw=2)
    txt(s, l, t, w, h, [[(big, 24, c, True)], [(small, 15, WHITE, False)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=4)


def footer(s, n):
    txt(s, Inches(0.6), Inches(7.02), Inches(6), Inches(0.4),
        [[("嘻諧們 ｜ 2026 TEL Robot Combat", 11, GREY, False)]])
    txt(s, Inches(11.2), Inches(7.02), Inches(1.5), Inches(0.4),
        [[(f"{n}/8", 11, GREY, False)]], align=PP_ALIGN.RIGHT)


def img_ph(s, l, t, w, h, label):
    box(s, l, t, w, h, fill=PANEL, line=BLUE, lw=1.25)
    txt(s, l, t, w, h, [[("🖼 圖片區", 15, BLUE, True)], [(label, 12, GREY, False)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=6)


def notes(s, t): s.notes_slide.notes_text_frame.text = t


# ==================================================== 1 封面・動機
s = slide()
box(s, 0, Inches(3.0), SW, Inches(0.06), fill=CYAN, round_=False)
txt(s, Inches(0.9), Inches(1.1), Inches(11.5), Inches(0.6),
    [[("2026 TEL Robot Combat ｜ 東京威力科創機器人大賽", 18, CYAN, True)]])
txt(s, Inches(0.9), Inches(1.7), Inches(11.5), Inches(1.4),
    [[("嘻諧們", 66, WHITE, True)], [("AI 自主棒球投手機器人", 30, BLUE, True)]], sa=2)
txt(s, Inches(0.9), Inches(3.35), Inches(11.5), Inches(1.1),
    [[("眼到　·　算到　·　球到", 40, CYAN, True)]])
txt(s, Inches(0.9), Inches(5.7), Inches(11.5), Inches(1.0),
    [[("NCKU 機械系・電機所　｜　林敬堯 · 阮紹銘 · 何詺威　｜　指導 張怡玲 教授", 16, GREY, False)]])
img_ph(s, Inches(9.4), Inches(5.3), Inches(3.3), Inches(1.5), "機器人整機圖")
notes(s, "①動機（20s）我們是嘻諧們。這次要做一台會自己投棒球的機器人——而且規定前 60 秒"
          "不能遙控、要靠 AI 自主。我們的目標一句話：眼到、算到、球到。")

# ==================================================== 2 遇到的問題
s = slide(); kicker(s, "這場比賽，難在哪？")
data = [("無遙控", "前 60 秒全自主", RED),
        ("隨機亮·雙倍", "得分格賽前才亮", AMBER),
        ("距離未知", "方位事先不知道", BLUE),
        ("現場干擾", "LED·眩光·別隊訊號", GREEN)]
x0 = Inches(0.85); w = Inches(2.85); g = Inches(0.15); y = Inches(2.6); h = Inches(2.3)
for i, (a, b, c) in enumerate(data):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    chip(s, l, y, w, h, a, b, c)
txt(s, Inches(0.85), Inches(5.5), Inches(11.6), Inches(0.7),
    [[("四個難點，決定了我們整台機器人的設計。", 18, GREY, False)]])
footer(s, 2)
notes(s, "②遇到的問題（40s）這場比賽真正難的是四件事：一、前 60 秒不能碰遙控，機器人得自己判斷；"
          "二、得分格是賽前隨機亮、還雙倍計分；三、目標距離方位事先都不知道；"
          "四、現場一堆干擾：LED、眩光、別隊訊號。")

# ==================================================== 3 核心理念 看算投
s = slide(); kicker(s, "我們的解法")
tri = [("眼", "AI 辨識場地", CYAN), ("算", "決策 + 校正", BLUE), ("球", "機構發射", GREEN)]
x0 = Inches(1.1); w = Inches(3.5); g = Inches(0.4); y = Inches(2.5); h = Inches(2.6)
for i, (a, b, c) in enumerate(tri):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=2.5)
    txt(s, l, Inches(2.75), w, Inches(1.6), [[(a, 72, c, True)]], align=PP_ALIGN.CENTER)
    txt(s, l, Inches(4.45), w, Inches(0.6), [[(b, 20, WHITE, True)]], align=PP_ALIGN.CENTER)
    if i < 2:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW,
            Emu(int(l) + int(w) + int(Inches(0.02))), Inches(3.5), Inches(0.36), Inches(0.6))
        ar.fill.solid(); ar.fill.fore_color.rgb = WHITE; ar.line.fill.background(); ar.shadow.inherit=False
txt(s, Inches(1.1), Inches(5.6), Inches(11.1), Inches(0.7),
    [[("眼到場地 → 算到落點 → 球到網框，接下來照這順序講。", 18, GREY, False)]], align=PP_ALIGN.CENTER)
footer(s, 3)
notes(s, "③核心理念（20s）所以我們把機器人拆成三件事——眼到、算到、球到，"
          "也就是看場地、算落點、投進框。接下來就照這順序講。")

# ==================================================== 4 看：AI辨識+通訊
s = slide(); kicker(s, "眼　｜　AI 影像辨識", must=True)
txt(s, Inches(0.85), Inches(1.55), Inches(7.0), Inches(3.6),
    [[("YOLO26", 34, CYAN, True), ("（備援 YOLO11）", 16, GREY, False)],
     [("辨識 九宮格 · 亮燈得分格 · 對手", 18, WHITE, False)],
     [("", 6, WHITE, False)],
     [("單目幾何測距", 28, BLUE, True)],
     [("網框尺寸已知 → 反推距離與角度", 17, WHITE, False)],
     [("不需額外測距感測器", 17, GREEN, True)]], sa=8)
img_ph(s, Inches(8.15), Inches(1.55), Inches(4.3), Inches(2.5), "YOLO 偵測畫面\n（九宮格＋亮燈格套框）")
# 通訊條（重點・修好不重疊）
box(s, Inches(0.85), Inches(5.35), Inches(11.6), Inches(1.35), fill=RGBColor(0x10,0x1A,0x30), line=AMBER, lw=1.5)
txt(s, Inches(1.15), Inches(5.45), Inches(11.0), Inches(1.2),
    [[("通訊設備　", 20, AMBER, True), ("Orin", 18, CYAN, True), ("  —UART/USB 序列→  ", 16, WHITE, False),
      ("Arduino", 18, GREEN, True), ("（自訂封包協定）", 14, GREY, False)],
     [("前 60 秒純機上視覺、不靠無線　→　", 16, WHITE, False), ("天生抗干擾", 18, AMBER, True)]],
    anchor=MSO_ANCHOR.MIDDLE, sa=6)
footer(s, 4)
notes(s, "④看＋通訊（55s）先講看。用 YOLO 即時辨識九宮格、亮起的得分格、還有對手。"
          "網框尺寸已知，我們用單目幾何從畫面反推距離和角度，不用額外測距感測器。"
          "通訊上：辨識在 Orin 上算完，用 UART 序列把指令傳給 Arduino。"
          "重點是——前 60 秒完全不靠無線、純機上視覺，所以天生抗干擾。")

# ==================================================== 5 算：決策+校正
s = slide(); kicker(s, "算　｜　決策 + 校正", must=True)
flow = ["拋體公式\n算理論值", "實機資料\n校正", "每投一顆\n微調"]
x0 = Inches(0.85); w = Inches(3.0); g = Inches(0.75); y = Inches(2.0); h = Inches(1.5)
for i, st in enumerate(flow):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=GREEN, lw=2)
    txt(s, l, y, w, h, [[(st, 18, WHITE, True)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    if i < 2:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW,
            Emu(int(l) + int(w) + int(Inches(0.12))), Inches(2.5), Inches(0.42), Inches(0.5))
        ar.fill.solid(); ar.fill.fore_color.rgb = GREEN; ar.line.fill.background(); ar.shadow.inherit=False
txt(s, Inches(0.85), Inches(3.9), Inches(11.6), Inches(0.9),
    [[("命中率越投越準　＝　", 22, WHITE, True), ("「找手感」", 30, GREEN, True)]])
box(s, Inches(0.85), Inches(5.1), Inches(11.6), Inches(1.1), fill=RGBColor(0x2A,0x1F,0x10), line=AMBER, lw=1.5)
txt(s, Inches(1.1), Inches(5.1), Inches(11.1), Inches(1.1),
    [[("策略：亮起的得分格 = 最高優先，衝雙倍分（最多 12 分）", 20, AMBER, True)]],
    anchor=MSO_ANCHOR.MIDDLE)
footer(s, 5)
notes(s, "⑤算（45s）再來算。瞄準先用物理拋體公式算理論值，再用實機資料校正真實世界的空氣阻力"
          "跟旋轉。比賽中每投一顆就依落點誤差微調，命中率越投越準，我們叫它找手感。"
          "亮起的得分格會被列為最高優先，衝雙倍分。")

# ==================================================== 6 投：機構+球速
s = slide(); kicker(s, "球　｜　機構 + 球速", must=True)
txt(s, Inches(0.85), Inches(1.55), Inches(7.0), Inches(3.4),
    [[("彈倉轉輪 ＋ 飛輪摩擦", 26, CYAN, True)],
     [("每彈倉 4 顆 · 高速飛輪投射", 16, WHITE, False)],
     [("", 6, WHITE, False)],
     [("步進馬達瞄準", 24, BLUE, True)],
     [("控制俯仰 + 旋轉方向", 16, WHITE, False)],
     [("", 6, WHITE, False)],
     [("四輪差速車體", 20, GREEN, True)],
     [("不用萬向輪 · 可靠又省成本", 15, GREY, False)]], sa=6)
box(s, Inches(8.15), Inches(1.55), Inches(4.3), Inches(2.15), fill=RGBColor(0x10,0x1A,0x30), line=CYAN, lw=2)
txt(s, Inches(8.15), Inches(1.7), Inches(4.3), Inches(1.9),
    [[("預計球速", 18, WHITE, True)], [("約 10 m/s", 44, CYAN, True)],
     [("≈ 36 km/h · 飛輪可調", 15, GREY, False)]], align=PP_ALIGN.CENTER, sa=4)
img_ph(s, Inches(8.15), Inches(3.95), Inches(4.3), Inches(2.05), "發射器 / 彈倉機構圖")
box(s, Inches(0.85), Inches(6.3), Inches(7.0), Inches(0.55), fill=RGBColor(0x2A,0x1F,0x10), line=AMBER, lw=1)
txt(s, Inches(1.0), Inches(6.3), Inches(6.8), Inches(0.55),
    [[("⚠ 球速上台前用實機量測填真值（8–12 m/s）", 14, AMBER, True)]], anchor=MSO_ANCHOR.MIDDLE)
footer(s, 6)
notes(s, "⑥投（45s）最後投。供彈用彈倉轉輪，靠高速飛輪摩擦把球投出去，方向靠步進馬達控制俯仰和旋轉。"
          "預計球速約 10 公尺每秒、大概時速 36 公里，飛輪轉速可調，配合校正即時微調。〔記得改實測值〕")

# ==================================================== 7 可行性+預算
s = slide(); kicker(s, "可行性 + 預算")
stg = [("設計", GREEN), ("加工", GREEN), ("模型訓練", GREEN), ("整合/校正", AMBER)]
x0 = Inches(0.85); w = Inches(2.4); g = Inches(0.35); y = Inches(1.9); h = Inches(1.1)
for i, (name, c) in enumerate(stg):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=2)
    mark = "✔ 完成" if c == GREEN else "▶ 進行"
    txt(s, l, y, w, h, [[(name, 18, WHITE, True)], [(mark, 13, c, True)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=3)
    if i < 3:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW,
            Emu(int(l) + int(w) + int(Inches(0.02))), Inches(2.28), Inches(0.3), Inches(0.35))
        ar.fill.solid(); ar.fill.fore_color.rgb = WHITE; ar.line.fill.background(); ar.shadow.inherit=False
txt(s, Inches(0.85), Inches(3.35), Inches(11.6), Inches(0.6),
    [[("校正走輕量方法 → ", 18, WHITE, False), ("現場可除錯、結果可解釋，不賭黑箱", 18, GREEN, True)]])
txt(s, Inches(0.85), Inches(4.3), Inches(11.6), Inches(0.7),
    [[("預算　", 22, WHITE, True), ("40,000 元", 40, CYAN, True)]])
b = [("AI 運算 · 感測", "~1.9 萬", CYAN), ("結構 · 電源 · 耗材", "~1.5 萬", BLUE), ("發射 · 驅動", "~0.6 萬", GREEN)]
x0 = Inches(0.85); w = Inches(3.75); g = Inches(0.2); y = Inches(5.2); h = Inches(1.3)
for i, (name, amt, c) in enumerate(b):
    l = Emu(int(x0) + i * (int(w) + int(g)))
    box(s, l, y, w, h, fill=PANEL, line=c, lw=1.5)
    txt(s, l, y, w, h, [[(name, 15, c, True)], [(amt, 26, WHITE, True)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, sa=3)
footer(s, 7)
notes(s, "⑦可行性＋預算（30s）進度上機構已加工、YOLO 模型已訓練，剩整合和賽前校正。"
          "校正刻意走輕量方法、現場可除錯，不賭黑箱。預算四萬元，主要花在最關鍵的兩塊："
          "AI 運算和穩定的發射動力。")

# ==================================================== 8 收尾
s = slide()
box(s, 0, Inches(3.05), SW, Inches(0.06), fill=CYAN, round_=False)
txt(s, Inches(0.9), Inches(1.7), Inches(11.5), Inches(1.3),
    [[("眼到 · 算到 · 球到", 46, CYAN, True)]])
txt(s, Inches(0.9), Inches(3.4), Inches(11.5), Inches(1.0),
    [[("看得到　·　算得到　·　投得到", 22, WHITE, True)]])
txt(s, Inches(0.9), Inches(5.0), Inches(11.5), Inches(1.4),
    [[("Thank you ｜ Q & A", 40, WHITE, True)], [("嘻諧們 ｜ NCKU", 18, GREY, False)]], sa=6)
img_ph(s, Inches(9.4), Inches(5.0), Inches(3.3), Inches(1.6), "機器人主視覺")
notes(s, "⑧收尾（15s）一句話總結——眼到、算到、球到；看得到、算得到、投得到。"
          "以上是嘻諧們，謝謝，歡迎提問！")

import os
base = os.path.dirname(__file__)
out = os.path.join(base, "嘻諧們_簡報日_6min_v2.pptx")
try:
    prs.save(out)
except PermissionError:
    out = os.path.join(base, "嘻諧們_簡報日_6min_v2_new.pptx")
    prs.save(out)
    print("!! 原檔被 PowerPoint 鎖住，已改存 _new.pptx。請關閉 PowerPoint 後重跑，即可合併回同一檔。")
print("saved:", out.encode("utf-8", "replace").decode("utf-8", "replace"), "slides:", len(prs.slides._sldIdLst))
