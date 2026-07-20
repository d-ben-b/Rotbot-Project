# -*- coding: utf-8 -*-
"""產生「嘻諧們」隊伍簡報日 pptx（6 分鐘設計、深色科技風、大關鍵字）。
逐字稿寫在每頁的備註欄，方便上台練習。"""
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn

# ---- 主題色 ----
BG      = RGBColor(0x0B, 0x12, 0x20)   # 深藍黑背景
PANEL   = RGBColor(0x15, 0x20, 0x38)   # 面板
CYAN    = RGBColor(0x22, 0xD3, 0xEE)   # 主色 青
BLUE    = RGBColor(0x38, 0xBD, 0xF8)   # 次色 藍
WHITE   = RGBColor(0xF1, 0xF5, 0xF9)
GREY    = RGBColor(0x94, 0xA3, 0xB8)
AMBER   = RGBColor(0xFB, 0xBF, 0x24)
GREEN   = RGBColor(0x34, 0xD3, 0x99)
RED     = RGBColor(0xF8, 0x71, 0x71)
FONT    = "Microsoft JhengHei"

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


def box(s, l, t, w, h, fill=None, line=None, line_w=1.0, round_=False):
    shp = s.shapes.add_shape(
        MSO_SHAPE.ROUNDED_RECTANGLE if round_ else MSO_SHAPE.RECTANGLE, l, t, w, h)
    if fill is None:
        shp.fill.background()
    else:
        shp.fill.solid(); shp.fill.fore_color.rgb = fill
    if line is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = line; shp.line.width = Pt(line_w)
    shp.shadow.inherit = False
    return shp


def txt(s, l, t, w, h, runs, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP,
        space_after=4, line_spacing=1.05):
    tb = s.shapes.add_textbox(l, t, w, h)
    tf = tb.text_frame; tf.word_wrap = True; tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Inches(0.05)
    tf.margin_top = tf.margin_bottom = Inches(0.02)
    if isinstance(runs[0], tuple):
        runs = [runs]
    for i, para in enumerate(runs):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align; p.space_after = Pt(space_after)
        p.line_spacing = line_spacing
        for (text, size, color, bold) in para:
            r = p.add_run(); r.text = text
            r.font.name = FONT; r.font.size = Pt(size)
            r.font.color.rgb = color; r.font.bold = bold
            r.font._rPr.set(qn('w:eastAsia'), FONT) if False else None
            ea = r.font._rPr.find(qn('a:latin'))
    return tb


def kicker(s, text):
    """左上角小標＋青色橫線"""
    box(s, Inches(0.6), Inches(0.55), Inches(0.14), Inches(0.55), fill=CYAN)
    txt(s, Inches(0.85), Inches(0.5), Inches(11), Inches(0.7),
        [[(text, 30, WHITE, True)]], anchor=MSO_ANCHOR.MIDDLE)


def footer(s, n, must=False):
    txt(s, Inches(0.6), Inches(7.02), Inches(6), Inches(0.4),
        [[("嘻諧們 ｜ 2026 TEL Robot Combat", 11, GREY, False)]])
    tag = f"★必講 ｜ {n}/12" if must else f"{n}/12"
    txt(s, Inches(11.2), Inches(7.02), Inches(1.5), Inches(0.4),
        [[(tag, 11, AMBER if must else GREY, must)]], align=PP_ALIGN.RIGHT)


def img_ph(s, l, t, w, h, label):
    b = box(s, l, t, w, h, fill=PANEL, line=BLUE, line_w=1.25, round_=True)
    # 虛線感：用文字提示
    txt(s, l, t, w, h, [
        [("🖼  圖片放置區", 15, BLUE, True)],
        [(label, 12, GREY, False)],
    ], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, space_after=6)
    return b


def notes(s, text):
    s.notes_slide.notes_text_frame.text = text


# ============================================================ 頁1 封面
s = slide()
box(s, 0, Inches(3.02), SW, Inches(0.06), fill=CYAN)
txt(s, Inches(0.9), Inches(1.15), Inches(11.5), Inches(0.6),
    [[("2026 TEL Robot Combat ｜ 東京威力科創機器人大賽", 18, CYAN, True)]])
txt(s, Inches(0.9), Inches(1.75), Inches(11.5), Inches(1.4),
    [[("嘻諧們", 66, WHITE, True)],
     [("AI 自主棒球投手機器人", 30, BLUE, True)]], space_after=2)
txt(s, Inches(0.9), Inches(3.35), Inches(11.5), Inches(1.1),
    [[("看得準　·　投得中　·　修得快", 40, CYAN, True)]])
txt(s, Inches(0.9), Inches(5.6), Inches(11.5), Inches(1.2),
    [[("國立成功大學　機械系・電機所", 18, WHITE, True)],
     [("隊長 林敬堯 ｜ 隊員 阮紹銘、何詺威 ｜ 指導 張怡玲 教授", 15, GREY, False)]],
    space_after=4)
img_ph(s, Inches(9.4), Inches(5.35), Inches(3.3), Inches(1.55), "機器人整機 3D 算圖")
notes(s, "（10s）大家好，我們是嘻諧們。我們做的，是一台會自己看、自己算、自己投的 "
          "AI 棒球投手機器人——看得準、投得中、修得快。")

# ============================================================ 頁2 定位＋三層架構
s = slide(); kicker(s, "我們在做什麼")
txt(s, Inches(0.85), Inches(1.35), Inches(11.6), Inches(0.9),
    [[("賽事規定 ", 24, WHITE, True), ("前 60 秒不能遙控、必須 AI 自主投球", 24, CYAN, True)]])
labels = [("感知", "YOLO 看場地", CYAN), ("決策", "拋體模型 + 校正", BLUE), ("致動", "Arduino 即時發射", GREEN)]
x = Inches(0.85); w = Inches(3.7); gap = Inches(0.25); y = Inches(3.0); h = Inches(2.2)
for i, (a, b, c) in enumerate(labels):
    l = Emu(int(x) + i * (int(w) + int(gap)))
    box(s, l, y, w, h, fill=PANEL, line=c, line_w=2, round_=True)
    txt(s, l, Inches(3.25), w, Inches(1.2),
        [[(a, 40, c, True)]], align=PP_ALIGN.CENTER)
    txt(s, l, Inches(4.35), w, Inches(0.7),
        [[(b, 18, WHITE, False)]], align=PP_ALIGN.CENTER)
    if i < 2:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW,
            Emu(int(l) + int(w) - Inches(0.05)), Inches(3.75), Inches(0.35), Inches(0.7))
        ar.fill.solid(); ar.fill.fore_color.rgb = WHITE; ar.line.fill.background()
        ar.shadow.inherit = False
txt(s, Inches(0.85), Inches(5.5), Inches(11.6), Inches(0.7),
    [[("這條路徑，就是接下來六分鐘的主線。", 18, GREY, False)]])
footer(s, 2)
notes(s, "（25s）賽事規定前 60 秒不能遙控、要 AI 自主投球。我們把系統拆成三層："
          "感知用 AI 看場地、決策算出要怎麼瞄、致動由控制板即時發射。"
          "這條路徑，就是接下來六分鐘的主線。")

# ============================================================ 頁3 痛點→解法
s = slide(); kicker(s, "四大痛點 → 我們的解法")
rows = [
    ("前 60 秒全自主、無遙控", "YOLO 自主感知 → 自動觸發 → 閉迴路"),
    ("指定得分格隨機亮、雙倍計分", "即時辨識亮燈格，策略列最高優先"),
    ("目標距離與方位未知", "已知網框尺寸 → 單目幾何測距"),
    ("現場 LED／眩光／攝影機干擾", "生成式資料增強、域隨機化"),
]
y = Inches(1.6); rh = Inches(1.15); lw = Inches(5.75)
txt(s, Inches(0.85), Inches(1.5), lw, Inches(0.0), [[("", 6, GREY, False)]])
for i, (a, b) in enumerate(rows):
    ty = Emu(int(y) + i * (int(rh) + int(Inches(0.12))))
    box(s, Inches(0.85), ty, lw, rh, fill=RGBColor(0x1E,0x1B,0x2E), line=RED, line_w=1, round_=True)
    txt(s, Inches(1.05), ty, Emu(int(lw)-int(Inches(0.4))), rh,
        [[("⚠ " + a, 17, WHITE, True)]], anchor=MSO_ANCHOR.MIDDLE)
    box(s, Inches(6.95), ty, Inches(5.5), rh, fill=PANEL, line=GREEN, line_w=1, round_=True)
    txt(s, Inches(7.15), ty, Inches(5.1), rh,
        [[("✔ " + b, 17, WHITE, True)]], anchor=MSO_ANCHOR.MIDDLE)
footer(s, 3)
notes(s, "（40s）這場比賽最難的四件事：不能遙控、目標隨機亮、距離要自己量、現場一堆干擾。"
          "每一項我們都有對應解法——右邊這欄，就是我們整台機器人的設計邏輯。")

# ============================================================ 頁4 系統總覽
s = slide(); kicker(s, "系統總覽")
chain = [("USB Webcam", CYAN), ("Jetson Orin\nNano Super", BLUE),
         ("Arduino", GREEN), ("步進 / 直流馬達", AMBER), ("發射器 / 車體", WHITE)]
x0 = Inches(0.85); bw = Inches(2.15); by = Inches(2.2); bh = Inches(1.5)
edge_labels = ["影像", "序列 UART", "PWM", "驅動"]
for i, (name, c) in enumerate(chain):
    l = Emu(int(x0) + i * (int(bw) + int(Inches(0.35))))
    box(s, l, by, bw, bh, fill=PANEL, line=c, line_w=2, round_=True)
    txt(s, l, by, bw, bh, [[(name, 16, c, True)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    if i < 4:
        ax = Emu(int(l) + int(bw))
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, ax, Inches(2.75), Inches(0.35), Inches(0.4))
        ar.fill.solid(); ar.fill.fore_color.rgb = WHITE; ar.line.fill.background(); ar.shadow.inherit=False
        txt(s, Emu(int(ax)-int(Inches(0.15))), Inches(3.2), Inches(0.65), Inches(0.4),
            [[(edge_labels[i], 10, GREY, False)]], align=PP_ALIGN.CENTER)
box(s, Inches(0.85), Inches(4.5), Inches(11.6), Inches(1.0), fill=RGBColor(0x10,0x1A,0x30), line=CYAN, line_w=1, round_=True)
txt(s, Inches(1.1), Inches(4.5), Inches(11.2), Inches(1.0),
    [[("核心 ", 18, WHITE, True), ("NVIDIA Jetson Orin Nano Super", 20, CYAN, True),
      ("　8GB ｜ 67 TOPS", 18, WHITE, True)],
     [("電源：3S 11.1V 鋰聚合物（飛輪高扭力） + 18650（電子零件）", 15, GREY, False)]],
    anchor=MSO_ANCHOR.MIDDLE, space_after=2)
footer(s, 4)
notes(s, "（35s）硬體核心是 NVIDIA Jetson Orin Nano Super，8GB、67 TOPS，專為邊緣 AI 設計，"
          "能在機上即時跑影像運算。它把瞄準指令用序列通訊丟給 Arduino，由 Arduino 即時驅動馬達。"
          "高運算跟即時控制分開，各司其職。")

# ============================================================ 頁5 投擲方式★
s = slide(); kicker(s, "投擲方式")
txt(s, Inches(0.85), Inches(1.35), Inches(11.6), Inches(0.9),
    [[("彈倉式轉輪供彈　＋　飛輪摩擦投射", 30, CYAN, True)]])
steps = ["轉輪對準\n發射口", "推進機構\n送球入軌", "高速飛輪\n摩擦加速", "出球"]
x0 = Inches(0.85); bw = Inches(2.0); by = Inches(2.5); bh = Inches(1.3)
for i, st in enumerate(steps):
    l = Emu(int(x0) + i * (int(bw) + int(Inches(0.45))))
    box(s, l, by, bw, bh, fill=PANEL, line=BLUE, line_w=1.5, round_=True)
    txt(s, l, by, bw, bh, [[(st, 16, WHITE, True)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    if i < 3:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Emu(int(l)+int(bw)+int(Inches(0.05))), Inches(2.95), Inches(0.32), Inches(0.4))
        ar.fill.solid(); ar.fill.fore_color.rgb = CYAN; ar.line.fill.background(); ar.shadow.inherit=False
txt(s, Inches(0.85), Inches(4.3), Inches(7.4), Inches(2.2),
    [[("· 每彈倉裝 4 顆比賽用發泡球", 18, WHITE, False)],
     [("· 步進馬達控制", 18, WHITE, False), ("俯仰 + 旋轉", 18, AMBER, True), (" 瞄準", 18, WHITE, False)],
     [("· 車體 ", 18, WHITE, False), ("四輪雙馬達差速轉向", 18, AMBER, True), ("（不用萬向輪）", 18, GREY, False)],
     [("  → 更高可靠性、更低成本、狹小空間更靈活", 15, GREY, False)]], space_after=10)
img_ph(s, Inches(8.5), Inches(4.3), Inches(3.95), Inches(2.3), "發射器機構圖 / 彈倉算圖")
footer(s, 5, must=True)
notes(s, "（45s）先講怎麼投。供彈用彈倉式轉輪，每個彈倉裝 4 顆發泡球；轉輪對準發射口後，"
          "推進機構把球送進軌道，靠高速飛輪摩擦把球投出去。瞄準方向靠步進馬達控制俯仰和旋轉。"
          "車體是四輪雙馬達差速轉向，不用萬向輪，換來更高的可靠性跟更低成本。")

# ============================================================ 頁6 球速★
s = slide(); kicker(s, "預計球速")
box(s, Inches(0.85), Inches(1.7), Inches(6.0), Inches(2.6), fill=RGBColor(0x10,0x1A,0x30), line=CYAN, line_w=2, round_=True)
txt(s, Inches(0.85), Inches(1.9), Inches(6.0), Inches(2.2),
    [[("預計球速", 22, WHITE, True)],
     [("約 10 m/s", 60, CYAN, True)],
     [("≈ 36 km/h", 26, BLUE, True)]], align=PP_ALIGN.CENTER, space_after=4)
txt(s, Inches(7.2), Inches(1.9), Inches(5.3), Inches(2.4),
    [[("· 距網框 ", 18, WHITE, False), ("約 250 cm", 18, AMBER, True)],
     [("· 球徑 7 cm ｜ 重 30 g", 18, WHITE, False)],
     [("· 用拋體運動反推初速與角度", 18, WHITE, False)],
     [("· 飛輪轉速可調 → 現場微調", 18, WHITE, False)]], space_after=12)
box(s, Inches(0.85), Inches(4.7), Inches(11.6), Inches(0.8), fill=RGBColor(0x2A,0x1A,0x10), line=AMBER, line_w=1, round_=True)
txt(s, Inches(1.1), Inches(4.7), Inches(11.2), Inches(0.8),
    [[("⚠ 上台前用實機量測，把「約 10 m/s」換成真實值（合理範圍 8–12 m/s）", 16, AMBER, True)]],
    anchor=MSO_ANCHOR.MIDDLE)
img_ph(s, Inches(0.85), Inches(5.7), Inches(11.6), Inches(1.15), "拋體軌跡示意圖（機器人 → 拋物線 → 九宮格網框）")
footer(s, 6, must=True)
notes(s, "（25s）投出去的球速預計約 10 公尺／秒，換算大概每小時 36 公里。距離網框約 2 公尺半，"
          "我們用拋體運動反推需要的初速和角度。飛輪轉速可調，所以球速能依現場微調——"
          "這也是下一頁校正的基礎。〔記得改成實測值〕")

# ============================================================ 頁7 AI①YOLO★
s = slide(); kicker(s, "AI ①　判別式 AI · YOLO 影像辨識")
txt(s, Inches(0.85), Inches(1.4), Inches(6.9), Inches(0.7),
    [[("YOLO26", 26, CYAN, True), ("（備援 YOLO11）", 18, GREY, False),
      ("　+　TensorRT", 22, BLUE, True)]])
txt(s, Inches(0.85), Inches(2.4), Inches(6.9), Inches(3.5),
    [[("辨識三種目標：", 18, WHITE, True)],
     [("· 九宮格網框", 18, WHITE, False)],
     [("· 隨機亮起的指定得分格", 18, AMBER, True)],
     [("· 下半場對手機器人", 18, WHITE, False)],
     [("", 8, WHITE, False)],
     [("單目幾何測距", 20, CYAN, True)],
     [("網框 40cm／加分格 20cm 為已知", 16, WHITE, False)],
     [("→ 從框大小反推距離與方位角", 16, WHITE, False)],
     [("→ 不需額外測距感測器", 16, GREEN, True)]], space_after=6)
img_ph(s, Inches(8.0), Inches(1.4), Inches(4.45), Inches(4.5), "YOLO 偵測畫面\n（九宮格 + 亮燈格套框）")
footer(s, 7, must=True)
notes(s, "（45s）AI 第一塊是看。我們用 YOLO26 即時辨識——九宮格網框、隨機亮起的指定得分格、"
          "還有下半場的對手機器人，在 Orin 上用 TensorRT 加速。網框尺寸是已知的，40 公分、"
          "加分格 20 公分，所以我們能用單目幾何，從影像裡框的大小直接反推出距離跟方位角，"
          "不用額外測距感測器。")

# ============================================================ 頁8 AI②決策+GAI★
s = slide(); kicker(s, "AI ②　決策 + 生成式 AI")
txt(s, Inches(0.85), Inches(1.4), Inches(11.6), Inches(0.7),
    [[("解析拋體模型　+　資料驅動校正　+　線上「找手感」", 24, CYAN, True)]])
# 校正流程
flow = ["物理理論值", "實測誤差", "查表內插", "每投一顆微調"]
x0 = Inches(0.85); bw = Inches(2.5); by = Inches(2.4); bh = Inches(1.0)
for i, st in enumerate(flow):
    l = Emu(int(x0) + i * (int(bw) + int(Inches(0.35))))
    box(s, l, by, bw, bh, fill=PANEL, line=GREEN, line_w=1.5, round_=True)
    txt(s, l, by, bw, bh, [[(st, 16, WHITE, True)]], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    if i < 3:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Emu(int(l)+int(bw)), Inches(2.7), Inches(0.3), Inches(0.4))
        ar.fill.solid(); ar.fill.fore_color.rgb = GREEN; ar.line.fill.background(); ar.shadow.inherit=False
txt(s, Inches(0.85), Inches(3.8), Inches(11.6), Inches(0.6),
    [[("命中率越投越準 → 我們叫它「找手感」", 18, GREEN, True)]])
box(s, Inches(0.85), Inches(4.7), Inches(5.7), Inches(1.9), fill=PANEL, line=BLUE, line_w=1.5, round_=True)
txt(s, Inches(1.05), Inches(4.85), Inches(5.3), Inches(1.7),
    [[("生成式 AI 訓練資料", 18, BLUE, True)],
     [("Unity 數位孿生 + 合成影像增強", 15, WHITE, False)],
     [("域隨機化模擬光線/眩光/視角 → 辨識更穩", 14, GREY, False)]], space_after=6)
box(s, Inches(6.75), Inches(4.7), Inches(5.7), Inches(1.9), fill=PANEL, line=AMBER, line_w=1.5, round_=True)
txt(s, Inches(6.95), Inches(4.85), Inches(5.3), Inches(1.7),
    [[("投球策略", 18, AMBER, True)],
     [("指定得分格 = 最高優先", 15, WHITE, False)],
     [("雙倍計分，最多可得 12 分", 14, GREY, False)]], space_after=6)
footer(s, 8, must=True)
notes(s, "（40s）AI 第二塊是算。瞄準用物理拋體方程先算理論值，再用實機資料校正真實世界的空氣阻力"
          "跟旋轉；比賽中每投一顆就依落點誤差微調，命中率越投越準，我們叫它找手感。"
          "訓練資料則用 Unity 數位孿生加生成式 AI 合成，模擬各種光線角度，讓辨識更穩。"
          "指定得分格會被策略列為最高優先，衝雙倍分。")

# ============================================================ 頁9 為何深度學習
s = slide(); kicker(s, "為何用深度學習，不用傳統影像")
box(s, Inches(0.85), Inches(1.7), Inches(5.7), Inches(3.6), fill=RGBColor(0x2A,0x14,0x14), line=RED, line_w=1.5, round_=True)
txt(s, Inches(1.1), Inches(1.9), Inches(5.2), Inches(3.3),
    [[("❌ 傳統影像處理", 22, RED, True)],
     [("顏色閾值 / 霍夫圓 / 模板比對", 16, WHITE, False)],
     [("", 8, WHITE, False)],
     [("現場有 LED、眩光、網框位移", 16, WHITE, False)],
     [("→ 光線一變，固定閾值就崩潰", 16, GREY, False)]], space_after=8)
box(s, Inches(6.75), Inches(1.7), Inches(5.7), Inches(3.6), fill=RGBColor(0x0E,0x24,0x1C), line=GREEN, line_w=1.5, round_=True)
txt(s, Inches(7.0), Inches(1.9), Inches(5.2), Inches(3.3),
    [[("✔ YOLO 深度學習", 22, GREEN, True)],
     [("學的是整體特徵，非單一顏色", 16, WHITE, False)],
     [("", 8, WHITE, False)],
     [("對光照、視角、形變有韌性", 16, WHITE, False)],
     [("一次輸出所有目標 → 契合即時需求", 16, GREY, False)]], space_after=8)
txt(s, Inches(0.85), Inches(5.6), Inches(11.6), Inches(0.8),
    [[("實質好處：", 18, WHITE, True), ("穩健於干擾、且單一模型涵蓋多類目標", 20, CYAN, True)]])
footer(s, 9)
notes(s, "（20s）有人會問：為什麼不用傳統影像處理就好？因為現場有 LED 指示燈、眩光、"
          "網框投擲後會位移，固定閾值一遇到光線變化就崩潰。YOLO 學的是整體特徵，對干擾有韌性，"
          "一次就輸出所有目標——這是我們選深度學習的實質理由。")

# ============================================================ 頁10 可行性+時程
s = slide(); kicker(s, "執行可行性 + 時程")
stages = [("設計", GREEN, "✔ 完成"), ("加工", GREEN, "✔ 完成"),
          ("模型訓練", GREEN, "✔ 完成"), ("整合測試", AMBER, "▶ 進行中"),
          ("賽前實機校正", CYAN, "▷ 規劃")]
x0 = Inches(0.85); bw = Inches(2.15); by = Inches(2.2); bh = Inches(1.4)
for i, (name, c, st) in enumerate(stages):
    l = Emu(int(x0) + i * (int(bw) + int(Inches(0.2))))
    box(s, l, by, bw, bh, fill=PANEL, line=c, line_w=2, round_=True)
    txt(s, l, by, bw, bh, [[(name, 17, WHITE, True)], [(st, 13, c, True)]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, space_after=4)
    if i < 4:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Emu(int(l)+int(bw)-int(Inches(0.02))), Inches(2.75), Inches(0.22), Inches(0.4))
        ar.fill.solid(); ar.fill.fore_color.rgb = WHITE; ar.line.fill.background(); ar.shadow.inherit=False
txt(s, Inches(0.85), Inches(4.4), Inches(11.6), Inches(2.0),
    [[("· 車體、彈倉、發射器已完成設計與加工；YOLO 模型已訓練", 18, WHITE, False)],
     [("· 校正刻意採輕量方法：", 18, WHITE, False), ("賽前十顆球量級即收斂", 18, AMBER, True)],
     [("· ", 18, WHITE, False), ("現場可除錯、結果可解釋", 18, GREEN, True), ("——不賭黑箱模型", 18, WHITE, False)]],
    space_after=12)
footer(s, 10)
notes(s, "（25s）可行性上，車體、彈倉、發射器都已完成設計與加工，YOLO 模型也已訓練。"
          "剩下的是整合測試與賽前實機校正——我們刻意把校正設計成賽前十顆球量級就能收斂的輕量方法，"
          "現場可除錯、結果可解釋，不賭黑箱模型。")

# ============================================================ 頁11 預算
s = slide(); kicker(s, "預算規劃")
txt(s, Inches(0.85), Inches(1.4), Inches(11.6), Inches(1.0),
    [[("總預算　", 24, WHITE, True), ("40,000 元", 48, CYAN, True)]])
cards = [("運算 / 感測", "~19,100", "Orin + Webcam + SD + Arduino", CYAN),
         ("結構 / 電源 / 耗材", "~15,300", "鋁擠、電子材料、電池、球", BLUE),
         ("發射 / 驅動", "~5,600", "步進+直流馬達 + 驅動器", GREEN)]
x0 = Inches(0.85); bw = Inches(3.75); by = Inches(2.9); bh = Inches(2.5)
for i, (name, amt, desc, c) in enumerate(cards):
    l = Emu(int(x0) + i * (int(bw) + int(Inches(0.2))))
    box(s, l, by, bw, bh, fill=PANEL, line=c, line_w=2, round_=True)
    txt(s, l, Inches(3.1), bw, Inches(2.1),
        [[(name, 18, c, True)], [(amt, 34, WHITE, True)], [("元", 14, GREY, False)],
         [(desc, 13, GREY, False)]], align=PP_ALIGN.CENTER, space_after=6)
txt(s, Inches(0.85), Inches(5.7), Inches(11.6), Inches(0.8),
    [[("錢花在真正決定成績的地方：", 18, WHITE, False), ("AI 運算精度 + 穩定發射動力", 20, CYAN, True)]])
footer(s, 11)
notes(s, "（20s）總預算四萬元。最大兩塊是運算核心跟結構與馬達——錢花在真正決定成績的地方："
          "AI 運算精度、跟穩定的發射動力。其餘是耗材與電源，整體配置合理、可控。")

# ============================================================ 頁12 結尾
s = slide()
box(s, 0, Inches(3.1), SW, Inches(0.06), fill=CYAN)
txt(s, Inches(0.9), Inches(1.7), Inches(11.5), Inches(1.4),
    [[("看得準 · 投得中 · 修得快", 44, CYAN, True)]])
txt(s, Inches(0.9), Inches(3.35), Inches(11.5), Inches(1.2),
    [[("用 AI 看得準　用物理投得中　用資料修得快", 22, WHITE, True)]])
txt(s, Inches(0.9), Inches(5.0), Inches(11.5), Inches(1.4),
    [[("Thank you ｜ Q & A", 40, WHITE, True)],
     [("嘻諧們 ｜ NCKU", 18, GREY, False)]], space_after=6)
img_ph(s, Inches(9.4), Inches(5.0), Inches(3.3), Inches(1.7), "機器人主視覺")
notes(s, "（15s）總結一句：我們用 AI 看得準、用物理投得中、用資料修得快。"
          "以上是嘻諧們的簡報，謝謝，歡迎提問！")

import os
out = os.path.join(os.path.dirname(__file__), "嘻諧們_簡報日_6min.pptx")
prs.save(out)
print("saved:", out, "slides:", len(prs.slides._sldIdLst))
