"""
Generate DWIN HMI background images (480 x 272) for the excavation depth monitor.

Two screens are produced:
  1. main_screen      - driver name, target depth, current depth, set-offset key,
                        HSE alarm, depth-over-dig alarm, and a menu key -> settings.
  2. settings_screen  - beam length, stick length, Wi-Fi SSID + password,
                        save + back-to-main keys.

Both are saved as PNG (preview) and 24-bit BMP (ready to import into the DGUS
image library). Value areas are drawn as recessed boxes with dim placeholder
text; at runtime the DGUS variable widgets paint the live values over them.

Run:  python generate_hmi.py
"""

import os
import math
from PIL import Image, ImageDraw, ImageFont

W, H = 480, 272
HERE = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------- palette --
BG_TOP   = (16, 30, 48)
BG_BOT   = (6, 13, 22)
TITLE_BG = (10, 20, 34)
PANEL    = (23, 41, 61)
PANEL_BR = (52, 84, 118)
ACCENT   = (0, 200, 255)
BOX_TOP  = (5, 12, 22)
BOX_BOT  = (10, 20, 33)
BOX_BR   = (46, 76, 108)
TXT      = (228, 238, 247)
LABEL    = (130, 162, 192)
DIM      = (78, 104, 132)
AMBER    = (255, 176, 32)
GREEN    = (46, 206, 128)
RED      = (232, 72, 72)
BTN_TOP  = (42, 124, 180)
BTN_BOT  = (24, 84, 130)
BTN_BR   = (78, 164, 216)
BTN2_TOP = (52, 150, 96)          # save button (green)
BTN2_BOT = (28, 104, 64)
BTN2_BR  = (76, 196, 132)

FONTS = "C:/Windows/Fonts/"


def font(name, size):
    return ImageFont.truetype(FONTS + name, size)


F_TITLE  = font("arialbd.ttf", 19)
F_HEAD   = font("arialbd.ttf", 13)
F_LABEL  = font("segoeui.ttf", 13)
F_LABELB = font("segoeuib.ttf", 14)
F_BIG    = font("arialbd.ttf", 38)
F_UNIT   = font("segoeui.ttf", 16)
F_BTN    = font("arialbd.ttf", 15)
F_SMALL  = font("segoeui.ttf", 11)
F_VAL    = font("segoeui.ttf", 17)
F_SIGN   = font("arialbd.ttf", 24)


# ----------------------------------------------------------------- helpers --
def grad_round(base, xy, c_top, c_bot, radius, border=None, bw=2):
    """Paint a vertical-gradient rounded rectangle onto `base`."""
    x0, y0, x1, y1 = xy
    w, h = x1 - x0, y1 - y0
    grad = Image.new("RGB", (w, h))
    gd = ImageDraw.Draw(grad)
    for i in range(h):
        t = i / max(1, h - 1)
        col = tuple(int(c_top[k] + (c_bot[k] - c_top[k]) * t) for k in range(3))
        gd.line([(0, i), (w, i)], fill=col)
    mask = Image.new("L", (w, h), 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, w - 1, h - 1], radius=radius, fill=255)
    base.paste(grad, (x0, y0), mask)
    if border:
        ImageDraw.Draw(base).rounded_rectangle(
            [x0, y0, x1 - 1, y1 - 1], radius=radius, outline=border, width=bw)


def bg(base):
    d = ImageDraw.Draw(base)
    for i in range(H):
        t = i / (H - 1)
        col = tuple(int(BG_TOP[k] + (BG_BOT[k] - BG_TOP[k]) * t) for k in range(3))
        d.line([(0, i), (W, i)], fill=col)


def titlebar(base, text):
    d = ImageDraw.Draw(base)
    d.rectangle([0, 0, W, 38], fill=TITLE_BG)
    d.line([(0, 39), (W, 39)], fill=ACCENT, width=2)
    d.text((14, 19), text, font=F_TITLE, fill=TXT, anchor="lm")


def value_box(base, xy, placeholder="", unit="", big=False, accent=False):
    grad_round(base, xy, BOX_TOP, BOX_BOT, 6,
               border=ACCENT if accent else BOX_BR, bw=2 if accent else 1)
    d = ImageDraw.Draw(base)
    x0, y0, x1, y1 = xy
    cy = (y0 + y1) // 2
    if big:
        if unit:
            d.text((x1 - 12, cy), unit, font=F_UNIT, fill=LABEL, anchor="rm")
        d.text(((x0 + x1) // 2 - 6, cy), placeholder, font=F_BIG, fill=DIM, anchor="mm")
    else:
        d.text((x0 + 12, cy), placeholder, font=F_VAL, fill=DIM, anchor="lm")
        if unit:
            d.text((x1 - 12, cy), unit, font=F_UNIT, fill=LABEL, anchor="rm")


def button(base, xy, text, kind="blue", radius=8):
    if kind == "green":
        ct, cb, br = BTN2_TOP, BTN2_BOT, BTN2_BR
    else:
        ct, cb, br = BTN_TOP, BTN_BOT, BTN_BR
    grad_round(base, xy, ct, cb, radius, border=br, bw=2)
    d = ImageDraw.Draw(base)
    x0, y0, x1, y1 = xy
    lines = text.split("\n")
    n = len(lines)
    cy = (y0 + y1) // 2
    lh = 18
    start = cy - (n - 1) * lh // 2
    for i, ln in enumerate(lines):
        d.text(((x0 + x1) // 2, start + i * lh), ln, font=F_BTN, fill=(245, 250, 255), anchor="mm")


def lamp(base, cx, cy, r, color):
    d = ImageDraw.Draw(base)
    # soft halo
    for k in range(3, 0, -1):
        a = tuple(int(c * (0.18 * k)) for c in color)
        d.ellipse([cx - r - k * 2, cy - r - k * 2, cx + r + k * 2, cy + r + k * 2], fill=None,
                  outline=a, width=1)
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=color, outline=(255, 255, 255))
    # glossy highlight
    d.ellipse([cx - r + 2, cy - r + 2, cx - r + r, cy - r + r],
              fill=tuple(min(255, c + 70) for c in color))


def menu_icon(base, xy):
    """Hamburger 'menu' key on the main screen -> opens settings."""
    grad_round(base, xy, BTN_TOP, BTN_BOT, 6, border=BTN_BR, bw=1)
    d = ImageDraw.Draw(base)
    x0, y0, x1, y1 = xy
    cx = (x0 + x1) // 2
    cy = (y0 + y1) // 2
    bw = (x1 - x0) - 16
    for off in (-6, 0, 6):
        d.rounded_rectangle([cx - bw // 2, cy + off - 1, cx + bw // 2, cy + off + 1],
                            radius=1, fill=(235, 244, 252))


def alarm_cell(base, xy, name, lamp_color):
    grad_round(base, xy, PANEL, (PANEL[0] - 6, PANEL[1] - 8, PANEL[2] - 10), 8,
               border=PANEL_BR, bw=1)
    d = ImageDraw.Draw(base)
    x0, y0, x1, y1 = xy
    lamp(base, x0 + 24, (y0 + y1) // 2, 12, lamp_color)
    d.text((x0 + 46, y0 + 24), name, font=F_LABELB, fill=TXT, anchor="lm")
    d.text((x0 + 46, y0 + 47), "NORMAL", font=F_SMALL, fill=DIM, anchor="lm")


# ------------------------------------------------------------- main screen --
def build_main():
    img = Image.new("RGB", (W, H))
    bg(img)
    titlebar(img, "EXCAVATION  DEPTH  MONITOR")
    menu_icon(img, (436, 7, 472, 33))
    d = ImageDraw.Draw(img)

    # driver row
    grad_round(img, (8, 46, 472, 84), PANEL, (17, 31, 47), 8, border=PANEL_BR, bw=1)
    d.text((22, 65), "DRIVER", font=F_HEAD, fill=ACCENT, anchor="lm")
    value_box(img, (110, 52, 466, 78), placeholder="DRIVER  NAME")

    # depth cards
    for (x0, x1, title, accent) in [(8, 236, "TARGET  DEPTH", False),
                                    (244, 472, "CURRENT  DEPTH", True)]:
        grad_round(img, (x0, 90, x1, 188), PANEL, (17, 31, 47), 8,
                   border=ACCENT if accent else PANEL_BR, bw=2 if accent else 1)
        d.text(((x0 + x1) // 2, 106), title, font=F_HEAD,
               fill=ACCENT if accent else LABEL, anchor="mm")
        value_box(img, (x0 + 12, 120, x1 - 12, 178),
                  placeholder="0.00", unit="m", big=True, accent=accent)

    # bottom row: offset key | HSE alarm | over-dig alarm
    button(img, (8, 194, 156, 264), "SET\nOFFSET")
    alarm_cell(img, (164, 194, 312, 264), "HSE", GREEN)
    alarm_cell(img, (320, 194, 472, 264), "OVER-DIG", GREEN)

    return img


# ---------------------------------------------------------- settings screen --
def build_settings():
    img = Image.new("RGB", (W, H))
    bg(img)
    titlebar(img, "SETTINGS")
    d = ImageDraw.Draw(img)

    h = 34

    def field(y, label, placeholder, unit=""):
        grad_round(img, (8, y, 472, y + h), PANEL, (17, 31, 47), 8, border=PANEL_BR, bw=1)
        d.text((22, y + h // 2), label, font=F_LABELB, fill=TXT, anchor="lm")
        value_box(img, (180, y + 5, 466, y + h - 5), placeholder=placeholder, unit=unit)

    # geometry section
    d.text((14, 50), "GEOMETRY", font=F_HEAD, fill=ACCENT, anchor="lm")
    field(60, "BEAM LENGTH", "0000", "mm")
    field(98, "STICK LENGTH", "0000", "mm")

    # wifi section
    d.text((14, 142), "Wi-Fi  CONFIG", font=F_HEAD, fill=ACCENT, anchor="lm")
    field(152, "SSID", "network-name")
    field(190, "PASSWORD", "* * * * * * * *")

    # keys
    button(img, (8, 232, 150, 266), "BACK")
    button(img, (330, 232, 472, 266), "SAVE", kind="green")

    return img


# ----------------------------------------------------------- keypad screen --
def build_keypad():
    """Numeric keypad only (480 x 272) - no value field. One page, reused for
    target depth (m), beam length (mm) and stick length (mm); the live value is
    shown by the field on the calling screen, not here.

    Suggested touch return codes: keys 0-9 -> that digit, '.' -> decimal point,
    DEL -> backspace, CLR -> clear field, OK -> commit value,
    X (title bar) -> cancel."""
    KEY_TOP, KEY_BOT, KEY_BR = (30, 52, 74), (18, 34, 52), (66, 106, 146)
    FN_TOP,  FN_BOT,  FN_BR  = (150, 96, 40), (108, 66, 24), (210, 150, 70)
    F_KEY   = font("arialbd.ttf", 23)
    F_KEYSM = font("arialbd.ttf", 15)

    img = Image.new("RGB", (W, H))
    bg(img)
    titlebar(img, "NUMERIC  ENTRY")
    d = ImageDraw.Draw(img)

    # cancel (X) key in the title bar
    cx0, cy0, cx1, cy1 = 436, 7, 472, 33
    grad_round(img, (cx0, cy0, cx1, cy1), (150, 60, 60), (108, 40, 40), 6,
               border=(214, 92, 92), bw=1)
    ccx, ccy, s = (cx0 + cx1) // 2, (cy0 + cy1) // 2, 7
    d.line([(ccx - s, ccy - s), (ccx + s, ccy + s)], fill=(246, 232, 232), width=2)
    d.line([(ccx - s, ccy + s), (ccx + s, ccy - s)], fill=(246, 232, 232), width=2)

    # key grid: 4 cols x 4 rows, fills below the title bar
    gx, gy, gap, cols, rows = 8, 48, 8, 4, 4
    cw = (W - 2 * gx - (cols - 1) * gap) // cols
    ch = (264 - gy - (rows - 1) * gap) // rows

    def cell(c, r):
        x0 = gx + c * (cw + gap)
        y0 = gy + r * (ch + gap)
        return (x0, y0, x0 + cw, y0 + ch)

    def key(xy, label, style):
        pal = {"digit": (KEY_TOP, KEY_BOT, KEY_BR, (236, 244, 252), F_KEY),
               "fn":    (FN_TOP, FN_BOT, FN_BR, (250, 244, 235), F_KEYSM),
               "dot":   (BTN_TOP, BTN_BOT, BTN_BR, (245, 250, 255), F_KEY),
               "ok":    (BTN2_TOP, BTN2_BOT, BTN2_BR, (245, 255, 248), F_KEY)}
        ct, cb, br, fg, fnt = pal[style]
        grad_round(img, xy, ct, cb, 8, border=br, bw=2)
        x0, y0, x1, y1 = xy
        d.text(((x0 + x1) // 2, (y0 + y1) // 2), label, font=fnt, fill=fg, anchor="mm")

    for r, rowkeys in enumerate([["7", "8", "9"], ["4", "5", "6"], ["1", "2", "3"]]):
        for c, lbl in enumerate(rowkeys):
            key(cell(c, r), lbl, "digit")

    # wide 0 across the bottom three digit columns
    z0, z2 = cell(0, 3), cell(2, 3)
    key((z0[0], z0[1], z2[2], z2[3]), "0", "digit")

    # right-hand function column
    key(cell(3, 0), "DEL", "fn")
    key(cell(3, 1), "CLR", "fn")
    key(cell(3, 2), ".", "dot")
    key(cell(3, 3), "OK", "ok")

    return img


# --------------------------------------------------------- text keyboard --
def build_keyboard():
    """Full QWERTY + numerals + symbols keyboard (480 x 272), no value field.
    One page, reused for SSID and password entry; the live text is shown by the
    field on the calling screen, not here. SHIFT (aA) toggles upper/lower case
    at runtime.

    Suggested touch return codes: any letter/number/symbol -> that character,
    aA -> shift, DEL -> backspace, SPACE -> space, OK -> commit,
    X (title bar) -> cancel."""
    KEY_TOP, KEY_BOT, KEY_BR = (30, 52, 74), (18, 34, 52), (66, 106, 146)
    FN_TOP,  FN_BOT,  FN_BR  = (150, 96, 40), (108, 66, 24), (210, 150, 70)
    F_K  = font("arialbd.ttf", 18)
    F_FN = font("arialbd.ttf", 13)
    F_OK = font("arialbd.ttf", 20)

    img = Image.new("RGB", (W, H))
    bg(img)
    titlebar(img, "TEXT  ENTRY")
    d = ImageDraw.Draw(img)

    # cancel (X) key in the title bar
    cx0, cy0, cx1, cy1 = 436, 7, 472, 33
    grad_round(img, (cx0, cy0, cx1, cy1), (150, 60, 60), (108, 40, 40), 6,
               border=(214, 92, 92), bw=1)
    ccx, ccy, s = (cx0 + cx1) // 2, (cy0 + cy1) // 2, 7
    d.line([(ccx - s, ccy - s), (ccx + s, ccy + s)], fill=(246, 232, 232), width=2)
    d.line([(ccx - s, ccy + s), (ccx + s, ccy - s)], fill=(246, 232, 232), width=2)

    # key grid: 10 cols x 5 rows, fills below the title bar
    gx, gy, gap, cols, rows = 6, 48, 4, 10, 5
    cw = (W - 2 * gx - (cols - 1) * gap) // cols
    kh = (264 - gy - (rows - 1) * gap) // rows

    def xc(c):
        return gx + c * (cw + gap)

    def yr(r):
        return gy + r * (kh + gap)

    def key(xy, label, style="key", fnt=F_K):
        pal = {"key": (KEY_TOP, KEY_BOT, KEY_BR, (236, 244, 252)),
               "fn":  (FN_TOP, FN_BOT, FN_BR, (250, 244, 235)),
               "mod": (BTN_TOP, BTN_BOT, BTN_BR, (245, 250, 255)),
               "ok":  (BTN2_TOP, BTN2_BOT, BTN2_BR, (245, 255, 248))}
        ct, cb, br, fg = pal[style]
        grad_round(img, xy, ct, cb, 6, border=br, bw=2)
        x0, y0, x1, y1 = xy
        d.text(((x0 + x1) // 2, (y0 + y1) // 2), label, font=fnt, fill=fg, anchor="mm")

    def gk(c, r, label, style="key", fnt=F_K):
        key((xc(c), yr(r), xc(c) + cw, yr(r) + kh), label, style, fnt)

    # row 0: digits 1..0
    for c, lbl in enumerate("1234567890"):
        gk(c, 0, lbl)
    # row 1: q..p
    for c, lbl in enumerate("qwertyuiop"):
        gk(c, 1, lbl)
    # row 2: a..l  + DEL
    for c, lbl in enumerate("asdfghjkl"):
        gk(c, 2, lbl)
    gk(9, 2, "DEL", "fn", F_FN)
    # row 3: shift + z..m + . _
    gk(0, 3, "aA", "mod", F_FN)
    for c, lbl in enumerate("zxcvbnm"):
        gk(c + 1, 3, lbl)
    gk(8, 3, ".")
    gk(9, 3, "_")
    # row 4: @ # -  [wide SPACE]  [wide OK]
    gk(0, 4, "@")
    gk(1, 4, "#")
    gk(2, 4, "-")
    key((xc(3), yr(4), xc(7) + cw, yr(4) + kh), "SPACE", "key", F_FN)
    key((xc(8), yr(4), xc(9) + cw, yr(4) + kh), "OK", "ok", F_OK)

    return img


# ----------------------------------------------------------- alarm signs --
def build_sign(active):
    """One status sign (160 x 120). Two are produced as a same-size PAIR so they
    can be the two frames of a DGUS icon variable: write 0 -> NORMAL (green),
    write 1 -> ALARM (red). Drawn on the same dark theme as the screens."""
    w, h = 160, 120
    color = RED if active else GREEN
    label = "ALARM" if active else "NORMAL"

    img = Image.new("RGB", (w, h), BG_BOT)          # dark corners
    grad_round(img, (2, 2, w - 2, h - 2), PANEL, (17, 31, 47), 12,
               border=color, bw=3)
    d = ImageDraw.Draw(img)

    # glossy status lamp
    cx, cy, r = w // 2, 46, 26
    lamp(img, cx, cy, r, color)

    # glyph inside the lamp: "!" for alarm, check-mark for normal
    if active:
        d.rounded_rectangle([cx - 3, cy - 13, cx + 3, cy + 5], radius=3,
                            fill=(255, 255, 255))
        d.ellipse([cx - 3, cy + 9, cx + 3, cy + 15], fill=(255, 255, 255))
    else:
        d.line([(cx - 12, cy + 1), (cx - 3, cy + 11)], fill=(255, 255, 255), width=4)
        d.line([(cx - 3, cy + 11), (cx + 13, cy - 11)], fill=(255, 255, 255), width=4)

    # label, in a brightened tint of the state colour
    bright = tuple(min(255, c + 60) for c in color)
    d.text((cx, 97), label, font=F_SIGN, fill=bright, anchor="mm")
    return img


def save(img, name):
    png = os.path.join(HERE, name + ".png")
    bmp = os.path.join(HERE, name + ".bmp")
    img.save(png)
    img.save(bmp)                      # 24-bit BMP, DGUS-import ready
    print("saved", png)
    print("saved", bmp)


if __name__ == "__main__":
    save(build_main(), "01_main_screen")
    save(build_settings(), "02_settings_screen")
    save(build_keypad(), "03_numeric_keypad")
    save(build_keyboard(), "04_text_keyboard")
    save(build_sign(False), "05_status_normal")
    save(build_sign(True), "06_status_alarm")
    print("done -", W, "x", H)
