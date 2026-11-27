#!/usr/bin/env python3
"""Printable ArUco stickers for the wooden cubes: one id per cube, the same id on all six faces.

    python3 tools/make_cube_stickers.py --cubes 15 --first-id 50 -o ~/so101_data/stickers/kup_etiketleri.pdf

Does not touch the servo bus. A4, 600 dpi. Each sticker: the black marker square (--marker-mm,
default 20 mm) with a white quiet zone (--quiet-mm, default 3 mm) inside a dashed cut line, so a
26 mm sticker on a ~30 mm face leaves 2 mm of wood around it. Dictionary DICT_4X4_100, ids
first_id .. first_id + cubes - 1: its first 50 ids are DICT_4X4_50, which the paper cubes use (ids
0-35, six per cube), so both sets can be on the table at once.

Print at 100 % ("actual size", not "fit to page"); the 100 mm bar on each page checks the scale.
"""

import argparse
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DPI = 600
MM = DPI / 25.4
A4 = (round(210 * MM), round(297 * MM))


def px(mm):
    return int(round(mm * MM))


def marker(dictionary, k, side_px):
    img = cv2.aruco.drawMarker(dictionary, k, side_px)      # OpenCV 4.6 (generateImageMarker in >= 4.7)
    return Image.fromarray(img).convert("L")


def font(size_px):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(f):
            return ImageFont.truetype(f, size_px)
    return ImageFont.load_default()


def dashed_rect(d, x0, y0, x1, y1, dash, width):
    for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)):
        n = max(1, int(np.hypot(bx - ax, by - ay) // (2 * dash)))
        for i in range(n):
            t0, t1 = (2 * i) / (2 * n), (2 * i + 1) / (2 * n)
            d.line([(ax + (bx - ax) * t0, ay + (by - ay) * t0), (ax + (bx - ax) * t1, ay + (by - ay) * t1)],
                   fill=110, width=width)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cubes", type=int, default=15)
    ap.add_argument("--first-id", type=int, default=50)
    ap.add_argument("--marker-mm", type=float, default=20.0)
    ap.add_argument("--quiet-mm", type=float, default=3.0)
    ap.add_argument("--gap-mm", type=float, default=0.0,
                    help="space between stickers; 0 = butted grid with straight cut lines through every row and column")
    ap.add_argument("-o", "--out", default=os.path.expanduser("~/so101_data/stickers/kup_etiketleri.pdf"))
    a = ap.parse_args()
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    assert a.first_id + a.cubes <= 100, "DICT_4X4_100 has ids 0..99"

    sticker = a.marker_mm + 2 * a.quiet_mm
    pitch = sticker + a.gap_mm
    margin_x, margin_y, label_w, head = 12.0, 14.0, 16.0, 16.0
    rows_per_page = int((297 - margin_y - head - 22) // pitch)
    f_big, f_small, f_head = font(px(4.2)), font(px(2.4)), font(px(4.0))

    pages = []
    for start in range(0, a.cubes, rows_per_page):
        page = Image.new("L", A4, 255)
        d = ImageDraw.Draw(page)
        d.text((px(margin_x), px(margin_y - 6)),
               f"SO-101 ahsap kup etiketleri - ArUco 4x4_100, kare {a.marker_mm:.0f} mm - %100 olcekte yazdirin",
               fill=0, font=f_head)
        y_bar = margin_y + 2
        d.line([(px(margin_x), px(y_bar)), (px(margin_x + 100), px(y_bar))], fill=0, width=px(0.5))
        for t in range(0, 101, 10):
            d.line([(px(margin_x + t), px(y_bar - 1.5)), (px(margin_x + t), px(y_bar + 1.5))], fill=0, width=px(0.3))
        d.text((px(margin_x + 103), px(y_bar - 2)), "100 mm: cetvelle kontrol (olcek dogru mu)", fill=0, font=f_small)
        for r, k in enumerate(range(start, min(a.cubes, start + rows_per_page))):
            mid = a.first_id + k
            y = margin_y + head + r * pitch
            d.text((px(margin_x), px(y + sticker / 2 - 4)), f"Kup {k}", fill=0, font=f_big)
            d.text((px(margin_x), px(y + sticker / 2 + 1.5)), f"id {mid}", fill=0, font=f_small)
            img = marker(dic, mid, px(a.marker_mm))
            for j in range(6):
                x = margin_x + label_w + j * pitch
                if a.gap_mm > 0:
                    dashed_rect(d, px(x), px(y), px(x + sticker), px(y + sticker), px(1.0), px(0.15))
                page.paste(img, (px(x + a.quiet_mm), px(y + a.quiet_mm)))
                # tiny id under the marker, inside the quiet zone, far from the black square
                d.text((px(x + a.quiet_mm), px(y + a.quiet_mm + a.marker_mm + 0.4)), f"{mid}", fill=150, font=font(px(1.6)))
        d.text((px(margin_x), px(297 - 14)),
               "Once uzun yatay cizgileri, sonra dikeyleri cetvelle kesin. Her satir = bir kupun 6 yuzu.",
               fill=0, font=f_small)
        d.text((px(margin_x), px(297 - 10)),
               "Ortalamak yeterli: 1-2 mm kayma ve birkac derece aci sorun degil.", fill=0, font=f_small)
        if a.gap_mm <= 0:
            # butted grid: one straight line per row / column boundary, run 6 mm past the grid so a
            # ruler can line up with it; cut the long lines first, then the short ones
            n_rows = min(a.cubes, start + rows_per_page) - start
            x0, x1 = margin_x + label_w, margin_x + label_w + 6 * sticker
            y0, y1 = margin_y + head, margin_y + head + n_rows * sticker
            for r in range(n_rows + 1):
                yy = px(y0 + r * sticker)
                d.line([(px(x0 - 6), yy), (px(x1 + 6), yy)], fill=120, width=px(0.12))
            for j in range(7):
                xx = px(x0 + j * sticker)
                d.line([(xx, px(y0 - 6)), (xx, px(y1 + 6))], fill=120, width=px(0.12))
        pages.append(page.convert("RGB"))

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    pages[0].save(a.out, save_all=True, append_images=pages[1:], resolution=DPI)
    print(f"{a.out}: {a.cubes} cubes x 6 stickers, ids {a.first_id}..{a.first_id + a.cubes - 1}, "
          f"{len(pages)} page(s), sticker {sticker:.0f} mm (marker {a.marker_mm:.0f} + 2 x {a.quiet_mm:.0f} quiet)")


if __name__ == "__main__":
    main()
