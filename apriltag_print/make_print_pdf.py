#!/usr/bin/env python3
"""
AprilTag 36h11 출력용 A4 PDF 생성.

- 원본: AprilRobotics/apriltag-imgs 의 tag36h11 PNG (10x10 px:
  흰 테두리 1px + 검은 사각형 8px)
- apriltag.yaml 의 size: 0.16 = '검은 사각형' 한 변이 160 mm 여야 함
  → 전체 태그(흰 테두리 포함)는 160 * 10/8 = 200 mm
- 확대는 반드시 NEAREST (보간 금지 — 모서리 흐려지면 검출 성능 저하)
- 페이지마다 ID 라벨 + 100 mm 검증용 자(scale bar) 인쇄
"""
import glob
import os
import re

from PIL import Image, ImageDraw, ImageFont

DPI         = 300
MM_PER_IN   = 25.4
BLACK_MM    = 150.0                       # apriltag.yaml size: 0.15
                                          # (160mm는 A4 여백 5mm뿐이라 잘림 위험)
TOTAL_MM    = BLACK_MM * 10.0 / 8.0       # 흰 테두리 포함 200 mm
A4_W_MM, A4_H_MM = 210.0, 297.0

def mm2px(mm: float) -> int:
    return round(mm / MM_PER_IN * DPI)

def main():
    raw_dir = os.path.join(os.path.dirname(__file__), 'raw')
    out_pdf = os.path.join(
        os.path.dirname(__file__),
        f'apriltag_36h11_{BLACK_MM:.0f}mm_A4.pdf')

    try:
        font_big = ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 72)
        font_sm = ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 48)
    except OSError:
        font_big = font_sm = ImageFont.load_default()

    pages = []
    for path in sorted(glob.glob(os.path.join(raw_dir, 'tag36_11_*.png'))):
        m = re.search(r'tag36_11_(\d+)\.png', path)
        tag_id = int(m.group(1))

        tag = Image.open(path).convert('L')
        size_px = mm2px(TOTAL_MM)
        tag_big = tag.resize((size_px, size_px), Image.NEAREST)

        page = Image.new('L', (mm2px(A4_W_MM), mm2px(A4_H_MM)), 255)
        d = ImageDraw.Draw(page)

        # 태그: 가로 중앙, 상단 20mm
        x0 = (page.width - size_px) // 2
        y0 = mm2px(20)
        page.paste(tag_big, (x0, y0))

        # 라벨
        y_text = y0 + size_px + mm2px(8)
        d.text((x0, y_text),
               f'tag36h11  ID = {tag_id}', fill=0, font=font_big)
        d.text((x0, y_text + mm2px(10)),
               f'black square must measure {BLACK_MM:.0f} mm '
               f'(config size: {BLACK_MM/1000:.2f})', fill=0, font=font_sm)
        d.text((x0, y_text + mm2px(18)),
               'print at 100% scale (no "fit to page")', fill=0, font=font_sm)

        # 100 mm 검증용 자
        bar_y = y_text + mm2px(30)
        bar_len = mm2px(100)
        d.line([(x0, bar_y), (x0 + bar_len, bar_y)], fill=0, width=6)
        for t_mm in range(0, 101, 10):
            tx = x0 + mm2px(t_mm)
            d.line([(tx, bar_y - mm2px(2)), (tx, bar_y + mm2px(2))], fill=0, width=4)
        d.text((x0, bar_y + mm2px(3)),
               'ruler check: this bar = 100 mm', fill=0, font=font_sm)

        pages.append(page)
        print(f'tag {tag_id}: page ready '
              f'(tag {TOTAL_MM:.0f} mm, black {BLACK_MM:.0f} mm)')

    pages[0].save(out_pdf, save_all=True, append_images=pages[1:],
                  resolution=DPI)
    print(f'\nPDF 저장: {out_pdf} ({len(pages)} pages)')

if __name__ == '__main__':
    main()
