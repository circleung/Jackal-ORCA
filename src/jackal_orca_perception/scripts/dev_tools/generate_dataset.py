#!/usr/bin/env python3
"""
generate_dataset.py
네트워크 없이 로컬에서만 YOLOv8 학습용 합성 데이터셋 자동 생성

사용법:
    pip3 install opencv-python numpy
    python3 scripts/generate_dataset.py

생성 결과: datasets/apriltag/ 폴더에 YOLO 포맷 데이터셋
약 1분 이내 완료
"""

import cv2
import numpy as np
import random
import math
from pathlib import Path

# ══════════════════════════════════════════════════════════════════
# 설정
# ══════════════════════════════════════════════════════════════════
NUM_IMAGES  = 2000
IMG_SIZE    = 640
TAG_IDS     = list(range(10))
TRAIN_RATIO = 0.85
OUTPUT_DIR  = Path("datasets/apriltag")

random.seed(42)
np.random.seed(42)

# ══════════════════════════════════════════════════════════════════
# Step 1: AprilTag 패턴 로컬 생성 (다운로드 불필요)
# ══════════════════════════════════════════════════════════════════
def generate_tag_image(tag_id: int, size: int = 96) -> np.ndarray:
    """
    tag_id 별로 고유한 AprilTag 스타일 패턴 생성
    - 흰색 외곽 여백
    - 검정 1셀 테두리
    - 내부 6x6 이진 패턴 (tag_id 기반 결정론적)
    """
    GRID = 8          # 외곽 테두리 포함 8x8
    INNER = 6         # 내부 데이터 6x6
    MARGIN_CELLS = 1  # 흰 여백

    total_cells = GRID + 2 * MARGIN_CELLS  # 10x10
    cell = size // total_cells
    canvas_size = cell * total_cells

    img = np.ones((canvas_size, canvas_size), dtype=np.uint8) * 255

    offset = MARGIN_CELLS * cell  # 내부 8x8 시작점

    # 검정 테두리 (1셀)
    for r in range(GRID):
        for c in range(GRID):
            if r == 0 or r == GRID - 1 or c == 0 or c == GRID - 1:
                y0 = offset + r * cell
                x0 = offset + c * cell
                img[y0:y0 + cell, x0:x0 + cell] = 0

    # 내부 6x6 데이터 패턴 (tag_id 마다 고유)
    rng = np.random.default_rng(tag_id * 99991 + 12347)
    for r in range(INNER):
        for c in range(INNER):
            if rng.random() > 0.5:
                y0 = offset + (r + 1) * cell
                x0 = offset + (c + 1) * cell
                img[y0:y0 + cell, x0:x0 + cell] = 0

    return img


# ══════════════════════════════════════════════════════════════════
# Step 2: 복도 벽 배경 생성
# ══════════════════════════════════════════════════════════════════
WALL_COLORS = [
    (220, 220, 215), (200, 210, 200), (215, 200, 185),
    (230, 225, 210), (190, 195, 200), (180, 180, 175),
]

def make_background(h: int, w: int) -> np.ndarray:
    base = random.choice(WALL_COLORS)
    bg = np.full((h, w, 3), base, dtype=np.uint8)

    # 밝기 그라디언트
    if random.random() > 0.4:
        grad = np.linspace(0.85, 1.05, w, dtype=np.float32)
        bg = (bg * np.clip(grad, 0, 1)[np.newaxis, :, np.newaxis]).clip(0, 255).astype(np.uint8)

    # 수직선 (기둥/문틀 모사)
    if random.random() > 0.6:
        x = random.randint(w // 4, 3 * w // 4)
        cv2.line(bg, (x, 0), (x, h), (155, 155, 150), random.randint(1, 4))

    return bg


# ══════════════════════════════════════════════════════════════════
# Step 3: 태그를 perspective warp 해서 배경에 합성
# ══════════════════════════════════════════════════════════════════
def paste_tag(bg: np.ndarray, tag_gray: np.ndarray):
    H, W = bg.shape[:2]

    # 태그 크기 (거리 모사: 화면의 5%~30%)
    tag_base = random.randint(int(W * 0.05), int(W * 0.30))
    tag_resized = cv2.resize(tag_gray, (tag_base, tag_base))
    tag_color = cv2.cvtColor(tag_resized, cv2.COLOR_GRAY2BGR)
    th, tw = tag_color.shape[:2]

    # 수평 기울기 (복도 측면에서 보이는 각도 모사: 0°~75°)
    horiz_angle = random.uniform(0, 75)
    side = random.choice([-1, 1])
    shrink = max(0.05, math.cos(math.radians(horiz_angle)))

    pts_src = np.float32([[0, 0], [tw, 0], [tw, th], [0, th]])

    if side == 1:
        pts_dst = np.float32([
            [tw * (1 - shrink), 0],
            [tw, 0],
            [tw, th],
            [tw * (1 - shrink), th]
        ])
    else:
        pts_dst = np.float32([
            [0, 0],
            [tw * shrink, 0],
            [tw * shrink, th],
            [0, th]
        ])

    out_w = max(4, int(pts_dst[:, 0].max()) + 1)
    out_h = th

    M = cv2.getPerspectiveTransform(pts_src, pts_dst)
    warped = cv2.warpPerspective(
        tag_color, M, (out_w, out_h),
        flags=cv2.INTER_LINEAR,
        borderValue=(255, 255, 255)
    )

    wh, ww = warped.shape[:2]
    if ww <= 0 or wh <= 0:
        return None

    max_x = W - ww
    max_y = H - wh
    if max_x <= 0 or max_y <= 0:
        return None

    px = random.randint(0, max_x)
    py = random.randint(0, max_y)

    result = bg.copy()
    roi = result[py:py + wh, px:px + ww]

    gray_w = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_w, 240, 255, cv2.THRESH_BINARY_INV)
    mask_inv = cv2.bitwise_not(mask)

    result[py:py + wh, px:px + ww] = (
        cv2.bitwise_and(roi, roi, mask=mask_inv) +
        cv2.bitwise_and(warped, warped, mask=mask)
    )

    cx = (px + ww / 2) / W
    cy = (py + wh / 2) / H
    bw = ww / W
    bh = wh / H

    return result, (cx, cy, bw, bh)


# ══════════════════════════════════════════════════════════════════
# Step 4: 어그멘테이션
# ══════════════════════════════════════════════════════════════════
def augment(img: np.ndarray) -> np.ndarray:
    img = cv2.convertScaleAbs(img,
        alpha=random.uniform(0.7, 1.3),
        beta=random.randint(-30, 30))
    if random.random() > 0.5:
        k = random.choice([3, 5])
        img = cv2.GaussianBlur(img, (k, k), 0)
    if random.random() > 0.5:
        noise = np.random.normal(0, random.uniform(5, 20), img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return img


# ══════════════════════════════════════════════════════════════════
# Step 5: 메인
# ══════════════════════════════════════════════════════════════════
def main():
    print("=" * 55)
    print(" AprilTag 합성 데이터셋 생성 (로컬, 네트워크 불필요)")
    print(f" 목표: {NUM_IMAGES}장  |  IMG_SIZE: {IMG_SIZE}")
    print("=" * 55)

    # 폴더 생성
    for split in ["train", "val"]:
        (OUTPUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)

    # 태그 원본 이미지 생성
    print("\n[1/3] 태그 패턴 생성 중...")
    tag_imgs = {tid: generate_tag_image(tid) for tid in TAG_IDS}
    print(f"  완료: {len(tag_imgs)}개 태그 패턴 생성")

    # 합성
    print(f"\n[2/3] 합성 이미지 {NUM_IMAGES}장 생성 중...")
    n_train = int(NUM_IMAGES * TRAIN_RATIO)
    done = 0

    for i in range(NUM_IMAGES):
        split = "train" if i < n_train else "val"
        fname = f"syn_{i:05d}"

        # Hard negative 10%: 태그 없는 배경만
        if random.random() < 0.10:
            img_out = augment(make_background(IMG_SIZE, IMG_SIZE))
            label = ""
        else:
            tid = random.choice(TAG_IDS)
            bg = make_background(IMG_SIZE, IMG_SIZE)
            ret = paste_tag(bg, tag_imgs[tid])
            if ret is None:
                img_out = augment(bg)
                label = ""
            else:
                img_out, (cx, cy, bw, bh) = ret
                img_out = augment(img_out)
                label = f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n"

        cv2.imwrite(
            str(OUTPUT_DIR / "images" / split / f"{fname}.jpg"),
            img_out,
            [cv2.IMWRITE_JPEG_QUALITY, 92]
        )
        with open(OUTPUT_DIR / "labels" / split / f"{fname}.txt", "w") as f:
            f.write(label)

        done += 1
        if (i + 1) % 500 == 0:
            print(f"  {i + 1}/{NUM_IMAGES} ...")

    # YAML
    print("\n[3/3] YAML 작성...")
    yaml = f"""path: {OUTPUT_DIR.resolve().as_posix()}
train: images/train
val:   images/val
nc: 1
names:
  0: apriltag
"""
    (OUTPUT_DIR / "apriltag.yaml").write_text(yaml)

    print(f"\n완료! {done}장 생성")
    print(f"경로: {OUTPUT_DIR.resolve()}")
    print("\n다음 단계:")
    print("  pip3 install ultralytics")
    print("  python3 scripts/train_yolo.py")


if __name__ == "__main__":
    main()
