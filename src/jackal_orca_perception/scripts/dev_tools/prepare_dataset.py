#!/usr/bin/env python3
"""
prepare_dataset.py
실측 수집 풀(dataset_collector_node) + 합성 데이터(generate_dataset)를 합쳐
train/val 분할된 학습용 데이터셋을 만든다. (PC에서 실행)

워크플로우:
    1. Jetson에서 dataset_collector_node 로 수집:
         mode:=positive (태그 벽 부착)  → pos_*.jpg
         mode:=negative (태그 전부 제거) → neg_*.jpg
    2. PC로 복사:
         scp -r jetson@<IP>:~/datasets/apriltag_real datasets/
    3. (선택) python generate_dataset.py   — 합성 데이터도 섞으려면
    4. python prepare_dataset.py
    5. python train_yolo.py                — combined yaml 자동 사용

분할 정책:
    - 실측 풀은 pos/neg 각각 따로 85:15 분할 (계층화 — val에 양쪽 모두 보장)
    - 파일명 정렬 후 균등 간격 추출 → 재실행해도 동일 분할 (재현성)
    - 합성 데이터는 기존 train/val 분할 그대로 복사
"""

import shutil
from pathlib import Path

REAL_POOL  = Path("datasets/apriltag_real")      # Jetson에서 복사해온 풀
SYNTH_DIR  = Path("datasets/apriltag")           # generate_dataset.py 결과 (선택)
OUT_DIR    = Path("datasets/apriltag_combined")
VAL_RATIO  = 0.15


def _split_pool(stems: list, val_ratio: float):
    """정렬된 stem 목록을 균등 간격으로 val 추출 (결정적)."""
    stems = sorted(stems)
    n_val = max(1, round(len(stems) * val_ratio)) if stems else 0
    if n_val == 0:
        return stems, []
    step = len(stems) / n_val
    val_idx = {int(i * step) for i in range(n_val)}
    train = [s for i, s in enumerate(stems) if i not in val_idx]
    val   = [s for i, s in enumerate(stems) if i in val_idx]
    return train, val


def _copy(stem: str, src_img: Path, src_lbl: Path, split: str):
    shutil.copy(src_img, OUT_DIR / "images" / split / src_img.name)
    lbl_out = OUT_DIR / "labels" / split / f"{stem}.txt"
    if src_lbl.exists():
        shutil.copy(src_lbl, lbl_out)
    else:                                   # 라벨 파일 누락 = negative 취급
        lbl_out.write_text("")


def main():
    if OUT_DIR.exists():
        shutil.rmtree(OUT_DIR)
    for split in ("train", "val"):
        (OUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)

    counts = {"train": 0, "val": 0}

    # ── 실측 풀: pos/neg 계층화 분할 ──────────────────────────────
    if REAL_POOL.exists():
        imgs = {p.stem: p for p in (REAL_POOL / "images").glob("*.jpg")}
        for prefix in ("pos", "neg"):
            stems = [s for s in imgs if s.startswith(prefix)]
            train, val = _split_pool(stems, VAL_RATIO)
            for split, group in (("train", train), ("val", val)):
                for stem in group:
                    _copy(stem, imgs[stem],
                          REAL_POOL / "labels" / f"{stem}.txt", split)
                    counts[split] += 1
            print(f"실측 {prefix}: train {len(train)} / val {len(val)}")
        if not imgs:
            print(f"⚠️  {REAL_POOL}/images 가 비어 있음")
    else:
        print(f"⚠️  실측 풀 없음: {REAL_POOL} — 합성 데이터만 사용")

    # ── 합성 데이터: 기존 분할 그대로 ─────────────────────────────
    if SYNTH_DIR.exists():
        n_synth = 0
        for split in ("train", "val"):
            for img in (SYNTH_DIR / "images" / split).glob("*.jpg"):
                _copy(img.stem, img,
                      SYNTH_DIR / "labels" / split / f"{img.stem}.txt", split)
                counts[split] += 1
                n_synth += 1
        print(f"합성: {n_synth}장 병합")
    else:
        print(f"합성 데이터 없음 (건너뜀): {SYNTH_DIR}")

    total = counts["train"] + counts["val"]
    if total == 0:
        raise SystemExit("❌ 데이터가 한 장도 없습니다. 경로를 확인하세요.")

    yaml = f"""path: {OUT_DIR.resolve().as_posix()}
train: images/train
val:   images/val
nc: 1
names:
  0: apriltag
"""
    (OUT_DIR / "apriltag.yaml").write_text(yaml)
    print(f"\n✅ 완료: train {counts['train']} / val {counts['val']} (총 {total}장)")
    print(f"   yaml: {OUT_DIR / 'apriltag.yaml'}")
    print("   다음: python train_yolo.py")


if __name__ == "__main__":
    main()
