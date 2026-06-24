#!/usr/bin/env python3
"""
train_yolo.py
YOLOv8 AprilTag 탐지 모델 학습

사용법:
    pip install ultralytics
    python train_yolo.py

RTX 5070 기준 약 30~60분 소요 (NUM_IMAGES=2000, epochs=100, yolov8s)
학습된 모델: models/apriltag_yolo.pt

학습 후 Jetson 배포:
    1. apriltag_yolo.pt 를 Jetson의 src/jackal_orca_perception/models/ 로 복사
    2. Jetson에서 dev_tools/export_trt.py 실행 → apriltag_yolo.engine 생성
       (TensorRT 엔진은 디바이스 종속 — 반드시 Jetson에서 변환할 것)
"""

from pathlib import Path
from ultralytics import YOLO
import shutil
import os

# ══════════════════════════════════════════════════════════════════
# 설정
# ══════════════════════════════════════════════════════════════════
# 실측+합성 병합본(prepare_dataset.py)이 있으면 우선, 없으면 합성만
_COMBINED  = Path("datasets/apriltag_combined/apriltag.yaml")
_SYNTH     = Path("datasets/apriltag/apriltag.yaml")
YAML_PATH  = _COMBINED if _COMBINED.exists() else _SYNTH
MODEL_OUT  = Path("models/apriltag_yolo.pt")
EPOCHS     = 100
IMG_SIZE   = 640
BATCH      = 16   # RTX 5070 (12GB+) — 메모리 부족 시 8로 낮출 것
DEVICE     = 0    # GPU 0번 사용

def train():
    print("=" * 60)
    print(" YOLOv8 AprilTag 탐지 모델 학습 시작")
    print("=" * 60)

    if not YAML_PATH.exists():
        raise FileNotFoundError(
            f"{YAML_PATH} 없음.\n"
            "실측 데이터가 있으면:  python prepare_dataset.py\n"
            "합성 데이터만 쓰려면:  python generate_dataset.py"
        )

    Path("models").mkdir(exist_ok=True)

    # YOLOv8s (small) — Orin NX 16GB + TensorRT FP16 기준 실시간(30fps+) 여유
    # 더 가볍게 가려면 yolov8n.pt (nano)로 변경
    model = YOLO("yolov8s.pt")

    print(f"\n데이터셋: {YAML_PATH}")
    print(f"에포크:   {EPOCHS}")
    print(f"배치:     {BATCH}")
    print(f"GPU:      {DEVICE}")
    print()

    results = model.train(
        data    = str(YAML_PATH),
        epochs  = EPOCHS,
        imgsz   = IMG_SIZE,
        batch   = BATCH,
        device  = DEVICE,
        project = "runs/apriltag",
        name    = "v1",
        # 어그멘테이션 (합성 데이터 보완)
        flipud  = 0.0,   # 상하 반전 끔 (벽에 붙은 태그는 뒤집히지 않음)
        fliplr  = 0.5,
        degrees = 5.0,   # 약간의 회전만 (원근 warp으로 이미 커버)
        hsv_h   = 0.01,
        hsv_s   = 0.3,
        hsv_v   = 0.3,
        mosaic  = 0.5,
        mixup   = 0.1,
        # 학습률
        lr0     = 0.01,
        lrf     = 0.01,
        # 저장
        save    = True,
        save_period = 10,
    )

    # 최적 모델을 models/ 폴더로 복사
    best_pt = Path("runs/apriltag/v1/weights/best.pt")
    if best_pt.exists():
        shutil.copy(best_pt, MODEL_OUT)
        print(f"\n✅ 최적 모델 저장: {MODEL_OUT}")
    else:
        print("\n⚠️  best.pt 를 찾지 못했습니다. runs/apriltag/v1/weights/ 확인")

    # 검증 결과 출력
    print("\n" + "=" * 60)
    print(" 학습 완료 — 검증 결과")
    print("=" * 60)
    metrics = model.val()
    print(f"  mAP50:    {metrics.box.map50:.3f}")
    print(f"  mAP50-95: {metrics.box.map:.3f}")
    print()
    print(" 다음 단계: ROS2 노드에 모델 탑재")
    print(f"  모델 경로: {MODEL_OUT.resolve()}")
    print("=" * 60)


if __name__ == "__main__":
    train()
