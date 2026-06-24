#!/usr/bin/env python3
"""
export_trt.py
apriltag_yolo.pt → TensorRT FP16 엔진 변환 (Jetson 전용)

TensorRT 엔진은 빌드한 디바이스(GPU/TensorRT 버전)에 종속되므로
반드시 배포 대상인 Jetson에서 실행해야 한다. PC에서 만든 .engine은
Jetson에서 동작하지 않는다.

사용법 (Jetson에서):
    python3 export_trt.py

입력:  models/apriltag_yolo.pt   (train_yolo.py 결과물, PC에서 복사)
출력:  models/apriltag_yolo.engine  (FP16)

변환 후 colcon build 하면 install/share 에도 복사되고,
tag_yolo_detector_node 가 .pt 보다 새 .engine 을 자동 우선 사용한다.
"""

from pathlib import Path
from ultralytics import YOLO

# train_yolo.py 와 동일한 경로 규약 (dev_tools/ 기준 패키지 루트)
PKG_ROOT = Path(__file__).resolve().parents[2]
PT_PATH  = PKG_ROOT / "models" / "apriltag_yolo.pt"
IMG_SIZE = 640   # train_yolo.py IMG_SIZE 와 일치해야 함


def export():
    if not PT_PATH.exists():
        raise FileNotFoundError(
            f"{PT_PATH} 없음.\n"
            "PC에서 train_yolo.py 로 학습한 apriltag_yolo.pt 를 먼저 복사하세요.")

    print("=" * 60)
    print(" TensorRT FP16 엔진 변환 시작 (수 분 소요)")
    print("=" * 60)
    print(f"입력: {PT_PATH}")

    model = YOLO(str(PT_PATH))
    engine_path = model.export(format="engine", half=True, imgsz=IMG_SIZE)

    # 중간 부산물 .onnx 정리 (배포 불필요 — install/share 용량 절약)
    onnx = PT_PATH.with_suffix(".onnx")
    if onnx.exists():
        onnx.unlink()

    print(f"\n✅ 변환 완료: {engine_path}")
    print("다음 단계:")
    print("  cd ~/ros2_ws/jackal-ORCA && colcon build --packages-select jackal_orca_perception")
    print("  (tag_yolo_detector_node 가 .engine 을 자동으로 우선 로드)")


if __name__ == "__main__":
    export()
