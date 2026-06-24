# AprilTag YOLO 재학습 키트

Jetson(Jackal)에서 수집한 실측 데이터로 AprilTag 탐지 YOLOv8 모델을 재학습하는 패키지.
**PC(GPU 머신)에서 이 폴더 전체를 복사한 뒤 폴더 안에서 실행할 것.**

## 폴더 구성

```
yolo_train_kit/
├── datasets/apriltag_real/   실측 수집 풀 (2026-06-05 수집, pos 795 + neg 639 = 1,434장)
│   ├── images/               pos_*.jpg (태그 O) / neg_*.jpg (태그 X)
│   └── labels/               YOLO 포맷 (pos: bbox 자동라벨, neg: 빈 파일)
├── prepare_dataset.py        실측(+합성) → train/val 분할 (85:15, 결정적)
├── generate_dataset.py       (선택) 합성 데이터 2,000장 생성
├── train_yolo.py             학습 본체 — yolov8s, 100 epochs
├── export_trt.py             ★ Jetson에서 실행 — .pt → TensorRT .engine 변환
└── models/apriltag_yolo_prev.pt   이전 학습 모델 (비교/백업용)
```

## PC에서 학습 순서

```bash
pip install ultralytics opencv-python numpy

cd yolo_train_kit
python generate_dataset.py    # (선택) 합성 데이터도 섞으려면
python prepare_dataset.py     # → datasets/apriltag_combined/ 생성
python train_yolo.py          # → models/apriltag_yolo.pt 생성
```

- RTX 5070 기준 약 30~60분
- 메모리 부족 시 train_yolo.py의 `BATCH = 16` → `8`

## 학습 후 Jetson 배포

1. `models/apriltag_yolo.pt` → Jetson `~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/models/` 로 복사
   ```bash
   scp models/apriltag_yolo.pt jetson@<IP>:~/ros2_ws/jackal-ORCA/src/jackal_orca_perception/models/
   ```
2. **Jetson에서** `export_trt.py` 실행 → `apriltag_yolo.engine` 생성
   (TensorRT 엔진은 디바이스 종속 — 반드시 Jetson에서 변환)
3. colcon build 후 perception 노드 재시작
