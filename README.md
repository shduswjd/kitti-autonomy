# KITTI Navigation, Perception & Local Planning

KITTI Raw 데이터를 사용해 자율주행의 위치 추정 → 주변 환경 인식 → 지역 경로 계획을 단계적으로 구현하는 Python 프로젝트입니다. 현재 GPS/IMU ESKF 기반 navigation과 결과 시각화를 구현했으며, perception과 local planning은 구현 예정입니다.

## 구성

| 경로 | 역할 / 상태 |
|---|---|
| `load_file.py` | 공통 KITTI 로더: 시간, GPS, IMU, 선택적 이미지·LiDAR 로딩 |
| `navigation/` | ESKF 예측·보정, GPS→ENU 변환, 실행 및 DataFrame 변환 |
| `perception/` | 예정: LiDAR 관심 영역 추출, 지면 제거, 장애물 검출 |
| `local_planning/` | 예정: 장애물과 목표 경로를 이용한 지역 경로 생성 |
| `results/navigation/` | CSV, 궤적·위치 차이 그래프, 시각화 스크립트 |

## 환경 준비

프로젝트 루트에서 실행합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install numpy pandas matplotlib Pillow pykitti scipy pymap3d
```

KITTI 데이터는 별도 다운로드가 필요합니다. 기본 시퀀스는 `2011_09_26_drive_0005_sync`입니다. 날짜별 calibration 파일도 다운로드해 아래 위치에 넣으세요. 데이터 압축파일의 폴더 구조를 유지합니다.

```text
data/
└── 2011_09_26/
    ├── calib_cam_to_cam.txt
    ├── calib_imu_to_velo.txt
    ├── calib_velo_to_cam.txt
    └── 2011_09_26_drive_0005_sync/
        ├── oxts/                 # data/, timestamps.txt
        ├── image_00/             # 각 카메라의 data/
        ├── image_01/
        ├── image_02/
        ├── image_03/
        └── velodyne_points/      # data/
```

## 실행

```bash
python load_file.py --frame 0 --lidar
python -m navigation.run_eskf
python results/navigation/plot.py
# 그래프 창도 표시하려면
python results/navigation/plot.py --show
```

ESKF 결과는 `results/navigation/result.csv`에 저장됩니다. 시각화 명령은 같은 폴더에 `trajectory.png`, `result_error.png`, `metrics.csv`를 저장합니다. 그래픽 화면이 없는 환경에서는 `MPLBACKEND=Agg python results/navigation/plot.py`로 실행할 수 있습니다.

자세한 초기화·좌표계·반환값은 [navigation README](navigation/README.md)를 참고하세요.

## 결과 해석 및 다음 단계

`gps_*`는 OXTS 위치를 ENU로 변환한 보정 입력이며, 독립적인 정답 궤적이 아닙니다. ESKF와 이 값의 차이는 GPS 입력에 대한 일치도를 보여줍니다. OXTS 자세·속도 역시 장비의 기존 GPS/IMU 융합 출력입니다.

다음 단계는 LiDAR 장애물 인식과 차량 좌표계에서의 local planning입니다. Navigation은 이후 GPS 이상치 거부, GPS 중단 실험, 잡음 튜닝, 독립적인 평가를 보강할 예정입니다.
