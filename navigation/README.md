# Navigation: GPS/IMU ESKF

KITTI Raw의 OXTS 가속도·각속도로 상태를 예측하고, ENU 위치 측정으로 보정하는 15차원 오차 상태 칼만 필터입니다. 필요하면 OXTS 속도도 함께 보정할 수 있습니다.

## 파일과 실행

- `eskf.py`: 명목 상태, IMU 예측, 측정 보정, 오차 주입, 공분산 reset
- `run_eskf.py`: GPS→ENU 변환, 초기화, 필터 실행, DataFrame 변환 및 CSV 저장
- `../load_file.py`: 공통 KITTI 데이터 로더
- `../results/navigation/plot.py`: 저장된 CSV 시각화 및 지표 계산

루트 README의 환경·데이터 준비 후 프로젝트 루트에서 실행합니다.

```bash
python -m navigation.run_eskf
# 파일을 직접 실행해도 동일하게 동작
python navigation/run_eskf.py
python results/navigation/plot.py --show
```

실행 결과는 프로젝트의 `results/navigation/`에 저장됩니다. 데이터·결과 경로는 현재 작업 디렉터리와 무관하게 소스 파일 위치를 기준으로 결정합니다.

## 상태와 좌표계

| 항목 | 규약 |
|---|---|
| `p`, `v` | 첫 GPS 위치를 원점으로 한 ENU, m 및 m/s |
| `q` | IMU body → ENU 회전 quaternion, `[w, x, y, z]` |
| `ba`, `bg` | body 가속도 bias [m/s²], 각속도 bias [rad/s] |
| 오차 상태 | `[δp, δv, δθ, δba, δbg]`, 15차원 |
| `P` | 위 오차 순서의 15×15 공분산 |
| 자세 오차 | 오른쪽 곱: `q_true = q_nominal ⊗ δq` |
| 중력 | ENU에서 `[0, 0, -9.81]` m/s² |

가속도 변환은 `R_world_body @ (accel - ba) + gravity_world`입니다. 자세는 bias를 제거한 body 각속도로 quaternion을 적분하며, Euler 각을 직접 적분하지 않습니다.

GPS는 `llh2enu(llh, origin_llh)`로 변환합니다. 입력은 위도·경도 [degree], 고도 [m]이며, 한 점 `(3,)` 또는 여러 점 `(N, 3)`을 지원합니다. 모든 프레임에 동일한 첫 GPS 원점을 사용해야 합니다.

## 초기화와 잡음

첫 프레임으로 `p0=0`, `v0=[ve,vn,vu]`, `q0=roll/pitch/yaw에서 변환한 quaternion`을 설정합니다. Bias는 0부터 시작하며, 예측은 두 번째 프레임부터 수행합니다.

`P0`는 초기 표준편차를 제곱해 구성합니다. `Q_imu`는 가속도 잡음, 각속도 잡음, 가속도 bias random walk, 각속도 bias random walk 순서의 12×12 연속시간 공분산 밀도입니다. 샘플별 측정 분산과 다릅니다. `R_gnss`는 위치만이면 3×3, 위치·속도이면 6×6 측정 공분산입니다. 코드의 수치는 시작용 예시이며 센서에 맞춘 튜닝이 필요합니다.

## Python API

```python
from load_file import KittiLoader
from navigation.run_eskf import run_eskf, result_to_dataframe

loader = KittiLoader(date='2011_09_26', drive='0005')
result = run_eskf(loader)  # 기본: 위치만 보정
# result = run_eskf(loader, use_velocity=True)
df = result_to_dataframe(result)
print(df.head())
df.to_csv('results/navigation/result.csv')
```

필터를 직접 사용할 때 `predict(gyro, accel, dt)`와 `update(position, velocity=None, covariance=None)`는 내부 상태를 갱신하고 상태 복사본을 반환합니다. `update()`에는 같은 시간·ENU 원점의 측정을 전달해야 합니다. 반환값에는 innovation(측정−보정 전 예측)과 주입한 15차원 correction도 포함됩니다. `dt`는 양수인 초 단위 값입니다.

`run_eskf()` 결과는 프레임별 `p`, `v`, `q`, `ba`, `bg`, 전체 `P`, `time_s`, `gps_enu`, 고정 `origin_llh`, 보정 innovation을 담은 dict입니다. Innovation은 초기화 프레임을 제외하므로 N−1개입니다.

## CSV와 그래프

DataFrame의 한 행은 한 프레임입니다.

| 열 | 의미 |
|---|---|
| `frame`, `time_s` | 프레임 번호, 시작 이후 경과 시간 [s] |
| `p_e_m`, `p_n_m`, `p_u_m` | 보정 후 ESKF 위치 [m] |
| `gps_e_m`, `gps_n_m`, `gps_u_m` | 보정에 사용한 OXTS 위치의 ENU 변환 [m] |
| `v_*_mps` | 추정 ENU 속도 |
| `q_w`, `q_x`, `q_y`, `q_z` | 추정 자세 quaternion |
| `ba_*_mps2`, `bg_*_radps` | 추정 body bias |
| `var_*` | 오차 공분산 대각 성분(분산) |
| `innovation_*` | 보정 전 측정 잔차; 첫 프레임은 NaN |

전체 공분산은 `result['P']`에 남아 있습니다. ENU 원점은 DataFrame의 `attrs['origin_llh']`에 있으나 CSV로 저장하면 attrs는 보존되지 않습니다. 다른 세션의 좌표와 통합할 때는 원점을 별도로 보관하세요.

시각화 스크립트는 궤적, 축별 위치 차이, `metrics.csv`를 저장합니다. 축별 RMSE는 `sqrt(mean(error_axis²))`, 수평/3D 거리 RMSE는 `sqrt(mean(sum(error_vector²)))`입니다. North와 Up을 각각 N/U 열로 비교합니다.

## 현재 한계와 후속 작업

- OXTS는 이미 GPS/IMU 융합 출력입니다. 입력 위치와의 작은 차이가 독립적인 위치 정확도를 의미하지 않습니다. 초기 자세·속도 및 선택적 속도 보정에도 이 출력이 사용됩니다.
- 동기화된 프레임을 사용하며, 원본 고주파 IMU 전체를 처리하는 구성은 아닙니다. 카메라·LiDAR·OXTS 시간은 완전히 동일하다고 보장하지 않습니다.
- 명목 상태는 구간 시작 자세의 가속도로 적분하고, 공분산은 1차 이산화를 사용합니다. 빠른 회전이나 긴 시간 간격에서는 더 정밀한 적분·이산화가 필요합니다.
- GPS 안테나와 IMU 사이 lever arm은 0으로 가정합니다.
- GPS 이상치 거부, 지연 측정 처리, GPS 중단 실험 및 독립적 정확도 평가는 후속 작업입니다.

Perception과 연결할 때 LiDAR → IMU 변환은 `inv(loader.calib.T_velo_imu)`입니다. 이후 `q`와 `p`로 IMU 좌표의 점을 ENU로 옮길 수 있습니다. Local planning에 전달하는 장애물과 목표 경로는 같은 좌표계·시각으로 맞춰야 합니다.
