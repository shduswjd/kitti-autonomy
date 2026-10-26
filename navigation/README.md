# GPS/IMU ESKF

IMU 적분으로 상태를 예측하고 GPS로 누적 오차를 줄이는 과정을 구현했다.
일단 KITTI Raw의 `0005` 시퀀스에서 전체 루프가 돌아가도록 만들었고, 위치만 보정하는 경우와 위치·속도를 같이 보정하는 경우를 둘 다 실행해봤다.

## 코드 구성

`eskf.py`에 예측과 보정, `run_eskf.py`에 초기화와 KITTI 실행 루프를 넣었다. 데이터는 루트의 `load_file.py`에서 읽고, 결과 그래프는 `results/navigation/plot.py`에서 그린다.

프로젝트 루트에서:

```bash
python -m navigation.run_eskf
python results/navigation/plot.py --show
```

`results/navigation/`에 저장되는 파일:

- `result.csv`: 프레임별 추정 상태, GPS 위치, 분산, innovation
- `trajectory.png`: East–North 궤적 비교
- `result_error.png`: 축별 ESKF − OXTS 위치 차이
- `metrics.csv`: 축별 및 수평·3D 거리 RMSE와 최대 차이

## 좌표계와 상태

World는 ENU로 두고 첫 GPS 위치를 원점으로 잡았다. 위도·경도·고도는 `pymap3d.geodetic2enu`로 변환한다. 매 프레임 원점을 다시 잡지 않고 처음 값을 계속 사용한다.

명목 상태는 위치 `p`, 속도 `v`, 자세 `q`, 가속도 bias `ba`, 각속도 bias `bg`다. 위치는 m, 속도는 m/s, bias는 각각 m/s²와 rad/s 단위다.

`q`는 body → world 회전이고 순서는 `[w, x, y, z]`로 통일했다. SciPy의 기본 순서가 `[x, y, z, w]`라서 초기화할 때 순서를 바꾼다.

오차 상태는 아래 순서로 15차원이다. Quaternion은 4개 값이지만 자세 오차는 3차원 회전벡터로 둔다.

```text
δx = [δp, δv, δθ, δba, δbg]
P: 15 × 15
```

## 초기화

첫 프레임은 초기화에 쓰고 두 번째 프레임부터 predict/update를 수행한다.

```text
p0  = [0, 0, 0]
v0  = OXTS의 [ve, vn, vu]
q0  = OXTS roll, pitch, yaw → quaternion
ba0 = [0, 0, 0]
bg0 = [0, 0, 0]
```

시작할 때 차량이 정지했다고 가정하지 않고 OXTS 속도를 사용했다. 초기 자세도 OXTS 추정값을 사용하므로, 센서 측정만으로 초기 자세를 구하는 과정은 아직 없다.

`P0`는 초기 표준편차를 제곱해 대각행렬로 만들었다. `Q_imu`와 `R_gnss`를 포함한 현재 잡음 값은 동작 확인용이며, 센서에 맞춰 튜닝한 값은 아니다.

## Predict

각속도와 가속도에서 bias를 제거한다. 각속도는 body 좌표계에 있으므로 회전량 quaternion을 오른쪽에 곱해 자세를 갱신한다.

```text
ω = gyro − bg
q_new = q ⊗ Exp(ω dt)

acc_world = R(q) (accel − ba) + g_world
g_world = [0, 0, −9.81]
```

가속도는 world로 회전한 뒤 중력을 반영한다. 수평 정지 상태에서 IMU가 `[0, 0, 9.81]`을 측정하면 world 가속도는 0이 되는 규약이다. 현재는 구간 시작 자세로 가속도를 변환해 위치·속도를 적분한다.

공분산은 연속시간 오차 모델을 1차 이산화해서 전파한다.

```text
Φ ≈ I + F_c dt
Q_d ≈ G Q_imu Gᵀ dt
P_new = Φ P Φᵀ + Q_d
```

`Q_imu`는 `[가속도 잡음, 각속도 잡음, ba random walk, bg random walk]` 순서의 12×12 연속시간 공분산 밀도다. 샘플별 분산을 그대로 넣는 것과는 구분해야 한다.

## Update

기본은 ENU 위치만 보정한다. 속도를 같이 넣으면 측정 벡터가 `[p_e, p_n, p_u, v_e, v_n, v_u]`가 된다. `R_gnss`도 위치만이면 3×3, 위치·속도이면 6×6으로 맞춘다.

Innovation으로 오차 상태를 계산하고 명목 상태에 주입한다. 자세는 오른쪽 오차 규약 `q_true = q_nominal ⊗ δq`를 사용했다. 공분산은 Joseph form으로 갱신한 뒤 자세 오차 주입에 따른 reset Jacobian을 적용한다.

```python
predicted = eskf.predict(gyro, accel, dt)
corrected = eskf.update(position_enu)
# corrected = eskf.update(position_enu, velocity_enu, covariance_6x6)
```

두 함수 모두 내부 상태를 갱신하고 복사본을 반환한다. `update()` 반환값에는 `innovation`과 15차원 `correction`도 들어 있다. 위치와 속도는 현재 필터 시각과 같은 ENU 기준으로 넣어야 한다.

## 결과 읽기

```python
from navigation.run_eskf import run_eskf, result_to_dataframe

result = run_eskf()
# result = run_eskf(use_velocity=True)
df = result_to_dataframe(result)
```

`p_e_m`, `p_n_m`, `p_u_m`은 보정 후 추정 위치이고, `gps_e_m`, `gps_n_m`, `gps_u_m`은 보정에 넣은 OXTS 위치를 ENU로 변환한 값이다. `time_s`는 시작 이후 경과 시간이다.

나머지는 `v_*_mps`, `q_w/x/y/z`, `ba_*_mps2`, `bg_*_radps`, `var_*`, `innovation_*`로 저장한다. `var_*`는 공분산 대각 성분이다. 전체 공분산은 `result['P']`에 남겨뒀다. 첫 프레임은 초기화만 하므로 innovation이 NaN이다.

ENU 원점은 `result['origin_llh']`와 DataFrame attrs에 있다. CSV에는 attrs가 저장되지 않으므로 다른 세션의 좌표와 합칠 때는 원점도 별도로 저장해야 한다.

## 확인한 결과와 남은 점

`0005` 시퀀스 154프레임을 실행했다. 위치만 보정했을 때 OXTS 입력 위치와의 차이는 다음과 같다.

| 방향 | RMSE [m] |
|---|---:|
| East | 0.062 |
| North | 0.151 |
| Up | 0.096 |
| 수평 거리 | 0.163 |
| 3D 거리 | 0.189 |

수평·3D RMSE는 프레임마다 거리의 제곱을 구한 뒤 평균하고 제곱근을 취했다.

이 값은 독립적인 정답에 대한 정확도가 아니다. OXTS는 이미 GPS/IMU 융합 출력이고, 비교에 쓴 위치를 필터 보정에도 넣었기 때문이다. 속도 보정도 같은 장비 출력을 다시 쓰는 것이어서 기본 실행에서는 꺼뒀다.

현재는 synced 데이터의 프레임 간격으로 적분한다. 원본 고주파 IMU 전체를 사용하는 구성은 아니고 센서별 시각 차이도 따로 보정하지 않는다. GPS 안테나와 IMU 사이 lever arm은 0으로 가정했다. 빠른 회전이나 긴 시간 간격에서의 적분 정확도, GPS 이상치, 지연 측정, GPS 중단 시 드리프트는 더 확인해야 한다.

Perception 연결 시에는 `inv(loader.calib.T_velo_imu)`로 LiDAR 점을 IMU 좌표로 옮긴 뒤, 추정한 `q`, `p`로 ENU에 배치할 수 있다. Local planning에 전달할 장애물과 목표 경로의 좌표계·시각을 맞추는 것이 다음 작업이다.
