"""Run with python -m navigation.run_eskf or python navigation/run_eskf.py."""
import sys
from pathlib import Path

# Direct file execution puts navigation/, rather than the project root,
# on sys.path. Resolve sibling modules relative to this file, not the cwd.
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pymap3d as pm
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

from load_file import KittiLoader
from navigation.eskf import ESKF


def llh2enu(llh: np.ndarray, origin_llh: np.ndarray) -> np.ndarray:
    """Convert (..., 3) [latitude deg, longitude deg, altitude m] to ENU m.

    Keep origin_llh fixed at the first GPS measurement for the entire drive.
    """
    llh = np.asarray(llh, dtype=float)
    origin = np.asarray(origin_llh, dtype=float)
    if llh.ndim < 1 or llh.shape[-1] != 3 or not np.all(np.isfinite(llh)):
        raise ValueError('llh must be finite with shape (..., 3)')
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError('origin_llh must be a finite 3-vector')
    east, north, up = pm.geodetic2enu(
        llh[..., 0], llh[..., 1], llh[..., 2], *origin,
    )
    return np.stack([east, north, up], axis=-1)


def run_eskf(loader=None, *, use_velocity=False):
    """Return histories; OXTS velocity correction is optional (vendor fused data).

    Noise values below are starting examples, not calibrated sensor parameters.
    """
    if loader is None:
        loader = KittiLoader()
    first = loader.get_frame(0)
    origin_llh = first.gps_lla.copy()

    p0 = np.zeros(3)
    v0 = np.array([first.oxts.ve, first.oxts.vn, first.oxts.vu])
    # SciPy defaults to xyzw; reorder explicitly for ESKF's wxyz convention.
    q_xyzw = R.from_euler('xyz', first.rpy).as_quat()
    q0 = q_xyzw[[3, 0, 1, 2]]
    ba0 = np.zeros(3)
    bg0 = np.zeros(3)

    # Initial standard deviations: p, v, attitude error, ba, bg.
    std0 = np.r_[
        [3.0, 3.0, 5.0],
        [1.0, 1.0, 1.0],
        np.deg2rad([3.0, 3.0, 10.0]),
        [0.1, 0.1, 0.1],
        np.deg2rad([0.5, 0.5, 0.5]),
    ]
    eskf = ESKF(p0, v0, q0, ba0, bg0, np.diag(std0**2))

    # Continuous-time noise densities: accel, gyro, ba walk, bg walk.
    imu_noise_std = np.r_[
        np.full(3, 0.1), np.full(3, 0.01),
        np.full(3, 0.001), np.full(3, 0.0001),
    ]
    gnss_std = np.array([3.0, 3.0, 5.0])
    if use_velocity:
        gnss_std = np.r_[gnss_std, [0.5, 0.5, 1.0]]
    eskf.noise_covariance(np.diag(imu_noise_std**2), np.diag(gnss_std**2))

    states = [eskf.state()]
    gps_positions = [llh2enu(first.gps_lla, origin_llh)]
    times = [first.elapsed_s]
    innovations = []
    for frame in loader.iter_frames(start=1):
        eskf.predict(frame.gyro, frame.accel, frame.dt_s)
        position = llh2enu(frame.gps_lla, origin_llh)
        velocity = (
            np.array([frame.oxts.ve, frame.oxts.vn, frame.oxts.vu])
            if use_velocity else None
        )
        corrected = eskf.update(position, velocity)
        states.append(corrected)
        gps_positions.append(position)
        times.append(frame.elapsed_s)
        innovations.append(corrected['innovation'])

    return {
        **{key: np.stack([state[key] for state in states])
           for key in ('p', 'v', 'q', 'ba', 'bg', 'P')},
        'time_s': np.asarray(times),
        'gps_enu': np.asarray(gps_positions),
        'origin_llh': origin_llh,
        # Innovations correspond to frames 1..N-1, not the initialization frame.
        'innovation': np.asarray(innovations).reshape(-1, 6 if use_velocity else 3),
    }


def result_to_dataframe(result):
    """Flatten histories to one row per frame; retain origin in df.attrs.

    ENU positions are meters, velocities m/s, biases m/s² and rad/s.
    Quaternion order is wxyz. Innovation at initialization is NaN because
    frame zero initializes the filter and has no measurement update.
    Full covariance remains available in result['P']; export its diagonal.
    """
    columns = {'time_s': result['time_s']}
    groups = {
        'p': ['p_e_m', 'p_n_m', 'p_u_m'],
        'gps_enu': ['gps_e_m', 'gps_n_m', 'gps_u_m'],
        'v': ['v_e_mps', 'v_n_mps', 'v_u_mps'],
        'q': ['q_w', 'q_x', 'q_y', 'q_z'],
        'ba': ['ba_x_mps2', 'ba_y_mps2', 'ba_z_mps2'],
        'bg': ['bg_x_radps', 'bg_y_radps', 'bg_z_radps'],
    }
    for key, names in groups.items():
        for i, name in enumerate(names):
            columns[name] = result[key][:, i]
    covariance_names = [
        'p_e', 'p_n', 'p_u', 'v_e', 'v_n', 'v_u',
        'theta_x', 'theta_y', 'theta_z', 'ba_x', 'ba_y', 'ba_z',
        'bg_x', 'bg_y', 'bg_z',
    ]
    diagonal = np.diagonal(result['P'], axis1=1, axis2=2)
    for i, name in enumerate(covariance_names):
        columns[f'var_{name}'] = diagonal[:, i]
    innovation_names = [
        'innovation_p_e_m', 'innovation_p_n_m', 'innovation_p_u_m',
        'innovation_v_e_mps', 'innovation_v_n_mps', 'innovation_v_u_mps',
    ]
    for i in range(result['innovation'].shape[1]):
        columns[innovation_names[i]] = np.r_[np.nan, result['innovation'][:, i]]
    df = pd.DataFrame(columns)
    df.index.name = 'frame'
    df.attrs['origin_llh'] = result['origin_llh'].tolist()
    return df


if __name__ == '__main__':
    result = run_eskf()
    result_df = result_to_dataframe(result)
    output_dir = Path(__file__).resolve().parents[1] / 'results' / 'navigation'
    output_dir.mkdir(parents=True, exist_ok=True)
    result_df.to_csv(output_dir / 'result.csv')
    print(result_df.head())
    print(f"Processed {len(result['time_s'])} frames")
    print('Final ESKF position [ENU m]:', result['p'][-1])
    print('Final GPS position [ENU m]:', result['gps_enu'][-1])
