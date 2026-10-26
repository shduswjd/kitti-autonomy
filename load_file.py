"""Shared KITTI Raw loader for navigation, perception and local planning.

Example:
    loader = KittiLoader()
    for frame in loader.iter_frames():  # GPS/IMU only; no image/LiDAR disk reads
        gps = frame.gps_lla
        accel, gyro = frame.accel, frame.gyro

    frame = loader.get_frame(0, image=True, lidar=True)
    calib = loader.calib  # e.g. K_cam2, T_cam2_velo, T_velo_imu

Sensor values are passed through without filtering or coordinate conversion.
OXTS poses are vendor GPS/IMU estimates, not independent ground truth.
"""

import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import pykitti

DEFAULT_DATA_DIR = Path(__file__).resolve().parent / "data"


@dataclass
class KittiFrame:
    """One synchronized frame; vectors use the original KITTI sensor axes.

    gps_lla: latitude/longitude [degrees], altitude [m].
    accel: ax/ay/az [m/s²]; gyro: wx/wy/wz [rad/s], IMU axes.
    rpy: OXTS roll/pitch/yaw [rad], for initialization/reference.
    image: RGB uint8 (H, W, 3); points: LiDAR (N, 4), x/y/z [m], reflectance.
    timestamp is the OXTS timestamp; sync sensor times are approximate.
    """

    index: int
    timestamp: datetime
    elapsed_s: float
    dt_s: float
    gps_lla: np.ndarray
    accel: np.ndarray
    gyro: np.ndarray
    rpy: np.ndarray
    oxts: Any
    image: np.ndarray | None = None
    points: np.ndarray | None = None


class KittiLoader:
    """Load a sync drive. Indices refer to the full drive, starting at zero.

    calib.T_destination_source maps source coordinates into destination.
    .raw exposes the original pykitti API (other cameras, poses, etc.).
    """

    def __init__(self, base_path=DEFAULT_DATA_DIR, date="2011_09_26", drive="0005"):
        self.base_path = Path(base_path).expanduser().resolve()
        self.date = date
        self.drive = str(drive).zfill(4)
        date_path = self.base_path / date
        drive_path = date_path / f"{date}_drive_{self.drive}_sync"
        for name in ("calib_cam_to_cam.txt", "calib_imu_to_velo.txt", "calib_velo_to_cam.txt"):
            if not (date_path / name).is_file():
                raise FileNotFoundError(f"Calibration file missing: {date_path / name}")
        if not drive_path.is_dir():
            raise FileNotFoundError(f"KITTI drive missing: {drive_path}")
        self.raw = pykitti.raw(str(self.base_path), date, self.drive)
        if not self.raw.timestamps:
            raise ValueError(f"No OXTS timestamps in {drive_path}")
        if len(self.raw.oxts) != len(self.raw.timestamps):
            raise ValueError("OXTS packet count does not match timestamp count")
        # Check IDs, not only counts, so a missing file cannot silently shift frames.
        expected = [f"{i:010d}" for i in range(len(self))]
        if [Path(p).stem for p in self.raw.oxts_files] != expected:
            raise ValueError("OXTS frame IDs must be consecutive from 0000000000")
        self._sensor_ids = {
            "image": [Path(p).stem for p in self.raw.cam2_files],
            "lidar": [Path(p).stem for p in self.raw.velo_files],
        }
        self._sensor_aligned = {key: ids == expected for key, ids in self._sensor_ids.items()}
        if any(b <= a for a, b in zip(self.timestamps, self.timestamps[1:])):
            raise ValueError("OXTS timestamps must be strictly increasing")

    def __len__(self):
        return len(self.raw.timestamps)

    @property
    def calib(self):
        return self.raw.calib

    @property
    def timestamps(self):
        return self.raw.timestamps

    def get_frame(self, index: int, *, image=False, lidar=False) -> KittiFrame:
        """Read GPS/IMU and optional left color camera / LiDAR for one frame.

        dt_s is time since the preceding full-drive frame (zero at index 0).
        """
        if not 0 <= index < len(self):
            raise IndexError(f"Frame {index} outside [0, {len(self)})")
        for sensor, enabled in (("image", image), ("lidar", lidar)):
            if enabled and not self._sensor_aligned[sensor]:
                raise ValueError(f"{sensor} files are missing or not aligned with OXTS frame IDs")
        packet = self.raw.oxts[index].packet
        timestamp = self.timestamps[index]
        return KittiFrame(
            index=index,
            timestamp=timestamp,
            elapsed_s=(timestamp - self.timestamps[0]).total_seconds(),
            dt_s=(timestamp - self.timestamps[index - 1]).total_seconds() if index else 0.0,
            gps_lla=np.array([packet.lat, packet.lon, packet.alt]),
            accel=np.array([packet.ax, packet.ay, packet.az]),
            gyro=np.array([packet.wx, packet.wy, packet.wz]),
            rpy=np.array([packet.roll, packet.pitch, packet.yaw]),
            oxts=packet,
            image=np.array(self.raw.get_cam2(index)) if image else None,
            points=self.raw.get_velo(index) if lidar else None,
        )

    def iter_frames(self, start=0, stop=None, *, image=False, lidar=False) -> Iterator[KittiFrame]:
        """Iterate [start, stop) without skipping IMU samples."""
        if stop is None:
            stop = len(self)
        if not 0 <= start <= stop <= len(self):
            raise ValueError(f"Expected 0 <= start <= stop <= {len(self)}")
        for index in range(start, stop):
            yield self.get_frame(index, image=image, lidar=lidar)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--date", default="2011_09_26")
    parser.add_argument("--drive", default="0005")
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--show", action="store_true", help="Display the left color image")
    parser.add_argument("--lidar", action="store_true", help="Read and summarize LiDAR")
    args = parser.parse_args()
    loader = KittiLoader(args.data_dir, args.date, args.drive)
    frame = loader.get_frame(args.frame, image=args.show, lidar=args.lidar)
    print(f"Drive: {loader.date}/{loader.drive} | Frames: {len(loader)}")
    print(f"Frame: {frame.index} | Time: {frame.timestamp} | dt: {frame.dt_s:.6f} s")
    print(f"GPS [deg, deg, m]: {frame.gps_lla}")
    print(f"Acceleration [m/s²]: {frame.accel}")
    print(f"Angular velocity [rad/s]: {frame.gyro}")
    if frame.points is not None:
        print(f"LiDAR shape: {frame.points.shape}")
    if args.show:
        import matplotlib.pyplot as plt

        plt.imshow(frame.image)
        plt.title(f"KITTI {loader.drive} / frame {frame.index}")
        plt.show()


if __name__ == "__main__":
    main()
