'''전처리, 지면 제거, 군집화'''

import sys
from pathlib import Path

# Direct file execution puts navigation/, rather than the project root,
# on sys.path. Resolve sibling modules relative to this file, not the cwd.
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import open3d as o3d
from load_file import KittiLoader


class LiDARProcessor:
    def __init__(self):
        self.voxel_size = 0.15
        self.eps = 0.6
        self.min_points = 10

    # 1. 로더에서 읽은 점 배열을 Open3D 포인트클라우드로 변환
    def load_point_cloud(self, points):
        """Convert a KittiFrame.points array (N, 4) to an Open3D cloud."""
        # KITTI: x, y, z, intensity
        xyz = points[:, :3]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    # 2. 전처리
    def preprocess(self, pcd):
        points = np.asarray(pcd.points)

        # KITTI Velodyne 좌표계:
        # x: 전방, y: 좌측, z: 위쪽
        mask = (
            (points[:, 0] > 0) &
            (points[:, 0] < 50) &
            (points[:, 1] > -20) &
            (points[:, 1] < 20) &
            (points[:, 2] > -2.5) &
            (points[:, 2] < 2.0)
        )

        indices = np.where(mask)[0]
        pcd = pcd.select_by_index(indices)

        # Voxel Downsampling
        pcd = pcd.voxel_down_sample(
            voxel_size=self.voxel_size
        )

        return pcd

    # 3. 지면 제거 (RANSAC)
    def remove_ground(self, pcd):
        if len(pcd.points) < 3:
            return pcd, o3d.geometry.PointCloud()

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.2,
            ransac_n=3,
            num_iterations=1000
        )

        ground = pcd.select_by_index(inliers)
        non_ground = pcd.select_by_index(
            inliers, invert=True
        )

        normal = np.asarray(plane_model[:3])
        tilt_deg = np.degrees(
            np.arccos(np.clip(abs(normal[2]) / np.linalg.norm(normal), 0, 1))
        )
        print(f'지면 평면 기울기: {tilt_deg:.1f}')

        print("Plane:", plane_model)
        return non_ground, ground

    # 4. 군집화 (DBSCAN)
    def clustering(self, pcd):
        if len(pcd.points) == 0:
            return np.array([], dtype=int)

        labels = np.array(
            pcd.cluster_dbscan(
                eps=self.eps,
                min_points=self.min_points,
                print_progress=False
            )
        )

        cluster_count = len(set(labels)) - (
            1 if -1 in labels else 0
        )
        print("Clusters:", cluster_count)

        return labels

    def extract_boxes(self, pcd, labels):
        boxes = []
        for label in np.unique(labels):
            if label == -1:
                continue

            indices = np.flatnonzero(labels == label)
            cluster = pcd.select_by_index(indices)
            box = cluster.get_axis_aligned_bounding_box()
            box.color = (1, 0, 0)
            boxes.append(box)

        return boxes

    # 5. 군집화 시각화
    def visualize_clusters(self, pcd, labels):
        if len(labels) == 0:
            print("No points to visualize.")
            return

        max_label = labels.max()
        colors = np.zeros((len(labels), 3))

        if max_label >= 0:
            import matplotlib.pyplot as plt

            cmap = plt.get_cmap("tab20")

            for label in range(max_label + 1):
                colors[labels == label] = (
                    cmap(label % 20)[:3]
                )

        # Noise(-1)는 회색
        colors[labels == -1] = [0.4, 0.4, 0.4]

        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    processor = LiDARProcessor()

    loader = KittiLoader()
    frame = loader.get_frame(0, lidar=True)

    # Load
    pcd = processor.load_point_cloud(frame.points)

    # Preprocessing
    pcd = processor.preprocess(pcd)

    # Ground Removal
    non_ground, ground = processor.remove_ground(pcd)

    # Clustering
    labels = processor.clustering(non_ground)

    # Visualization
    # processor.visualize_clusters(non_ground, labels)
    boxes = processor.extract_boxes(non_ground, labels)
    # o3d.visualization.draw_geometries([non_ground, *boxes])
    o3d.visualization.draw_geometries([ground])
    o3d.visualization.draw_geometries([non_ground])

