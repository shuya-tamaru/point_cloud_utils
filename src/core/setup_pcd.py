import open3d as o3d
import numpy as np

from ..utils.scale_point_cloud import scale_point_cloud
from ..utils.origin_translate import origin_translate
from .load_point_cloud import load_point_cloud


def setup_pcd(input_file_path: str):
    pcd = load_point_cloud(input_file_path)

    scale_factor = 0.001
    pcd_scaled = scale_point_cloud(pcd, scale_factor)

    pcd_origin = origin_translate(pcd_scaled)

    nn = 40
    std_multiplier = 1.0
    pcd_filtered, inliners = pcd_origin.remove_statistical_outlier(
        nb_neighbors=nn, std_ratio=std_multiplier)
    print(f"filtered point count: {len(pcd_filtered.points)}")

    # outliners = pcd_origin.select_by_index(inliners, invert=True)
    # outliners.paint_uniform_color([1, 0, 0])  # red

    voxel_size = 0.05
    pcd_down = pcd_filtered.voxel_down_sample(voxel_size)
    print(f"downsampled point count: {len(pcd_down.points)}")

    # o3d.visualization.draw_geometries([pcd_down])
    return pcd_down
