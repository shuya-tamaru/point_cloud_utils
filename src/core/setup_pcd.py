import open3d as o3d

from ..utils.scale_point_cloud import scale_point_cloud
from ..utils.origin_translate import origin_translate
from .load_point_cloud import load_point_cloud


def setup_pcd(input_file_path: str, scale_factor: float = 1.0, voxel_size=0.01, translate_center=True, nn=40, std_multiplier=2.5) -> o3d.geometry.PointCloud:
    pcd = load_point_cloud(input_file_path)

    if (scale_factor != 1.0):
        pcd = scale_point_cloud(pcd, scale_factor)

    if translate_center:
        pcd = origin_translate(pcd)

    pcd_filtered, inliners = pcd.remove_statistical_outlier(
        nb_neighbors=nn, std_ratio=std_multiplier)

    print(f"filtered point count: {len(pcd_filtered.points)}")
    # outliners = pcd_origin.select_by_index(inliners, invert=True)
    # outliners.paint_uniform_color([1, 0, 0])  # red

    pcd_down = pcd_filtered.voxel_down_sample(voxel_size)
    print(f"downsampled point count: {len(pcd_down.points)}")

    return pcd_down
