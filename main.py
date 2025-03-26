import numpy as np
import open3d as o3d

from src.core.convert_point_cloud_to_mesh import \
    convert_segmented_point_clouds_to_meshes
from src.core.load_point_cloud import load_point_cloud
from src.core.segment_floors import segment_floors
from src.core.segmentation_walls_and_floors import segment_walls_and_floors
from src.utils.origin_translate import origin_translate
from src.utils.scale_point_cloud import scale_point_cloud


def main():
    # input_file = "data/fragment.ply"
    input_file = "data/opt.ply"
    
    pcd = load_point_cloud(input_file)
    voxel_size=0.05
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    pcd_filtered, _ = pcd_down.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    print(f"filtered point count: {len(pcd_filtered.points)}")
    

    pcd_scaled = scale_point_cloud(pcd_filtered, 0.001)
    # pcd_scaled = scale_point_cloud(pcd_filtered, 1.0)
    pcd_origin = origin_translate(pcd_scaled)

    floor_point_clouds = segment_floors(pcd_origin)
    # _, all_direction_clouds = segment_walls_and_floors(floor_point_clouds)

    mesh = convert_segmented_point_clouds_to_meshes([pcd])

    return

if __name__ == "__main__":
    main()
