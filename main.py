import numpy as np
import open3d as o3d

from src.core.convert_point_cloud_to_mesh import \
    convert_segmented_point_clouds_to_meshes
from src.core.load_point_cloud import load_point_cloud
from src.core.segment_floors import segment_floors
from src.core.segment_planes import segment_planes
from src.core.segmentation_walls_and_floors import segment_walls_and_floors
from src.utils.calculate_dynamic_sampling_size import \
    calculate_dynamic_sampling_size
from src.utils.export_point_clouds_to_ply import export_point_clouds_to_ply
from src.utils.export_point_clouds_to_ply_individual import \
    export_point_clouds_to_ply_individual
from src.utils.origin_translate import origin_translate
from src.utils.scale_point_cloud import scale_point_cloud


def main():
    # input_file = "data/fragment.ply"
    input_file = "data/opt.ply"

    pcd = load_point_cloud(input_file)

    pcd_scaled = scale_point_cloud(pcd, 0.001)

    pcd_origin = origin_translate(pcd_scaled)

    voxel_size = 0.05
    pcd_down = pcd_origin.voxel_down_sample(voxel_size)
    print(f"downsampled point count: {len(pcd_down.points)}")


    pcd_filtered, _ = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    print(f"filtered point count: {len(pcd_filtered.points)}")
    # floor_point_clouds = segment_floors(pcd_origin)
    planes = segment_planes(pcd_filtered)
    # o3d.visualization.draw_geometries(planes)
    # all_wall_planes, all_direction_clouds = segment_walls_and_floors(
    #     floor_point_clouds)
    # export_point_clouds_to_ply(planes)
    export_point_clouds_to_ply_individual(planes)
    o3d.visualization.draw_geometries(planes)
    return

    # mesh = convert_segmented_point_clouds_to_meshes(planes)
    # mesh = convert_segmented_point_clouds_to_meshes([pcd_origin])

    return


if __name__ == "__main__":
    main()
