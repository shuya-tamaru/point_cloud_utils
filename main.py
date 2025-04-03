import open3d as o3d

from src.core.load_point_cloud import load_point_cloud
from src.core.segment_planes import segment_planes
from src.utils.export_point_clouds_to_ply_individual import \
    export_point_clouds_to_ply_individual, export_point_clouds_by_point_count
from src.utils.origin_translate import origin_translate
from src.utils.scale_point_cloud import scale_point_cloud
from src.core.merge_similar_planes import merge_similar_planes
from src.core.assign_unique_color import assign_unique_color
from src.utils.filter_by_point_count import filter_by_point_count
from src.utils.calculate_pcd_size import calculate_pcd_size


def main():
    input_file = "data/point_bi.ply"

    pcd = load_point_cloud(input_file)
    pcd_scaled = scale_point_cloud(pcd, 0.001)
    pcd_origin = origin_translate(pcd_scaled)

    voxel_size = 0.03
    pcd_down = pcd_origin.voxel_down_sample(voxel_size)
    print(f"downsampled point count: {len(pcd_down.points)}")

    pcd_filtered, _ = pcd_down.remove_statistical_outlier(
        nb_neighbors=20, std_ratio=1.0)

    print(f"filtered point count: {len(pcd_filtered.points)}")
    planes = segment_planes(pcd_filtered)

    merge_planes = merge_similar_planes(planes)
    # filtered_planes = filter_by_point_count(merge_planes, min_points=100)
    color_planes = assign_unique_color(merge_planes)
    small_planes, medium_planes, large_planes = export_point_clouds_by_point_count(
        color_planes)
    pcd_filtered.translate((10, 0, 0))
    vis_plane = large_planes + [pcd_filtered]
    vis_plane2 = medium_planes + [pcd_filtered]
    vis_plane3 = small_planes + [pcd_filtered]
    o3d.visualization.draw_geometries(vis_plane)
    o3d.visualization.draw_geometries(vis_plane2)
    o3d.visualization.draw_geometries(vis_plane3)
    return


if __name__ == "__main__":
    main()
