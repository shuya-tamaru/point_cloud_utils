import open3d as o3d
import numpy as np

from src.core.segment_planes import segment_planes
from src.utils.export_point_clouds_to_ply_individual import \
    export_point_clouds_to_ply_individual, export_point_clouds_by_point_count
from src.core.merge_similar_planes import merge_similar_planes
from src.core.assign_unique_color import assign_unique_color
from src.core.setup_pcd import setup_pcd
from src.core.cleanup_planes import cleanup_planes
from src.core.split_pcd import split_pcd
from src.utils.export_ply import export_ply


def main():
    input_file_path = "data/1F_interior.ply"
    pcd_optimize = setup_pcd(input_file_path)

    o3d.visualization.draw_geometries([pcd_optimize])
    # export_ply(pcd_optimize)

    planes = segment_planes(pcd_optimize)

    merge_planes = merge_similar_planes(planes)
    cleaned_up_planes = cleanup_planes(merge_planes)
    # split_planes = split_pcd(cleaned_up_planes)
    color_planes = assign_unique_color(cleaned_up_planes)
    small_planes, medium_planes, large_planes = export_point_clouds_by_point_count(
        color_planes)
    pcd_optimize.translate((10, 0, 0))

    vis_plane = large_planes + [pcd_optimize]
    vis_plane2 = medium_planes + [pcd_optimize]
    vis_plane3 = small_planes + [pcd_optimize]
    o3d.visualization.draw_geometries(vis_plane)
    o3d.visualization.draw_geometries(vis_plane2)
    o3d.visualization.draw_geometries(vis_plane3)

    return


if __name__ == "__main__":
    main()
