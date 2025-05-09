import open3d as o3d

from .segment_planes import segment_planes
from ..utils.export_point_clouds_to_ply_individual import export_point_clouds_by_point_count, export_point_clouds_to_ply_individual
from .assign_unique_color import assign_unique_color
from .setup_pcd import setup_pcd
from .cleanup_planes import cleanup_planes


def building_segmentation(config):
    input_file_path = config["input_file_path"]
    use_original_color = config.get("b_use_original_color", True)
    visualize = config.get("b_visualize_planes_by_size", True)
    scale_factor = config.get("b_scale_factor", 1.0)
    voxel_size = config.get("b_voxel_size", 0.05)
    translate_center = config.get("b_translate_center", True)
    output_dir = config.get("b_output_dir", "./results/point_clouds_by_size")
    nn = config.get("b_nn", 40)
    std_multiplier = config.get("b_std_multiplier", 2.5)
    min_points = config.get("b_min_points", 500)
    distance_threshold = config.get("b_distance_threshold", 0.02)
    dbscan_eps = config.get("b_dbscan_eps", 0.4)
    dbscan_min_points = config.get("b_dbscan_min_points", 10)
    small_threshold = config.get("b_small_folder_threshold", 500)
    medium_threshold = config.get("b_medium_folder_threshold", 10000)

    pcd_optimize = setup_pcd(input_file_path=input_file_path, scale_factor=scale_factor,
                             voxel_size=voxel_size, translate_center=translate_center, nn=nn, std_multiplier=std_multiplier)

    planes = segment_planes(
        pcd=pcd_optimize,
        use_original_color=use_original_color,
        min_points=min_points,
        distance_threshold=distance_threshold,
        dbscan_eps=dbscan_eps,
        dbscan_min_points=dbscan_min_points
    )

    cleaned_up_planes = cleanup_planes(planes)

    export_planes = (
        assign_unique_color(cleaned_up_planes)
        if not use_original_color else
        cleaned_up_planes
    )
    small, medium, large = export_point_clouds_by_point_count(
        point_clouds=export_planes,
        output_dir=output_dir,
        small_threshold=small_threshold,
        medium_threshold=medium_threshold
    )

    if visualize:
        pcd_optimize.translate((10, 0, 0))
        o3d.visualization.draw_geometries(large + [pcd_optimize])
        o3d.visualization.draw_geometries(medium + [pcd_optimize])
        o3d.visualization.draw_geometries(small + [pcd_optimize])
