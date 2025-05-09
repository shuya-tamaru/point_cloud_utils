from src.utils.get_normals import get_normals
from src.core.extract_directional_planes import extract_directional_planes_xyz
from src.core.segment_plane_dbscan import segment_plane_dbscan


def stair_segment(pcd, config):
    x_planes = []
    y_planes = []
    z_planes = []
    unclassified_planes = []

    normal_threshold = config.get("s_normal_threshold", 0.8)
    min_points = config.get("s_filtered_min_points", 100)

    pcd_with_normal, normals = get_normals(pcd)
    direction_clouds = extract_directional_planes_xyz(
        pcd=pcd_with_normal,
        normals=normals,
        normal_threshold=normal_threshold,
        min_points=min_points
    )

    for direction, cloud in direction_clouds.items():
        segmented_dbscan_planes = segment_stair_component(
            pcd=cloud, direction=direction, config=config,)
        if direction == "x":
            x_planes.extend(segmented_dbscan_planes)
        elif direction == "y":
            y_planes.extend(segmented_dbscan_planes)
        elif direction == "z":
            z_planes.extend(segmented_dbscan_planes)
        else:
            unclassified_planes.extend(segmented_dbscan_planes)

    all_planes = {
        "x": x_planes,
        "y": y_planes,
        "z": z_planes,
        "unclassified": unclassified_planes
    }

    return all_planes


def segment_stair_component(pcd, direction, config):
    if (direction == "x"):
        min_points = config.get("s_dbscan_min_points_x", 500)
        distance_threshold = config.get("s_dbscan_eps_x", 0.02)
    elif (direction == "y"):
        min_points = config.get("s_dbscan_min_points_y", 500)
        distance_threshold = config.get("s_dbscan_eps_y", 0.02)
    elif (direction == "z"):
        min_points = config.get("s_dbscan_min_points_z", 500)
        distance_threshold = config.get("s_dbscan_eps_z", 0.02)
    else:
        min_points = config.get("s_dbscan_min_points_unclassified", 500)
        distance_threshold = config.get("s_dbscan_eps_unclassified", 0.02)

    segmented_dbscan_planes = segment_plane_dbscan(
        plane_pcd=pcd, eps=distance_threshold, min_points=min_points)

    return segmented_dbscan_planes
