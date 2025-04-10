import os
import open3d as o3d

from src.core.cleanup_planes import cleanup_planes
from src.core.assign_unique_color import assign_unique_color


def export_planes_direction(all_planes: dict[str, list], output_dir='./results/point_clouds_by_direction'):
    base_dir = output_dir
    dir_index = 1

    while os.path.exists(output_dir):
        output_dir = f"{base_dir}_{dir_index}"
        dir_index += 1

    for direction, planes in all_planes.items():
        print(f"Number of {direction} planes: {len(planes)}")
        cleaned_up_planes = cleanup_planes(planes)
        color_planes = assign_unique_color(cleaned_up_planes)
        o3d.visualization.draw_geometries(color_planes)

        output_dir_direction = os.path.join(output_dir, direction)
        os.makedirs(output_dir_direction, exist_ok=True)
        for i, pcd in enumerate(color_planes):
            individual_output_path = os.path.join(
                output_dir_direction, f'point_cloud_{i}_{len(pcd.points)}.ply')
            o3d.io.write_point_cloud(individual_output_path, pcd)
