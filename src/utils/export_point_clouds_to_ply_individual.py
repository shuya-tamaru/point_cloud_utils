import os

import open3d as o3d


def export_point_clouds_to_ply_individual(point_clouds: list, output_dir='./results/point_clouds_individual'):
    os.makedirs(output_dir, exist_ok=True)

    for i, pcd in enumerate(point_clouds):
        individual_output_path = os.path.join(output_dir, f'point_cloud_{i}.ply')
        print(f"Full output path: {individual_output_path}")
        o3d.io.write_point_cloud(individual_output_path, pcd)
        print(f"Point cloud {i} exported to: {individual_output_path}")
