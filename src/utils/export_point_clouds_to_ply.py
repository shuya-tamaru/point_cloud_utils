import os

import open3d as o3d


def export_point_clouds_to_ply(point_clouds: list, output_dir='./results/point_clouds'):
    os.makedirs(output_dir, exist_ok=True)

    merged_pcd = o3d.geometry.PointCloud()
    output_path = os.path.join(output_dir, 'merged_point_cloud.ply')
    print(f"Full output path: {output_path}")

    for pcd in point_clouds:
        merged_pcd.points.extend(pcd.points)

        if pcd.has_colors():
            merged_pcd.colors.extend(pcd.colors)

    o3d.io.write_point_cloud(output_path, merged_pcd)
    print(f"Merged point cloud exported to: {output_path}")
    return
