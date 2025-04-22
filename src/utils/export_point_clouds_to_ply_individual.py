import os

import open3d as o3d


def export_point_clouds_to_ply_individual(point_clouds: list, output_dir='./results/point_clouds_individual'):
    os.makedirs(output_dir, exist_ok=True)

    for i, pcd in enumerate(point_clouds):
        individual_output_path = os.path.join(
            output_dir, f'point_cloud_{i}.ply')
        print(f"Full output path: {individual_output_path}")
        o3d.io.write_point_cloud(individual_output_path, pcd)
        print(f"Point cloud {i} exported to: {individual_output_path}")


def export_point_clouds_by_point_count(point_clouds: list, output_dir='./results/point_clouds_by_count'):

    small_threshold = 500
    medium_threshold = 1000

    base_dir = output_dir
    dir_index = 1

    while os.path.exists(output_dir):
        output_dir = f"{base_dir}_{dir_index}"
        dir_index += 1

    small_count = 0
    medium_count = 0
    large_count = 0

    small_planes = []
    medium_planes = []
    large_planes = []

    ordered_point_clouds = sorted(
        point_clouds, key=lambda pcd: len(pcd.points), reverse=True)

    for i, pcd in enumerate(ordered_point_clouds):
        num_points = len(pcd.points)

        if num_points <= small_threshold:
            target_dir = os.path.join(output_dir, 'small_clouds')
            file_prefix = f'small_{small_count // 5}'
            small_planes.append(pcd)
            small_count += 1
        elif num_points <= medium_threshold:
            target_dir = os.path.join(output_dir, 'medium_clouds')
            file_prefix = f'medium_{medium_count // 5}'
            medium_planes.append(pcd)
            medium_count += 1
        else:
            target_dir = os.path.join(output_dir, 'large_clouds')
            file_prefix = f'large_{large_count // 5}'
            large_planes.append(pcd)
            large_count += 1

        nested_dir_index = (small_count if num_points <= small_threshold else
                            medium_count if num_points <= medium_threshold else
                            large_count) // 5

        nested_dir = os.path.join(target_dir, f'group_{nested_dir_index}')
        os.makedirs(nested_dir, exist_ok=True)

        output_path = os.path.join(
            nested_dir, f'{file_prefix}_points_{num_points}.ply')
        o3d.io.write_point_cloud(output_path, pcd)
        print(
            f"Point cloud with {num_points} points exported to: {output_path}")

    print(f"\nExport summary:")
    print(f"- Small point clouds (≤ {small_threshold} points): {small_count}")
    print(
        f"- Medium point clouds ({small_threshold + 1}-{medium_threshold} points): {medium_count}")
    print(f"- Large point clouds (> {medium_threshold} points): {large_count}")
    print(f"- Total: {small_count + medium_count + large_count}")

    return small_planes, medium_planes, large_planes
