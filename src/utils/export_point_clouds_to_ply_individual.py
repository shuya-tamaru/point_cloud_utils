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

    small_dir = os.path.join(
        output_dir, f'small_clouds_below_{small_threshold}')
    medium_dir = os.path.join(
        output_dir, f'medium_clouds_{small_threshold + 1}_to_{medium_threshold}')
    large_dir = os.path.join(
        output_dir, f'large_clouds_above_{medium_threshold}')

    os.makedirs(small_dir, exist_ok=True)
    os.makedirs(medium_dir, exist_ok=True)
    os.makedirs(large_dir, exist_ok=True)

    small_count = 0
    medium_count = 0
    large_count = 0

    small_planes = []
    medium_planes = []
    large_planes = []

    for i, pcd in enumerate(point_clouds):
        num_points = len(pcd.points)

        if num_points <= small_threshold:
            target_dir = small_dir
            file_prefix = f'small_{small_count}'
            small_planes.append(pcd)
            small_count += 1
        elif num_points <= medium_threshold:
            target_dir = medium_dir
            file_prefix = f'medium_{medium_count}'
            medium_planes.append(pcd)
            medium_count += 1
        else:
            target_dir = large_dir
            file_prefix = f'large_{large_count}'
            large_planes.append(pcd)
            large_count += 1

        # ファイル名の作成と保存
        output_path = os.path.join(
            target_dir, f'{file_prefix}_points_{num_points}.ply')
        o3d.io.write_point_cloud(output_path, pcd)
        print(
            f"Point cloud with {num_points} points exported to: {output_path}")

    # 結果の要約を表示
    print(f"\nExport summary:")
    print(f"- Small point clouds (≤ {small_threshold} points): {small_count}")
    print(
        f"- Medium point clouds ({small_threshold + 1}-{medium_threshold} points): {medium_count}")
    print(f"- Large point clouds (> {medium_threshold} points): {large_count}")
    print(f"- Total: {small_count + medium_count + large_count}")

    return small_planes, medium_planes, large_planes
