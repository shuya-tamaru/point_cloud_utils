import os

import open3d as o3d


def load_point_cloud(input_file: str):
    print(f"point cloud loading...: {input_file}")

    if not os.path.exists(input_file):
        print(f"Error: File {input_file} is not found.")
        return

    pcd = o3d.io.read_point_cloud(input_file)
    print(f"raw data point count: {len(pcd.points)}")

    return pcd
