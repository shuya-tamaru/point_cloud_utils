import os
import open3d as o3d


def export_ply(point_cloud, output_dir='./results/optimize_ply'):
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, 'optimize.ply')
    print(f"Full output path: {out_path}")
    o3d.io.write_point_cloud(out_path, point_cloud)
    return
