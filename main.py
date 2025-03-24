import os

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from open3d.cpu.pybind.geometry import PointCloud

from src.segmentation import segment_point_cloud
from src.visualization import visualize_segments


def main():
    input_file = "data/bunny.ply"
    # input_file = "data/point_bi.ply"
    output_dir = "results"
    try:
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        print(f"point cloud loading...: {input_file}")
        if not os.path.exists(input_file):
            print(f"エラー: ファイル {input_file} が見つかりません。")
            return
        
        pcd = o3d.io.read_point_cloud(input_file)

        print(f"points count: {len(pcd.points)}")
        pcd_center = pcd.get_center()
        pcd.translate(-pcd_center)
        print(f"point center: {pcd_center}")

        nn = 20
        std_multiplier = 2.0

        filtered_pcd = pcd.remove_statistical_outlier(nb_neighbors = nn, std_ratio = std_multiplier)
        outliers = pcd.select_by_index(filtered_pcd[1],invert=True)
        outliers.paint_uniform_color([1,0,0])

        filtered_pcd = filtered_pcd[0]
        filtered_pcd.paint_uniform_color([0,1,0])

        voxel_size = 0.001

        pcd_downsampled = filtered_pcd.voxel_down_sample(voxel_size = voxel_size)

        nn_distance = np.mean(pcd.compute_nearest_neighbor_distance())
        radius_normals = nn_distance * 4
        pcd_downsampled.estimate_normals(
            search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = radius_normals, max_nn = 16),
            fast_normal_computation = True
        )

        pcd_downsampled.paint_uniform_color([0.5,0.5,0.5])

        front = [ -0.51667969318101614, 0.010350475092077739, 0.85611620842017666 ]
        lookat = [ 0.01008188624696018, 0.0129460901813781, -0.0099816412608373477 ]
        up = [ 0.0026773784657139086, 0.9999415671351739, -0.010473488425733157 ]
        zoom = 0.68599999999999994

        pcd = pcd_downsampled

        pt_to_plane = 0.002
        plane_model, inliers = pcd.segment_plane(distance_threshold=pt_to_plane, ransac_n=3, num_iterations=1000)
        [a,b,c,d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers,invert=True)   
        inlier_cloud.paint_uniform_color([0.6,0.6,0.6])
        outlier_cloud.paint_uniform_color([1.0,0.0,0.0])


        max_plane_idx = 6
        pt_to_plane_dist = 0.002

        segment_models = {}
        segments = {}
        rest = pcd

        for i in range(max_plane_idx):
            colors = plt.get_cmap("tab20")(i)
            segment_models[i], inliers = rest.segment_plane(distance_threshold = pt_to_plane_dist, ransac_n = 3, num_iterations = 1000)
            segments[i] = rest.select_by_index(inliers)
            segments[i].paint_uniform_color(list(colors[:3]))
            rest = rest.select_by_index(inliers,invert=True)
            print("pass", i , "/", max_plane_idx, " done.")

        o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)] + [rest],zoom=zoom,front=front,lookat=lookat,up=up)

    except Exception as e:
        print(f"エラー: {e}")
        return

if __name__ == "__main__":
    main()