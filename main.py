import numpy as np
import open3d as o3d

from src.core.load_point_cloud import load_point_cloud
from src.core.segment_floors import segment_floors
from src.core.segmentation_walls_and_floors import segment_walls_and_floors
from src.utils.origin_translate import origin_translate
from src.utils.scale_point_cloud import scale_point_cloud


def main():
    # input_file = "data/fragment.ply"
    input_file = "data/point_bi.ply"
    
    pcd = load_point_cloud(input_file)
    voxel_size=0.05
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    pcd_filtered, _ = pcd_down.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    print(f"filtered point count: {len(pcd_filtered.points)}")
    
    # aligned_pcd = align_point_cloud(pcd_filtered)

    pcd_scaled = scale_point_cloud(pcd_filtered, 0.001)
    pcd_origin = origin_translate(pcd_scaled)

    floor_point_clouds = segment_floors(pcd_origin)
    segment_walls_and_floors(floor_point_clouds)

    return

def check_point_cloud_orientation(pcd):
    points = np.asarray(pcd.points)
    
    # 主成分分析
    mean = np.mean(points, axis=0)
    centered_points = points - mean
    
    # 共分散行列の固有値と固有ベクトル
    cov_matrix = np.cov(centered_points.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    
    # 主成分の表示
    print("主成分:")
    for i, (val, vec) in enumerate(zip(eigenvalues, eigenvectors.T), 1):
        print(f"第{i}主成分:")
        print(f"  固有値: {val}")
        print(f"  固有ベクトル: {vec}")
    
    # 最も分散の大きい方向
    primary_direction = eigenvectors[:, np.argmax(eigenvalues)]
    print("\n最も分散の大きい方向:", primary_direction)
    
    return primary_direction, eigenvalues

def visualize_point_cloud_planes(pcd):
    # 主成分分析
    direction, eigenvalues = check_point_cloud_orientation(pcd)
    
    # 最小二乗平面の推定
    points = np.asarray(pcd.points)
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    
    print("\n推定平面の方程式:")
    print(f"ax + by + cz + d = 0")
    print(f"係数: {plane_model}")
    
    # 平面上の点を抽出
    plane_cloud = pcd.select_by_index(inliers)
    plane_cloud.paint_uniform_color([1, 0, 0])  # 赤色で平面を強調
    
    # 可視化
    o3d.visualization.draw_geometries([pcd, plane_cloud])

def align_point_cloud(pcd):
    # 点群をコピー
    points = np.asarray(pcd.points)
    aligned_pcd = o3d.geometry.PointCloud()
    aligned_pcd.points = o3d.utility.Vector3dVector(points)
    
    # 点群をコピー
    mean = np.mean(points, axis=0)
    
    # 主成分分析
    cov_matrix = np.cov(points.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    
    # Z軸方向に最も近い主成分を使用して回転
    rotation_matrix = np.eye(3)
    rotation_matrix[:3, :3] = eigenvectors
    
    # 点群を回転
    aligned_points = (rotation_matrix @ (points - mean).T).T + mean
    
    aligned_pcd.points = o3d.utility.Vector3dVector(aligned_points)
    
    # 元の色情報があれば引き継ぐ
    if hasattr(pcd, 'colors') and len(pcd.colors) > 0:
        aligned_pcd.colors = pcd.colors
    
    return aligned_pcd

if __name__ == "__main__":
    main()
