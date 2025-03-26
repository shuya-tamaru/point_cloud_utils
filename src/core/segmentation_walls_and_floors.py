import numpy as np
import open3d as o3d

from src.core.extract_directional_planes import extract_directional_planes
from src.utils.get_normals import get_normals


def segment_walls_and_floors(floor_point_clouds):
    all_direction_clouds = {}
    all_wall_planes = []

    for i, floor in enumerate(floor_point_clouds.values()):
        pcd, normals = get_normals(floor)
        direction_clouds = extract_directional_planes(pcd, normals)
        for direction, cloud in direction_clouds.items():
            all_direction_clouds[f"{direction}_floor{i}"] = cloud
        
        wall_directions = ["east_wall", "west_wall", "north_wall", "south_wall"]
        for direction in wall_directions:
            if direction in direction_clouds:
                wall_cloud = direction_clouds[direction]

                wall_planes = extract_wall_planes_optimized(wall_cloud, direction)
                all_wall_planes.extend(wall_planes)

    o3d.visualization.draw_geometries(list(all_direction_clouds.values()))
    
    o3d.visualization.draw_geometries(all_wall_planes)
    
    return all_wall_planes, all_direction_clouds

    
def extract_wall_planes_optimized(wall_cloud, direction):
    wall_planes = []
    remaining_points = wall_cloud
    total_points = len(np.asarray(wall_cloud.points))
    
    # 方向に基づいてパラメータを最適化
    if direction in ["east_wall", "west_wall"]:
        # X方向の壁は面積が大きいことが多い
        min_points = max(300, int(total_points * 0.1))
        distance_threshold = 0.05
    elif direction in ["north_wall", "south_wall"]:
        # Y方向の壁
        min_points = max(300, int(total_points * 0.1))
        distance_threshold = 0.05
    else:
        # その他の方向
        min_points = max(300, int(total_points * 0.1))
        distance_threshold = 0.05
    
    iteration = 0
    max_iterations = 30
    
    while len(np.asarray(remaining_points.points)) > min_points and iteration < max_iterations:
        plane_model, inliers = remaining_points.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=1000
        )
        
        plane_points = remaining_points.select_by_index(inliers)
        plane_points_count = len(np.asarray(plane_points.points))
        
        # 法線ベクトルを正規化
        [a, b, c, d] = plane_model
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)
        
        # 方向に応じて異なる制約をかける
        if direction in ["east_wall", "west_wall"]:
            # X方向の壁は法線のX成分が大きいはず
            is_valid_direction = abs(normal[0]) > 0.9
        elif direction in ["north_wall", "south_wall"]:
            # Y方向の壁は法線のY成分が大きいはず
            is_valid_direction = abs(normal[1]) > 0.9
        else:
            is_valid_direction = True
        
        # 平面性の評価
        points = np.asarray(plane_points.points)
        distances = np.abs(points[:, 0]*a + points[:, 1]*b + points[:, 2]*c + d) / np.sqrt(a**2 + b**2 + c**2)
        rmse = np.sqrt(np.mean(distances**2))
        
        print(f"方向: {direction}, 検出された平面 - 点数: {plane_points_count}, RMSE: {rmse:.4f}")
        
        # 条件を満たす平面のみ保存
        if (plane_points_count >= min_points and 
            is_valid_direction and 
            rmse <= 0.05):
            
            colored_plane = o3d.geometry.PointCloud(plane_points)
            
            # 方向ごとに色を変える
            if direction == "east_wall":
                color = [1.0, 0.7, 0.7]
            elif direction == "west_wall":
                color = [0.7, 0.0, 0.0]
            elif direction == "north_wall":
                color = [0.7, 1.0, 0.7]
            elif direction == "south_wall":
                color = [0.0, 0.7, 0.0]
            else:
                color = [0.5, 0.5, 0.5]
            
            colored_plane.paint_uniform_color(color)
            wall_planes.append(colored_plane)
            
            print(f"壁として検出: {direction} 壁 #{len(wall_planes)}, 点数: {plane_points_count}")
        
        # 残りの点群を更新
        remaining_points = remaining_points.select_by_index(inliers, invert=True)
        iteration += 1
    
    return wall_planes

