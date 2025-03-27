import numpy as np


def segment_plane_ransac(remaining_points, distance_threshold, direction):
    # ransacで平面を検出。distance_thresholdは平面として認識する距離の閾値
    # plane_modelは平面の方程式の係数
    # inliersは平面に含まれる点のインデックス
    plane_model, inliers = remaining_points.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=1000
    )

    plane_pcd = remaining_points.select_by_index(inliers)
    plane_points = plane_pcd.points
    plane_points_count = len(np.asarray(plane_points))

    # 平面に対する法線取得
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    normalized_vec = normal / np.linalg.norm(normal)

    # 法線方向の成分で有効な添加を判定
    is_valid_direction = check_valid_direction(direction, normalized_vec, threshold=0.8)

    # 平面のRMSEを計算
    points = np.asarray(plane_points)
    distances = np.abs(points[:, 0]*a + points[:, 1]*b +
                       points[:, 2]*c + d) / np.sqrt(a**2 + b**2 + c**2)
    rmse = np.sqrt(np.mean(distances**2))

    return plane_pcd, plane_points, plane_points_count, is_valid_direction, rmse, inliers


def check_valid_direction(direction, normalized_vec, threshold=0.9):
    # 法線方向の成分で有効な添加を判定
    if direction == "x":
        # X方向の壁は法線のX成分が大きいはず
        is_valid_direction = abs(normalized_vec[0]) > threshold
    elif direction == "y":
        # Y方向の壁は法線のY成分が大きいはず
        is_valid_direction = abs(normalized_vec[1]) > threshold
    elif direction == "z":
        # Z方向の壁は法線のZ成分が大きいはず
        is_valid_direction = abs(normalized_vec[2]) > threshold
    else:
        is_valid_direction = True

    return is_valid_direction
