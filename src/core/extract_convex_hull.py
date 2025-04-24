import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import open3d as o3d
from src.core.load_point_cloud import load_point_cloud


def extract_convex_hull(inputPath):
    # Point cloudの読み込み
    point_cloud = load_point_cloud(inputPath)

    # NumPy配列に変換
    points = np.asarray(point_cloud.points)

    # 凸包の計算
    hull = ConvexHull(points)

    # 凸包の頂点を取得（NumPy配列から）
    hull_points = points[hull.vertices]

    # 3D可視化
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 全点群
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               c='blue', alpha=0.1, label='Point Cloud')

    # 凸包の頂点
    ax.scatter(hull_points[:, 0], hull_points[:, 1], hull_points[:, 2],
               c='red', s=100, label='Convex Hull Vertices')
    # print(hull.simplices)
    # # 凸包の辺を描画
    # for simplex in hull.simplices:
    #     print(simplex)
    #     simplex_points = hull_points[simplex]
    #     ax.plot3D(simplex_points[:, 0],
    #               simplex_points[:, 1],
    #               simplex_points[:, 2], 'r-')

    ax.set_title('3D Convex Hull Extraction')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    plt.show()

    return hull_points
