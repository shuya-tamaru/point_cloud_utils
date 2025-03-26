import numpy as np
import open3d as o3d


def get_random_color():
    """ランダムな色を生成する"""
    return np.random.random(3)

def visualize_segments(original_pcd, segments):
    """
    セグメンテーション結果を可視化する
    
    Args:
        original_pcd: 元の点群
        segments: セグメント化された点群のリスト
    """
    # 可視化用のジオメトリリスト
    geometries = []
    
    # 各セグメントに色を付けて表示
    for i, segment in enumerate(segments):
        # セグメントに色を割り当て
        segment_colored = segment.clone()
        segment_colored.paint_uniform_color(get_random_color())
        
        # 結果のリストに追加
        geometries.append(segment_colored)
    
    # 残りの点（どのセグメントにも属さない点）
    remaining_points = original_pcd
    for segment in segments:
        segment_points = np.asarray(segment.points)
        original_points = np.asarray(original_pcd.points)
        
        # 非効率だがシンプルな実装（大規模点群では最適化が必要）
        if len(remaining_points.points) > 0:
            remaining_mask = np.ones(len(remaining_points.points), dtype=bool)
            for point in segment_points:
                # 点が近いかどうかをチェック
                distances = np.linalg.norm(np.asarray(remaining_points.points) - point, axis=1)
                close_points = distances < 0.001  # しきい値
                remaining_mask = remaining_mask & ~close_points
            
            # 残りの点を更新
            if np.any(~remaining_mask):
                remaining_points = remaining_points.select_by_index(np.where(remaining_mask)[0])
    
    # 残りの点を灰色で表示
    if len(remaining_points.points) > 0:
        remaining_points.paint_uniform_color([0.5, 0.5, 0.5])
        geometries.append(remaining_points)
    
    # 座標軸を追加
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    geometries.append(coord_frame)
    
    # 可視化
    o3d.visualization.draw_geometries(geometries)

def visualize_planes_with_normals(segments, normals, scale=0.2):
    """
    平面と法線ベクトルを可視化する
    
    Args:
        segments: 平面の点群のリスト
        normals: 法線ベクトルのリスト
        scale: 法線ベクトルの表示スケール
    """
    # 可視化用のジオメトリリスト
    geometries = []
    
    for i, (segment, normal) in enumerate(zip(segments, normals)):
        # セグメントに色を割り当て
        segment_colored = segment.clone()
        color = get_random_color()
        segment_colored.paint_uniform_color(color)
        
        # セグメントの中心を計算
        center = segment_colored.get_center()
        
        # 法線ベクトル可視化用の線分
        points = [center, center + normal * scale]
        lines = [[0, 1]]
        
        # 線分のジオメトリを作成
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        # 法線ベクトルには赤色を使用
        line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
        
        # 結果のリストに追加
        geometries.append(segment_colored)
        geometries.append(line_set)
    
    # 座標軸を追加
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    geometries.append(coord_frame)
    
    # 可視化
    o3d.visualization.draw_geometries(geometries)