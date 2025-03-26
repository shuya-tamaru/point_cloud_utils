import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.signal import find_peaks

from src.utils.show_hist import show_hist


def segment_floors(pcd):
    points = np.asarray(pcd.points)
    z_values = points[:, 2] 

    # ヒストグラムを作成 bin_edgeはz方向の距離を0.05mで分割した範囲を配列で返す[0.0, 0.05, 0.1, ...]
    # histは各bin_edgeの範囲に含まれる点の数を配列で返す
    hist, bin_edges = np.histogram(z_values, bins=int((max(z_values) - min(z_values)) / 0.06))
    # show_hist(hist, bin_edges)
    # return

    # ヒストグラムのピークを検出 点群全体の1%以上の点が存在し、かつピーク間の距離が30以上のピークつまり(0.05m * 30 = 1.5m)以内は同一のピークとして扱う
    # 一般的な部屋を想定すると2.5m - 3mが天井の高さとして考えられるため、ピーク間の距離が30以上のピークを天井と床のペアとして扱う
    peaks = find_peaks(hist, height=0.005*len(pcd.points), distance=5, prominence=10)

    floor_ceiling_pairs = identify_floor_ceiling_pairs(peaks, bin_edges)
    
    # color_palette = [
    #     [1, 0, 0],    # 赤
    #     [0, 1, 0],    # 緑
    #     [0, 0, 1],    # 青
    #     [1, 1, 0],    # 黄
    #     [1, 0, 1],    # マゼンタ
    #     [0, 1, 1],    # シアン
    #     [0.5, 0.5, 0.5]  # グレー
    # ]
    
    floor_point_clouds = {}
    for floor_id, (floor_z, ceiling_z) in enumerate(floor_ceiling_pairs):
        mask = (z_values >= floor_z) & (z_values <= ceiling_z)

        floor_cloud = o3d.geometry.PointCloud()
        floor_cloud.points = o3d.utility.Vector3dVector(points[mask])
        
        # floor_color = color_palette[floor_id % len(color_palette)]
        # floor_cloud.paint_uniform_color(floor_color)
        
        if hasattr(pcd, 'colors') and len(pcd.colors) > 0:
            colors = np.asarray(pcd.colors)
            floor_cloud.colors = o3d.utility.Vector3dVector(colors[mask])
        
        floor_point_clouds[floor_id] = floor_cloud
    
    print(f"床と天井のペア数: {len(floor_point_clouds)}")
    
    return floor_point_clouds
    

def identify_floor_ceiling_pairs(peaks, bin_edges):
    peak_indices = peaks[0]

    peak_z_values = [(bin_edges[index] + bin_edges[index+1]) / 2 for index in peak_indices]
    
    peak_z_values.sort()
    
    floor_ceiling_pairs = []
        
    i = 0
    while i < len(peak_z_values) - 1:
        floor_z = peak_z_values[i]
        ceiling_z = peak_z_values[i + 1]
        
        height = ceiling_z - floor_z
        if 0.0 <= height <= 5.0:
            floor_ceiling_pairs.append((floor_z, ceiling_z))
            i += 1
        else:
            i += 1
    
    return floor_ceiling_pairs


def analyze_z_distribution(z_values):
    print("Z座標分布の分析:")
    print("最小値:", np.min(z_values))
    print("最大値:", np.max(z_values))
    print("平均値:", np.mean(z_values))
    print("標準偏差:", np.std(z_values))
    
    # ヒストグラムの詳細
    hist, bin_edges = np.histogram(z_values, bins=50)
    plt.figure(figsize=(15, 5))
    plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), align='edge')
    plt.title('Z座標のヒストグラム')
    plt.xlabel('Z座標')
    plt.ylabel('点の数')
    plt.show()