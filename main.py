import open3d as o3d

from src.core.load_point_cloud import load_point_cloud
from src.core.segment_floors import segment_floors
from src.utils.origin_translate import origin_translate
from src.utils.scale_point_cloud import scale_point_cloud


def main():
    # input_file = "data/fragment.ply"
    input_file = "data/point_bi.ply"
    
    pcd = load_point_cloud(input_file)
    
    pcd_filtered, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"filtered point count: {len(pcd_filtered.points)}")

    pcd_scaled = scale_point_cloud(pcd_filtered, 0.001)
    pcd_origin = origin_translate(pcd_scaled)

    floor_point_clouds = segment_floors(pcd_origin)
    o3d.visualization.draw_geometries(list(floor_point_clouds.values()))
    return

if __name__ == "__main__":
    main()