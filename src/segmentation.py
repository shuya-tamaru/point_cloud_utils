import numpy as np
import open3d as o3d


def segment_point_cloud(pcd):
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.05,
        ransac_n=3,
        num_iterations=1000
    )
    
    return plane_model, inliers