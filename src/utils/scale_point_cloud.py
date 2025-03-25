
import numpy as np
import open3d as o3d


def scale_point_cloud(pcd,scaleFactor):
    pcd_scaled = o3d.geometry.PointCloud()
    pcd_scaled.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) * scaleFactor)
   
    if hasattr(pcd, 'colors') and len(pcd.colors) > 0:
        pcd_scaled.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))
    
    return pcd_scaled