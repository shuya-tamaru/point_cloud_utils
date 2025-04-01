import copy

import numpy as np
import open3d as o3d


def get_normals(point_cloud):
    pcd = copy.deepcopy(point_cloud)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    normals = np.asarray(pcd.normals)
    return pcd, normals