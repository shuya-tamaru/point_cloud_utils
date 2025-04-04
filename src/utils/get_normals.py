import copy

import numpy as np
from ..core.calculate_normal import calculate_normal


def get_normals(point_cloud):
    pcd = copy.deepcopy(point_cloud)
    pcd = calculate_normal(pcd)
    normals = np.asarray(pcd.normals)
    return pcd, normals
