import numpy as np
import open3d as o3d
from ..core.calculate_normal import calculate_normal


def convert_unit(pcd, scale_factor):
    # Scale the points
    pcd.points = o3d.utility.Vector3dVector(
        np.asarray(pcd.points) * scale_factor)

    # Retain colors if present
    pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))
    # Recompute normals if present
    pcd = calculate_normal(pcd)
    return pcd
