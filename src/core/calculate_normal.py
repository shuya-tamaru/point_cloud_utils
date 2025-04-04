import open3d as o3d
import numpy as np


def calculate_normal(pcd, radius_normal=None, max_nn=None, orient_normals_factor=None):
    if radius_normal is None:
        nn_distance = np.mean(pcd.compute_nearest_neighbor_distance())
        radius_normal = nn_distance * 4

    if max_nn is None:
        max_nn = 16

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=max_nn), fast_normal_computation=True)
    pcd.normalize_normals()

    if orient_normals_factor is not None:
        pcd.orient_normals_consistent_tangent_plane(orient_normals_factor)

    return pcd
