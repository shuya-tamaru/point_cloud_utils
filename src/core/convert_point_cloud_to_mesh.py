import os

import numpy as np
import open3d as o3d


def convert_point_cloud_to_mesh(point_cloud, method='poisson', depth=8, alpha=0.1):
    if not point_cloud.has_normals():
        point_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        point_cloud.orient_normals_consistent_tangent_plane(100)

    if method == 'poisson':
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            point_cloud, depth=depth
        )
        
        vertices_to_remove = densities < np.quantile(densities, 0.05)
        mesh.remove_vertices_by_mask(vertices_to_remove)
    elif method == 'alpha_shape':
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
                point_cloud, alpha=alpha
            )        
    else:
        raise ValueError("method must be 'poisson' or 'alpha_shape'")
    mesh = mesh.simplify_vertex_clustering(
        voxel_size=mesh.get_max_bound().min() * 0.01
    )
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()
    return mesh

def convert_segmented_point_clouds_to_meshes(segmented_point_clouds, output_dir='./results/mesh_outputs', method='poisson'):
    os.makedirs(output_dir, exist_ok=True)
    
    meshes = []
    
    # 点群を辞書として扱う場合と、リストとして扱う場合の両方に対応
    if isinstance(segmented_point_clouds, dict):
        for key, point_cloud in segmented_point_clouds.items():
            # 複数の点群がある場合（壁など）
            if isinstance(point_cloud, list):
                for i, pc in enumerate(point_cloud):
                    mesh = convert_point_cloud_to_mesh(pc, method=method)
                    meshes.append(mesh)
                    
                    # OBJとして出力
                    output_path = os.path.join(output_dir, f'{key}_mesh_{i}.obj')
                    o3d.io.write_triangle_mesh(output_path, mesh)
            else:
                # 単一の点群の場合
                mesh = convert_point_cloud_to_mesh(point_cloud, method=method)
                meshes.append(mesh)
                
                # OBJとして出力
                output_path = os.path.join(output_dir, f'{key}_mesh.obj')
                o3d.io.write_triangle_mesh(output_path, mesh)
    
    elif isinstance(segmented_point_clouds, list):
        for i, point_cloud in enumerate(segmented_point_clouds):
            mesh = convert_point_cloud_to_mesh(point_cloud, method=method)
            meshes.append(mesh)
            
            output_path = os.path.join(output_dir, f'mesh_{i}.obj')
            o3d.io.write_triangle_mesh(output_path, mesh)
    
    return meshes