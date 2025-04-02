import numpy as np
import open3d as o3d


def merge_similar_planes(planes, normal_threshold=0.95, distance_threshold=0.05):
    # 結合処理を実装
    merged_planes = []
    processed = [False] * len(planes)
    
    for i in range(len(planes)):
        if processed[i]:
            continue
            
        current_plane = planes[i]
        current_points = np.asarray(current_plane.points)

        
        normals = current_plane.normals
        reference_normal = normals[0]
        aligned_normals = np.array([
            normal if np.dot(normal, reference_normal) > 0 else -normal
            for normal in normals
        ])
        current_normal = np.mean(aligned_normals, axis=0)
        current_normal /= np.linalg.norm(current_normal)
        
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(current_points)
        plane_model, _ = temp_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=100)
        a, b, c, d = plane_model 

        merged_points = current_points.copy()
        processed[i] = True
        
        for j in range(i+1, len(planes)):
            if processed[j]:
                continue
                
            plane_j = planes[j]
            points_j = np.asarray(plane_j.points)

            normals_j = np.asarray(plane_j.normals)
            reference_normal_j = normals_j[0]
            aligned_normals_j = np.array([
                normal if np.dot(normal, reference_normal_j) > 0 else -normal
                for normal in normals_j
            ])
            normal_j = np.mean(aligned_normals_j, axis=0)
            normal_j /= np.linalg.norm(normal_j)
            
            dot_product = np.abs(np.dot(current_normal, normal_j))
            
            if dot_product > normal_threshold:
                temp_pcd_j = o3d.geometry.PointCloud()
                temp_pcd_j.points = o3d.utility.Vector3dVector(points_j)
                plane_model_j, _ = temp_pcd_j.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=100)
                a_j, b_j, c_j, d_j = plane_model_j

                center_i = np.mean(current_points, axis=0)
                center_j = np.mean(points_j, axis=0)
                
                distance_i_to_j = np.abs(a_j * center_i[0] + b_j * center_i[1] + c_j * center_i[2] + d_j) / np.sqrt(a_j**2 + b_j**2 + c_j**2)
                distance_j_to_i = np.abs(a * center_j[0] + b * center_j[1] + c * center_j[2] + d) / np.sqrt(a**2 + b**2 + c**2)
                distance = (distance_i_to_j + distance_j_to_i) / 2

                if distance < distance_threshold:
                    merged_points = np.vstack([merged_points, points_j])
                    processed[j] = True
        
        # 結合した点から新しい平面を作成
        new_plane = o3d.geometry.PointCloud()
        new_plane.points = o3d.utility.Vector3dVector(merged_points)
        new_plane.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        merged_planes.append(new_plane)
        
    return merged_planes