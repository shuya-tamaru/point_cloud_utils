def calculate_dynamic_sampling_size(point_cloud):
    bbox = point_cloud.get_axis_aligned_bounding_box()
    bbox_diagonal = max(bbox.get_max_bound() - bbox.get_min_bound())
    print(f"bbox_diagonal: {bbox_diagonal}")
    return bbox_diagonal * 0.01
