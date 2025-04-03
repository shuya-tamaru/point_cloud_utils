def calculate_pcd_size(pcd):
    bounding_box = pcd.get_axis_aligned_bounding_box()
    min_bound = bounding_box.min_bound
    max_bound = bounding_box.max_bound
    size = max_bound - min_bound
    print("-------------------------")
    print(f"Bounding Box Size: {size}")
    return