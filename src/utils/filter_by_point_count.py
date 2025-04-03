import numpy as np

def filter_by_point_count(merge_planes, min_points=500):
    filtered_planes = []
    for plane in merge_planes:
        if len(np.asarray(plane.points)) >= min_points:
            filtered_planes.append(plane)
    print(f"Filtered planes count: {len(filtered_planes)}")
    return filtered_planes