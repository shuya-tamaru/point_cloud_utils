import numpy as np
import scipy.ndimage as ndimage


def split_pcd(planes, grid_size=0.04):
    all_planes = []
    for i, plane in enumerate(planes):
        print(f"Processing plane {i + 1}/{len(planes)}")
        segments = segment_plane_by_grid(plane, grid_size=grid_size)
        all_planes.extend(segments)
    return all_planes


def segment_plane_by_grid(plane_points, grid_size=0.05):
    points = np.asarray(plane_points.points)
    plane_model, _ = plane_points.segment_plane(
        distance_threshold=0.01, ransac_n=3, num_iterations=100)
    normal = plane_model[:3]

    if abs(normal[0]) > abs(normal[1]):
        v1 = np.array([-normal[2], 0, normal[0]])
    else:
        v1 = np.array([0, -normal[2], normal[1]])
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(normal, v1)

    origin = np.mean(points, axis=0)
    points_2d = np.zeros((len(points), 2))

    for i, point in enumerate(points):
        points_2d[i, 0] = np.dot(point - origin, v1)
        points_2d[i, 1] = np.dot(point - origin, v2)

    x_min, y_min = np.min(points_2d, axis=0)
    x_max, y_max = np.max(points_2d, axis=0)

    grid_width = int(np.ceil((x_max - x_min) / grid_size)) + 1
    grid_height = int(np.ceil((y_max - y_min) / grid_size)) + 1

    grid = np.zeros((grid_height, grid_width), dtype=bool)
    for px, py in points_2d:
        x_idx = min(grid_width - 1, max(0, int((px - x_min) / grid_size)))
        y_idx = min(grid_height - 1, max(0, int((py - y_min) / grid_size)))
        grid[y_idx, x_idx] = True

    grid = ndimage.binary_dilation(grid, iterations=1)

    grid = ndimage.binary_erosion(grid, iterations=2)
    grid = ndimage.binary_dilation(grid, iterations=1)

    labeled_grid, num_components = ndimage.label(grid)

    point_labels = np.zeros(len(points), dtype=int)
    for i, (px, py) in enumerate(points_2d):
        x_idx = min(grid_width - 1, max(0, int((px - x_min) / grid_size)))
        y_idx = min(grid_height - 1, max(0, int((py - y_min) / grid_size)))
        point_labels[i] = labeled_grid[y_idx, x_idx]

    segments = []
    for label in range(1, num_components + 1):
        indices = np.where(point_labels == label)[0]
        if len(indices) > 100:
            segment = plane_points.select_by_index(indices)
            segments.append(segment)

    return segments
