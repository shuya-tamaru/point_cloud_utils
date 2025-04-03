from src.utils.generate_distinct_color_by_direction import generate_unique_color

def assign_unique_color(planes):
    all_planes = []
    for i, plane in enumerate(planes):
        color = generate_unique_color(i)
        plane.paint_uniform_color(color)
        all_planes.append(plane)
    return all_planes