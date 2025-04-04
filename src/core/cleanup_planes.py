def cleanup_planes(planes, nb_neighbors=20, std_ratio=2.0):
    cleaned_planes = []
    for plane in planes:
        if len(plane.points) > nb_neighbors:
            cleaned, _ = plane.remove_statistical_outlier(
                nb_neighbors=nb_neighbors,
                std_ratio=std_ratio
            )
            cleaned_planes.append(cleaned)

    return cleaned_planes
