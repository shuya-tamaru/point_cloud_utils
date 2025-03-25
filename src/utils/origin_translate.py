def origin_translate(pcd):
    pcd_center = pcd.get_center()
    pcd.translate(-pcd_center)
    return pcd