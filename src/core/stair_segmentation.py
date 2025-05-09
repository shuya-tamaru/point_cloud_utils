from .setup_pcd import setup_pcd
from .stair_segment import stair_segment
from ..utils.export_planes_direction import export_planes_direction


def stair_segmentation(config):
    input_file_path = config["input_file_path"]
    scale_factor = config.get("s_scale_factor", 1.0)
    voxel_size = config.get("s_voxel_size", 0.05)
    translate_center = config.get("s_translate_center", True)
    output_dir = config.get("s_output_dir", "./results/point_clouds_by_size")

    pcd_optimize = setup_pcd(input_file_path=input_file_path, scale_factor=scale_factor,
                             voxel_size=voxel_size, translate_center=translate_center)

    all_planes = stair_segment(pcd_optimize, config)
    export_planes_direction(all_planes=all_planes, output_dir=output_dir)
