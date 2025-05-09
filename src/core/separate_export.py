from ..utils.export_point_clouds_to_ply_individual import export_point_clouds_to_ply_individual
from ..utils.export_ply_by_color import export_ply_by_color


def separate_export(config):
    input_file_path = config["input_file_path"]
    output_dir = config["se_output_dir"]
    convert_unit_factor = config["se_convert_unit_factor"]

    segmented_planes = export_ply_by_color(input_file_path)
    export_point_clouds_to_ply_individual(
        segmented_planes, output_dir, convert_unit_factor=convert_unit_factor)
