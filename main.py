import sys
import yaml

from src.core.all_segmentation import building_segmentation
from src.core.stair_segmentation import stair_segmentation
from src.core.separate_export import separate_export


def load_config(path="config.yaml"):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
    config = load_config(config_path)

    mode = config.get("mode", "building")

    if mode == "stair":
        stair_segmentation(config)
    elif mode == "separate":
        separate_export(config)
    else:
        building_segmentation(config)


if __name__ == "__main__":
    main()
