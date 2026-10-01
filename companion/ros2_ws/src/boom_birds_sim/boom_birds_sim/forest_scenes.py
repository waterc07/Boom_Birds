"""加载显式森林布局；不覆盖控制阈值。"""
from pathlib import Path
import math
import yaml

FIELDS = {"forest_obs_num", "forest_circle_num", "forest_x_size", "forest_center_x"}


def load_profile(name, path=None):
    if path is None:
        path = Path(__file__).resolve().parents[1] / "config/forest_scenes.yaml"
        if not path.is_file():
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory("boom_birds_sim")) / "config/forest_scenes.yaml"
    profiles = yaml.safe_load(Path(path).read_text())
    if name not in profiles:
        raise ValueError(f"未知森林布局：{name}")
    values = profiles[name]
    if set(values) != FIELDS:
        raise ValueError(f"森林布局字段不匹配：{name}")
    for key in ("forest_obs_num", "forest_circle_num"):
        if type(values[key]) is not int or values[key] < 0:
            raise ValueError(key)
    for key in ("forest_x_size", "forest_center_x"):
        if isinstance(values[key], bool) or not math.isfinite(values[key]):
            raise ValueError(key)
    if values["forest_x_size"] <= 0:
        raise ValueError("forest_x_size")
    return dict(values)
