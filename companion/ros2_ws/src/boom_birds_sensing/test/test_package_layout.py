"""拆包第三批的结构断言：迁移到位、兼容转发、生产包不依赖 sim。"""

import ast
import importlib
from pathlib import Path

import pytest

MOVED = ["stereo_source", "stereo_capture", "camera_timestamp", "timebase", "depth_node",
         "depth_core", "pose_adapter", "mavlink_imu_node", "mavlink_imu_core",
         "mavlink_clock", "mavlink_imu_replay"]
PKG_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PKG_ROOT.parent


@pytest.mark.parametrize("name", MOVED)
def test_module_lives_in_sensing_package(name):
    impl = importlib.import_module(f"boom_birds_sensing.{name}")
    assert impl.__file__.endswith(f"boom_birds_sensing/{name}.py")


@pytest.mark.parametrize("name", MOVED)
def test_nav_shim_forwards_the_same_objects(name):
    impl = importlib.import_module(f"boom_birds_sensing.{name}")
    shim = importlib.import_module(f"boom_birds_nav.{name}")
    for attr in dir(impl):
        if attr.startswith("_") or getattr(impl, attr).__class__.__name__ == "module":
            continue
        assert getattr(shim, attr) is getattr(impl, attr), f"{name}.{attr} 不是同一个对象"


def test_sensing_package_does_not_depend_on_nav():
    """方向必须是 control ← 本包，而不是本包 → nav（拆包时写反的元数据）。

    这条断言原来是 ``assert "<depend>boom_birds_nav</depend>" in xml``：它把旧包名
    依赖当成规格锁住，代价是 ``--packages-up-to boom_birds_nav`` 只构建 nav，得到
    一个能构建、但 import 全断的安装树（第六批实测，第一版 CI 就是这么写的）。
    """
    pkg_dir = Path(__file__).resolve().parents[1]
    xml = (pkg_dir / "package.xml").read_text(encoding="utf-8")
    assert "<depend>boom_birds_nav</depend>" not in xml, "不得依赖上层转发包 nav"
    assert "<depend>boom_birds_control</depend>" in xml, "必须声明基础层 control"

def test_sensing_does_not_depend_on_sim():
    for path in (PKG_ROOT / "boom_birds_sensing").glob("*.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            imports = ([node.module] if isinstance(node, ast.ImportFrom) else
                       [alias.name for alias in node.names] if isinstance(node, ast.Import) else [])
            assert not any(name and name.startswith("boom_birds_sim") for name in imports), path.name


def test_production_source_rejects_synthetic_mode_without_opening_capture():
    from boom_birds_sensing.stereo_source import StereoSourceNode
    node = object.__new__(StereoSourceNode)
    node.mode = "synth"
    with pytest.raises(RuntimeError, match="synthetic_stereo_source"):
        node._setup_source()
