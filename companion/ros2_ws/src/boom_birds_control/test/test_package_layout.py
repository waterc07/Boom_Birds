"""拆包第一批的结构断言：迁移到位、兼容转发、且生产包不依赖 sim。"""

import importlib
from pathlib import Path

import pytest

MOVED = ["px4_backend", "px4_frames", "px4_failsafe", "px4_interface_node",
         "control_protocol", "handoff", "sih_guard"]


@pytest.mark.parametrize("name", MOVED)
def test_module_lives_in_control_package(name):
    impl = importlib.import_module(f"boom_birds_control.{name}")
    assert impl.__file__.endswith(f"boom_birds_control/{name}.py")


@pytest.mark.parametrize("name", MOVED)
def test_nav_shim_forwards_the_same_objects(name):
    """兼容入口必须是**转发**，不是第二份实现：对象身份必须一致。"""
    impl = importlib.import_module(f"boom_birds_control.{name}")
    shim = importlib.import_module(f"boom_birds_nav.{name}")
    public = [n for n in dir(impl) if not n.startswith("_")]
    assert public, f"{name} 没有可转发的公共名"
    for attr in public:
        if getattr(impl, attr).__class__.__name__ == "module":
            continue
        assert getattr(shim, attr) is getattr(impl, attr), f"{name}.{attr} 不是同一个对象"


def test_control_package_does_not_depend_on_sim():
    """生产包不得依赖 sim 包（提示词 D）。"""
    for path in (SRC_ROOT := Path(__file__).resolve().parents[1] / "boom_birds_control").glob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert "boom_birds_sim" not in text, f"{path.name} 依赖了 sim 包"


def test_dependency_direction_is_control_then_nav():
    """方向必须反过来：control 是基础层，nav 才是兼容转发层。

    这条断言原来是 ``assert "<depend>boom_birds_nav</depend>" in xml``——它把拆包时
    留在 control 里的旧包名依赖**当成规格**锁住了，于是"谁是底层"在元数据里写反还能
    全绿。实测代价：``--packages-up-to boom_birds_nav`` 只构建 nav，得到一个能构建、
    但 import 全断的安装树（第一版 CI 的包清单就是这么来的）。
    """
    pkg_dir = Path(__file__).resolve().parents[1]
    xml = (pkg_dir / "package.xml").read_text(encoding="utf-8")
    assert "<depend>boom_birds_nav</depend>" not in xml, "control 不得依赖上层转发包 nav"
    nav_xml = (pkg_dir.parent / "boom_birds_nav" / "package.xml").read_text(encoding="utf-8")
    for name in ("boom_birds_control", "boom_birds_sensing", "boom_birds_sim", "boom_birds_bringup"):
        assert f"<depend>{name}</depend>" in nav_xml, f"nav 必须声明它转发的 {name}"
