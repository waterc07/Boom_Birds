"""拆包第二批的结构断言：迁移到位、兼容转发、且生产包不依赖 sim。"""

import importlib
from pathlib import Path

import pytest

MOVED = ["sitl_truth_source", "sitl_hold_relay", "vio_source", "synthetic", "pointcloud_scene", "deep_checks"]
PKG_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PKG_ROOT.parent


@pytest.mark.parametrize("name", MOVED)
def test_module_lives_in_sim_package(name):
    impl = importlib.import_module(f"boom_birds_sim.{name}")
    assert impl.__file__.endswith(f"boom_birds_sim/{name}.py")


@pytest.mark.parametrize("name", MOVED)
def test_nav_shim_forwards_the_same_objects(name):
    impl = importlib.import_module(f"boom_birds_sim.{name}")
    shim = importlib.import_module(f"boom_birds_nav.{name}")
    for attr in dir(impl):
        if attr.startswith("_"):
            continue
        if getattr(impl, attr).__class__.__name__ == "module":
            continue
        assert getattr(shim, attr) is getattr(impl, attr), f"{name}.{attr} 不是同一个对象"
    assert shim.__doc__ == impl.__doc__


def test_sim_package_does_not_depend_on_nav():
    """方向必须是 control ← 本包，而不是本包 → nav（拆包时写反的元数据）。

    这条断言原来是 ``assert "<depend>boom_birds_nav</depend>" in xml``：它把旧包名
    依赖当成规格锁住，代价是 ``--packages-up-to boom_birds_nav`` 只构建 nav，得到
    一个能构建、但 import 全断的安装树（第六批实测，第一版 CI 就是这么写的）。
    """
    pkg_dir = Path(__file__).resolve().parents[1]
    xml = (pkg_dir / "package.xml").read_text(encoding="utf-8")
    assert "<depend>boom_birds_nav</depend>" not in xml, "不得依赖上层转发包 nav"
    assert "<depend>boom_birds_control</depend>" in xml, "必须声明基础层 control"

def test_production_packages_do_not_depend_on_sim():
    """提示词 D：生产包不能依赖 sim 包。只扫生产源码，不扫测试。"""
    offenders = []
    for pkg in ("boom_birds_nav", "boom_birds_control"):
        base = SRC_ROOT / pkg / pkg
        for path in base.glob("*.py"):
            text = path.read_text(encoding="utf-8")
            # 兼容转发层本身必须 import sim，那是它的职责；只查非转发模块。
            if "兼容转发：" in text.split("\n")[0]:
                continue
            if "boom_birds_sim" in text:
                offenders.append(f"{pkg}/{path.name}")
    assert not offenders, f"生产包依赖了 sim 包：{offenders}"
