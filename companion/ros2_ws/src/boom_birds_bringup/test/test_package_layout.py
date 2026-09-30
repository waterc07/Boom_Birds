"""实现属主、兼容导入和生产包依赖方向。"""

import importlib
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

MOVED = ["lifecycle", "lifecycle_node", "lifecycle_cli", "config_io", "sih_params"]
PKG_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("name", MOVED)
def test_module_lives_in_bringup(name):
    impl = importlib.import_module(f"boom_birds_bringup.{name}")
    owner = "boom_birds_sensing" if name == "config_io" else "boom_birds_bringup"
    assert impl.__file__.endswith(f"{owner}/{name}.py")


@pytest.mark.parametrize("name", MOVED)
def test_nav_shim_forwards_the_same_objects(name):
    impl = importlib.import_module(f"boom_birds_bringup.{name}")
    shim = importlib.import_module(f"boom_birds_nav.{name}")
    for attr in dir(impl):
        if attr.startswith("_") or getattr(impl, attr).__class__.__name__ == "module":
            continue
        assert getattr(shim, attr) is getattr(impl, attr), f"{name}.{attr} 不是同一个对象"


def test_no_package_cycle_with_bringup():
    graph = {}
    for directory in PKG_ROOT.parent.glob("boom_birds_*"):
        manifest = ET.parse(directory / "package.xml").getroot()
        graph[directory.name] = {e.text for e in manifest
            if e.tag in ("depend", "exec_depend") and e.text.startswith("boom_birds_")}
    completed = set()
    def visit(name, stack):
        assert name not in stack, "包依赖环: " + " -> ".join([*stack, name])
        if name in completed: return
        for target in graph.get(name, set()): visit(target, [*stack, name])
        completed.add(name)
    for name in graph: visit(name, [])
    def dependencies(name, visited=None):
        visited = set() if visited is None else visited
        for target in graph.get(name, set()) - visited:
            visited.add(target)
            dependencies(target, visited)
        return visited
    for name in ("boom_birds_control", "boom_birds_sensing", "boom_birds_bringup"):
        assert "boom_birds_sim" not in dependencies(name), name
