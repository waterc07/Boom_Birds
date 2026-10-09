"""nav 兼容入口的结构断言：只转发，不保留第二份实现。"""

from pathlib import Path

NAV_PKG = Path(__file__).resolve().parents[1] / "boom_birds_nav"
SHIM_MARK = "兼容转发：实现已迁至"


def test_every_module_is_either_a_shim_or_the_package_init():
    offenders = []
    for path in sorted(NAV_PKG.glob("*.py")):
        if path.name in ("__init__.py", "_compat.py"):
            continue
        text = path.read_text(encoding="utf-8")
        if SHIM_MARK not in text.splitlines()[0]:
            offenders.append(path.name)
    assert not offenders, f"这些模块仍保留实现（应迁出或改为转发）：{offenders}"


def test_no_duplicate_implementations():
    """转发层不得包含函数/类定义（那会是第二份实现）。"""
    import ast

    bad = []
    for path in sorted(NAV_PKG.glob("*.py")):
        if path.name == "_compat.py":
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        defs = [n.name for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
        if defs:
            bad.append(f"{path.name}: {defs}")
    assert not bad, bad
