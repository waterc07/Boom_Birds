import sys
from types import ModuleType
import pytest
from boom_birds_nav._compat import forward


def test_public_exports_ignore_all_and_preserve_module_metadata(monkeypatch):
    impl = ModuleType("compat_fixture", "implementation doc")
    impl.__all__ = ["listed"]
    impl.listed, impl.unlisted, impl._private = object(), object(), object()
    monkeypatch.setitem(sys.modules, impl.__name__, impl)
    namespace = {"__name__": "old_module", "__file__": "old_module.py"}
    forward(namespace, impl.__name__)
    assert namespace["listed"] is impl.listed
    assert namespace["unlisted"] is impl.unlisted
    assert "_private" not in namespace
    assert namespace["__doc__"] == impl.__doc__
    assert namespace["__name__"] == "old_module"
    assert namespace["__file__"] == "old_module.py"


@pytest.mark.parametrize("with_main, expected", [(False, 0), (True, 7)])
def test_main_dispatch_and_exit_code(monkeypatch, with_main, expected):
    impl = ModuleType("compat_main_fixture")
    calls = []
    if with_main:
        def main():
            calls.append(True)
            return 7
        impl.main = main
    monkeypatch.setitem(sys.modules, impl.__name__, impl)
    with pytest.raises(SystemExit) as result:
        forward({"__name__": "__main__"}, impl.__name__)
    assert result.value.code == expected
    assert calls == ([True] if with_main else [])
