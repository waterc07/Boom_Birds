"""旧模块导入与 python -m 入口的公共转发。"""
from importlib import import_module


def forward(namespace, target):
    implementation = import_module(target)
    # 不受实现模块 __all__ 限制；旧入口转发所有公共名。
    namespace.update({name: getattr(implementation, name)
                      for name in dir(implementation) if not name.startswith("_")})
    namespace["__doc__"] = implementation.__doc__
    if namespace["__name__"] == "__main__":
        raise SystemExit(getattr(implementation, "main", lambda: 0)())
