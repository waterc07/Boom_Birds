"""开发工具从工作空间导入，运行库使用已加载的安装前缀。"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
