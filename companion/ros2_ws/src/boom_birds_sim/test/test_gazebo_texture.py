"""纹理派生模型保留顶点、碰撞和原文件，并拒绝损坏资产。"""
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from boom_birds_sim.gazebo_texture import addSurfaceTexture, validateTextureModel


def test_texture_variant_geometry_collision_and_tamper(tmp_path):
    source = tmp_path / "source"
    (source / "meshes").mkdir(parents=True)
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    data = np.zeros(1, dtype=dtype)
    data["normal"][0] = [0, 0, 1]
    data["vertices"][0] = [[0.123456789, 0, 0], [1, 0, 0], [0, 1, 0]]
    original = bytes(80) + (1).to_bytes(4, "little") + data.tobytes()
    (source / "meshes/visual.stl").write_bytes(original)
    (source / "meshes/collision.stl").write_bytes(original)
    (source / "model.config").write_text("<model><name>test</name></model>")
    (source / "model.sdf").write_text('''<sdf version="1.9"><model name="test"><link name="link">
      <visual name="visual"><geometry><mesh><uri>model://test/meshes/visual.stl</uri></mesh></geometry>
      <material><diffuse>1 0.5 0.25 1</diffuse></material></visual>
      <collision name="collision"><geometry><mesh><uri>model://test/meshes/collision.stl</uri></mesh></geometry></collision>
    </link></model></sdf>''')
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob("*") if p.is_file()}
    destination = tmp_path / "textured"
    result = addSurfaceTexture(source, destination)
    assert result["visual_triangles"] == 1
    assert validateTextureModel(source, destination)["geometry_changed"] is False
    obj = (destination / "meshes/visual_000.obj").read_text().splitlines()
    positions = np.asarray([[float(v) for v in line.split()[1:]] for line in obj if line.startswith("v ")], dtype=np.float32)
    face = next(line for line in obj if line.startswith("f ")).split()[1:]
    indices = [int(value.split("/")[0])-1 for value in face]
    np.testing.assert_array_equal(positions[indices], data["vertices"][0])
    assert (destination / "meshes/collision.stl").read_bytes() == original
    assert before == {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob("*") if p.is_file()}
    with pytest.raises(FileExistsError): addSurfaceTexture(source, destination)
    (destination / "meshes/collision.stl").write_bytes(b"changed")
    with pytest.raises(ValueError): validateTextureModel(source, destination)
    with pytest.raises(ValueError): addSurfaceTexture(source, source / "nested")
