"""生成可选的 CAD 纹理诊断模型；几何与碰撞保留，资产不覆盖原件。"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import cv2
import numpy as np


def addSurfaceTexture(source, destination, seed=12):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists():
        raise FileExistsError(destination)
    if source == destination or source.is_relative_to(destination) or destination.is_relative_to(source):
        raise ValueError("destination must be independent of source")
    (destination / "meshes").mkdir(parents=True)
    (destination / "textures").mkdir()
    tree = ET.parse(source / "model.sdf")
    rng = np.random.default_rng(seed)
    # 4 m 周期、62.5 mm 单元；单元内保持常值，图像由渲染器生成而非后加噪声。
    tile = rng.integers(85, 256, (64, 64), dtype=np.uint8)
    texture = cv2.resize(tile, (1024, 1024), interpolation=cv2.INTER_NEAREST)
    triangles = 0
    records = []
    stlType = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    for index, visual in enumerate(tree.findall(".//visual")):
        uri = visual.find("geometry/mesh/uri")
        if uri is None:
            continue
        relative = uri.text.split("/", 3)[-1]
        path = source / relative
        with path.open("rb") as stream:
            stream.seek(80)
            count = int.from_bytes(stream.read(4), "little")
            data = np.fromfile(stream, dtype=stlType, count=count)
        if len(data) != count:
            raise ValueError(f"truncated STL: {path}")
        positions = data["vertices"].reshape(-1, 3)
        # 顶点仅按位置去重；每个三角面仍保留原 STL 法向与独立 UV。
        vertices, inverse = np.unique(positions, axis=0, return_inverse=True)
        normals = data["normal"]
        axis = np.argmax(np.abs(normals), axis=1)
        uv = np.empty((count, 3, 2), dtype=np.float32)
        for normalAxis, axes in ((0, (1, 2)), (1, (0, 2)), (2, (0, 1))):
            selected = axis == normalAxis
            uv[selected] = data["vertices"][selected][..., list(axes)] / 4.0
        obj = destination / "meshes" / f"visual_{index:03d}.obj"
        with obj.open("w") as stream:
            for vertex in vertices:
                stream.write("v " + " ".join(format(float(x), ".9g") for x in vertex) + "\n")
            for value in uv.reshape(-1, 2):
                stream.write("vt " + " ".join(format(float(x), ".9g") for x in value) + "\n")
            for normal in normals:
                stream.write("vn " + " ".join(format(float(x), ".9g") for x in normal) + "\n")
            faces = inverse.reshape(-1, 3) + 1
            for faceIndex, indices in enumerate(faces):
                refs = [f"{int(vertex)}/{faceIndex*3+j+1}/{faceIndex+1}" for j,vertex in enumerate(indices)]
                stream.write("f " + " ".join(refs) + "\n")
        color = visual.findtext("material/diffuse", "1 1 1 1").split()
        rgb = np.asarray([float(v) for v in color[:3]], dtype=np.float32)
        image = np.rint(texture[..., None] * rgb).clip(0,255).astype(np.uint8)
        texturePath = destination / "textures" / f"visual_{index:03d}.png"
        if not cv2.imwrite(str(texturePath), image[..., ::-1]):
            raise RuntimeError(f"cannot write texture: {texturePath}")
        material = visual.find("material")
        if material is not None:
            visual.remove(material)
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = "1 1 1 1"
        ET.SubElement(material, "diffuse").text = "1 1 1 1"
        metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
        ET.SubElement(metal, "albedo_map").text = texturePath.as_uri()
        ET.SubElement(metal, "metalness").text = "0"
        ET.SubElement(metal, "roughness").text = "1"
        uri.text = obj.as_uri()
        # 反向索引必须逐位还原原始 float32 顶点；OBJ 的 9 位有效数字可往返 float32。
        assert np.array_equal(vertices[inverse], positions)
        records.append(dict(source=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            vertex_sha256=hashlib.sha256(positions.tobytes()).hexdigest(),
                            triangles=count, unique_vertices=len(vertices), obj=obj.name))
        triangles += count
    # 碰撞网格直接复制，保留模型 URI 及源文件字节。
    for collision in tree.findall(".//collision/geometry/mesh/uri"):
        relative = collision.text.split("/", 3)[-1]
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / relative, target)
        assert hashlib.sha256(target.read_bytes()).digest() == hashlib.sha256((source/relative).read_bytes()).digest()
    shutil.copyfile(source / "model.config", destination / "model.config")
    tree.write(destination / "model.sdf", encoding="utf-8", xml_declaration=True)
    manifest = dict(scope="procedural_texture_diagnostic_only", seed=seed, tile_period_m=4,
                    source_model=str(source), source_sdf_sha256=hashlib.sha256((source/"model.sdf").read_bytes()).hexdigest(),
                    visual_triangles=triangles, geometry_changed=False, collision_changed=False,
                    appearance_changed=True, geometry=records,
                    files={str(p.relative_to(destination)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in sorted(destination.rglob("*")) if p.is_file()})
    (destination / "texture-manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest


def validateTextureModel(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    manifest = json.loads((destination / "texture-manifest.json").read_text())
    if (manifest.get("scope") != "procedural_texture_diagnostic_only" or
            manifest.get("geometry_changed") is not False or manifest.get("collision_changed") is not False):
        raise ValueError("invalid texture diagnostic scope")
    if manifest["source_sdf_sha256"] != hashlib.sha256((source / "model.sdf").read_bytes()).hexdigest():
        raise ValueError("texture model source differs from arena")
    if not manifest.get("files"):
        raise ValueError("texture model requires generated file hashes")
    for relative, expected in manifest["files"].items():
        path = (destination / relative).resolve()
        if not path.is_relative_to(destination) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f"texture asset differs: {relative}")
    for collision in ET.parse(source / "model.sdf").findall(".//collision/geometry/mesh/uri"):
        relative = collision.text.split("/", 3)[-1]
        if (source / relative).read_bytes() != (destination / relative).read_bytes():
            raise ValueError(f"collision differs: {relative}")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--seed", type=int, default=12)
    args = parser.parse_args()
    result = addSurfaceTexture(args.source, args.out, args.seed)
    print(json.dumps({key: value for key, value in result.items() if key not in ("geometry", "files")}, indent=2))


if __name__ == "__main__":
    main()
