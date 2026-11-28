"""从原始 STL 生成 27UC 显示网格，保留碰撞网格。"""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

FACET = np.dtype([
    ("normal", "<f4", (3,)),
    ("position", "<f4", (3, 3)),
    ("attribute", "<u2"),
])
BASE = [(10.50, -1.25, -1, 12.65, 1.25, 4)]
OUTPOST = [(2.72, 3.22, -1, 3.55, 4.50, 4)]
ARMOR = [
    (11.74, -.14, .48, 12.10, .14, 1.15),
    (11.36, -.24, .97, 11.81, .24, 1.17),
    (11.29, .16, .49, 11.56, .47, 1.14),
    (11.29, -.47, .49, 11.56, -.16, 1.14),
    (2.80, 3.53, 1.32, 3.13, 3.80, 1.61),
    (2.80, 3.99, 1.42, 3.13, 4.26, 1.70),
    (3.25, 3.72, 1.22, 3.48, 4.05, 1.50),
]
GREEN_LAMPS = [
    (11.43, -.095, 1.15, 11.56, .095, 1.33),
    (2.87, 3.81, 1.60, 3.00, 3.99, 1.78),
]
STRIP_FRAMES = [
    (2.925, 3.83, .74, 3.02, 3.94, 1.42),
    (3.04, 3.66, .74, 3.16, 3.78, 1.42),
    (3.04, 4.00, .74, 3.16, 4.09, 1.42),
]
# Bounds in the northeast tower frame. Mirror XY and offset for the southwest.
SHAFT_STRIPS = [
    ((2.956, 3.860, .785, 2.994, 3.907, 1.378), 0, -1),
    ((3.077, 3.694, .785, 3.124, 3.747, 1.382), 1, -1),
    ((3.077, 4.035, .785, 3.124, 4.059, 1.375), 1, 1),
]
STRIP_GROUPS = {"visual_005.stl": 1, "visual_043.stl": -1}
SHAFT_OFFSET_M = .012


def mirrorBox(box):
    x0, y0, z0, x1, y1, z1 = box
    return -x1, -y1, z0, -x0, -y0, z1


def intersectsBoxes(lower, upper, boxes):
    mask = np.zeros(len(lower), dtype=bool)
    for *start, x1, y1, z1 in boxes:
        mask |= np.all((lower <= (x1, y1, z1)) & (upper >= start), axis=1)
    return mask


def mirrored(boxes):
    return boxes + [mirrorBox(box) for box in boxes]


def compactTerrainVisuals(terrainDir, world, out, padXY):
    model = ET.parse(terrainDir / "model.sdf").getroot().find("model")
    visualDir = out / "terrain-visuals"
    visualDir.mkdir()
    px, py = padXY
    landingBox = (px - .60, py - .60, -1, px + .60, py + .60, 4)
    energyBox = (-1.65, -1.65, -1, 1.65, 1.65, 4)
    for visual in model.findall("./link/visual"):
        uri = visual.find("geometry/mesh/uri")
        source = terrainDir / "meshes" / Path(uri.text).name
        original = np.memmap(source, mode="r", dtype=FACET, offset=84)
        vertices = original["position"]
        lower = vertices.min(axis=1)
        upper = vertices.max(axis=1)
        bases = intersectsBoxes(lower, upper, mirrored(BASE))
        outposts = intersectsBoxes(lower, upper, mirrored(OUTPOST))
        landing = intersectsBoxes(lower, upper, [landingBox])
        detailed = bases | outposts | landing | intersectsBoxes(lower, upper, [energyBox])

        u = (vertices[:, :, 0] + vertices[:, :, 1]) * np.float32(.707106781)
        v = (vertices[:, :, 0] - vertices[:, :, 1]) * np.float32(.707106781)
        rotor = (
            (u.min(1) <= .96) & (u.max(1) >= -.96)
            & (v.min(1) <= .55) & (v.max(1) >= -.55)
            & (lower[:, 2] <= 3.52) & (upper[:, 2] >= 1.77)
        )
        rear = rotor & (np.abs(v).max(1) < .292)
        red, green, blue = map(float, visual.findtext("material/diffuse").split()[:3])
        redBlue = max(red, blue) >= .45 and max(red, blue) - max(green, min(red, blue)) >= .28
        coloredStrips = redBlue & (((bases | outposts) & (upper[:, 2] >= .30)) | rotor)
        features = intersectsBoxes(lower, upper, mirrored(ARMOR + GREEN_LAMPS + STRIP_FRAMES))
        oneCm = (bases | outposts | rear) & detailed & ~landing & ~features & ~coloredStrips
        twoCm = (len(original) > 50_000) & ~detailed
        grid = np.where(oneCm, 100., 50.).astype(np.float32)
        quantized = oneCm | twoCm
        stripSide = STRIP_GROUPS.get(source.name)
        if not quantized.any() and stripSide is None:
            continue

        selected = vertices[quantized]
        scale = grid[quantized, None, None]
        snapped = np.rint(selected * scale) / scale
        normals = np.cross(snapped[:, 1] - snapped[:, 0], snapped[:, 2] - snapped[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        valid = lengths > 1.e-10
        keep = np.ones(len(original), dtype=bool)
        keep[quantized] = valid
        if not keep.any():
            continue
        compact = original[keep].copy()
        changed = quantized[keep]
        compact["position"][changed] = snapped[valid]
        compact["normal"][changed] = normals[valid] / lengths[valid, None]

        if stripSide is not None:
            # Original colored shaft faces intersect the tower shell by up to 10 mm.
            for box, axis, sign in SHAFT_STRIPS:
                bounds = box if stripSide == 1 else mirrorBox(box)
                strip = intersectsBoxes(lower, upper, [bounds])
                if not strip.any() or (strip & quantized).any():
                    raise ValueError(f"Unexpected shaft strip geometry in {source}")
                compact["position"][strip[keep], :, axis] += np.float32(SHAFT_OFFSET_M * sign * stripSide)

        target = visualDir / source.name
        with target.open("wb") as stream:
            stream.write(bytes(80))
            stream.write(struct.pack("<I", len(compact)))
            compact.tofile(stream)
        uri.text = str(target)

    world.remove(world.find("include"))
    world.append(model)
