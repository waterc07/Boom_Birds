"""显示 Gazebo 下视 Tag、局部特征和任务 3 的轨迹对照。"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

SRC = Path(__file__).resolve().parents[1] / "src"


class TrackingPreview:
    def __init__(self):
        for name in ("boom_birds_sensing", "boom_birds_control"):
            sys.path.insert(0, str(SRC / name))
        from boom_birds_sensing.platform_feature_track import BoardFeatureTrack
        from boom_birds_sensing.platform_observation import BoardDetector

        sim = SRC / "boom_birds_sim/config"
        geometry = yaml.safe_load((sim / "aircraft_340g.yaml").read_text())
        profile = yaml.safe_load((sim / "platform_landing_70mm_test.yaml").read_text())
        fx, fy, cx, cy = geometry["downward"]["intrinsics"]
        profile["camera"]["K"] = [[fx, 0., cx], [0., fy, cy], [0., 0., 1.]]
        for index in range(3):
            profile["camera"]["T_body_camera"][index][3] = geometry["downward"]["pose"][index]
        self.detector = BoardDetector(profile, test_only=True)
        self.tracker = BoardFeatureTrack(self.detector)
        self.previousPoints = {}
        self.lastSource = None

    def render(self, gray, stamp):
        candidates, rejected = self.detector.findCorners(gray)
        observation = self.detector.estimate(candidates, stamp, "gazebo_camera", rejected)
        # Prefer propagation across frames to show measured LK tracks. Decode can reseed the tracker.
        tracked = (self.tracker.update(gray, stamp) if self.tracker.previous is not None
                   else None)
        if tracked is None or not tracked["valid"]:
            tracked = self.tracker.seedDecoded(gray, candidates, stamp)
            self.previousPoints = {}
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for identity, corners in candidates:
            if identity not in self.detector.tags:
                continue
            polygon = np.rint(corners).astype(np.int32)
            cv2.polylines(image, [polygon], True, (60, 225, 255), 2, cv2.LINE_AA)
            for point in polygon:
                cv2.circle(image, tuple(point), 4, (60, 225, 255), -1, cv2.LINE_AA)
            anchor = tuple(polygon[np.argmin(polygon[:, 1])] + [10, -8])
            cv2.putText(image, f"ID {identity}", anchor, cv2.FONT_HERSHEY_SIMPLEX,
                        .62, (60, 225, 255), 2, cv2.LINE_AA)
        self.lastSource = tracked["source"] if tracked is not None and tracked["valid"] else None
        self.drawTracks(image, tracked)
        detail = "Tag pose valid" if observation.valid else f"Tag: {observation.reason.replace('_', ' ')}"
        count = len(self.previousPoints)
        cv2.rectangle(image, (8, 8), (420, 42), (19, 25, 30), -1)
        cv2.putText(image, f"{detail}   |   {count} features", (18, 31),
                    cv2.FONT_HERSHEY_SIMPLEX, .54, (245, 245, 245), 1, cv2.LINE_AA)
        return image

    def drawTracks(self, image, tracked):
        if tracked is None or not tracked["valid"]:
            self.previousPoints.clear()
            return
        points = {identity: tuple(np.rint(point).astype(int))
                  for identity, point in zip(tracked["point_ids"], tracked["image_points"])}
        for identity, point in points.items():
            if identity in self.previousPoints:
                cv2.line(image, self.previousPoints[identity], point, (255, 220, 0), 1, cv2.LINE_AA)
            cv2.circle(image, point, 3, (255, 220, 0), -1, cv2.LINE_AA)
        landing = tuple(np.rint(tracked["landing_pixel"]).astype(int))
        cv2.drawMarker(image, landing, (75, 240, 75), cv2.MARKER_CROSS, 20, 2, cv2.LINE_AA)
        self.previousPoints = points


class FrontPreview:
    def __init__(self):
        self.previous = self.points = None

    def update(self, gray):
        view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if self.previous is not None and self.points is not None:
            points, valid, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, self.points, None)
            if points is not None and valid is not None:
                for old, new in zip(self.points[valid[:, 0] == 1], points[valid[:, 0] == 1]):
                    start = tuple(np.rint(old[0]).astype(int))
                    end = tuple(np.rint(new[0]).astype(int))
                    cv2.line(view, start, end, (255, 220, 0), 1, cv2.LINE_AA)
                    cv2.circle(view, end, 2, (255, 220, 0), -1, cv2.LINE_AA)
        self.previous = gray
        self.points = cv2.goodFeaturesToTrack(gray, 80, .015, 9)
        if self.points is not None:
            for point in self.points:
                xy = tuple(np.rint(point[0]).astype(int))
                cv2.circle(view, xy, 2, (95, 240, 90), -1, cv2.LINE_AA)
        return view


def caption(image, text, x, y, color=(216, 228, 240), size=.50):
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)


def cameraTile(frame, title, width, height):
    tile = np.full((height, width, 3), (26, 30, 38), dtype=np.uint8)
    caption(tile, title, 9, 20)
    if frame is None:
        caption(tile, "Waiting for camera", 12, height//2)
        return tile
    ratio = min(width/frame.shape[1], (height-27)/frame.shape[0])
    scaled = cv2.resize(frame, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
    h, w = scaled.shape[:2]
    x = (width-w)//2
    tile[26:26+h, x:x+w] = scaled
    return tile


def yawOf(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class TrajectoryPreview:
    def __init__(self, runDirectory, node):
        from gz.msgs10.pose_pb2 import Pose

        self.truth = deque(maxlen=200)
        self.history = deque(maxlen=8000)
        self.runDirectory = runDirectory
        self.reference = self.stream = self.route = None
        self.pending = None
        if not node.subscribe(Pose, "/model/aircraft_340g/pose", self.receiveTruth):
            raise RuntimeError("could not subscribe to Gazebo model truth")

    def receiveTruth(self, pose):
        stamp = pose.header.stamp.sec + pose.header.stamp.nsec*1e-9
        if self.truth and stamp <= self.truth[-1][0]:
            return
        xyz = pose.position.x, pose.position.y, pose.position.z
        q = pose.orientation
        self.truth.append((stamp, xyz, yawOf((q.x, q.y, q.z, q.w))))

    def update(self):
        path = self.runDirectory / "vio-trajectory.jsonl"
        if self.stream is None:
            if not path.exists():
                return
            self.stream = path.open()
        truth = tuple(self.truth)
        if len(truth) < 2:
            return
        routePath = self.runDirectory / "arena-route.json"
        if self.route is None and routePath.exists():
            try:
                self.route = np.asarray(json.loads(routePath.read_text()))
            except json.JSONDecodeError:
                pass
        times = np.asarray([sample[0] for sample in truth])
        while True:
            if self.pending is None:
                offset = self.stream.tell()
                line = self.stream.readline()
                if not line.endswith("\n"):
                    self.stream.seek(offset)
                    break
                self.pending = json.loads(line)
            sample = self.pending
            stamp = sample["stamp"]
            if stamp > times[-1]:
                break
            self.pending = None
            index = int(np.searchsorted(times, stamp))
            if index == 0 or times[index] - times[index-1] > .15:
                continue
            left, right = truth[index-1], truth[index]
            t = (stamp-left[0])/(right[0]-left[0])
            measured = (1-t)*np.asarray(left[1])+t*np.asarray(right[1])
            xy = np.asarray(sample["position"])
            lastVisual = sample["last_visual_stamp"]
            fresh = lastVisual is not None and -.004 <= stamp-lastVisual <= .50
            if self.reference is None:
                if not fresh:
                    continue
                yaw = left[2] + t*math.remainder(right[2]-left[2], 2*math.pi)
                a = yaw-yawOf(sample["quaternion"])
                c, s = math.cos(a), math.sin(a)
                self.reference = xy, measured, np.array(((c,-s,0),(s,c,0),(0,0,1)))
            origin, target, rotation = self.reference
            estimate = rotation @ (xy-origin) + target
            self.history.append((stamp, measured, estimate, np.linalg.norm(measured-estimate), fresh))

    def draw(self, width):
        self.update()
        canvas = np.full((420, width, 3), (24, 28, 35), dtype=np.uint8)
        caption(canvas, "POSITION   |   GAZEBO / OPENVINS", 12, 20)
        if not self.history:
            caption(canvas, "Waiting for time-aligned position", 12, 48)
            return canvas
        rows = tuple(self.history)
        stride = max(1, len(rows)//650)
        sampled = (*rows[::stride], rows[-1])
        truth = np.asarray([p[1][:2] for p in sampled])
        vio = np.asarray([p[2][:2] for p in sampled])
        frame = np.vstack((truth, vio, self.route)) if self.route is not None else np.vstack((truth, vio))
        low, high = frame.min(axis=0), frame.max(axis=0)
        center = (low+high)/2
        scale = min((width-40)/max(1., high[0]-low[0]), 135/max(1., high[1]-low[1]))
        def pixels(xy):
            return np.rint((xy-center)*[scale,-scale]+[width/2,124]).astype(np.int32)
        if self.route is not None:
            cv2.polylines(canvas, [pixels(self.route)], False, (78, 86, 97), 1, cv2.LINE_AA)
        for data, color in ((truth, (255, 194, 81)), (vio, (80, 158, 255))):
            points = pixels(data)
            cv2.polylines(canvas, [points], False, color, 1, cv2.LINE_AA)
            cv2.circle(canvas, tuple(points[-1]), 3, color, -1, cv2.LINE_AA)
        caption(canvas, "Gazebo truth", 12, 211, (255, 194, 81), .44)
        caption(canvas, "OpenVINS", 148, 211, (80, 158, 255), .44)
        caption(canvas, "visual update" if rows[-1][4] else "IMU propagation", 270, 211, size=.44)
        t = rows[-1][0]
        caption(canvas, f"t {t:.1f} s   |   XYZ and 3D error   |   last 30 s", 12, 237, size=.44)
        recent = [row for row in sampled if row[0] >= t-30]
        plotLeft, plotRight = 160, width-13
        gx = plotRight - (t-np.asarray([row[0] for row in recent]))*(plotRight-plotLeft)/30
        error = np.asarray([row[3] for row in recent])
        series = [(name, (np.asarray([row[1][i] for row in recent]),
                          np.asarray([row[2][i] for row in recent])))
                  for i, name in enumerate("XYZ")]
        series.append(("Error", (error,)))
        for index, (name, curves) in enumerate(series):
            top = 247 + 42*index
            lo = min(float(curve.min()) for curve in curves) if index < 3 else 0.
            hi = max(float(curve.max()) for curve in curves)
            cv2.rectangle(canvas, (plotLeft,top), (plotRight,top+34), (75,82,95), 1)
            colors = [(255,194,81), (80,158,255)] if index < 3 else [(194,146,244)]
            caption(canvas, name, 12, top+23, size=.47)
            for j, (curve, color) in enumerate(zip(curves, colors)):
                gy = top + 31 - (curve-lo)*28/max(.02, hi-lo)
                line = np.column_stack((gx, gy)).astype(np.int32)
                cv2.polylines(canvas, [line], False, color, 1, cv2.LINE_AA)
                cv2.circle(canvas, tuple(line[-1]), 2, color, -1, cv2.LINE_AA)
                caption(canvas, f"{curve[-1]:.2f} m", 67, top+13+16*j, color, .41)
        return canvas


def showLive(runDirectory, preview, stereo=False, trajectory=False):
    os.environ.update(GZ_PARTITION=(runDirectory / "partition").read_text().strip(), GZ_IP="127.0.0.1")
    from gz.msgs10 import image_pb2
    from gz.transport13 import Node

    node = Node()
    fronts = {side: FrontPreview() for side in ("left", "right")} if stereo else {}
    latest, rendered, stamps = {}, {}, {}
    snapshot = runDirectory / "downward-tracking.png"

    def receive(side):
        def onImage(msg):
            if (msg.pixel_format_type != image_pb2.L_INT8 or len(msg.data) != msg.height * msg.step
                    or msg.step < msg.width):
                return
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec/1e9
            latest[side] = stamp, gray
        return onImage

    for side in ("downward", *fronts):
        if not node.subscribe(image_pb2.Image, f"/aircraft_340g/{side}/image", receive(side)):
            raise RuntimeError(f"could not subscribe to Gazebo {side} camera")
    publisher = node.advertise("/boom_birds/gui/flight_instruments", image_pb2.Image)
    position = TrajectoryPreview(runDirectory, node) if trajectory else None
    lastSnapshot = -math.inf
    while True:
        for side, (stamp, gray) in tuple(latest.items()):
            if stamp <= stamps.get(side, -math.inf):
                continue
            stamps[side] = stamp
            rendered[side] = (preview.render(gray, stamp) if side == "downward"
                              else fronts[side].update(gray))
            if side == "downward" and preview.lastSource == "local_features" and not snapshot.exists():
                cv2.imwrite(str(snapshot), rendered[side])
        width = 500
        tiles = []
        if stereo:
            tiles.append(np.hstack([cameraTile(rendered.get(side), f"FRONT {side.upper()}", width//2, 185)
                                    for side in fronts]))
        tiles.append(cameraTile(rendered.get("downward"), "DOWNWARD  /  APRILTAG", width, 292 if trajectory else 390))
        if position is not None:
            tiles.append(position.draw(width))
        dashboard = np.vstack(tiles)
        rgb = cv2.cvtColor(dashboard, cv2.COLOR_BGR2RGB)
        msg = image_pb2.Image(width=width, height=rgb.shape[0], step=width*3,
                             pixel_format_type=image_pb2.RGB_INT8, data=rgb.tobytes())
        publisher.publish(msg)
        now = time.monotonic()
        if now-lastSnapshot >= 5 and "downward" in rendered:
            temp = runDirectory / "flight-instruments.tmp.png"
            cv2.imwrite(str(temp), dashboard)
            temp.replace(runDirectory / "flight-instruments.png")
            lastSnapshot = now
        time.sleep(.14)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", nargs="?", type=Path)
    parser.add_argument("--image", type=Path, help="annotate a saved real Gazebo frame")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--stereo", action="store_true", help="also show live left and right feature previews")
    parser.add_argument("--trajectory", action="store_true", help="include Gazebo and OpenVINS trajectory")
    args = parser.parse_args()
    if args.image is not None:
        if args.run_directory is not None or args.out is None:
            parser.error("--image requires --out and is independent of the live session")
        image = cv2.imread(str(args.image), cv2.IMREAD_GRAYSCALE)
        if image is None:
            parser.error(f"could not read {args.image}")
        if not cv2.imwrite(str(args.out), TrackingPreview().render(image, 1.)):
            raise RuntimeError(f"could not write {args.out}")
    elif args.run_directory is None or args.out is not None:
        parser.error("provide a Gazebo run directory or --image and --out")
    else:
        cv2.setNumThreads(1)
        showLive(args.run_directory.resolve(strict=True), TrackingPreview(), args.stereo, args.trajectory)


if __name__ == "__main__":
    main()
