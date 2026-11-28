"""按 RMUC2027 世界坐标生成单圈、基地掉头与返库航段。"""

import math

ARENA_CRUISE_SPEED_M_S = 5.0
ARENA_YAW_RATE_RAD_S = .55 * ARENA_CRUISE_SPEED_M_S / .72

ENTRY = (-1.40, 4.00)
WEST_NORTH = (-10.15, 4.00)
WEST_SOUTH = (-10.15, -3.40)
EAST_SOUTH = (10.17, -3.40)
EAST_NORTH = (10.17, 4.00)
RED_BASE = (-11.35, 0.00)


def arenaRoute(spawn, *, baseTurnCheck=False):
    origin = tuple(spawn[:2])
    yaw = spawn[5]
    circuit = [WEST_NORTH, WEST_SOUTH, EAST_SOUTH, EAST_NORTH, ENTRY]
    # Follow one closed circuit before visiting the red base.
    points = [ENTRY]
    if not baseTurnCheck:
        points.extend(circuit)
    points.extend([WEST_NORTH, RED_BASE, WEST_NORTH, ENTRY, origin])
    steps = [{"action": "hold", "duration_s": 1, "require_tag": True}]
    heading = yaw
    c, s = math.cos(yaw), math.sin(yaw)
    for start, end in zip([origin, *points], points):
        raw = math.atan2(end[1]-start[1], end[0]-start[0])
        delta = math.atan2(math.sin(raw-heading), math.cos(raw-heading))
        if start == RED_BASE and delta < 0. and not baseTurnCheck:
            # World ENU positive yaw turns counterclockwise for the return.
            delta += 2.*math.pi
        if abs(delta) > 1.e-6:
            heading += delta
            steps.append({"action": "turn", "yaw_deg": math.degrees(heading-yaw)})
        x, y = end[0]-origin[0], end[1]-origin[1]
        steps.append({"action": "waypoint", "forward_m": c*x+s*y, "left_m": -s*x+c*y})
    return steps + [{"action": "land"}], [origin, *points]


def visualRefusionReset(previous, current, position, velocity):
    """Bound the estimator correction when PX4 restarts visual aiding."""
    counts = tuple((new - old) % 256 for old, new in zip(previous, current))
    xy, z, vxy, vz, yaw = counts
    return (any(counts) and z == yaw == 0 and max(xy, vxy, vz) <= 1
            and math.hypot(*position[:2]) <= .40 and abs(position[2]) <= .02
            and math.hypot(*velocity[:2]) <= .10 and abs(velocity[2]) <= .07)


class ImuTurnRecovery:
    """Short turn continuation using PX4's current inertial prediction."""

    def __init__(self):
        self.start = None
        self.recoveredAt = None
        self.startUpdates = 0
        self.deadline = None

    def update(self, now, *, navigating, visualReady, visualUpdates, safe, moving=False):
        if self.start is None:
            if not navigating or visualReady or not safe:
                return "idle"
            self.start = now
            self.startUpdates = visualUpdates
            self.deadline = now + (3.50 if moving else 2.60)
            return "started"
        if not navigating or not safe:
            return "failed"
        if visualReady:
            if self.recoveredAt is None and now < self.deadline:
                self.recoveredAt = now
            if self.recoveredAt is not None and now < self.deadline+.35:
                if now-self.recoveredAt >= .16 and visualUpdates-self.startUpdates >= 2:
                    self.start = self.recoveredAt = None
                    return "recovered"
                return "scanning"
        else:
            self.recoveredAt = None
        if now >= self.deadline:
            return "failed"
        return "scanning"
