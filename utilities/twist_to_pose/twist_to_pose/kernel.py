import math
from typing import Any, Dict, Optional

from common.polyflow_kernel import PolyflowKernel


def _q_mul(a: Dict[str, float], b: Dict[str, float]) -> Dict[str, float]:
    return {
        "x": a["w"] * b["x"] + a["x"] * b["w"] + a["y"] * b["z"] - a["z"] * b["y"],
        "y": a["w"] * b["y"] - a["x"] * b["z"] + a["y"] * b["w"] + a["z"] * b["x"],
        "z": a["w"] * b["z"] + a["x"] * b["y"] - a["y"] * b["x"] + a["z"] * b["w"],
        "w": a["w"] * b["w"] - a["x"] * b["x"] - a["y"] * b["y"] - a["z"] * b["z"],
    }


def _q_norm(q: Dict[str, float]) -> Dict[str, float]:
    n = (q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2 + q["w"] ** 2) ** 0.5
    if n < 1e-12:
        return {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    return {"x": q["x"] / n, "y": q["y"] / n, "z": q["z"] / n, "w": q["w"] / n}


def _q_from_axis_angle(ax: float, ay: float, az: float, angle: float) -> Dict[str, float]:
    h = angle * 0.5
    s = math.sin(h)
    return {"x": ax * s, "y": ay * s, "z": az * s, "w": math.cos(h)}


def _q_rotate_vec(q: Dict[str, float], vx: float, vy: float, vz: float):
    """Rotate (vx, vy, vz) by quaternion q. Returns [x, y, z]."""
    qx, qy, qz, qw = q["x"], q["y"], q["z"], q["w"]
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return [
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    ]


class TwistToPoseKernel(PolyflowKernel):
    """
    Integrates a Twist (linear/angular velocity) into a Pose.

    Bridges teleop sources (gamepad, joystick) to pose-controlled orchestrators
    (inverse kinematics, mobile-base waypoints). Holds an internal pose,
    integrates the most recent Twist at the configured rate, and emits the
    running pose every tick.

    Subscriptions:
        cmd_vel:    Twist  — latest velocity command (m/s, rad/s).
        seed_pose:  Pose   — anchor the integrator. Must be set once before
                             output is meaningful when require_seed is true.
        enable:     Bool   — deadman gate. While false, velocity is zeroed
                             and (on the falling edge) the pose snaps back
                             to the last seed.

    Publications:
        target_pose: Pose  — the integrated pose, emitted once per tick.

    Parameters:
        rate_hz, frame ("body" | "world"), require_seed, require_enable,
        ws_min_x/y/z, ws_max_x/y/z (optional position clamps).
    """

    def setup(self):
        self.rate_hz = float(self.get_param("rate_hz", 50.0))
        self.dt = 1.0 / max(self.rate_hz, 1e-3)
        self.frame = str(self.get_param("frame", "body"))
        self.require_seed = bool(self.get_param("require_seed", True))
        self.require_enable = bool(self.get_param("require_enable", True))

        self._ws_min = [self.get_param(k) for k in ("ws_min_x", "ws_min_y", "ws_min_z")]
        self._ws_max = [self.get_param(k) for k in ("ws_max_x", "ws_max_y", "ws_max_z")]

        self._position = [0.0, 0.0, 0.0]
        self._orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self._seeded = not self.require_seed
        self._seed_pose: Optional[Dict[str, Any]] = None

        self._linear = [0.0, 0.0, 0.0]
        self._angular = [0.0, 0.0, 0.0]

        self._enabled = not self.require_enable
        self._was_enabled = self._enabled

    def _apply_seed(self, pose: Dict[str, Any]):
        pos = pose.get("position", {}) or {}
        ori = pose.get("orientation", {}) or {"x": 0, "y": 0, "z": 0, "w": 1}
        self._position = [float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), float(pos.get("z", 0.0))]
        self._orientation = _q_norm({
            "x": float(ori.get("x", 0.0)),
            "y": float(ori.get("y", 0.0)),
            "z": float(ori.get("z", 0.0)),
            "w": float(ori.get("w", 1.0)),
        })
        self._seeded = True

    def _clamp_position(self):
        for i in range(3):
            lo, hi = self._ws_min[i], self._ws_max[i]
            if lo is not None and self._position[i] < lo:
                self._position[i] = float(lo)
            if hi is not None and self._position[i] > hi:
                self._position[i] = float(hi)

    def _emit_pose(self):
        self.emit("target_pose", {
            "position": {"x": self._position[0], "y": self._position[1], "z": self._position[2]},
            "orientation": dict(self._orientation),
        })

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "cmd_vel":
            lin = data.get("linear", {}) or {}
            ang = data.get("angular", {}) or {}
            self._linear = [float(lin.get("x", 0.0)), float(lin.get("y", 0.0)), float(lin.get("z", 0.0))]
            self._angular = [float(ang.get("x", 0.0)), float(ang.get("y", 0.0)), float(ang.get("z", 0.0))]

        elif pin_id == "seed_pose":
            # Always store the latest seed so falling-edge resnap uses the
            # current EE pose, but only adopt it as our running pose on the
            # *first* message — otherwise a continuously-emitting source
            # (e.g. an IK current_pose stream) would erase user motion.
            self._seed_pose = data
            if not self._seeded:
                self._apply_seed(data)

        elif pin_id == "enable":
            value = data.get("data", data) if isinstance(data, dict) else data
            now_enabled = bool(value)
            # Falling edge: snap back to seed so re-enable starts where the EE
            # actually is, not where the integrator drifted to.
            if self.require_enable and self._was_enabled and not now_enabled and self._seed_pose is not None:
                self._apply_seed(self._seed_pose)
            self._enabled = now_enabled
            self._was_enabled = now_enabled

    def poll(self):
        if not self.should_run(trigger_info={"pin_id": "__tick__"}):
            return
        if not self._seeded:
            return

        if self.require_enable and not self._enabled:
            self._emit_pose()
            return

        if self.frame == "body":
            v_world = _q_rotate_vec(self._orientation, *self._linear)
        else:
            v_world = list(self._linear)
        self._position[0] += v_world[0] * self.dt
        self._position[1] += v_world[1] * self.dt
        self._position[2] += v_world[2] * self.dt

        wx, wy, wz = self._angular
        w_mag = (wx * wx + wy * wy + wz * wz) ** 0.5
        if w_mag > 1e-9:
            ax, ay, az = wx / w_mag, wy / w_mag, wz / w_mag
            dq = _q_from_axis_angle(ax, ay, az, w_mag * self.dt)
            self._orientation = _q_norm(_q_mul(self._orientation, dq))

        self._clamp_position()
        self._emit_pose()
