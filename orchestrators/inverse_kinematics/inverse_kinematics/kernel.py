import math
from typing import Any, Dict, List, Optional

import numpy as np

from common.polyflow_kernel import PolyflowKernel


def _rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula — axis-angle to 3x3 rotation matrix."""
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def _euler_to_rotation(rx: float, ry: float, rz: float) -> np.ndarray:
    """Convert Euler angles (rx, ry, rz) to a 3x3 rotation matrix (XYZ order)."""
    Rx = _rotation_matrix(np.array([1, 0, 0], dtype=float), rx)
    Ry = _rotation_matrix(np.array([0, 1, 0], dtype=float), ry)
    Rz = _rotation_matrix(np.array([0, 0, 1], dtype=float), rz)
    return Rz @ Ry @ Rx


def _homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a 3x3 rotation and 3-vector."""
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


class InverseKinematicsKernel(PolyflowKernel):
    """
    Portable inverse kinematics solver using Jacobian pseudoinverse.

    Builds a kinematic chain from the robot's RobotComponent[] and Joint[]
    parameters. Joints define the connections between components and carry
    the kinematic properties (type, axis, limits, origin).

    Input pins (as dicts):
        target_pose  — {"position": {"x", "y", "z"}, "orientation": {"x", "y", "z", "w"}}
        joint_states — {"positions": [float, ...]}  (current joint angles feedback)

    Output pins (as dicts):
        joint_commands — {"positions": [float, ...]}  (solved joint angles)

    Parameters:
        root_component_id:    string — _id of the root component for this chain
                           (e.g. "front_left_hip"). The chain is built by walking
                           joints downward from this component to the end-effector.
        components:     RobotComponent[] — auto-injected by studio (hidden).
        joints:         Joint[] — auto-injected by studio (hidden). Each joint has:
                          - joint_type: "Revolute" | "Continuous" | "Prismatic" | "Fixed" | ...
                          - axis: {x, y, z}
                          - origin: {x, y, z, rx, ry, rz}
                          - limit: {lower, upper, velocity, effort}  (optional)
                          - parent: component _id
                          - child: component _id
                          - dynamics: {damping, friction}  (optional)
        max_iterations: int   — solver iteration limit (default: 100)
        tolerance:      float — position tolerance in meters (default: 0.001)
        damping:        float — damping factor for DLS pseudoinverse (default: 0.01)
    """

    def setup(self):
        self.root_component_id: str = self.get_param("root_component_id", "")
        self.end_effector_component_id: str = self.get_param("end_effector_component_id", "")
        self.components: List[Dict[str, Any]] = self.get_param("components", [])
        self.joints: List[Dict[str, Any]] = self.get_param("joints", [])
        self.max_iterations = int(self.get_param("max_iterations", 100))
        self.tolerance = float(self.get_param("tolerance", 0.001))
        self.damping = float(self.get_param("damping", 0.01))

        self._chain = self._build_chain(self.root_component_id, self.end_effector_component_id, self.components, self.joints)
        self._num_joints = len(self._chain)
        self._current_joint_positions: Optional[List[float]] = None

        # Map joint IDs (and names as fallback) to chain indices for fan-in state accumulation
        self._joint_name_to_idx: Dict[str, int] = {}
        for i, joint in enumerate(self._chain):
            if joint.get("_id"):
                self._joint_name_to_idx[joint["_id"]] = i
            self._joint_name_to_idx[joint["name"]] = i

        self.log(f"IK chain from '{self.root_component_id}' to '{self.end_effector_component_id}' with {self._num_joints} actuated joints, {len(self.components)} components, {len(self.joints)} joints")

    def _build_chain(
        self,
        root_component_id: str,
        end_effector_component_id: str,
        components: List[Dict[str, Any]],
        joints: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        """
        Build a kinematic chain from root_component_id to end_effector_component_id.

        Uses BFS to find the unique path through the joint tree, then extracts
        only the actuated joints along that path.
        """
        comp_by_id = {c.get("_id", c.get("component_id", "")): c for c in components}

        if not root_component_id or root_component_id not in comp_by_id:
            self.log(f"Root component '{root_component_id}' not found — chain will be empty")
            return []

        if not end_effector_component_id or end_effector_component_id not in comp_by_id:
            self.log(f"End effector '{end_effector_component_id}' not found — chain will be empty")
            return []

        # Index joints by parent component id
        joints_by_parent: Dict[str, List[Dict[str, Any]]] = {}
        for j in joints:
            pid = j.get("parent", "")
            if pid not in joints_by_parent:
                joints_by_parent[pid] = []
            joints_by_parent[pid].append(j)

        # BFS from root to end-effector, tracking the path
        from collections import deque

        self._debug_log = []
        self._debug_log.append(f"BFS: root={repr(root_component_id)} ee={repr(end_effector_component_id)}")
        self._debug_log.append(f"BFS: joints_by_parent keys={[repr(k) for k in joints_by_parent.keys()]}")
        self._debug_log.append(f"BFS: root in joints_by_parent={root_component_id in joints_by_parent}")
        if root_component_id in joints_by_parent:
            for j in joints_by_parent[root_component_id]:
                self._debug_log.append(f"BFS: root joint child={repr(j.get('child'))}")

        queue: deque = deque([(root_component_id, [])])
        visited = {root_component_id}
        path_joints: Optional[List[Dict[str, Any]]] = None

        while queue:
            comp_id, path = queue.popleft()
            self._debug_log.append(f"BFS visiting: {repr(comp_id)}, path_len={len(path)}")

            if comp_id == end_effector_component_id:
                path_joints = path
                break

            for joint in joints_by_parent.get(comp_id, []):
                child_id = joint.get("child", "")
                if child_id and child_id not in visited:
                    visited.add(child_id)
                    queue.append((child_id, path + [joint]))

        if path_joints is None:
            self.log(f"No path from '{root_component_id}' to '{end_effector_component_id}'")
            return []

        # Extract actuated joints along the path
        chain = []
        for joint in path_joints:
            joint_type = joint.get("joint_type", "fixed").lower()

            if joint_type not in ("revolute", "continuous", "prismatic"):
                continue

            origin = joint.get("origin", {})
            translation = np.array([
                origin.get("x", 0),
                origin.get("y", 0),
                origin.get("z", 0),
            ], dtype=float)
            rotation_euler = np.array([
                origin.get("rx", 0),
                origin.get("ry", 0),
                origin.get("rz", 0),
            ], dtype=float)

            axis_raw = joint.get("axis", {"x": 0, "y": 0, "z": 1})
            axis = np.array([
                axis_raw.get("x", 0),
                axis_raw.get("y", 0),
                axis_raw.get("z", 0),
            ], dtype=float)
            # If axis is all zeros (sparse object with no matching keys), default to Z
            if np.linalg.norm(axis) < 1e-9:
                axis = np.array([0, 0, 1], dtype=float)

            limit = joint.get("limit", {})
            lower = limit.get("lower", -math.pi)
            upper = limit.get("upper", math.pi)
            if joint_type == "continuous":
                lower = -math.pi
                upper = math.pi

            child_id = joint.get("child", "")
            child_comp = comp_by_id.get(child_id, {})
            # Use _id (output ID) as the canonical joint identifier —
            # downstream controllers (e.g. ODrive) match on this, not display name.
            joint_id = joint.get("_id", joint.get("parent_output", ""))
            chain.append({
                "_id": joint_id,
                "name": joint.get("name", child_comp.get("name", "unnamed")),
                "type": joint_type,
                "axis": axis,
                "origin_translation": translation,
                "origin_rotation": rotation_euler,
                "lower": lower,
                "upper": upper,
            })

        return chain

    def _forward_kinematics(self, q: np.ndarray) -> List[np.ndarray]:
        """
        Compute forward kinematics for all joints.

        Each joint's origin (translation + Euler rotation) positions the joint
        frame relative to the parent. The joint variable then actuates about
        the joint axis.

        Returns a list of 4x4 homogeneous transforms (len = num_joints + 1).
        """
        transforms = [np.eye(4)]
        for i, joint in enumerate(self._chain):
            # Static transform from parent to joint frame (from joint origin)
            R_static = _euler_to_rotation(*joint["origin_rotation"])
            T_static = _homogeneous(R_static, joint["origin_translation"])

            # Joint actuation
            if joint["type"] in ("revolute", "continuous"):
                R_joint = _rotation_matrix(joint["axis"], q[i])
                T_joint = _homogeneous(R_joint, np.zeros(3))
            elif joint["type"] == "prismatic":
                T_joint = _homogeneous(np.eye(3), joint["axis"] * q[i])
            else:
                T_joint = np.eye(4)

            transforms.append(transforms[-1] @ T_static @ T_joint)
        return transforms

    def _compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Compute the 6xN geometric Jacobian (linear + angular velocity).

        For revolute joints:  J_i = [z_i × (p_ee - p_i); z_i]
        For prismatic joints: J_i = [z_i; 0]
        """
        transforms = self._forward_kinematics(q)
        p_ee = transforms[-1][:3, 3]

        J = np.zeros((6, self._num_joints))
        for i, joint in enumerate(self._chain):
            T_i = transforms[i + 1]
            z_i = T_i[:3, :3] @ joint["axis"]

            if joint["type"] in ("revolute", "continuous"):
                p_i = T_i[:3, 3]
                J[:3, i] = np.cross(z_i, p_ee - p_i)
                J[3:, i] = z_i
            elif joint["type"] == "prismatic":
                J[:3, i] = z_i

        return J

    def _clamp_joints(self, q: np.ndarray) -> np.ndarray:
        """Clamp joint values to their limits."""
        for i, joint in enumerate(self._chain):
            q[i] = np.clip(q[i], joint["lower"], joint["upper"])
        return q

    def _pose_error(
        self,
        current_T: np.ndarray,
        target_pos: np.ndarray,
        target_quat: Optional[np.ndarray],
    ) -> np.ndarray:
        """
        Compute 6D pose error (position + orientation).

        If no target orientation is given, only position error is used.
        """
        pos_error = target_pos - current_T[:3, 3]

        if target_quat is not None:
            x, y, z, w = target_quat
            R_target = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
            ])
            R_current = current_T[:3, :3]
            R_err = R_target @ R_current.T
            trace = np.clip((np.trace(R_err) - 1) / 2, -1, 1)
            angle = math.acos(trace)
            if abs(angle) < 1e-6:
                orient_error = np.zeros(3)
            else:
                orient_error = (angle / (2 * math.sin(angle))) * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])
        else:
            orient_error = np.zeros(3)

        return np.concatenate([pos_error, orient_error])

    def _solve_ik(self, target: Dict[str, Any], current: List[float]) -> Optional[List[float]]:
        """
        Solve IK via damped least-squares (Levenberg-Marquardt) pseudoinverse.

        Uses J^T (J J^T + λ²I)^{-1} to avoid singularity issues.
        """
        q = np.array(current, dtype=float)

        pos = target.get("position", {})
        target_pos = np.array([pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)])

        orient = target.get("orientation")
        target_quat = None
        if orient:
            # Only enable orientation tracking if a non-identity quaternion is provided
            x, y, z, w = orient.get("x", 0), orient.get("y", 0), orient.get("z", 0), orient.get("w", 1)
            is_identity = abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6 and abs(w - 1) < 1e-6
            if not is_identity:
                target_quat = np.array([x, y, z, w])

        lam2 = self.damping ** 2

        for iteration in range(self.max_iterations):
            transforms = self._forward_kinematics(q)
            error = self._pose_error(transforms[-1], target_pos, target_quat)

            pos_err_norm = np.linalg.norm(error[:3])
            if pos_err_norm < self.tolerance:
                if target_quat is None or np.linalg.norm(error[3:]) < self.tolerance:
                    self.log(f"IK converged in {iteration + 1} iterations (err={pos_err_norm:.6f})")
                    return self._clamp_joints(q).tolist()

            J = self._compute_jacobian(q)
            JJT = J @ J.T + lam2 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJT, error)

            q += dq
            q = self._clamp_joints(q)

        self.log(f"IK did not converge after {self.max_iterations} iterations (err={pos_err_norm:.6f})")
        # Return best effort solution — for interactive dragging, partial progress is better than nothing
        return self._clamp_joints(q).tolist()

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "joint_states":
            # Initialize positions array on first use
            if self._current_joint_positions is None:
                self._current_joint_positions = [0.0] * self._num_joints

            # Handle individual joint state messages (fan-in from controllers)
            # Format: {"name": ["joint_id"], "position": [pos], ...}
            names = data.get("name", [])
            positions = data.get("position", [])

            if names and positions:
                for name, pos in zip(names, positions):
                    idx = self._joint_name_to_idx.get(name)
                    if idx is not None:
                        self._current_joint_positions[idx] = float(pos)
            else:
                # Fallback: combined positions array
                combined = data.get("positions", [])
                if combined:
                    self._current_joint_positions = list(combined)

        elif pin_id == "target_pose":
            if self._current_joint_positions is None:
                self._current_joint_positions = [0.0] * self._num_joints

            # Debug: log current FK end-effector position vs target
            q_current = np.array(self._current_joint_positions, dtype=float)
            fk_transforms = self._forward_kinematics(q_current)
            ee_pos = fk_transforms[-1][:3, 3]
            pos = data.get("position", {})
            target_pos = [pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)]
            self.log(f"FK end-effector pos: [{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}]")
            self.log(f"Target pos: [{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}]")
            self.log(f"Current joint angles: {self._current_joint_positions}")
            self.log(f"Chain origins: {[list(j['origin_translation']) for j in self._chain]}")

            solution = self._solve_ik(data, self._current_joint_positions)
            self.log(f"IK solution: {solution}")
            if solution is not None:
                joint_ids = [joint["_id"] for joint in self._chain]
                self.emit("joint_commands", {
                    "name": joint_ids,
                    "positions": solution,
                })
