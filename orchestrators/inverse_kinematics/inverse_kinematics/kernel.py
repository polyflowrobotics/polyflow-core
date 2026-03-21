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
        root_component_id: string — _id of the root component for this chain
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
        self.components: List[Dict[str, Any]] = self.get_param("components", [])
        self.joints: List[Dict[str, Any]] = self.get_param("joints", [])
        self.max_iterations = int(self.get_param("max_iterations", 100))
        self.tolerance = float(self.get_param("tolerance", 0.001))
        self.damping = float(self.get_param("damping", 0.01))

        self._chain = self._build_chain(self.root_component_id, self.components, self.joints)
        self._num_joints = len(self._chain)
        self._current_joint_positions: Optional[List[float]] = None

        self.log(f"IK chain from '{self.root_component_id}' with {self._num_joints} actuated joints")

    def _build_chain(
        self,
        root_component_id: str,
        components: List[Dict[str, Any]],
        joints: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        """
        Walk the component tree from root_component_id downward via joints,
        building an ordered kinematic chain of actuated joints to the end-effector.
        """
        comp_by_id = {c.get("_id", c.get("component_id", "")): c for c in components}

        if not root_component_id or root_component_id not in comp_by_id:
            self.log(f"Root component '{root_component_id}' not found — chain will be empty")
            return []

        base_id = root_component_id

        # Index joints by parent component id
        joints_by_parent: Dict[str, List[Dict[str, Any]]] = {}
        for j in joints:
            pid = j.get("parent", "")
            if pid not in joints_by_parent:
                joints_by_parent[pid] = []
            joints_by_parent[pid].append(j)

        # BFS/DFS from base to build ordered chain
        chain = []
        visited = set()
        stack = [base_id]

        while stack:
            comp_id = stack.pop()
            if comp_id in visited:
                continue
            visited.add(comp_id)

            for joint in joints_by_parent.get(comp_id, []):
                child_id = joint.get("child", "")
                joint_type = joint.get("joint_type", "Fixed")

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
                    axis_raw.get("z", 1),
                ], dtype=float)

                limit = joint.get("limit", {})

                # Only actuated joints go into the chain
                if joint_type in ("Revolute", "Continuous", "Prismatic"):
                    lower = limit.get("lower", -math.pi)
                    upper = limit.get("upper", math.pi)
                    if joint_type == "Continuous":
                        lower = -math.pi
                        upper = math.pi

                    child_comp = comp_by_id.get(child_id, {})
                    chain.append({
                        "name": joint.get("name", child_comp.get("name", "unnamed")),
                        "type": joint_type,
                        "axis": axis,
                        "origin_translation": translation,
                        "origin_rotation": rotation_euler,
                        "lower": lower,
                        "upper": upper,
                    })

                if child_id:
                    stack.append(child_id)

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
            if joint["type"] in ("Revolute", "Continuous"):
                R_joint = _rotation_matrix(joint["axis"], q[i])
                T_joint = _homogeneous(R_joint, np.zeros(3))
            elif joint["type"] == "Prismatic":
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

            if joint["type"] in ("Revolute", "Continuous"):
                p_i = T_i[:3, 3]
                J[:3, i] = np.cross(z_i, p_ee - p_i)
                J[3:, i] = z_i
            elif joint["type"] == "Prismatic":
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
        if orient and any(orient.get(k, 0) != 0 for k in ("x", "y", "z", "w")):
            target_quat = np.array([
                orient.get("x", 0), orient.get("y", 0),
                orient.get("z", 0), orient.get("w", 1),
            ])

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
        return None

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "joint_states":
            self._current_joint_positions = data.get("positions", [])

        elif pin_id == "target_pose":
            if self._current_joint_positions is None:
                self._current_joint_positions = [0.0] * self._num_joints

            solution = self._solve_ik(data, self._current_joint_positions)
            if solution is not None:
                self.emit("joint_commands", {"positions": solution})
