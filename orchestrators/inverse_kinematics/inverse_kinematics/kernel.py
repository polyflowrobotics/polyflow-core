import math
from typing import Any, Dict, List, Optional

import numpy as np

from common.polyflow_kernel import PolyflowKernel


def _euler_to_rotation(rx: float, ry: float, rz: float) -> np.ndarray:
    """Euler XYZ intrinsic → 3x3 rotation matrix."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cy*cz, -cy*sz, sy],
        [sx*sy*cz + cx*sz, -sx*sy*sz + cx*cz, -sx*cy],
        [-cx*sy*cz + sx*sz, cx*sy*sz + sx*cz, cx*cy],
    ])


def _rotation_to_rotvec(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to rotation vector (axis * angle)."""
    trace = np.clip((np.trace(R) - 1) / 2, -1, 1)
    angle = math.acos(trace)
    if abs(angle) < 1e-10:
        return np.zeros(3)
    if abs(angle - math.pi) < 1e-6:
        # Near 180°: find eigenvector with eigenvalue 1
        _, vecs = np.linalg.eigh(R)
        axis = vecs[:, -1]
        return axis * angle
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2 * math.sin(angle))
    return axis * angle


def _align_x_to(direction: np.ndarray) -> np.ndarray:
    """Rotation matrix that maps X → direction. Mirrors physicsSimWorker's quatAlignXTo."""
    d = direction / np.linalg.norm(direction)
    from_vec = np.array([1.0, 0.0, 0.0])
    dot = float(np.dot(from_vec, d))
    if dot > 0.999999:
        return np.eye(3)
    if dot < -0.999999:
        return np.diag([-1.0, -1.0, 1.0])
    cross = np.cross(from_vec, d)
    K = np.array([
        [0, -cross[2], cross[1]],
        [cross[2], 0, -cross[0]],
        [-cross[1], cross[0], 0],
    ])
    return np.eye(3) + K + (K @ K) / (1 + dot)


def _homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


class InverseKinematicsKernel(PolyflowKernel):
    """
    IK solver that exactly mirrors the PhysX articulation FK.

    Builds a kinematic chain matching the physics worker's joint geometry:
        child_body = parent_body @ parentPose @ joint_rot(q) @ inv(childPose)
    where:
        parentPose = T(output_translation, R_output @ alignXTo(axis))
        childPose  = T(input_translation,  R_input  @ alignXTo(axis))
        joint_rot  = rotation/translation around X (PhysX eTWIST)

    Parameters:
        root_component_id, end_effector_component_id, components, joints,
        max_iterations, tolerance, damping
    """

    def setup(self):
        self.root_component_id: str = self.get_param("root_component_id", "")
        self.end_effector_component_id: str = self.get_param("end_effector_component_id", "")
        self.components: List[Dict[str, Any]] = self.get_param("components", [])
        self.joints: List[Dict[str, Any]] = self.get_param("joints", [])
        self.max_iterations = int(self.get_param("max_iterations", 100))
        self.tolerance = float(self.get_param("tolerance", 0.001))
        self.damping = float(self.get_param("damping", 0.01))

        self._chain = self._build_chain()
        self._num_joints = len(self._chain)
        self._current_joint_positions: Optional[List[float]] = None

        # Map joint IDs to chain indices for fan-in state accumulation
        self._joint_name_to_idx: Dict[str, int] = {}
        for i, joint in enumerate(self._chain):
            if joint.get("_id"):
                self._joint_name_to_idx[joint["_id"]] = i
            self._joint_name_to_idx[joint["name"]] = i

        self.log(f"IK chain: {self._num_joints} joints from '{self.root_component_id}' to '{self.end_effector_component_id}'")
        for i, j in enumerate(self._chain):
            self.log(f"  [{i}] {j['name']} type={j['type']} axis={j['axis'].tolist()}")

        # Dump FK body positions at q=0 for comparison with PhysX
        q0 = np.zeros(self._num_joints)
        fk0 = self._forward_kinematics(q0)
        self.log(f"FK at q=0 body positions:")
        for i, T in enumerate(fk0):
            p = T[:3, 3]
            self.log(f"  body[{i}] pos=({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f})")

    def _build_chain(self) -> List[Dict[str, Any]]:
        """Build kinematic chain via BFS from root to end-effector."""
        comp_by_id = {c.get("_id", c.get("component_id", "")): c for c in self.components}

        if not self.root_component_id or self.root_component_id not in comp_by_id:
            self.log(f"Root component '{self.root_component_id}' not found")
            return []
        if not self.end_effector_component_id or self.end_effector_component_id not in comp_by_id:
            self.log(f"End effector '{self.end_effector_component_id}' not found")
            return []

        # Index joints by parent component
        joints_by_parent: Dict[str, List[Dict[str, Any]]] = {}
        for j in self.joints:
            pid = j.get("parent", "")
            joints_by_parent.setdefault(pid, []).append(j)

        # BFS to find path
        from collections import deque
        queue: deque = deque([(self.root_component_id, [])])
        visited = {self.root_component_id}
        path_joints = None

        while queue:
            comp_id, path = queue.popleft()
            if comp_id == self.end_effector_component_id:
                path_joints = path
                break
            for joint in joints_by_parent.get(comp_id, []):
                child_id = joint.get("child", "")
                if child_id and child_id not in visited:
                    visited.add(child_id)
                    queue.append((child_id, path + [joint]))

        if path_joints is None:
            self.log(f"No path from root to end-effector")
            return []

        # Build chain entries with pre-computed transforms
        chain = []
        for joint in path_joints:
            joint_type = joint.get("joint_type", "fixed").lower()
            if joint_type not in ("revolute", "continuous", "prismatic"):
                continue

            # Output (parent-side) origin — already in meters from logic worker
            origin = joint.get("origin", {})
            output_t = np.array([origin.get("x", 0), origin.get("y", 0), origin.get("z", 0)], dtype=float)
            output_r = _euler_to_rotation(origin.get("rx", 0), origin.get("ry", 0), origin.get("rz", 0))

            # Child input origin — look up from component data (in mm → meters)
            child_id = joint.get("child", "")
            child_input_id = joint.get("child_input", "")
            child_comp = comp_by_id.get(child_id, {})
            input_t = np.zeros(3)
            input_r = np.eye(3)
            if child_input_id:
                for inp in child_comp.get("inputs", []):
                    if inp.get("_id") == child_input_id:
                        io = inp.get("origin", {})
                        input_t = np.array([io.get("x", 0), io.get("y", 0), io.get("z", 0)], dtype=float) / 1000.0
                        input_r = _euler_to_rotation(io.get("rx", 0), io.get("ry", 0), io.get("rz", 0))
                        break

            # Joint axis
            axis_raw = joint.get("axis", {"x": 0, "y": 0, "z": 1})
            axis = np.array([axis_raw.get("x", 0), axis_raw.get("y", 0), axis_raw.get("z", 0)], dtype=float)
            if np.linalg.norm(axis) < 1e-9:
                axis = np.array([0, 0, 1], dtype=float)
            axis = axis / np.linalg.norm(axis)

            # Joint limits (degrees → radians)
            limit = joint.get("limit", {})
            lower_raw = limit.get("lower_position", limit.get("lower", None))
            upper_raw = limit.get("upper_position", limit.get("upper", None))
            if joint_type == "continuous" or (lower_raw is None and upper_raw is None):
                lower, upper = -math.pi, math.pi
            else:
                lower = math.radians(lower_raw) if lower_raw is not None else -math.pi
                upper = math.radians(upper_raw) if upper_raw is not None else math.pi

            # Pre-compute parent and child pose transforms (matching PhysX exactly)
            R_align = _align_x_to(axis)
            T_parent_pose = _homogeneous(output_r @ R_align, output_t)
            T_child_pose = _homogeneous(input_r @ R_align, input_t)
            T_child_pose_inv = np.linalg.inv(T_child_pose)

            joint_id = joint.get("_id", joint.get("parent_output", ""))
            chain.append({
                "_id": joint_id,
                "name": joint.get("name", child_comp.get("name", "unnamed")),
                "type": joint_type,
                "axis": axis,
                "lower": lower,
                "upper": upper,
                "T_parent_pose": T_parent_pose,
                "T_child_pose_inv": T_child_pose_inv,
            })

        return chain

    def _forward_kinematics(self, q: np.ndarray) -> List[np.ndarray]:
        """
        FK matching PhysX articulation exactly:
            child = parent @ parentPose @ joint_rot(q) @ inv(childPose)
        Returns body transforms [root, child0, child1, ...].
        """
        transforms = [np.eye(4)]
        for i, joint in enumerate(self._chain):
            # Joint actuation around X axis (PhysX eTWIST)
            if joint["type"] in ("revolute", "continuous"):
                c, s = math.cos(q[i]), math.sin(q[i])
                T_joint = np.array([
                    [1, 0, 0, 0],
                    [0, c, -s, 0],
                    [0, s, c, 0],
                    [0, 0, 0, 1],
                ])
            elif joint["type"] == "prismatic":
                T_joint = np.eye(4)
                T_joint[0, 3] = q[i]
            else:
                T_joint = np.eye(4)

            T_child = transforms[-1] @ joint["T_parent_pose"] @ T_joint @ joint["T_child_pose_inv"]
            transforms.append(T_child)
        return transforms

    def _compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Geometric Jacobian (6xN). Joint axis = X column of joint frame."""
        transforms = [np.eye(4)]
        pivot_frames = []
        for i, joint in enumerate(self._chain):
            T_pivot = transforms[-1] @ joint["T_parent_pose"]
            pivot_frames.append(T_pivot)

            if joint["type"] in ("revolute", "continuous"):
                c, s = math.cos(q[i]), math.sin(q[i])
                T_joint = np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])
            elif joint["type"] == "prismatic":
                T_joint = np.eye(4)
                T_joint[0, 3] = q[i]
            else:
                T_joint = np.eye(4)

            transforms.append(T_pivot @ T_joint @ joint["T_child_pose_inv"])

        p_ee = transforms[-1][:3, 3]
        J = np.zeros((6, self._num_joints))
        for i, joint in enumerate(self._chain):
            z_i = pivot_frames[i][:3, 0]  # X column = joint axis in world
            if joint["type"] in ("revolute", "continuous"):
                p_i = pivot_frames[i][:3, 3]
                J[:3, i] = np.cross(z_i, p_ee - p_i)
                J[3:, i] = z_i
            elif joint["type"] == "prismatic":
                J[:3, i] = z_i
        return J

    def _solve_ik(self, target_pos: np.ndarray, current: List[float]) -> List[float]:
        """DLS (damped least-squares) IK solver — position only."""
        q = np.array(current, dtype=float)
        lam2 = self.damping ** 2
        prev_err = float("inf")
        stall_count = 0

        for iteration in range(self.max_iterations):
            transforms = self._forward_kinematics(q)
            ee_pos = transforms[-1][:3, 3]
            error_3 = target_pos - ee_pos
            err_norm = np.linalg.norm(error_3)

            if err_norm < self.tolerance:
                self.log(f"IK converged in {iteration + 1} iters (err={err_norm:.6f})")
                return np.clip(q, [j["lower"] for j in self._chain], [j["upper"] for j in self._chain]).tolist()

            # Stall detection
            if abs(prev_err - err_norm) < 1e-8:
                stall_count += 1
                if stall_count >= 5:
                    q += np.random.uniform(-0.1, 0.1, size=q.shape)
                    q = np.clip(q, [j["lower"] for j in self._chain], [j["upper"] for j in self._chain])
                    stall_count = 0
            else:
                stall_count = 0
            prev_err = err_norm

            # Position-only: use 3xN Jacobian
            J = self._compute_jacobian(q)[:3, :]
            JJT = J @ J.T + lam2 * np.eye(3)
            dq = J.T @ np.linalg.solve(JJT, error_3)

            q += dq
            q = np.clip(q, [j["lower"] for j in self._chain], [j["upper"] for j in self._chain])

        self.log(f"IK did not converge after {self.max_iterations} iters (err={err_norm:.6f})")
        return q.tolist()

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "joint_states":
            if self._current_joint_positions is None:
                self._current_joint_positions = [0.0] * self._num_joints

            # Fan-in from individual controllers: {"name": [id], "position": [pos]}
            names = data.get("name", [])
            positions = data.get("position", [])
            if names and positions:
                for name, pos in zip(names, positions):
                    idx = self._joint_name_to_idx.get(name)
                    if idx is not None:
                        self._current_joint_positions[idx] = float(pos)
            else:
                combined = data.get("positions", [])
                if combined:
                    self._current_joint_positions = list(combined)

        elif pin_id == "target_pose":
            if self._current_joint_positions is None:
                self._current_joint_positions = [0.0] * self._num_joints

            pos = data.get("position", {})
            target_pos = np.array([pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)])

            # Debug logging
            q_current = np.array(self._current_joint_positions, dtype=float)
            fk = self._forward_kinematics(q_current)
            ee = fk[-1][:3, 3]
            self.log(f"FK EE (m): [{ee[0]:.4f}, {ee[1]:.4f}, {ee[2]:.4f}]")
            self.log(f"Target (m): [{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}]")
            self.log(f"Current q (rad): {[f'{a:.4f}' for a in self._current_joint_positions]}")

            solution = self._solve_ik(target_pos, self._current_joint_positions)

            # Verify
            sol_fk = self._forward_kinematics(np.array(solution))
            sol_ee = sol_fk[-1][:3, 3]
            self.log(f"Solution: {[f'{a:.4f}' for a in solution]}")
            self.log(f"FK at solution (m): [{sol_ee[0]:.4f}, {sol_ee[1]:.4f}, {sol_ee[2]:.4f}]")

            joint_ids = [joint["_id"] for joint in self._chain]
            self.emit("joint_commands", {
                "name": joint_ids,
                "positions": solution,
            })
