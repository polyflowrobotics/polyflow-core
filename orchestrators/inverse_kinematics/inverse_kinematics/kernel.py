import math
from collections import deque
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


def _rotation_to_rpy(R: np.ndarray) -> np.ndarray:
    """Extract roll-pitch-yaw (XYZ intrinsic) from a 3x3 rotation matrix."""
    sy = R[0, 2]
    sy = np.clip(sy, -1, 1)
    pitch = math.asin(sy)
    if abs(abs(sy) - 1) < 1e-6:
        # Gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    else:
        roll = math.atan2(-R[1, 2], R[2, 2])
        yaw = math.atan2(-R[0, 1], R[0, 0])
    return np.array([roll, pitch, yaw])


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
    IK solver using ikpy for robust inverse kinematics.

    Converts the PhysX-style joint geometry into ikpy URDFLink objects,
    then delegates solving to ikpy's scipy-backed optimizer with
    built-in regularization.

    Parameters:
        root_component_id, end_effector_component_id, components, joints,
        max_iterations, tolerance, regularization
    """

    def setup(self):
        self.root_component_id: str = self.get_param("root_component_id", "")
        self.end_effector_component_id: str = self.get_param("end_effector_component_id", "")
        self.components: List[Dict[str, Any]] = self.get_param("components", [])
        self.joints: List[Dict[str, Any]] = self.get_param("joints", [])
        self.max_iterations = int(self.get_param("max_iterations", 100))
        self.tolerance = float(self.get_param("tolerance", 0.001))
        self._regularization = float(self.get_param("regularization", 0.01))

        # Build the joint path and ikpy chain
        self._joint_path = self._find_joint_path()
        self._joint_ids: List[str] = []
        self._num_joints = 0
        self._ikpy_chain = None
        self._active_mask: List[bool] = []
        self._current_joint_positions: Optional[List[float]] = None

        if self._joint_path:
            self._build_ikpy_chain()

        # Map joint IDs to indices for fan-in state accumulation
        self._joint_name_to_idx: Dict[str, int] = {}
        for i, jid in enumerate(self._joint_ids):
            self._joint_name_to_idx[jid] = i

        self.log(f"IK chain: {self._num_joints} joints from '{self.root_component_id}' to '{self.end_effector_component_id}'")

        # Dump FK at q=0 for comparison with PhysX
        if self._ikpy_chain:
            q_full = np.zeros(len(self._ikpy_chain.links))
            fk = self._ikpy_chain.forward_kinematics(q_full, full_kinematics=True)
            self.log("FK at q=0 body positions (ikpy):")
            for i, T in enumerate(fk):
                p = T[:3, 3]
                self.log(f"  link[{i}] pos=({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f})")

    def _find_joint_path(self) -> List[Dict[str, Any]]:
        """BFS from root to end-effector, returning the list of joints on the path."""
        comp_by_id = {c.get("_id", c.get("component_id", "")): c for c in self.components}

        if not self.root_component_id or self.root_component_id not in comp_by_id:
            self.log(f"Root component '{self.root_component_id}' not found")
            return []
        if not self.end_effector_component_id or self.end_effector_component_id not in comp_by_id:
            self.log(f"End effector '{self.end_effector_component_id}' not found")
            return []

        joints_by_parent: Dict[str, List[Dict[str, Any]]] = {}
        for j in self.joints:
            pid = j.get("parent", "")
            joints_by_parent.setdefault(pid, []).append(j)

        queue: deque = deque([(self.root_component_id, [])])
        visited = {self.root_component_id}

        while queue:
            comp_id, path = queue.popleft()
            if comp_id == self.end_effector_component_id:
                return path
            for joint in joints_by_parent.get(comp_id, []):
                child_id = joint.get("child", "")
                if child_id and child_id not in visited:
                    visited.add(child_id)
                    queue.append((child_id, path + [joint]))

        self.log("No path from root to end-effector")
        return []

    def _build_ikpy_chain(self):
        """Convert PhysX joint geometry to ikpy Chain.

        PhysX FK formula per joint:
            child = parent @ T_parent_pose @ Rot_x(q) @ inv(T_child_pose)
        where:
            T_parent_pose = H(R_output @ alignXTo(axis), output_translation)
            T_child_pose  = H(R_input  @ alignXTo(axis), input_translation)

        At q=0 the net transform from parent body to child body is:
            T_link = T_parent_pose @ inv(T_child_pose)

        ikpy wants each link defined as:
            origin_translation, origin_orientation (RPY), rotation axis
        We decompose T_link into translation + RPY, and pass the joint axis
        directly (ikpy rotates around the given axis vector, not X).
        """
        from ikpy.chain import Chain
        from ikpy.link import URDFLink, OriginLink

        comp_by_id = {c.get("_id", c.get("component_id", "")): c for c in self.components}

        links = [OriginLink()]
        active_mask = [False]  # base link is not active
        joint_ids = []

        for joint in self._joint_path:
            joint_type = joint.get("joint_type", "fixed").lower()

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
            if joint_type in ("continuous",) or (lower_raw is None and upper_raw is None):
                lower, upper = -math.pi, math.pi
            else:
                lower = math.radians(lower_raw) if lower_raw is not None else -math.pi
                upper = math.radians(upper_raw) if upper_raw is not None else math.pi

            # Compute net link transform at q=0: T_link = T_parent_pose @ inv(T_child_pose)
            R_align = _align_x_to(axis)
            T_parent_pose = _homogeneous(output_r @ R_align, output_t)
            T_child_pose = _homogeneous(input_r @ R_align, input_t)
            T_link = T_parent_pose @ np.linalg.inv(T_child_pose)

            link_translation = T_link[:3, 3]
            link_rpy = _rotation_to_rpy(T_link[:3, :3])

            joint_id = joint.get("_id", joint.get("parent_output", ""))
            name = joint.get("name", child_comp.get("name", "unnamed"))

            is_active = joint_type in ("revolute", "continuous", "prismatic")

            if is_active:
                rotation_axis = axis if joint_type != "prismatic" else None
                translation_axis = axis if joint_type == "prismatic" else None
            else:
                rotation_axis = None
                translation_axis = None

            self.log(f"  link '{name}': t=({link_translation[0]:.6f}, {link_translation[1]:.6f}, {link_translation[2]:.6f}) "
                     f"rpy=({link_rpy[0]:.4f}, {link_rpy[1]:.4f}, {link_rpy[2]:.4f}) "
                     f"axis={axis.tolist()} type={joint_type} bounds=({lower:.3f}, {upper:.3f})")

            links.append(URDFLink(
                name=name,
                origin_translation=link_translation,
                origin_orientation=link_rpy,
                rotation=rotation_axis,
                translation=translation_axis,
                bounds=(lower, upper),
                use_symbolic_matrix=False,
            ))
            active_mask.append(is_active)

            if is_active:
                joint_ids.append(joint_id)

        self._ikpy_chain = Chain(
            links=links,
            active_links_mask=active_mask,
            name="ik_chain",
        )
        self._joint_ids = joint_ids
        self._num_joints = len(joint_ids)
        self._active_mask = active_mask

    def _joints_to_full(self, joint_angles: List[float]) -> np.ndarray:
        """Convert active-only joint angles to full ikpy array (including inactive links)."""
        full = np.zeros(len(self._active_mask))
        j = 0
        for i, active in enumerate(self._active_mask):
            if active:
                full[i] = joint_angles[j]
                j += 1
        return full

    def _full_to_joints(self, full: np.ndarray) -> List[float]:
        """Extract active joint angles from full ikpy array."""
        return [float(full[i]) for i, active in enumerate(self._active_mask) if active]

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
            if not self._ikpy_chain:
                self.log("No IK chain built, ignoring target_pose")
                return

            if self._current_joint_positions is None:
                self._current_joint_positions = [0.0] * self._num_joints

            pos = data.get("position", {})
            target_pos = np.array([pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)])

            # Debug logging
            q_full = self._joints_to_full(self._current_joint_positions)
            fk = self._ikpy_chain.forward_kinematics(q_full)
            ee = fk[:3, 3]
            self.log(f"FK EE (m): [{ee[0]:.4f}, {ee[1]:.4f}, {ee[2]:.4f}]")
            self.log(f"Target (m): [{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}]")
            self.log(f"Current q (rad): {[f'{a:.4f}' for a in self._current_joint_positions]}")

            # Solve IK using ikpy
            solution_full = self._ikpy_chain.inverse_kinematics(
                target_position=target_pos,
                initial_position=q_full,
                regularization_parameter=self._regularization,
            )

            solution = self._full_to_joints(solution_full)

            # Verify
            sol_fk = self._ikpy_chain.forward_kinematics(self._joints_to_full(solution))
            sol_ee = sol_fk[:3, 3]
            err = np.linalg.norm(target_pos - sol_ee)
            self.log(f"Solution: {[f'{a:.4f}' for a in solution]}")
            self.log(f"FK at solution (m): [{sol_ee[0]:.4f}, {sol_ee[1]:.4f}, {sol_ee[2]:.4f}] (err={err:.6f})")

            # Update current positions so the next solve starts from this solution
            self._current_joint_positions = list(solution)

            self.emit("joint_commands", {
                "name": self._joint_ids,
                "positions": solution,
            })
