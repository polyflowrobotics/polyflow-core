# Inverse Kinematics

Solves joint positions for a target end-effector pose. Builds a kinematic chain from the configured root component to the end-effector component using the Robot's joint geometry, then runs a damped-least-squares solver (SciPy `least_squares`) on each new target. Outputs a per-joint trajectory point for the downstream Joint Controllers and republishes the achieved pose for monitoring or feedback loops.

The forward-kinematics chain matches the physics worker's convention exactly (`child = parent · parentPose · joint_rot(q) · inv(childPose)`), so simulated and on-robot results agree.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `target_pose` | subscribe | `geometry_msgs/Pose` | Desired end-effector pose. Bound to the Studio IK gizmo by default. |
| `joint_states` | subscribe | `sensor_msgs/JointState[]` | Fan-in of per-joint feedback; used to seed the next solve. |
| `joint_commands` | publish | `trajectory_msgs/JointTrajectoryPoint[]` | Solved joint positions for downstream controllers. |
| `current_pose` | publish | `geometry_msgs/Pose` | FK of the most recent solved joint state. Useful as a `seed_pose` source for Twist To Pose. |

## Parameters

- **Root Component** (`root_component_id`) — base of the kinematic chain.
- **End Effector** (`end_effector_component_id`) — tip of the chain (gripper, hand, etc.). The solver walks from root to this component.
- **Robot Components / Robot Joints** (`components`, `joints`) — auto-injected from the Robot on deployment. Hidden in the UI.
- **Max Solver Iterations** (`max_iterations`) — iteration cap for `least_squares`. Default `100`.
- **Position Tolerance** (`tolerance`) — convergence tolerance in meters. Default `0.001`.
- **Damping Factor** (`damping`) — damping (λ) for the DLS solver. Higher values improve stability near singularities at the cost of step size. Default `0.01`.

## Typical use

Drop into a graph with one Joint Controller per actuated joint. Wire `joint_commands` to the trajectory inputs (each Joint Controller picks its own joint by name), and fan the Joint Controller `state` outputs back into `joint_states`. For teleop, route `current_pose` → Twist To Pose `seed_pose`, and `Twist To Pose.target_pose` → `target_pose` here — the operator drives the EE with a stick and IK keeps the joints consistent.

The solver dedups targets within ~0.1 mm to avoid re-solving when an upstream node (e.g. Twist To Pose) re-emits an unchanged target every tick.
