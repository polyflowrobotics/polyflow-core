"""
Constants mirroring the polyflow_msgs enum fields.

Adapters run outside the ROS environment and cannot import polyflow_msgs
directly. These constants let adapter code compare against typed command
fields (mode, request, level) without importing ROS.
"""

# --- Diagnostic levels (diagnostic_msgs/DiagnosticStatus) ---
LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2
LEVEL_STALE = 3

# --- JointCommand / JointState.mode (polyflow_msgs/JointCommand) ---
JOINT_MODE_POSITION = 0
JOINT_MODE_VELOCITY = 1
JOINT_MODE_TORQUE = 2

# --- MotorCommand / MotorState.mode (polyflow_msgs/MotorCommand) ---
MOTOR_MODE_SPEED = 0
MOTOR_MODE_DUTY = 1
MOTOR_MODE_IDLE = 2

# --- HardwareLifecycle.request (polyflow_msgs/HardwareLifecycle) ---
LIFECYCLE_INIT = 0
LIFECYCLE_CALIBRATE = 1
LIFECYCLE_HOMING = 2
LIFECYCLE_ENABLE = 3
LIFECYCLE_DISABLE = 4
LIFECYCLE_ESTOP = 5
LIFECYCLE_RESET = 6
