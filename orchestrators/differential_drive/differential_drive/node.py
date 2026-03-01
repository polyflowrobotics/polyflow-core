import asyncio
import math
import time

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from polyflow_msgs.msg import ModeState, EncoderState
from common.polyflow_node import PolyflowNode


class DifferentialDriveNode(PolyflowNode):
    """
    Differential drive orchestrator node for a 4-wheel skid-steer rover.

    Converts (linear, angular) velocity commands into per-wheel speed commands
    for four motors (2 per side). Both motors on the same side receive identical
    speed values. Optionally computes odometry from encoder feedback.

    Input pins:
        cmd_vel         (Twist)         — linear.x (m/s) and angular.z (rad/s)
        mode            (ModeState)     — from mode_switcher; "stopped" zeroes all motors
        encoder_left    (EncoderState)  — left-side encoder feedback
        encoder_right   (EncoderState)  — right-side encoder feedback

    Output pins:
        front_left_motor    (Float64)   — front-left motor speed command
        rear_left_motor     (Float64)   — rear-left motor speed command
        front_right_motor   (Float64)   — front-right motor speed command
        rear_right_motor    (Float64)   — rear-right motor speed command
        odometry            (Odometry)  — computed pose and velocity

    Parameters (via POLYFLOW_PARAMETERS):
        wheel_radius:       Wheel radius in meters (default: 0.05).
        wheel_separation:   Distance between left and right wheels in meters (default: 0.3).
        max_wheel_speed:    Maximum wheel speed in m/s (default: 1.0).
    """

    def __init__(self):
        super().__init__()

        self.wheel_radius = float(self.get_param("wheel_radius", 0.05))
        self.wheel_separation = float(self.get_param("wheel_separation", 0.3))
        self.max_wheel_speed = float(self.get_param("max_wheel_speed", 1.0))

        self._current_mode = "stopped"

        # Odometry state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_left_pos = None
        self._last_right_pos = None

        # Register input pins
        self.register_input_pin("cmd_vel", Twist)
        self.register_input_pin("mode", ModeState)
        self.register_input_pin("encoder_left", EncoderState)
        self.register_input_pin("encoder_right", EncoderState)

        # Register output pins
        self.register_output_pin("front_left_motor", Float64)
        self.register_output_pin("rear_left_motor", Float64)
        self.register_output_pin("front_right_motor", Float64)
        self.register_output_pin("rear_right_motor", Float64)
        self.register_output_pin("odometry", Odometry)

        self.get_logger().info(
            f"DifferentialDrive | wheel_radius={self.wheel_radius} | "
            f"wheel_separation={self.wheel_separation} | max_wheel_speed={self.max_wheel_speed}"
        )

    def _clamp(self, value: float) -> float:
        return max(-self.max_wheel_speed, min(self.max_wheel_speed, value))

    def _publish_motor_commands(self, left_speed: float, right_speed: float):
        """Send speed commands to all four motors."""
        left_speed = self._clamp(left_speed)
        right_speed = self._clamp(right_speed)

        left_msg = Float64()
        left_msg.data = left_speed
        right_msg = Float64()
        right_msg.data = right_speed

        self.publish_to_pin("front_left_motor", left_msg)
        self.publish_to_pin("rear_left_motor", left_msg)
        self.publish_to_pin("front_right_motor", right_msg)
        self.publish_to_pin("rear_right_motor", right_msg)

    def _update_odometry(self, left_pos: float, right_pos: float):
        """Compute odometry from encoder positions (radians)."""
        if self._last_left_pos is None or self._last_right_pos is None:
            self._last_left_pos = left_pos
            self._last_right_pos = right_pos
            return

        dl = (left_pos - self._last_left_pos) * self.wheel_radius
        dr = (right_pos - self._last_right_pos) * self.wheel_radius
        self._last_left_pos = left_pos
        self._last_right_pos = right_pos

        # Differential drive forward kinematics
        d_center = (dl + dr) / 2.0
        d_theta = (dr - dl) / self.wheel_separation

        self._x += d_center * math.cos(self._theta + d_theta / 2.0)
        self._y += d_center * math.sin(self._theta + d_theta / 2.0)
        self._theta += d_theta

        # TODO: Populate full Odometry message with covariance, twist, etc.
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        # Convert theta to quaternion (rotation about z-axis)
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        self.publish_to_pin("odometry", odom)

    def process_input(self, pin_id: str, data):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "mode":
            self._current_mode = data.mode
            self.get_logger().info(f"Mode updated: {self._current_mode}")
            if self._current_mode == "stopped":
                self._publish_motor_commands(0.0, 0.0)

        elif pin_id == "cmd_vel":
            if self._current_mode == "stopped":
                self._publish_motor_commands(0.0, 0.0)
                return

            linear = data.linear.x
            angular = data.angular.z

            # Unicycle to differential drive kinematics
            left_speed = linear - (angular * self.wheel_separation / 2.0)
            right_speed = linear + (angular * self.wheel_separation / 2.0)

            self._publish_motor_commands(left_speed, right_speed)

        elif pin_id == "encoder_left":
            # Store left encoder position; odometry updated when right arrives
            self._last_left_pos_pending = data.position_rad

        elif pin_id == "encoder_right":
            left_pos = getattr(self, '_last_left_pos_pending', None)
            if left_pos is not None:
                self._update_odometry(left_pos, data.position_rad)

    async def run_async(self):
        self.get_logger().info("DifferentialDrive ready")

        # Publish zero commands on startup (rover starts stopped)
        self._publish_motor_commands(0.0, 0.0)

        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    DifferentialDriveNode.main(args)
