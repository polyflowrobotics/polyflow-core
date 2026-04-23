"""Polyflow Adapter SDK.

Base classes and constants for writing hardware adapters that run inside
polyflow-os's system-manager. Adapters consume typed PRP commands from
the node graph and produce hardware state on PRP state topics.
"""

from .adapter import HardwareAdapter
from .host import AdapterHost, ScheduleHandle
from .manifest import DriverEntry, Manifest
from .constants import (
    LEVEL_OK,
    LEVEL_WARN,
    LEVEL_ERROR,
    LEVEL_STALE,
    JOINT_MODE_POSITION,
    JOINT_MODE_VELOCITY,
    JOINT_MODE_TORQUE,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_DUTY,
    MOTOR_MODE_IDLE,
    LIFECYCLE_INIT,
    LIFECYCLE_CALIBRATE,
    LIFECYCLE_HOMING,
    LIFECYCLE_ENABLE,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ESTOP,
    LIFECYCLE_RESET,
)

__version__ = "0.1.0"

__all__ = [
    "HardwareAdapter",
    "AdapterHost",
    "ScheduleHandle",
    "DriverEntry",
    "Manifest",
    "LEVEL_OK",
    "LEVEL_WARN",
    "LEVEL_ERROR",
    "LEVEL_STALE",
    "JOINT_MODE_POSITION",
    "JOINT_MODE_VELOCITY",
    "JOINT_MODE_TORQUE",
    "MOTOR_MODE_SPEED",
    "MOTOR_MODE_DUTY",
    "MOTOR_MODE_IDLE",
    "LIFECYCLE_INIT",
    "LIFECYCLE_CALIBRATE",
    "LIFECYCLE_HOMING",
    "LIFECYCLE_ENABLE",
    "LIFECYCLE_DISABLE",
    "LIFECYCLE_ESTOP",
    "LIFECYCLE_RESET",
]
