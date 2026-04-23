"""
Unit conversion between graph-side SI units and ODrive-native turns.

motor_turns = output_turns * gear_ratio
"""

import math
from dataclasses import dataclass


@dataclass
class JointScales:
    """Forward (command) and inverse (state) scales for one joint."""

    cmd_position_scale: float
    cmd_velocity_scale: float
    state_position_scale: float
    state_velocity_scale: float


def compute_scales(gear_ratio: float, units: str = "radians") -> JointScales:
    """
    Return the conversion factors between user-level units and ODrive turns.

    `units` selects the graph-side unit:
      - "radians" (default): position in rad, velocity in rad/s
      - "turns":             position in turns, velocity in turns/s
    """
    units = units.lower().strip()
    gr = float(gear_ratio)
    if units in ("turn", "turns"):
        return JointScales(
            cmd_position_scale=gr,
            cmd_velocity_scale=gr,
            state_position_scale=1.0 / gr,
            state_velocity_scale=1.0 / gr,
        )

    tau = 2.0 * math.pi
    return JointScales(
        cmd_position_scale=(1.0 / tau) * gr,
        cmd_velocity_scale=(1.0 / tau) * gr,
        state_position_scale=tau / gr,
        state_velocity_scale=tau / gr,
    )
