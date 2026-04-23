# polyflow-hiwonder-rrc-adapter

Polyflow hardware adapter for the Hiwonder RRC Lite controller board.

The RRC Lite exposes 4 motor channels, 4 PWM servo channels, an LX-series bus-servo bus, a 6-axis IMU, and battery voltage over a single USB serial link (binary protocol at 1 Mbaud).

Provides four drivers:

- **`hiwonder_rrc_board`** — serial-port owner. Instantiates the `HiwonderRRC` driver, publishes `sensor_msgs/Imu` and `sensor_msgs/BatteryState`. Exposes the shared driver handle to motor/servo adapters as a resource.
- **`hiwonder_rrc_motor`** — one adapter per motor channel (1–4). Consumes `MotorCommand` (SPEED in rad/s, DUTY in [-1,1], IDLE) with watchdog on `timeout_s`. Publishes `MotorState` (echoed commanded value — the firmware does not report encoder ticks).
- **`hiwonder_rrc_pwm_servo`** — one per PWM channel (1–4). Consumes `ServoCommand`, linearly maps rad → pulse µs.
- **`hiwonder_rrc_bus_servo`** — one per LX-bus servo ID. Consumes `ServoCommand`, maps rad → 0–1000 raw, polls position/temperature/voltage back.

## hardware.yaml example

```yaml
hardware:
  - id: rrc0
    driver: hiwonder_rrc_board
    params:
      device: /dev/ttyAMA0
      baud: 1000000
      poll_hz: 100

  - id: left_drive
    driver: hiwonder_rrc_motor
    params:
      board: rrc0
      port: 1
      gear_ratio: 30.0

  - id: right_drive
    driver: hiwonder_rrc_motor
    params:
      board: rrc0
      port: 2
      gear_ratio: 30.0
      invert: true

  - id: pan
    driver: hiwonder_rrc_pwm_servo
    params:
      board: rrc0
      channel: 1
      pulse_min_us: 500
      pulse_max_us: 2500
      angle_min_rad: -1.5708
      angle_max_rad:  1.5708

  - id: gripper
    driver: hiwonder_rrc_bus_servo
    params:
      board: rrc0
      servo_id: 5
      angle_min_rad: -2.0944
      angle_max_rad:  2.0944
```

## Limitations

- **No encoder feedback.** The RRC firmware does not report motor tick counts to the host. `MotorState.measured_value` mirrors the last commanded value.
- **IMU units.** The board emits six `float32` per sample; scaling is assumed SI (m/s² for accel, rad/s for gyro). If your board variant emits g / deg/s, scale in the user graph.
- **Battery cell-voltage array.** The RRC SYS frame reports total pack voltage only; `BatteryState.cell_voltage` is left empty.
