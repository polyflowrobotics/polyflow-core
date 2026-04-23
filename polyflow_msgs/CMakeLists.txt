cmake_minimum_required(VERSION 3.8)
project(polyflow_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraFrame.msg"
  "msg/EncoderState.msg"
  "msg/GamepadAxes.msg"
  "msg/GamepadButtons.msg"
  "msg/HardwareLifecycle.msg"
  "msg/JointCommand.msg"
  "msg/JointState.msg"
  "msg/LogEntry.msg"
  "msg/ModeCommand.msg"
  "msg/ModeState.msg"
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/ServoCommand.msg"
  "msg/ServoState.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
