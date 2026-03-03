cmake_minimum_required(VERSION 3.8)
project(polyflow_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/EncoderState.msg"
  "msg/CameraFrame.msg"
  "msg/GamepadAxes.msg"
  "msg/GamepadButtons.msg"
  "msg/LogEntry.msg"
  "msg/ModeCommand.msg"
  "msg/ModeState.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
