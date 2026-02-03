cmake_minimum_required(VERSION 3.8)
project(control_input_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Inputs.msg"
        "msg/ContactInfo.msg"
        "msg/AMPMotionData.msg"
        DEPENDENCIES std_msgs builtin_interfaces
)

file(GLOB SCRIPTS
  "scripts/*.py"
)

install(PROGRAMS
  ${SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
