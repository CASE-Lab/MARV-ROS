
cmake_minimum_required(VERSION 3.5)

project(marv_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CmdSteering.msg"
  "msg/ScenarioConfig.msg"
  "msg/Notification.msg"
  "msg/Status.msg"
  "msg/Log1NCU.msg"
  "msg/Log1PDU.msg"    
  "msg/Log1TCU.msg"
  "msg/Log2TCU.msg"
  "msg/DataVarHeader.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
