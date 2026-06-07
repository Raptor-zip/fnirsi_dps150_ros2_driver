cmake_minimum_required(VERSION 3.16)
project(fnirsi_dps150_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Dps150State.msg"
  "srv/SelectPreset.srv"
  "srv/SetFloat32.srv"
  "srv/SetPreset.srv"
  "srv/SetProtection.srv"
  "srv/SetUInt8.srv"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_core
  src/device_state.cpp
  src/dps150_client.cpp
  src/protocol.cpp
  src/serial_port.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}_core Threads::Threads)

add_library(${PROJECT_NAME}_node
  src/dps150_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
  "${cpp_typesupport_target}"
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}_node
  diagnostic_updater
  rclcpp
  std_msgs
  std_srvs
)

add_executable(dps150_node src/dps150_node_main.cpp)
target_link_libraries(dps150_node
  ${PROJECT_NAME}_node
)
ament_target_dependencies(dps150_node
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node dps150_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol
    test/test_protocol.cpp
  )
  target_link_libraries(test_protocol
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_protocol
    PRIVATE include
  )

  ament_add_gtest(test_dps150_node_integration
    test/test_dps150_node_integration.cpp
  )
  target_link_libraries(test_dps150_node_integration
    ${PROJECT_NAME}_node
  )
  target_include_directories(test_dps150_node_integration
    PRIVATE include
  )
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
