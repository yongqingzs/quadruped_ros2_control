cmake_minimum_required(VERSION 3.8)
project(hardware_unitree_ros2)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        unitree_go
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_sdk2 REQUIRED)

foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach ()

include(GNUInstallDirs)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/HardwareUnitreeRos2.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE src
)

ament_target_dependencies(
        ${PROJECT_NAME}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface HardwareUnitreeRos2.xml)

# Add DDS-ROS2 bridge node
add_executable(dds_ros2_bridge_node src/dds_ros2_bridge_node_main.cpp src/dds_ros2_bridge_node.cpp)
target_include_directories(dds_ros2_bridge_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_link_libraries(dds_ros2_bridge_node unitree_sdk2)
ament_target_dependencies(dds_ros2_bridge_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

install(
        TARGETS ${PROJECT_NAME} dds_ros2_bridge_node
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
