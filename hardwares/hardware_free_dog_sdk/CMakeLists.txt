cmake_minimum_required(VERSION 3.16)
project(hardware_free_dog_sdk)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fdsc_utils REQUIRED)
find_package(unitree_go REQUIRED)

# Build
add_library(
  hardware_free_dog_sdk
  SHARED
  src/HardwareFreeDogSdk.cpp
)

target_link_libraries(hardware_free_dog_sdk)

target_compile_features(hardware_free_dog_sdk PUBLIC cxx_std_17)

target_include_directories(hardware_free_dog_sdk PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hardware_free_dog_sdk>
)

ament_target_dependencies(
  hardware_free_dog_sdk PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  fdsc_utils
  unitree_go
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hardware_free_dog_sdk PRIVATE "HARDWARE_FREE_DOG_SDK_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface HardwareFreeDogSdk.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/hardware_free_dog_sdk
)
install(
  TARGETS hardware_free_dog_sdk
  EXPORT export_hardware_free_dog_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/hardware_free_dog_sdk
)

install(FILES HardwareFreeDogSdk.xml
  DESTINATION share/hardware_free_dog_sdk
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_hardware_free_dog_sdk HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  fdsc_utils
  unitree_go
)

ament_package()