# =============================================================================
# @file CMakeLists.txt
# @brief Build script for the laser_uav_lib package.
# @author Wagner Dantas Garcia / Laser UAV Team
# @date September 16, 2025
# =============================================================================

# =============================================================================
# Project Definition and CMake Version
#
# Specifies the minimum version of CMake required to build this project and
# declares the project name. The project name is used for variables like
# ${PROJECT_NAME} and in IDEs.
# =============================================================================
cmake_minimum_required(VERSION 3.8)
project(laser_uav_lib)

# =============================================================================
# Compiler Options
#
# Sets compiler flags to improve code quality and catch potential issues early.
# -Wall: Enables all standard compiler warnings.
# -Wextra: Enables additional, more stringent warnings.
# -Wpedantic: Enforces strict ISO C++ compliance.
# These options are applied only for GCC (GNUCXX) and Clang compilers.
# =============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Position Independent Code (PIC)
#
# This is crucial for creating shared libraries (.so files). It ensures that
# the generated machine code can be loaded at any address in memory, which is
# a requirement for libraries linked by other components in ROS 2.
# =============================================================================
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# =============================================================================
# Dependency Discovery
#
# Uses `find_package` to locate and configure all required dependencies.
# This makes their headers and libraries available for linking.
# - ament_cmake: Core build tools for ROS 2.
# - rclcpp: The ROS 2 C++ client library.
# - std_msgs, geometry_msgs, nav_msgs, sensor_msgs: Common ROS message types.
# - tf2_ros, tf2_geometry_msgs: The transform library for ROS.
# - Eigen3: A C++ template library for linear algebra.
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# =============================================================================
# Common Dependencies Variable
#
# A variable holding a list of all commonly used ROS dependencies.
# This avoids repetition and makes the file cleaner and easier to maintain.
# =============================================================================
set(common_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# =============================================================================
# SECTION 1: INDIVIDUAL COMPONENT LIBRARY DEFINITIONS
#
# This section defines each functional part of the project as a separate,
# modular library. This approach improves organization and reusability.
# =============================================================================

# --- IRR Filter Library ---
# Creates a shared library named 'irr_filter'.
add_library(irr_filter src/filter/irr_filter.cpp)
# Specifies the include directories.
# - $<BUILD_INTERFACE:...> makes headers available during the build of this package.
# - $<INSTALL_INTERFACE:...> makes headers available to other packages that use this one.
# - PUBLIC: Applies these directories to this library and any other target that links to it.
target_include_directories(irr_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Links the library against its ROS dependencies using the ament macro.
ament_target_dependencies(irr_filter ${common_dependencies})
# Links against the non-ament dependency Eigen.
target_link_libraries(irr_filter Eigen3::Eigen)

# --- Notch Filter Library ---
add_library(notch_filter src/filter/notch_filter.cpp)
target_include_directories(notch_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(notch_filter ${common_dependencies})
# The notch filter depends on the IRR filter, so we link them.
target_link_libraries(notch_filter irr_filter)

# --- IMU Filter Library ---
add_library(imu_filter src/filter/imu_filter.cpp)
target_include_directories(imu_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_filter ${common_dependencies})
# The IMU filter depends on the notch filter.
target_link_libraries(imu_filter notch_filter)

# --- Attitude Converter Library ---
add_library(attitude_converter src/attitude_converter/attitude_converter.cpp)
target_include_directories(attitude_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(attitude_converter ${common_dependencies})
target_link_libraries(attitude_converter Eigen3::Eigen)

# --- EKF Base Library ---
add_library(ekf src/kalman_filter/ekf/ekf.cpp)
target_include_directories(ekf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ekf ${common_dependencies})
target_link_libraries(ekf Eigen3::Eigen)

# --- IMU Propagator Library ---
add_library(imu_propagator src/kalman_filter/imu_propagator/imu_propagator.cpp)
target_include_directories(imu_propagator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_propagator ${common_dependencies})
target_link_libraries(imu_propagator Eigen3::Eigen)

# =============================================================================
# SECTION 2: MAIN AGGREGATE LIBRARY
#
# Defines a single "INTERFACE" library that bundles all the component libraries.
# An INTERFACE library doesn't compile any sources itself; it's a convenient
# way to group usage requirements (includes, dependencies) for other targets.
# Another package can simply link against `${PROJECT_NAME}` to get all components.
# =============================================================================
add_library(${PROJECT_NAME} INTERFACE)

# The main library also exposes the include directories.
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Links the main interface library to all the component libraries. Any target
# linking against `${PROJECT_NAME}` will automatically link against these.
target_link_libraries(${PROJECT_NAME} INTERFACE
  irr_filter
  notch_filter
  imu_filter
  attitude_converter
  ekf
  imu_propagator
)

# =============================================================================
# SECTION 3: INSTALLATION AND EXPORT
#
# This section handles installing the built targets (libraries, executables)
# and header files, and exporting the package information so that other
# ROS 2 packages can find and use it with `find_package`.
# =============================================================================

# A list of all library targets to be installed.
set(component_libraries
  irr_filter
  notch_filter
  imu_filter
  attitude_converter
  ekf
  imu_propagator
  ${PROJECT_NAME}
)

# Installs the compiled library files and creates an export set named
# 'export_${PROJECT_NAME}'.
install(
  TARGETS ${component_libraries}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Installs the public header files from the 'include/' directory.
install(
  DIRECTORY include/
  DESTINATION include
)

# The `ament_export_*` functions generate the necessary files (`.cmake`)
# in the install directory so other packages can use this one.
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${common_dependencies} Eigen3)
ament_export_include_directories(include)

# =============================================================================
# SECTION 4: TEST CONFIGURATION
#
# Defines the build targets for tests. This block is only processed if
# tests are enabled (e.g., via `colcon test`).
# =============================================================================
if(BUILD_TESTING)
  # Find GTest and linting tools.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Defines a GTest executable for the 'irr_filter' library.
  ament_add_gtest(test_irr_filter tests/filter/test_irr_filter.cpp)
  ament_target_dependencies(test_irr_filter ${common_dependencies})
  target_link_libraries(test_irr_filter irr_filter)

  # Defines a GTest executable for the 'notch_filter' library.
  ament_add_gtest(test_notch_filter tests/filter/test_notch_filter.cpp)
  ament_target_dependencies(test_notch_filter ${common_dependencies})
  target_link_libraries(test_notch_filter notch_filter)

  # Defines a GTest executable for the 'imu_filter' library.
  ament_add_gtest(test_imu_filter tests/filter/test_imu_filter.cpp)
  ament_target_dependencies(test_imu_filter ${common_dependencies})
  target_link_libraries(test_imu_filter imu_filter)
endif()

# =============================================================================
# Finalization
#
# This is a mandatory macro call for all ament packages. It registers the
# package with the build system and finalizes the configuration. It must be
# the last command in the file.
# =============================================================================
ament_package()