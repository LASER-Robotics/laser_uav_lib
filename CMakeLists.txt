cmake_minimum_required(VERSION 3.8)
project(laser_uav_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(common_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

macro(add_laser_component TARGET_NAME SOURCE_FILE)
  add_library(${TARGET_NAME} ${SOURCE_FILE})
  target_include_directories(${TARGET_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${TARGET_NAME} ${common_dependencies})
  
  if(${ARGC} GREATER 2)
    target_link_libraries(${TARGET_NAME} ${ARGN})
  endif()
  
  list(APPEND component_libraries ${TARGET_NAME})
endmacro()

add_laser_component(irr_filter src/filter/irr_filter.cpp Eigen3::Eigen)
add_laser_component(notch_filter src/filter/notch_filter.cpp irr_filter)
add_laser_component(imu_filter src/filter/imu_filter.cpp notch_filter)
add_laser_component(attitude_converter src/attitude_converter/attitude_converter.cpp Eigen3::Eigen)
add_laser_component(ekf src/kalman_filter/ekf/ekf.cpp Eigen3::Eigen)
add_laser_component(rmse src/metrics/rmse.cpp)
add_laser_component(kdtree src/data_structure/kdtree.cpp)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE ${component_libraries})

list(APPEND component_libraries ${PROJECT_NAME})

install(
  TARGETS ${component_libraries}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${common_dependencies} Eigen3)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  macro(add_laser_test TEST_NAME SOURCE_FILE LINK_LIB)
    ament_add_gtest(${TEST_NAME} ${SOURCE_FILE})
    ament_target_dependencies(${TEST_NAME} ${common_dependencies})
    target_link_libraries(${TEST_NAME} ${LINK_LIB})
  endmacro()

  add_laser_test(test_irr_filter tests/filter/test_irr_filter.cpp irr_filter)
  add_laser_test(test_notch_filter tests/filter/test_notch_filter.cpp notch_filter)
  add_laser_test(test_imu_filter tests/filter/test_imu_filter.cpp imu_filter)
endif()

ament_package()
