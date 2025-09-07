cmake_minimum_required(VERSION 3.8)
project(laser_uav_lib)

# Adiciona opções de compilação para GCC ou Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================== Encontra Pacotes Necessários ==========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ========================== Encontra Eigen3 ==========================
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
  message(STATUS "Eigen3 encontrado - versão ${Eigen3_VERSION}")
else()
  message(FATAL_ERROR "Eigen3 não encontrado. Instale com: sudo apt install libeigen3-dev")
endif()

# ========================== Encontra Autodiff ==========================
find_package(autodiff QUIET)
if(autodiff_FOUND)
    message(STATUS "Autodiff encontrado - versão ${autodiff_VERSION}")
else()
    message(FATAL_ERROR "Autodiff não encontrado. Instale com: sudo apt install libautodiff-dev")
endif()

# Define as dependências comuns
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# Inclui diretórios de cabeçalho
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${autodiff_INCLUDE_DIRS} # Diretórios do Autodiff
)
  
# ========================== Attitude Converter ==========================
add_library(attitude_converter SHARED
  src/attitude_converter/attitude_converter.cpp
)

target_include_directories(attitude_converter PRIVATE include)
ament_target_dependencies(attitude_converter ${dependencies})

# Export only your attitude_converter library
ament_export_libraries(attitude_converter)
ament_export_dependencies(${dependencies})

# ========================== filter ==========================
add_library(filter SHARED
  src/filter/irr_filter.cpp
  src/filter/notch_filter.cpp
  src/filter/imu_filter.cpp
)

target_include_directories(filter PRIVATE include)
ament_target_dependencies(filter ${dependencies})

# Export only your filter library
ament_export_libraries(filter)
ament_export_dependencies(${dependencies})

# ========================== Drone EKF ==========================
add_library(drone_ekf SHARED
  src/kalman_filter/drone_ekf/drone_ekf.cpp
  src/kalman_filter/ekf/ekf.cpp
)

target_include_directories(drone_ekf PRIVATE include)
ament_target_dependencies(drone_ekf
  ${dependencies}
  ${autodiff_LIBRARIES}
)

target_link_libraries(drone_ekf
  ${Eigen3_LIBRARIES}
  ${autodiff_LIBRARIES} # Vincula o Autodiff
)

ament_export_libraries(drone_ekf)
ament_export_dependencies(${dependencies} ${autodiff_LIBRARIES})

# ========================== Instalação das Bibliotecas ==========================
install(
  TARGETS attitude_converter filter drone_ekf
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ========================== Configuração dos Testes ==========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(GTest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  include_directories(${GTEST_INCLUDE_DIRS})

  # Add tests
  ament_add_gtest(test_irr_filter
    tests/filter/test_irr_filter.cpp
  )
  target_link_libraries(test_irr_filter filter)
  ament_target_dependencies(test_irr_filter ${dependencies})

  ament_add_gtest(test_notch_filter
    tests/filter/test_notch_filter.cpp
  )
  target_link_libraries(test_notch_filter filter)
  ament_target_dependencies(test_notch_filter ${dependencies})

  ament_add_gtest(test_imu_filter
    tests/filter/test_imu_filter.cpp
  )
  target_link_libraries(test_imu_filter filter)
  ament_target_dependencies(test_imu_filter ${dependencies})
  
  ament_add_gtest(test_drone_ekf
    tests/ekf/test_drone_ekf.cpp
  )
  target_link_libraries(test_drone_ekf
    drone_ekf
    attitude_converter
  )
  ament_target_dependencies(test_drone_ekf
    ${dependencies}
    ${autodiff_LIBRARIES}
  )
endif()

# Finaliza a configuração do pacote
ament_package()