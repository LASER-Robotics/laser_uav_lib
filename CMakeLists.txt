cmake_minimum_required(VERSION 3.8)
project(laser_uav_lib)

# Adiciona opções de compilação para GCC ou Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontra pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Define as dependências
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# Inclui diretórios de cabeçalho
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Adiciona a biblioteca
add_library(laser_uav_lib SHARED
  src/attitude_converter/attitude_converter.cpp
)

# Define diretórios de inclusão para a biblioteca
target_include_directories(laser_uav_lib PRIVATE
  include
)

# Vincula as dependências à biblioteca
ament_target_dependencies(laser_uav_lib
  ${dependencies}
)

# Instala a biblioteca
install(TARGETS laser_uav_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Instala os cabeçalhos
install(DIRECTORY include/
  DESTINATION include
)

# Configura testes (se necessário)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Exporta diretórios de inclusão e dependências
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

# Finaliza a configuração do pacote
ament_package()