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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)

# ========================== Encontra Eigen3 ==========================
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
message(STATUS "Eigen3 encontrado - versão ${Eigen3_VERSION}")
else()
message(FATAL_ERROR "Eigen3 não encontrado. Instale com: sudo apt install libeigen3-dev")
endif()

# ========================== Encontra OpenCV ==========================
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
message(STATUS "OpenCV encontrado - versão ${OpenCV_VERSION}")
else()
message(FATAL_ERROR "OpenCV não encontrado. Instale com: sudo apt install libopencv-dev")
endif()

# ========================== Encontra PCL ==========================
find_package(PCL QUIET)
if(PCL_FOUND)
message(STATUS "PCL encontrado - versão ${PCL_VERSION}")
else()
message(FATAL_ERROR "PCL não encontrado. Instale com: sudo apt install libpcl-dev")
endif()

# ========================== Encontra pcl_conversions ==========================
find_package(pcl_conversions QUIET)
if(pcl_conversions_FOUND)
    message(STATUS "pcl_conversions encontrado")
else()
    message(FATAL_ERROR "pcl_conversions não encontrado. Instale com: sudo apt install ros-${ROS_DISTRO}-pcl-conversions")
endif()

# ========================== Encontra ZBar ==========================
find_package(ZBar QUIET)
if(NOT ZBar_FOUND)
    find_library(ZBAR_LIBRARIES NAMES zbar)
    if(ZBAR_LIBRARIES)
        set(ZBar_FOUND TRUE)
        set(ZBAR_INCLUDE_DIRS "/usr/include")
        message(STATUS "ZBar encontrado via find_library")
    else()
        message(FATAL_ERROR "ZBar não encontrado. Instale com: sudo apt install libzbar-dev")
    endif()
else()
    message(STATUS "ZBar encontrado - versão ${ZBar_VERSION}")
endif()



# Define as dependências comuns
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

set(vision_dependencies
  ${dependencies}  
  OpenCV
  ZBar
  image_geometry
)

set(pcl_dependencies
  ${dependencies}
  PCL
  pcl_conversions
  image_geometry
)

# Inclui diretórios de cabeçalho
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}  # Diretórios do OpenCV
  ${PCL_INCLUDE_DIRS}     # Diretórios do PCL
  ${pcl_conversions_INCLUDE_DIRS} # Diretórios do pcl_conversions
  ${ZBAR_INCLUDE_DIRS}    # Diretórios do ZBar
  )
  
# ========================== Attitude Converter ==========================
add_library(attitude_converter SHARED
  src/attitude_converter/attitude_converter.cpp
)

target_include_directories(attitude_converter PRIVATE include)
ament_target_dependencies(attitude_converter ${dependencies})

# ament_export_libraries(attitude_converter)
# ament_export_targets(attitude_converter)
# ament_export_dependencies(${dependencies})

# ========================== Vision (Usa OpenCV e ZBar) ==========================
add_library(vision SHARED
  src/vision/image_utils.cpp
  src/vision/image_processing.cpp
)

target_include_directories(vision PRIVATE include)
ament_target_dependencies(vision ${vision_dependencies})
target_link_libraries(vision 
  ${OpenCV_LIBS}    # Vincula o OpenCV
  ${ZBAR_LIBRARIES} # Vincula o ZBar
  ${Eigen3_LIBRARIES} # Vincula o Eigen3
)

# ament_export_libraries(vision)
# ament_export_targets(vision
#   ${OpenCV_LIBS}    # Vincula o OpenCV
#   ${ZBAR_LIBRARIES} # Vincula o ZBar
#   ${Eigen3_LIBRARIES} # Vincula o Eigen3
#   )
# ament_export_dependencies(${vision_dependencies})

# ========================== PCL Utils (Usa PCL) ==========================
add_library(pcl_utils SHARED
  src/pcl_utils/pointcloud_utils.cpp
)

target_include_directories(pcl_utils PRIVATE include)
ament_target_dependencies(pcl_utils ${pcl_dependencies})
target_link_libraries(pcl_utils 
  ${PCL_LIBRARIES} 
  ${pcl_conversions_LIBRARIES}
) # Vincula o PCL

# ament_export_libraries(pcl_utils)
# ament_export_targets(pcl_utils
#   ${PCL_LIBRARIES} 
#   ${pcl_conversions_LIBRARIES}
#   )
# ament_export_dependencies(${pcl_dependencies})


# ========================== Instalação das Bibliotecas ==========================

# Exporta a biblioteca attitude_converter para outros pacotes

# Exportação correta dos targets
install(
  TARGETS attitude_converter vision pcl_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Exporta os targets para outros pacotes
ament_export_targets(export_${PROJECT_NAME})

# Exporta diretórios de inclusão e dependências
ament_export_include_directories(include)

# Exporta todas as dependências necessárias
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# Instala os cabeçalhos
install(
  DIRECTORY include/
  DESTINATION include
)


# Configura testes (se necessário)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Finaliza a configuração do pacote
ament_package()