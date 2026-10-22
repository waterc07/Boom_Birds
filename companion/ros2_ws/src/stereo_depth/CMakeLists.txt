cmake_minimum_required(VERSION 3.8)
project(stereo_depth)

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_library(stereo_depth_kernels SHARED native/kernels.cpp)
target_compile_features(stereo_depth_kernels PRIVATE cxx_std_17)
target_compile_options(stereo_depth_kernels PRIVATE -O3 -Wall -Wextra)
install(TARGETS stereo_depth_kernels LIBRARY DESTINATION lib/${PROJECT_NAME})
set(ELAS_SOURCE_DIR "" CACHE PATH "Optional external libelas directory; not vendored")
if(ELAS_SOURCE_DIR)
  add_library(stereo_depth_elas SHARED native/elas_wrapper.cpp
    ${ELAS_SOURCE_DIR}/src/elas.cpp ${ELAS_SOURCE_DIR}/src/descriptor.cpp
    ${ELAS_SOURCE_DIR}/src/filter.cpp ${ELAS_SOURCE_DIR}/src/matrix.cpp
    ${ELAS_SOURCE_DIR}/src/triangle.cpp)
  target_include_directories(stereo_depth_elas PRIVATE ${ELAS_SOURCE_DIR}/src)
  target_compile_options(stereo_depth_elas PRIVATE -O3)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|i.86")
    target_compile_options(stereo_depth_elas PRIVATE -msse3)
  endif()
  install(TARGETS stereo_depth_elas LIBRARY DESTINATION lib/${PROJECT_NAME})
endif()

# Python 构建文件写入本包的 CMake binary 目录。
set(BB_INSTALL_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/install_${PROJECT_NAME}.cmake")
file(GENERATE OUTPUT "${BB_INSTALL_SCRIPT}" CONTENT
"execute_process(
  COMMAND bash \"${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/install_python_package.sh\" \"${Python3_EXECUTABLE}\" \"${CMAKE_CURRENT_SOURCE_DIR}\" \"${CMAKE_INSTALL_PREFIX}\" \"${CMAKE_CURRENT_BINARY_DIR}\" \"${PROJECT_NAME}\"
  RESULT_VARIABLE _bb_result
  OUTPUT_VARIABLE _bb_output
  ERROR_VARIABLE _bb_output
)
if(NOT _bb_result EQUAL 0)
  message(FATAL_ERROR \"${PROJECT_NAME} 安装失败：\${_bb_output}\")
endif()
")
install(SCRIPT "${BB_INSTALL_SCRIPT}")

# 仅安装运行必需的默认标定。
install(FILES calibration/upright_20261022/candidate.npz
        DESTINATION share/${PROJECT_NAME}/calibration/upright_20261022)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
