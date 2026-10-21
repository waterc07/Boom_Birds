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

# 1) 算法模块安装到 site-packages。
#    安装逻辑放在 bin/install_module.sh，由 configure 阶段生成的 install 脚本执行：
#    install(CODE ...) 的多行字符串会被 CMake 压平，容易造成参数粘连与解析错误，
#    因此这里用 generate/install(SCRIPT) 形式，参数只拼接一次且可读。
set(SD_INSTALL_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/install_stereo_depth_module.cmake")
file(GENERATE OUTPUT "${SD_INSTALL_SCRIPT}" CONTENT
"execute_process(
  COMMAND \"${CMAKE_CURRENT_SOURCE_DIR}/bin/install_module.sh\" \"${Python3_EXECUTABLE}\" \"${CMAKE_CURRENT_SOURCE_DIR}\" \"${CMAKE_INSTALL_PREFIX}\" \"${CMAKE_CURRENT_BINARY_DIR}\"
  RESULT_VARIABLE _sd_result
  OUTPUT_VARIABLE _sd_output
  ERROR_VARIABLE _sd_output
)
if(NOT _sd_result EQUAL 0)
  message(FATAL_ERROR \"stereo_depth 算法模块安装失败：\${_sd_output}\")
endif()
")
install(SCRIPT "${SD_INSTALL_SCRIPT}")

# 仅安装运行必需的默认标定。
install(FILES calibration/live_20261017_141337_524478/candidate.npz
        DESTINATION share/${PROJECT_NAME}/calibration/live_20261017_141337_524478)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
