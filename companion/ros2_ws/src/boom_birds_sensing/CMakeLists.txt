cmake_minimum_required(VERSION 3.8)
project(boom_birds_sensing)

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

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

# 运行配置随它的属主模块一起安装（各包的 config/ 由本包安装）。
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(boom_birds_sensing_unit test
    APPEND_ENV PYTHONPATH="${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages")
endif()

ament_package()
