cmake_minimum_required(VERSION 3.8)
project(boom_birds_nav)

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# 与 stereo_depth 相同的方式安装 Python 包与资源：具体逻辑在 bin/install_package.sh，
# 由 configure 阶段生成的脚本执行，避免 install(CODE ...) 多行字符串被压平导致的解析问题。
set(BB_INSTALL_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/install_boom_birds_nav.cmake")
file(GENERATE OUTPUT "${BB_INSTALL_SCRIPT}" CONTENT
"execute_process(
  COMMAND \"${CMAKE_CURRENT_SOURCE_DIR}/bin/install_package.sh\" \"${Python3_EXECUTABLE}\" \"${CMAKE_CURRENT_SOURCE_DIR}\" \"${CMAKE_INSTALL_PREFIX}\"
  RESULT_VARIABLE _bb_result
  OUTPUT_VARIABLE _bb_output
  ERROR_VARIABLE _bb_output
)
if(NOT _bb_result EQUAL 0)
  message(FATAL_ERROR \"boom_birds_nav 安装失败：\${_bb_output}\")
endif()
")
install(SCRIPT "${BB_INSTALL_SCRIPT}")

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})
install(FILES README.md
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(boom_birds_nav_unit test
    APPEND_ENV PYTHONPATH="${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages")
endif()

ament_package()
