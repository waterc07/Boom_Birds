cmake_minimum_required(VERSION 3.16)
project(bb_sensor_motion LANGUAGES CXX)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
add_library(bb_sensor_motion SHARED sensor_motion.cpp)
target_link_libraries(bb_sensor_motion PRIVATE gz-sim8::gz-sim8 gz-transport13::gz-transport13)
target_compile_features(bb_sensor_motion PRIVATE cxx_std_17)

install(TARGETS bb_sensor_motion LIBRARY DESTINATION lib/boom_birds_sim)
