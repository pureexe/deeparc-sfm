cmake_minimum_required(VERSION 2.8)
add_compile_options("-W" "-Wall" "-O3" "-pedantic")

project(sfm)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# library
add_library(DeepArcManager src/DeepArcManager.cc)

# Make SFM executable
add_executable(sfm src/sfm.cc)
target_link_libraries(sfm  DeepArcManager)
target_link_libraries(sfm ${CERES_LIBRARIES})