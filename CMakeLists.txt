cmake_minimum_required(VERSION 2.8)

project(sfm)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# deeparc manager
add_library(deeparc_manager deeparc_manager.cc)

# Make SFM executable
add_executable(sfm sfm.cc)
target_link_libraries(sfm  deeparc_manager)
target_link_libraries(sfm ${CERES_LIBRARIES})