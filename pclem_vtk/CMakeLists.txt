cmake_minimum_required (VERSION 3.1)
include(ExternalProject)

project(pclem_vtk)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(VTK REQUIRED)
find_package(glog REQUIRED)

include_directories(src ${VTK_INCLUDE_DIRS} ${pclem_INCLUDE_DIRS})
include(${VTK_USE_FILE})

file(GLOB SOURCES src/*.cpp)
set(SOURCES
  src/visualizable_weighted_gaussian.cpp
  src/visualizable_gaussian_mixture.cpp
  src/visualizable_point_cloud.cpp
  src/vtk_visualization.cpp
  src/vtk_pointcloud_reader.cpp)

add_library(pclem_vtk_lib ${SOURCES})
target_link_libraries(pclem_vtk_lib pclem glog::glog ${VTK_LIBRARIES})

add_executable(run_em src/run_em.cpp ${SOURCES})
target_link_libraries(run_em pclem_vtk_lib)

add_executable(run_hierarchical_em src/run_hierarchical_em.cpp ${SOURCES})
target_link_libraries(run_hierarchical_em pclem_vtk_lib)

add_executable(profile src/profile.cpp ${SOURCES})
target_link_libraries(profile pclem pclem_vtk_lib)
