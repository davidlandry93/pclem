cmake_minimum_required (VERSION 3.0)
include(ExternalProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project (pclem)
find_package(CUDA REQUIRED)
find_package(Armadillo REQUIRED)
find_package(glog REQUIRED)

include_directories(src "${ARMADILLO_INCLUDE_DIRS}")

list(APPEND CUDA_NVCC_FLAGS "-arch=sm_30;-std=c++11;-D_MWAITXINTRIN_H_INCLUDED;-D_FORCE_INLINES")
SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

file(GLOB SOURCES src/*.cpp src/*.cu)

cuda_add_library(pclem ${SOURCES})
target_link_libraries(pclem
  ${CUDA_LIBRARIES} ${ARMADILLO_LIBRARIES} glog::glog)

set(PUBLIC_HEADERS src/pointcloud.h src/em_algorithm.h src/gaussian_mixture.h src/point.h src/weighted_gaussian.h src/covariance_matrix.h src/boundingbox.h)
INSTALL(FILES ${PUBLIC_HEADERS} DESTINATION include/pclem)
INSTALL(TARGETS pclem DESTINATION lib/pclem EXPORT pclem-targets)
INSTALL(EXPORT pclem-targets DESTINATION lib/pclem)
INSTALL(FILES pclem-config.cmake DESTINATION lib/pclem)
export(PACKAGE pclem)
