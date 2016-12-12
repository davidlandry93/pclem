cmake_minimum_required (VERSION 3.0)
include(ExternalProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project (pclem)
find_package(CUDA REQUIRED)
find_package(Armadillo REQUIRED)
find_package(glog REQUIRED)

include_directories(src include "${ARMADILLO_INCLUDE_DIRS}")

list(APPEND CUDA_NVCC_FLAGS "-arch=sm_30;-std=c++11;-D_MWAITXINTRIN_H_INCLUDED;-D_FORCE_INLINES")
SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

file(GLOB SOURCES src/*.cpp src/*.cu src/*.h)

cuda_add_library(pclem ${SOURCES})
target_link_libraries(pclem
  ${CUDA_LIBRARIES} ${ARMADILLO_LIBRARIES} glog::glog)

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
  CACHE INTERNAL  "${PROJECT_NAME}: Include Directories" FORCE)

set(PUBLIC_HEADERS
  src/boundingbox.h
  src/covariance_matrix.h
  src/em_algorithm.h
  src/file_exporter.h
  src/gaussian_mixture.h
  src/hierarchical_gaussian_mixture.h
  src/matrix33.h
  src/point.h
  src/pointcloud.h
  src/vector3.h
  src/visualization.h
  src/weighted_gaussian.h)
INSTALL(FILES ${PUBLIC_HEADERS} DESTINATION include/pclem)
INSTALL(TARGETS pclem DESTINATION lib/pclem EXPORT pclem-targets)
INSTALL(EXPORT pclem-targets DESTINATION lib/pclem)
INSTALL(FILES pclem-config.cmake DESTINATION lib/pclem)
export(PACKAGE pclem)

option(tests "Build tests." ON) # Turn off with -Dtests=OFF

if(tests)
  enable_testing()
  find_package(GTest)
  find_package(GMock)

  file(GLOB TEST_FILES test/*.cpp test/*.cu)
  cuda_add_executable(tests ${TEST_FILES})
  target_link_libraries(tests pclem ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
endif(tests)
