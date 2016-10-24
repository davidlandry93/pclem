cmake_minimum_required (VERSION 3.0)
include(ExternalProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project (pclem)
find_package(VTK REQUIRED)
find_package(CUDA REQUIRED)
find_package(Armadillo REQUIRED)
find_package(glog REQUIRED)

if (NOT VTK_FOUND)
  ExternalProject_Add(
    VTK

    PREFIX "vtk"
    INSTALL_DIR "${PROJECT_BINARY_DIR}/vtk"
    CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${PROJECT_BINARY_DIR}/vtk"

    GIT_REPOSITORY "https://gitlab.kitware.com/vtk/vtk.git"
    GIT_TAG "v7.0.0")
  find_package(VTK)
endif()

include(${VTK_USE_FILE})
include_directories(src "${ARMADILLO_INCLUDE_DIRS}")

list(APPEND CUDA_NVCC_FLAGS "-arch=sm_30;-std=c++11;-D_MWAITXINTRIN_H_INCLUDED;-D_FORCE_INLINES")
SET(CUDA_PROPAGATE_HOST_FLAGS OFF)


file(GLOB SOURCES src/*.cpp src/*.cu)

cuda_add_library(pclem
  src/associated_point.cuh
  src/boundingbox.cu
  src/covariance_matrix.cuh
  src/covariance_matrix.cu
  src/em_algorithm.cu
  src/gaussian_mixture.cu
  src/likelihood_matrix.cu
  src/pointcloud.cu
  src/point.cuh
  src/weighted_gaussian.cu
  src/weighted_gaussian.cuh)
target_link_libraries(pclem ${VTK_LIBRARIES}
  ${CUDA_LIBRARIES} ${ARMADILLO_LIBARIRES} glog::glog)

add_executable(run_em src/run_em.cpp src/raw_point_cloud.cpp)
target_link_libraries(run_em pclem)
