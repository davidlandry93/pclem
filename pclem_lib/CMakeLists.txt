cmake_minimum_required (VERSION 3.0)
include(ExternalProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project (pclem)
find_package(glog REQUIRED)
find_package(CUDA REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(src include "${ARMADILLO_INCLUDE_DIRS}")

list(APPEND CUDA_NVCC_FLAGS "-arch=sm_30;-std=c++11;-D_MWAITXINTRIN_H_INCLUDED;-D_FORCE_INLINES")
SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

file(GLOB SOURCES src/*.cpp src/*.cu src/*.h)

cuda_add_library(pclem ${SOURCES})
target_link_libraries(pclem glog::glog ${CUDA_LIBRARIES} ${ARMADILLO_LIBRARIES})

target_include_directories(pclem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include/pclem>)

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
  CACHE INTERNAL  "${PROJECT_NAME}: Include Directories" FORCE)

file(GLOB PUBLIC_HEADERS include/*.h)
file(COPY ${PUBLIC_HEADERS} DESTINATION ${CMAKE_BINARY_DIR}/include/pclem/)

INSTALL(FILES ${PUBLIC_HEADERS} DESTINATION ${INSTALL_INCLUDE_DIR})
INSTALL(TARGETS pclem DESTINATION ${INSTALL_LIB_DIR} EXPORT pclem-targets)
export(EXPORT pclem-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/pclem-targets.cmake)

set(CMAKE_INSTALL_DIR lib/cmake/pclem CACHE PATH "The location of the package files.")

INSTALL(EXPORT pclem-targets DESTINATION ${CMAKE_INSTALL_DIR})

option(tests "Build tests." ON) # Turn off with -Dtests=OFF

if(tests)
  enable_testing()
  find_package(GTest)
  find_package(GMock)

  set(SHORT_TESTS
    test/test_gaussian_in_visualization.cpp
    test/test_gaussian_mixture.cpp
    test/test_gaussian_mixture_factory.cpp
    test/test_rotation.cpp)

  cuda_add_executable(tests test/run_tests.cpp ${SHORT_TESTS})
  target_link_libraries(tests pclem ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
endif(tests)
