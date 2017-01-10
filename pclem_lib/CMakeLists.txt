cmake_minimum_required (VERSION 3.0)
include(ExternalProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project (pclem)
find_package(CUDA REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(src include "${ARMADILLO_INCLUDE_DIRS}")

list(APPEND CUDA_NVCC_FLAGS "-arch=sm_30;-std=c++11;-D_MWAITXINTRIN_H_INCLUDED;-D_FORCE_INLINES")
SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

file(GLOB SOURCES src/*.cpp src/*.cu src/*.h)

cuda_add_library(pclem ${SOURCES})
target_link_libraries(pclem glog::glog ${CUDA_LIBRARIES} ${ARMADILLO_LIBRARIES})

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
  CACHE INTERNAL  "${PROJECT_NAME}: Include Directories" FORCE)

file(GLOB PUBLIC_HEADERS include/*.h)

set(INSTALL_LIB_DIR lib/pclem)
set(INSTALL_INCLUDE_DIR include/pclem)
set(INSTALL_CMAKE_DIR lib/cmake/pclem)


INSTALL(FILES ${PUBLIC_HEADERS} DESTINATION ${INSTALL_INCLUDE_DIR})
INSTALL(TARGETS pclem DESTINATION ${INSTALL_LIB_DIR} EXPORT pclem-targets)
INSTALL(EXPORT pclem-targets DESTINATION ${INSTALL_LIB_DIR})
INSTALL(FILES ${PROJECT_BINARY_DIR}/pclem-config.cmake DESTINATION lib/pclem)
export(PACKAGE pclem)

foreach(p LIB INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")

set(CONF_INCLUDE_DIRS "\${PCLEM_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(pclem-config.cmake.in ${PROJECT_BINARY_DIR}/pclem-config.cmake @ONLY)

option(tests "Build tests." ON) # Turn off with -Dtests=OFF

if(tests)
  enable_testing()
  find_package(GTest)
  find_package(GMock)

  file(GLOB TEST_FILES test/*.cpp test/*.cu)
  cuda_add_executable(tests ${TEST_FILES})
  target_link_libraries(tests pclem ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})
endif(tests)
