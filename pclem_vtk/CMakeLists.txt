cmake_minimum_required (VERSION 3.1)
include(ExternalProject)

project(pclem_vtk)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(VTK REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)

include_directories(src ${VTK_INCLUDE_DIRS} ${pclem_INCLUDE_DIRS})
include(${VTK_USE_FILE})

file(GLOB SOURCES src/*.cpp)

add_library(pclem_vtk ${SOURCES})
target_link_libraries(pclem_vtk pclem glog::glog ${VTK_LIBRARIES})

add_executable(run_hierarchical_em ${SOURCES})
target_link_libraries(run_hierarchical_em pclem_vtk gflags)

set(PUBLIC_HEADERS src/vtk_pointcloud_reader.h src/vtk_visualization.h)

file(COPY ${PUBLIC_HEADERS} DESTINATION ${CMAKE_BINARY_DIR}/include/pclem/)
install(TARGETS pclem_vtk DESTINATION ${INSTALL_LIB_DIR} EXPORT pclem-vtk-targets)
install(FILES ${PUBLIC_HEADERS} DESTINATION ${INSTALL_INCLUDE_DIR})
export(EXPORT pclem-vtk-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/pclem-vtk-targets.cmake)

SET(CMAKE_INSTALL_DIR lib/cmake/pclem} CACHE PATH "The location of the package files.")

install(EXPORT pclem-vtk-targets
  FILE pclem-vtk-targets.cmake
  DESTINATION ${INSTALL_CMAKE_DIR})

option(tests "Build tests." ON)

if(tests)
  enable_testing()
  find_package(GTest)

  set(SHORT_TESTS
    test/test_vtk_visualization.cpp)

  set(LONG_TESTS
    test/test_one_hierarchy.cpp)

  add_executable(tests_vtk test/run_tests.cpp ${SHORT_TESTS})
  target_link_libraries(tests_vtk pclem_vtk ${GTEST_BOTH_LIBRARIES})

  cuda_add_executable(long_tests_vtk test/run_tests.cpp ${LONG_TESTS})
  target_link_libraries(long_tests_vtk pclem_vtk ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES})

  configure_file(res/26.vtk ${CMAKE_BINARY_DIR}/example.vtk)
endif(tests)
