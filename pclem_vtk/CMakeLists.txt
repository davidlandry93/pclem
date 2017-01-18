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

install(TARGETS pclem_vtk DESTINATION ${INSTALL_LIB_DIR} EXPORT pclem-vtk-targets)
install(FILES ${PUBLIC_HEADERS} DESTINATION ${INSTALL_INCLUDE_DIR})
install(EXPORT pclem-vtk-targets DESTINATION ${INSTALL_LIB_DIR})

option(tests "Build tests." ON)

if(tests)
  enable_testing()
  find_package(GTest)

  set(TESTS test/vtk_visualization.cpp)
  add_executable(tests_vtk test/run_tests.cpp ${TEST_FILES})
  target_link_libraries(tests_vtk pclem_vtk ${GTEST_BOTH_LIBRARIES})

  set(LONG_TESTS test/test_hierarchy.cpp)
  add_executable(long_tests_vtk test/run_tests.cpp ${LONG_TESTS})
  target_link_libraries(long_tests_vtk pclem_vtk ${GTEST_BOTH_LIBRARIES})
  configure_file(res/26.vtk ${CMAKE_BINARY_DIR}/example.vtk)
endif(tests)
