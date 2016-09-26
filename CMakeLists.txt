cmake_minimum_required (VERSION 3.0)
include(ExternalProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project (pclem)
find_package(VTK)

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

include_directories(src "${VTK_INCLUDE_DIRS}")


file(GLOB SOURCES src/*.cpp)

add_executable (run_em ${SOURCES})
target_link_libraries(run_em ${VTK_LIBRARIES})
