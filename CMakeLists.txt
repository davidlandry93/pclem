cmake_minimum_required (VERSION 2.6)
include(ExternalProject)


project (pclem)

ExternalProject_Add(
  vtk

  PREFIX "vtk"
  GIT_REPOSITORY "https://gitlab.kitware.com/vtk/vtk.git"
  GIT_TAG "v7.0.0")


file(GLOB SOURCES src/*.cpp)


add_executable (run_em ${SOURCES})
