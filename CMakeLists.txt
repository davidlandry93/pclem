cmake_minimum_required (VERSION 2.6)

project (pclem)


file(GLOB SOURCES src/*.cpp)


add_executable (run_em ${SOURCES})
