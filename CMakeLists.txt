
cmake_minimum_required(VERSION 3.0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")

set(INSTALL_LIB_DIR lib/pclem)
set(INSTALL_INCLUDE_DIR include/pclem)
set(INSTALL_CMAKE_DIR lib/cmake/pclem CACHE PATH "The location of the package files.")

add_subdirectory(pclem_lib)
add_subdirectory(pclem_vtk)

foreach(p LIB INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")

set(CONF_INCLUDE_DIRS "\${PCLEM_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(pclem-config.cmake.in ${PROJECT_BINARY_DIR}/pclem-config.cmake @ONLY)
INSTALL(FILES ${PROJECT_BINARY_DIR}/pclem-config.cmake DESTINATION ${INSTALL_CMAKE_DIR})

export(PACKAGE pclem)
