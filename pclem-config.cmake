get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(${SELF_DIR}/pclem-targets.cmake)
get_filename_component(pclem_INCLUDE_DIRS "${SELF_DIR}/../../include/pclem" ABSOLUTE)
