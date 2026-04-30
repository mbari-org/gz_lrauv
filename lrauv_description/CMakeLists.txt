#
# Development of this module has been funded by the Monterey Bay Aquarium
# Research Institute (MBARI) and the David and Lucile Packard Foundation
#

cmake_minimum_required(VERSION 3.5)

project(lrauv_description)

find_program(EMPY_EXECUTABLE NAMES empy em REQUIRED)

include(CTest)
#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)

configure_file(
  "hooks/hook.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.sh" @ONLY
)

#============================================================================
# Model Generation
set(TETHYS_MODEL_TEMPLATE
  ${CMAKE_CURRENT_SOURCE_DIR}/models/tethys/model.sdf.in)
set(TETHYS_MODEL_OUTPUT
  ${CMAKE_CURRENT_BINARY_DIR}/models/tethys/model.sdf)

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${TETHYS_MODEL_TEMPLATE})

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models/tethys)

execute_process(
  COMMAND ${EMPY_EXECUTABLE}
    -o ${TETHYS_MODEL_OUTPUT}
    ${TETHYS_MODEL_TEMPLATE}
  COMMAND_ERROR_IS_FATAL ANY
)

install(DIRECTORY
  models
  ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})

install(FILES
  ${TETHYS_MODEL_OUTPUT}
  DESTINATION share/${PROJECT_NAME}/models/tethys
)
