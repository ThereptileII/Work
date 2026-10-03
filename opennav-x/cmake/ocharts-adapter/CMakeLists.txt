cmake_minimum_required(VERSION 3.20)
project(skager_ocharts_adapter VERSION 2.0.0.49 LANGUAGES C CXX)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(SKAGER_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${CMAKE_CURRENT_LIST_DIR}/PreparedPath.cmake")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${SKAGER_ROOT}/tools/prepare-ocharts-adapter.py"
  --verify-prepared "${SKAGER_PREPARED}" COMMAND_ERROR_IS_FATAL ANY)
include("${CMAKE_CURRENT_LIST_DIR}/Targets.cmake")
