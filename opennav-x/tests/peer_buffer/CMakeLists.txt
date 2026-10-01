cmake_minimum_required(VERSION 3.20)
project(opennav_peer_buffer_tests LANGUAGES CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT DEFINED PEER_CLIENT_SOURCE OR NOT EXISTS "${PEER_CLIENT_SOURCE}")
  message(FATAL_ERROR "PEER_CLIENT_SOURCE must name the patched peer_client.cpp")
endif()
if(NOT DEFINED OPENNAV_ROOT)
  message(FATAL_ERROR "OPENNAV_ROOT is required")
endif()

set(extracted "${CMAKE_CURRENT_BINARY_DIR}/peer_buffer_under_test.inc")
add_custom_command(
  OUTPUT "${extracted}"
  COMMAND "${Python3_EXECUTABLE}" "${OPENNAV_ROOT}/tools/extract-peer-buffer.py"
          --source "${PEER_CLIENT_SOURCE}" --output "${extracted}"
  DEPENDS "${PEER_CLIENT_SOURCE}" "${OPENNAV_ROOT}/tools/extract-peer-buffer.py"
  VERBATIM)

add_executable(peer_buffer_tests peer_buffer_tests.cpp "${extracted}")
target_compile_features(peer_buffer_tests PRIVATE cxx_std_17)
target_include_directories(peer_buffer_tests PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
if(MSVC)
  target_compile_options(peer_buffer_tests PRIVATE /W4 /WX)
else()
  target_compile_options(peer_buffer_tests PRIVATE -Wall -Wextra -Werror)
endif()

enable_testing()
add_test(NAME peer_buffer_tests COMMAND peer_buffer_tests)
