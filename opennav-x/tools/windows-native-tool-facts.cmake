# Observe the generator-selected tools after the top-level project() call.
# This file changes no compiler, SDK, runtime, or project options.
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(_xnav_fact_path "${CMAKE_BINARY_DIR}/xnav-native-cmake-tools.txt")
  file(WRITE "${_xnav_fact_path}" "")
  foreach(_xnav_key IN ITEMS CMAKE_GENERATOR CMAKE_GENERATOR_INSTANCE
      CMAKE_GENERATOR_PLATFORM CMAKE_VS_MSBUILD_COMMAND
      CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION CMAKE_VS_PLATFORM_TOOLSET
      CMAKE_LINKER CMAKE_C_COMPILER CMAKE_MAKE_PROGRAM)
    if((NOT _xnav_key STREQUAL "CMAKE_MAKE_PROGRAM" AND
        (NOT DEFINED ${_xnav_key} OR "${${_xnav_key}}" STREQUAL "")) OR
        "${${_xnav_key}}" MATCHES "[\r\n]")
      message(FATAL_ERROR "Missing, empty or multiline native tool fact: ${_xnav_key}")
    endif()
    file(APPEND "${_xnav_fact_path}" "${_xnav_key}=${${_xnav_key}}\n")
  endforeach()
endif()
