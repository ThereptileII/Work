# Reuse OpenCPN's JSON parser in integrated builds. Standalone CI uses the exact
# v1.1.0 archive selected by the pinned OpenCPN libs/rapidjson/CMakeLists.txt,
# strengthened from the upstream MD5 to SHA-256. No AIS network access in CI.
if(TARGET ocpn::rapidjson)
  set(opennav_json_target ocpn::rapidjson)
else()
  include(FetchContent)
  FetchContent_Declare(opennav_rapidjson
    URL https://codeload.github.com/Tencent/rapidjson/tar.gz/refs/tags/v1.1.0
    URL_HASH SHA256=bf7ced29704a1e696fbccf2a2b4ea068e7774fa37f6d7dd4039d0787f8bed98e
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  # Header-only; do not configure RapidJSON's examples or its own test runner.
  FetchContent_GetProperties(opennav_rapidjson)
  if(NOT opennav_rapidjson_POPULATED)
    if(POLICY CMP0169)
      cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Populate(opennav_rapidjson)
  endif()
  add_library(opennav_json_headers INTERFACE)
  target_include_directories(opennav_json_headers SYSTEM INTERFACE "${opennav_rapidjson_SOURCE_DIR}/include")
  set(opennav_json_target opennav_json_headers)
endif()
add_library(opennav_ais_codec "${CMAKE_CURRENT_LIST_DIR}/../src/ais/AisStreamCodec.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../src/ais/AisStreamSession.cpp")
target_link_libraries(opennav_ais_codec PUBLIC opennav_ais PRIVATE ${opennav_json_target})
target_compile_features(opennav_ais_codec PUBLIC cxx_std_17)
set_target_properties(opennav_ais_codec PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16)
  # Same narrowly scoped compatibility as tools/arch-gcc-compat.cmake: the
  # upstream parser contains an unused invalid assignment template. Do not
  # disable other diagnostics or change the pinned dependency's runtime code.
  target_compile_options(opennav_ais_codec PRIVATE -Wno-template-body)
endif()
