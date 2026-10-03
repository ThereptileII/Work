cmake_minimum_required(VERSION 3.20)
project(skager_trust_compile_probe LANGUAGES CXX)
if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
  message(FATAL_ERROR "Trust compile preflight requires native MSVC Win32")
endif()
get_filename_component(TRUST_RECIPE "${CMAKE_CURRENT_LIST_DIR}/../downloader_trust" ABSOLUTE)
include("${TRUST_RECIPE}/InputPaths.cmake")
# Compile-only source input was independently fetched and patched from the same
# private lock by the runner. It is deliberately not a production preparation.
if(TRUST_PRIVATE_SOURCE)
  string(REPLACE "\\" "/" TRUST_PRIVATE_SOURCE "${TRUST_PRIVATE_SOURCE}")
  set(trust_wxcurl_source "${TRUST_PRIVATE_SOURCE}/libs/wxcurl/src")
  set(trust_wxcurl_include "${trust_wxcurl_source}")
else()
  set(trust_wxcurl_source "${OPENNAV_SOURCE_DIR}/libs/wxcurl/src")
  set(trust_wxcurl_include "${OPENNAV_SOURCE_DIR}/libs/wxcurl/include")
endif()
set(wxWidgets_USE_REL_AND_DBG OFF)
find_package(wxWidgets REQUIRED COMPONENTS base core)
include(${wxWidgets_USE_FILE})
# Exact executable source lists, settings and link declarations. The runner calls
# MSBuild /t:ClCompile only; no curl import library, executable or TLS run exists.
include("${TRUST_RECIPE}/Targets.cmake")
