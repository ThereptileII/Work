cmake_minimum_required(VERSION 3.20)
project(opennav_downloader_trust_probe LANGUAGES CXX)

if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
  message(FATAL_ERROR "The downloader trust probe requires native MSVC Win32")
endif()

foreach(required IN ITEMS OPENNAV_SOURCE_DIR OPENNAV_TOOLS_DIR CURL_ROOT)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "${required} is required")
  endif()
endforeach()

include("${CMAKE_CURRENT_LIST_DIR}/InputPaths.cmake")

# The default still qualifies core wxCurl. An explicit verified adapter workspace
# selects its actual production copy; the core downloader controls remain intact.
set(trust_wxcurl_source "${OPENNAV_SOURCE_DIR}/libs/wxcurl/src")
set(trust_wxcurl_include "${OPENNAV_SOURCE_DIR}/libs/wxcurl/include")
if(SKAGER_OCHARTS_PREPARED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  execute_process(COMMAND "${Python3_EXECUTABLE}"
    "${OPENNAV_TOOLS_DIR}/prepare-ocharts-adapter.py" --verify-prepared
    "${SKAGER_OCHARTS_PREPARED}" COMMAND_ERROR_IS_FATAL ANY)
  set(trust_wxcurl_source "${SKAGER_OCHARTS_PREPARED}/source/libs/wxcurl/src")
  set(trust_wxcurl_include "${trust_wxcurl_source}")
endif()

set(wxWidgets_USE_REL_AND_DBG OFF)
find_package(wxWidgets REQUIRED COMPONENTS base core)
include(${wxWidgets_USE_FILE})

foreach(required_file IN ITEMS
    "${OPENNAV_SOURCE_DIR}/model/src/downloader.cpp"
    "${OPENNAV_SOURCE_DIR}/model/include/model/downloader.h"
    "${OPENNAV_TOOLS_DIR}/downloader-trust-probe.cpp"
    "${trust_wxcurl_source}/base.cpp"
    "${trust_wxcurl_source}/http.cpp"
    "${trust_wxcurl_include}/wx/curl/base.h"
    "${OPENNAV_TOOLS_DIR}/wxcurl-trust-probe.cpp"
    "${CURL_ROOT}/include/curl/curl.h"
    "${CURL_ROOT}/libcurl.lib")
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR "Required native trust input is missing: ${required_file}")
  endif()
endforeach()

include("${CMAKE_CURRENT_LIST_DIR}/Targets.cmake")
