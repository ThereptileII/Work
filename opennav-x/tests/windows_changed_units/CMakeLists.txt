cmake_minimum_required(VERSION 3.20)
project(opennav_windows_changed_units LANGUAGES CXX)
if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
  message(FATAL_ERROR "Native MSVC Win32 required")
endif()
foreach(required OPENNAV_SOURCE_DIR CURL_INCLUDE)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "Missing ${required}")
  endif()
endforeach()
set(wxWidgets_USE_REL_AND_DBG OFF)
find_package(wxWidgets REQUIRED COMPONENTS base core)
include(${wxWidgets_USE_FILE})

# peer_client's SemanticVersion header includes the real generated config.h.
# This narrow compile configuration is not the full application's feature probe.
include("${OPENNAV_SOURCE_DIR}/VERSION.cmake")
set(PACKAGE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}${VERSION_TAIL}")
set(PKG_TARGET "MSVC")
set(PKG_TARGET_VERSION "Win32")
set(COMPILER_SUPPORTS_CXX11 1)
set(OCPN_USE_CURL 1)
configure_file("${OPENNAV_SOURCE_DIR}/cmake/in-files/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

# Complete production translation units, never extracted callbacks or wx shims.
# Object compilation needs headers only: no dependency build, link or execution.
set(units
  model/src/downloader.cpp
  model/src/peer_client.cpp
  libs/wxcurl/src/base.cpp
  libs/wxcurl/src/http.cpp)
foreach(unit IN LISTS units)
  get_filename_component(name "${unit}" NAME_WE)
  add_library("check_${name}" OBJECT "${OPENNAV_SOURCE_DIR}/${unit}")
  target_compile_features("check_${name}" PRIVATE cxx_std_17)
  set_property(TARGET "check_${name}" PROPERTY MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
  target_compile_options("check_${name}" PRIVATE /EHa /diagnostics:column)
  # Match the relevant production Windows model definitions. In particular,
  # NEVER add NOMINMAX, test-only macros, forced includes, or precompiled headers.
  target_compile_definitions("check_${name}" PRIVATE
    WIN32 _WINDOWS __MSVC__ _CRT_NONSTDC_NO_DEPRECATE
    _CRT_SECURE_NO_DEPRECATE _HAS_STD_BYTE=0
    _DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR
    _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS=1
    IXWEBSOCKET_USE_TLS IXWEBSOCKET_USE_OPEN_SSL IXWEBSOCKET_USE_ZLIB)
  # Maintained curl precedes wxCurl's bundled legacy header snapshot.
  target_include_directories("check_${name}" BEFORE PRIVATE "${CURL_INCLUDE}")
  target_include_directories("check_${name}" PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/include"
    "${OPENNAV_SOURCE_DIR}/model/include"
    "${OPENNAV_SOURCE_DIR}/model/include/model"
    "${OPENNAV_SOURCE_DIR}/include"
    "${OPENNAV_SOURCE_DIR}/libs/observable/include"
    "${OPENNAV_SOURCE_DIR}/libs/pugixml"
    "${OPENNAV_SOURCE_DIR}/libs/geoprim/src"
    "${OPENNAV_SOURCE_DIR}/libs/nmea0183/src"
    "${OPENNAV_SOURCE_DIR}/libs/s52plib/src"
    "${OPENNAV_SOURCE_DIR}/libs/std_filesystem/include"
    "${OPENNAV_SOURCE_DIR}/libs/wxJSON/include"
    "${OPENNAV_SOURCE_DIR}/libs/wxcurl/include"
    "${OPENNAV_SOURCE_DIR}/libs/IXWebSocket")
endforeach()
