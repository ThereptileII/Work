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

option(OPENNAV_CHECK_UI "Compile the real UI target and complete SettingsStore" OFF)
option(OPENNAV_CHECK_SETTINGS_COMPONENT "Link the offline Settings component for native interaction checks" OFF)
option(OPENNAV_CHECK_PROTOTYPE "Compile complete navigation bridge units and run offline prototype components" OFF)
option(OPENNAV_CHECK_CHART_COMPONENT "Link the offline chart-presentation component" OFF)
if(OPENNAV_CHECK_CHART_COMPONENT AND NOT OPENNAV_CHECK_PROTOTYPE)
  message(FATAL_ERROR "Chart component requires the prototype proof")
endif()
if((OPENNAV_CHECK_SETTINGS_COMPONENT OR OPENNAV_CHECK_PROTOTYPE) AND NOT OPENNAV_CHECK_UI)
  message(FATAL_ERROR "Settings component requires the production UI target")
endif()
if(OPENNAV_CHECK_UI)
  get_filename_component(OPENNAV_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
  # Reuse the production target and its source list, not a mirrored UI fixture.
  # Base gate compiles static libraries; the optional offline component below
  # adds no OpenCPN application link or dependency producer build.
  set(OPENNAV_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(OPENNAV_BUILD_UI_COMPONENTS ON CACHE BOOL "" FORCE)
  set(XNAV_ENABLE_TEST_FIXTURES OFF CACHE BOOL "" FORCE)
  set(XNAV_ENABLE_PILOT_LOOPBACK_TESTS OFF CACHE BOOL "" FORCE)
  set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
  add_subdirectory("${OPENNAV_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/opennav")
  add_library(check_SettingsStore OBJECT "${OPENNAV_ROOT}/src/integration/SettingsStore.cpp")
  target_include_directories(check_SettingsStore PRIVATE "${OPENNAV_ROOT}/src" ${wxWidgets_INCLUDE_DIRS})
  target_link_libraries(check_SettingsStore PRIVATE opennav_build_policy)
  target_compile_features(check_SettingsStore PRIVATE cxx_std_17)
  target_compile_options(check_SettingsStore PRIVATE /utf-8 /EHa /diagnostics:column)
  target_compile_definitions(check_SettingsStore PRIVATE
    WIN32 _WINDOWS __MSVC__ _HAS_STD_BYTE=0 _DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR)
  if(OPENNAV_CHECK_SETTINGS_COMPONENT)
    # Existing non-installed component: no OpenCPN app, profile or devices.
    add_executable(settings_drawer_test "${OPENNAV_ROOT}/tests/settings_drawer_test.cpp")
    target_link_libraries(settings_drawer_test PRIVATE opennav_ui)
    target_compile_features(settings_drawer_test PRIVATE cxx_std_17)
    target_compile_options(settings_drawer_test PRIVATE /utf-8)
  endif()
endif()

if(OPENNAV_CHECK_PROTOTYPE)
  if(NOT IS_DIRECTORY "${OPENNAV_GLEW_INCLUDE}" OR NOT EXISTS "${OPENNAV_GLEW_INCLUDE}/glew.h")
    message(FATAL_ERROR "Prototype proof requires the locked production Windows GLEW header")
  endif()
  # Derived from pinned OpenCPN CMakeLists.txt's application include block and
  # its transitive gui/geoprim/gdal/iso8211/s52plib/s57-charts/model targets.
  # Complete source files below compile as production Win32 objects. No app
  # link, producer dependency build, header substitution or macro suppression.
  set(navigation_includes
    gui/include/gui include resources gui/src/mbtiles gui/src/s57/include
    model/include model/include/model buildwin
    libs/gui/include libs/geoprim/src libs/gdal/include libs/gdal/include/gdal
    libs/iso8211/include libs/s52plib/src libs/s57-charts/include
    libs/observable/include libs/pugixml libs/nmea0183/src
    libs/std_filesystem/include libs/wxJSON/include libs/wxcurl/include
    libs/IXWebSocket libs/tinyxml/include libs/libtess2/Include
    libs/SQLiteCpp/include libs/sqlite/include libs/serial/include
    libs/gl_headers/windows)
  list(TRANSFORM navigation_includes PREPEND "${OPENNAV_SOURCE_DIR}/")
  foreach(directory IN LISTS navigation_includes)
    if(NOT IS_DIRECTORY "${directory}")
      message(FATAL_ERROR "Pinned navigation include directory missing: ${directory}")
    endif()
  endforeach()
  # The same config.h.in as production, with the installed Windows feature
  # profile needed by these headers. Other probe families keep their old config.
  set(OPENGL_FOUND 1)
  set(OCPN_USE_NEWSERIAL 1)
  set(OCPN_USE_LZMA 1)
  configure_file("${OPENNAV_SOURCE_DIR}/cmake/in-files/config.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/navigation-include/config.h")
  foreach(name NavigationActions NavigationObjects)
    add_library(check_${name} OBJECT "${OPENNAV_ROOT}/src/integration/${name}.cpp")
    target_include_directories(check_${name} BEFORE PRIVATE
      "${CMAKE_CURRENT_BINARY_DIR}/navigation-include" "${OPENNAV_ROOT}/src"
      "${CURL_INCLUDE}" "${OPENNAV_GLEW_INCLUDE}" ${navigation_includes})
    target_link_libraries(check_${name} PRIVATE opennav_build_policy opennav_json_headers)
    target_compile_features(check_${name} PRIVATE cxx_std_17)
    target_compile_options(check_${name} PRIVATE /utf-8 /EHa /diagnostics:column)
    target_compile_definitions(check_${name} PRIVATE
      WIN32 _WINDOWS __MSVC__ _CRT_NONSTDC_NO_DEPRECATE _CRT_SECURE_NO_DEPRECATE
      _HAS_STD_BYTE=0 _DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR PSAPI_VERSION=1
      _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS=1
      IXWEBSOCKET_USE_TLS IXWEBSOCKET_USE_OPEN_SSL IXWEBSOCKET_USE_ZLIB
      OPENNAV_X=1 HAVE_WX_GESTURE_EVENTS __OCPN_USE_GLEW__
      ocpnUSE_GL ocpnUSE_GLSL ocpnUSE_SVG ocpnUSE_wxBitmapBundle
      RAPIDJSON_HAS_STDSTRING=1 TIXML_USE_STL)
  endforeach()
  add_executable(search_drawer_test "${OPENNAV_ROOT}/tests/search_drawer_test.cpp")
  target_link_libraries(search_drawer_test PRIVATE opennav_ui opennav_integration)
  target_compile_features(search_drawer_test PRIVATE cxx_std_17)
  target_compile_options(search_drawer_test PRIVATE /utf-8)
  if(OPENNAV_CHECK_CHART_COMPONENT)
    add_executable(chart_presentation_drawer_test "${OPENNAV_ROOT}/tests/chart_presentation_drawer_test.cpp")
    target_link_libraries(chart_presentation_drawer_test PRIVATE opennav_ui)
    target_compile_features(chart_presentation_drawer_test PRIVATE cxx_std_17)
    target_compile_options(chart_presentation_drawer_test PRIVATE /utf-8)
  endif()
endif()
