# SCRUM-247: complete production objects, no executable/dependency library link.
foreach(required OPENNAV_SOURCE_DIR OPENNAV_CHART_LOCAL OPENNAV_CHART_UPSTREAM
    OPENNAV_CHART_SDK OPENNAV_CHART_RESOURCES)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "Missing ${required}")
  endif()
endforeach()
get_filename_component(OPENNAV_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(wxWidgets_USE_REL_AND_DBG OFF)
find_package(wxWidgets REQUIRED COMPONENTS base core adv aui net xml html gl)
include(${wxWidgets_USE_FILE})
include("${OPENNAV_SOURCE_DIR}/VERSION.cmake")
set(PACKAGE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}${VERSION_TAIL}")
set(PKG_TARGET MSVC)
set(PKG_TARGET_VERSION Win32)
set(COMPILER_SUPPORTS_CXX11 1)
set(OPENGL_FOUND 1)
set(OCPN_USE_CURL 1)
set(OCPN_USE_NEWSERIAL 1)
set(OCPN_USE_LZMA 1)
set(USE_GARMINHOST 1)
configure_file("${OPENNAV_SOURCE_DIR}/cmake/in-files/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")
# Actual default production configuration: no unqualified private adapter.
# Windows still compiles the complete loader and hash/refusal branches.
set(skager_ocharts_available false)
set(skager_ocharts_sha256 "")
set(skager_ocharts_bytes 0)
configure_file("${OPENNAV_ROOT}/src/integration/SkagerOChartsPackage.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/SkagerOChartsPackage.h" @ONLY)
# chcanv includes SystemCmdSound even with Windows' native sound backend.
# Reuse the pinned production generator with the Windows default backend flags;
# do not supply a hand-written header or build sound dependencies for this gate.
set(OCPN_ENABLE_PORTAUDIO OFF)
set(OCPN_ENABLE_SNDFILE OFF)
set(OCPN_ENABLE_SYSTEM_CMD_SOUND OFF)
include("${OPENNAV_SOURCE_DIR}/libs/sound/cmake/SoundConfig.cmake")
configure_file("${OPENNAV_SOURCE_DIR}/libs/sound/snd_config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/snd_config.h")
# Derived from pinned root application include block, S52PLIB and transitive
# gui/model/geoprim/gdal/s57/shapefile targets; no replacement headers or PCH.
set(chart_includes
  gui/include/gui include resources gui/src/mbtiles model/include model/include/model buildwin
  libs/gui/include libs/geoprim/src libs/gdal/include libs/gdal/include/gdal
  libs/iso8211/include libs/s52plib/src libs/s57-charts/include
  libs/observable/include libs/pugixml libs/nmea0183/src libs/std_filesystem/include
  libs/wxJSON/include libs/wxcurl/include libs/IXWebSocket libs/tinyxml/include
  libs/libtess2/Include libs/SQLiteCpp/include libs/sqlite/include libs/serial/include
  libs/gl_headers/windows libs/picosha2 libs/lz4/src
  libs/sound/include libs/manual/include libs/ssl_sha1/include
  libs/garmin/jeeps libs/texcmp/squish libs/mipmap/include
  libs/mdns/include libs/mdns/mdns-1.4.3 libs/mongoose/include
  libs/N2KParser/include libs/wxservdisc)
list(TRANSFORM chart_includes PREPEND "${OPENNAV_SOURCE_DIR}/")
list(PREPEND chart_includes
  "${CMAKE_CURRENT_BINARY_DIR}/include" "${OPENNAV_ROOT}/src"
  "${OPENNAV_CHART_RESOURCES}" "${OPENNAV_CHART_SDK}/glew"
  "${OPENNAV_CHART_SDK}/shapelib" "${OPENNAV_CHART_SDK}/rapidjson/include"
  "${OPENNAV_CHART_SDK}/shapefile/lib/include" "${OPENNAV_CHART_SDK}/curl")
foreach(directory IN LISTS chart_includes)
  if(NOT IS_DIRECTORY "${directory}")
    message(FATAL_ERROR "Pinned chart include directory missing: ${directory}")
  endif()
endforeach()
foreach(unit IN LISTS OPENNAV_CHART_LOCAL OPENNAV_CHART_UPSTREAM)
  get_filename_component(name "${unit}" NAME_WE)
  if(unit IN_LIST OPENNAV_CHART_LOCAL)
    set(source "${OPENNAV_ROOT}/${unit}")
  else()
    set(source "${OPENNAV_SOURCE_DIR}/${unit}")
  endif()
  add_library(check_chart_${name} OBJECT "${source}")
  target_include_directories(check_chart_${name} BEFORE PRIVATE ${chart_includes})
  target_compile_features(check_chart_${name} PRIVATE cxx_std_17)
  set_property(TARGET check_chart_${name} PROPERTY MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
  target_compile_options(check_chart_${name} PRIVATE /utf-8 /EHa /diagnostics:column)
  target_compile_definitions(check_chart_${name} PRIVATE
    WIN32 _WINDOWS __MSVC__ _CRT_NONSTDC_NO_DEPRECATE _CRT_SECURE_NO_DEPRECATE
    _HAS_STD_BYTE=0 _DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR PSAPI_VERSION=1
    _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS=1 OPENNAV_X=1
    HAVE_WX_GESTURE_EVENTS __OCPN_USE_GLEW__ ocpnUSE_GL ocpnUSE_GLSL
    ocpnUSE_SVG ocpnUSE_wxBitmapBundle RAPIDJSON_HAS_STDSTRING=1 TIXML_USE_STL
    IXWEBSOCKET_USE_TLS IXWEBSOCKET_USE_OPEN_SSL IXWEBSOCKET_USE_ZLIB
    XNAV_ENABLE_TEST_FIXTURES=0 XNAV_ENABLE_PILOT_LOOPBACK_TESTS=0)
endforeach()
