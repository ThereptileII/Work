cmake_minimum_required(VERSION 3.20)
project(skager_ocharts_adapter VERSION 2.0.0.49 LANGUAGES C CXX)
if(NOT WIN32 OR NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
  message(FATAL_ERROR "Adapter requires native MSVC Win32")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(SKAGER_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${SKAGER_ROOT}/tools/prepare-ocharts-adapter.py"
  --verify-prepared "${SKAGER_PREPARED}" COMMAND_ERROR_IS_FATAL ANY)
set(wxWidgets_ROOT_DIR "${SKAGER_PREPARED}/sdk/wx")
set(wxWidgets_LIB_DIR "${SKAGER_PREPARED}/sdk/wx/lib/vc14x_dll")
set(wxWidgets_CONFIGURATION mswu)
set(wxWidgets_USE_DEBUG OFF)
set(wxWidgets_USE_STATIC OFF)
find_package(wxWidgets 3.2 REQUIRED COMPONENTS base core net xml html adv stc aui gl)
include(${wxWidgets_USE_FILE})
find_package(OpenGL REQUIRED)
# The preparer supplies only verified, open source and qualified SDK outputs.
set(P "${SKAGER_PREPARED}/source")
set(API_VERSION "1.17")
configure_file("${P}/config.h.in" "${CMAKE_BINARY_DIR}/include/config.h" @ONLY)
include_directories(BEFORE "${P}/opencpn-libs/api-17" "${CMAKE_BINARY_DIR}/include"
  "${P}/src" "${P}/libs/gdal/src" "${SKAGER_PREPARED}/local/src"
  "${SKAGER_PREPARED}/resources" "${SKAGER_PREPARED}/sdk/include"
  "${SKAGER_PREPARED}/sdk/glew/include/glew")
add_compile_definitions(UNICODE _UNICODE ocpnUSE_GL ocpnUSE_GLSL __OCPN_USE_GLEW__
  __OCPN_USE_CURL__ SKAGER_OCHARTS_ADAPTER _CRT_NONSTDC_NO_DEPRECATE _CRT_SECURE_NO_DEPRECATE)
# API17 otherwise dllexports host API classes and their generated methods.
# The .def owns create/destroy; private bind/status have explicit exports.
add_compile_definitions("DECL_EXP=")
add_subdirectory("${P}/opencpn-libs/api-17" api)
foreach(lib cpl dsa wxJSON iso8211 tinyxml geoprim pugixml s52plib)
  add_subdirectory("${P}/libs/${lib}" "libs/${lib}")
endforeach()
file(GLOB WXCURL CONFIGURE_DEPENDS "${P}/libs/wxcurl/src/*.cpp")
add_library(skager_wxcurl STATIC ${WXCURL})
target_include_directories(skager_wxcurl PUBLIC "${P}/libs/wxcurl/src")
target_link_libraries(skager_wxcurl PUBLIC "${SKAGER_PREPARED}/sdk/lib/libcurl.lib")
set(SRC chart.cpp cutil.cpp eSENCChart.cpp fpr.cpp georef.cpp InstallDirs.cpp
  ochartShop.cpp o-charts_pi.cpp OCPNRegion.cpp oernc_inStream.cpp Osenc.cpp
  piScreenLog.cpp s57RegistrarMgr.cpp sha256.c uKey.cpp validator.cpp viewport.cpp)
list(TRANSFORM SRC PREPEND "${P}/src/")
add_library(skager-ocharts-adapter SHARED ${SRC} "${P}/libs/gdal/src/s57classregistrar.cpp"
  "${SKAGER_PREPARED}/local/src/plugin-adapters/ocharts/ChartPresentationAdapter.cpp"
  "${SKAGER_PREPARED}/local/src/integration/ChartNameAlphaWindows.cpp")
file(WRITE "${CMAKE_BINARY_DIR}/adapter.def"
  "LIBRARY skager-ocharts-adapter\nEXPORTS\n create_pi\n destroy_pi\n skager_bind_chart_presentation_v1\n skager_chart_presentation_status_v1\n")
target_sources(skager-ocharts-adapter PRIVATE "${CMAKE_BINARY_DIR}/adapter.def")
set_target_properties(skager-ocharts-adapter PROPERTIES PREFIX "" OUTPUT_NAME "skager-ocharts-adapter")
target_link_libraries(skager-ocharts-adapter PRIVATE ocpn::api_wx32
  ocpn::cpl ocpn::dsa ocpn::wxjson ocpn::iso8211 ocpn::tinyxml ocpn::geoprim
  ocpn::pugixml ocpn::s52plib skager_wxcurl ${wxWidgets_LIBRARIES}
  "${SKAGER_PREPARED}/sdk/lib/zlib1.lib" "${SKAGER_PREPARED}/sdk/glew/glew32.lib"
  OpenGL::GL OpenGL::GLU gdiplus)
# Deliberately no upstream install/package target, oeserverd, helper, chart or license payload.
