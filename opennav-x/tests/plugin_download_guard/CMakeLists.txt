cmake_minimum_required(VERSION 3.20)
project(plugin_download_guard LANGUAGES CXX)
if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
  message(FATAL_ERROR "Native MSVC Win32 required")
endif()
foreach(required SOURCE_DIR TOOLS_DIR CURL_INCLUDE CURL_IMPORT ARCHIVE_INCLUDE ARCHIVE_IMPORT)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "Missing ${required}")
  endif()
endforeach()
set(wxWidgets_USE_REL_AND_DBG OFF)
find_package(wxWidgets REQUIRED COMPONENTS base core)
include(${wxWidgets_USE_FILE})
add_executable(plugin-download-guard-probe
  "${SOURCE_DIR}/model/src/downloader.cpp"
  "${TOOLS_DIR}/plugin-download-guard-probe.cpp")
target_compile_features(plugin-download-guard-probe PRIVATE cxx_std_17)
target_compile_definitions(plugin-download-guard-probe PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
target_include_directories(plugin-download-guard-probe PRIVATE
  "${SOURCE_DIR}" "${SOURCE_DIR}/model/include" "${CURL_INCLUDE}" "${ARCHIVE_INCLUDE}")
target_link_libraries(plugin-download-guard-probe PRIVATE
  "${CURL_IMPORT}" "${ARCHIVE_IMPORT}" ${wxWidgets_LIBRARIES})
set_property(TARGET plugin-download-guard-probe PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
# No Linux wx shim, OPENNAV_DOWNLOADER_TLS_TEST, or LIBARCHIVE_STATIC.
