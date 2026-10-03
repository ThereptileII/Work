add_executable(downloader-trust-probe
  "${OPENNAV_SOURCE_DIR}/model/src/downloader.cpp"
  "${OPENNAV_TOOLS_DIR}/downloader-trust-probe.cpp")
target_compile_features(downloader-trust-probe PRIVATE cxx_std_17)
target_compile_definitions(downloader-trust-probe PRIVATE _CRT_SECURE_NO_WARNINGS)
target_include_directories(downloader-trust-probe PRIVATE
  "${OPENNAV_SOURCE_DIR}/model/include"
  "${CURL_ROOT}/include")
target_link_libraries(downloader-trust-probe PRIVATE
  "${CURL_ROOT}/libcurl.lib" ${wxWidgets_LIBRARIES})
set_property(TARGET downloader-trust-probe PROPERTY
  MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

add_executable(wxcurl-trust-probe
  "${trust_wxcurl_source}/base.cpp"
  "${trust_wxcurl_source}/http.cpp"
  "${OPENNAV_TOOLS_DIR}/wxcurl-trust-probe.cpp")
target_compile_features(wxcurl-trust-probe PRIVATE cxx_std_17)
target_compile_definitions(wxcurl-trust-probe PRIVATE _CRT_SECURE_NO_WARNINGS)
# Maintained curl must precede wxCurl's bundled legacy curl snapshot.
target_include_directories(wxcurl-trust-probe BEFORE PRIVATE "${CURL_ROOT}/include")
target_include_directories(wxcurl-trust-probe PRIVATE
  "${trust_wxcurl_include}")
target_link_libraries(wxcurl-trust-probe PRIVATE
  "${CURL_ROOT}/libcurl.lib" ${wxWidgets_LIBRARIES})
set_property(TARGET wxcurl-trust-probe PROPERTY
  MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

# Deliberately no OPENNAV_DOWNLOADER_TLS_TEST definition: this executable must
# exercise the production _WIN32/CURLSSLOPT_NATIVE_CA path.
# The wxCurl target likewise has no OPENNAV_WXCURL_TLS_TEST definition.
