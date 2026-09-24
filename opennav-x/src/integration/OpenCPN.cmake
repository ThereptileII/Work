# Included only by the explicit OpenNav source hook. Upstream defaults unchanged.
if(NOT EXISTS "${OPENNAV_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "OPENNAV_ROOT must point to the OpenNav X source root")
endif()
set(OPENNAV_BUILD_UI_COMPONENTS ON CACHE BOOL "" FORCE)
set(OPENNAV_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${OPENNAV_ROOT}" "${CMAKE_BINARY_DIR}/opennav")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/OpenCPNIntegration.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/NavigationBridge.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/OpenCPNRouteReader.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/PreviewDiagnostics.cpp"
  "${OPENNAV_ROOT}/src/integration/RuntimeDiagnostics.cpp")
add_library(opennav_marine
  "${OPENNAV_ROOT}/src/integration/N2kInstruments.cpp"
  "${OPENNAV_ROOT}/src/integration/NmeaInstruments.cpp"
  "${OPENNAV_ROOT}/src/integration/SignalKInstruments.cpp")
target_include_directories(opennav_marine PUBLIC "${OPENNAV_ROOT}/src" PRIVATE ${wxWidgets_INCLUDE_DIRS})
target_link_libraries(opennav_marine PUBLIC opennav_vessel ocpn::N2KParser ocpn::nmea0183 ocpn::wxjson ${wxWidgets_LIBRARIES})
target_compile_features(opennav_marine PUBLIC cxx_std_17)
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/MarineBridge.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/NavigationObjects.cpp"
  "${OPENNAV_ROOT}/src/integration/NavigationActions.cpp"
  "${OPENNAV_ROOT}/src/integration/SettingsStore.cpp"
  "${OPENNAV_ROOT}/src/integration/RecoveryStore.cpp")
target_link_libraries(${PACKAGE_NAME} PRIVATE opennav_marine)
set(OPENNAV_BUILD_COMMIT "$ENV{GITHUB_SHA}")
if(NOT OPENNAV_BUILD_COMMIT)
  execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY "${OPENNAV_ROOT}"
    OUTPUT_VARIABLE OPENNAV_BUILD_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
set(OPENNAV_BUILD_RUN "$ENV{GITHUB_RUN_ID}")
if(NOT OPENNAV_BUILD_RUN)
  set(OPENNAV_BUILD_RUN "local development")
endif()
string(TIMESTAMP OPENNAV_BUILD_DATE "%Y-%m-%dT%H:%M:%SZ" UTC)
configure_file("${OPENNAV_ROOT}/src/integration/OpenNavBuild.h.in" "${CMAKE_BINARY_DIR}/include/OpenNavBuild.h" @ONLY)
target_include_directories(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src")
target_compile_definitions(${PACKAGE_NAME} PRIVATE OPENNAV_X=1)
# Preserve normal plugin preferences while upstream Safe Mode blocks loading.
# Limit this additional definition to the one affected model translation unit.
set_property(SOURCE "${CMAKE_SOURCE_DIR}/model/src/plugin_loader.cpp"
  DIRECTORY "${CMAKE_SOURCE_DIR}/model" APPEND PROPERTY COMPILE_DEFINITIONS OPENNAV_X=1)
target_link_libraries(${PACKAGE_NAME} PRIVATE opennav_integration opennav_platform opennav_ui)
option(OPENNAV_ENABLE_ROUTE_SCENARIO "Compile isolated route integration test driver" OFF)
if(OPENNAV_ENABLE_ROUTE_SCENARIO)
  if(NOT OCPN_BUILD_TEST)
    message(FATAL_ERROR "Route scenario is permitted only with upstream tests enabled")
  endif()
  target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/tests/RouteProgressScenario.cpp"
    "${OPENNAV_ROOT}/tests/NavigationObjectScenario.cpp")
  target_include_directories(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/tests")
  target_compile_definitions(${PACKAGE_NAME} PRIVATE OPENNAV_ROUTE_TESTS=1)
  if(WIN32)
    add_executable(opennav-test-dpi "${OPENNAV_ROOT}/tests/WindowsDpi.cpp")
    target_compile_features(opennav-test-dpi PRIVATE cxx_std_17)
    target_compile_definitions(opennav-test-dpi PRIVATE _WIN32_WINNT=0x0A00)
    target_link_libraries(opennav-test-dpi PRIVATE user32)
  endif()
endif()
if(WIN32)
  install(TARGETS opennav-restart RUNTIME DESTINATION .)
  add_custom_command(TARGET ${PACKAGE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:opennav-restart> $<TARGET_FILE_DIR:${PACKAGE_NAME}>)
  add_dependencies(${PACKAGE_NAME} opennav-restart)
endif()

# The upstream test target is declared after this optional integration hook.
# Defer attaching model-bound tests; test sources stay outside upstream.
function(opennav_attach_route_tests)
  if(TARGET tests)
    target_sources(tests PRIVATE
      "${OPENNAV_ROOT}/tests/route_progress_upstream_tests.cpp"
      "${OPENNAV_ROOT}/tests/marine_decoder_upstream_tests.cpp"
      "${OPENNAV_ROOT}/tests/settings_store_upstream_tests.cpp"
      "${OPENNAV_ROOT}/tests/recovery_store_upstream_tests.cpp"
      "${OPENNAV_ROOT}/src/integration/RecoveryStore.cpp"
      "${OPENNAV_ROOT}/src/integration/SettingsStore.cpp"
      "${OPENNAV_ROOT}/src/integration/OpenCPNRouteReader.cpp")
    target_include_directories(tests PRIVATE "${OPENNAV_ROOT}/src")
    target_link_libraries(tests PRIVATE opennav_integration opennav_marine opennav_application)
  endif()
endfunction()
cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL opennav_attach_route_tests)
