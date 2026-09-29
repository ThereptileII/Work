# Included only by the explicit OpenNav source hook. Upstream defaults unchanged.
if(NOT EXISTS "${OPENNAV_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "OPENNAV_ROOT must point to the OpenNav X source root")
endif()
set(OPENNAV_BUILD_UI_COMPONENTS ON CACHE BOOL "" FORCE)
set(OPENNAV_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${OPENNAV_ROOT}" "${CMAKE_BINARY_DIR}/opennav")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/OpenCPNIntegration.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/AnchorGeometry.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/OnlineAis.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/OnlineAisOverlay.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/DashboardPresentation.cpp")
# Only the bundled, source-pinned Dashboard opts into transient XNav
# presentation. No third-party plugin ABI or normal upstream build is changed.
if(TARGET dashboard_pi)
  target_include_directories(dashboard_pi PRIVATE "${OPENNAV_ROOT}/src")
  target_compile_definitions(dashboard_pi PRIVATE OPENNAV_DASHBOARD_PLUGIN=1)
endif()
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/NavigationBridge.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/OpenCPNRouteReader.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/PreviewDiagnostics.cpp"
  "${OPENNAV_ROOT}/src/integration/RuntimeDiagnostics.cpp"
  "${OPENNAV_ROOT}/src/integration/InstallerSelfTest.cpp")
add_library(opennav_marine
  "${OPENNAV_ROOT}/src/integration/N2kInstruments.cpp"
  "${OPENNAV_ROOT}/src/integration/NmeaInstruments.cpp"
  "${OPENNAV_ROOT}/src/integration/SignalKInstruments.cpp")
target_include_directories(opennav_marine PUBLIC "${OPENNAV_ROOT}/src" PRIVATE ${wxWidgets_INCLUDE_DIRS})
target_link_libraries(opennav_marine PUBLIC opennav_vessel ocpn::N2KParser ocpn::nmea0183 ocpn::rapidjson ${wxWidgets_LIBRARIES})
target_compile_features(opennav_marine PUBLIC cxx_std_17)
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/MarineBridge.cpp")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/OpenCPNPilot.cpp")
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
# Bound untrusted Signal K before the upstream recursive parser, not only after
# the driver has already decoded it. The pristine build has no OpenNav include.
set_property(SOURCE "${CMAKE_SOURCE_DIR}/model/src/comm_drv_signalk_net.cpp"
  DIRECTORY "${CMAKE_SOURCE_DIR}/model" APPEND PROPERTY COMPILE_DEFINITIONS OPENNAV_X=1)
set_property(SOURCE "${CMAKE_SOURCE_DIR}/model/src/comm_drv_signalk_net.cpp"
  DIRECTORY "${CMAKE_SOURCE_DIR}/model" APPEND PROPERTY INCLUDE_DIRECTORIES "${OPENNAV_ROOT}/src")
target_link_libraries(${PACKAGE_NAME} PRIVATE opennav_integration opennav_platform opennav_ui)
option(OPENNAV_ENABLE_ROUTE_SCENARIO "Compile isolated route integration test driver" OFF)
if(OPENNAV_ENABLE_ROUTE_SCENARIO)
  if(NOT XNAV_ENABLE_TEST_FIXTURES)
    message(FATAL_ERROR "Route scenarios require XNAV_ENABLE_TEST_FIXTURES=ON; they cannot be compiled into the installed product")
  endif()
  if(NOT OCPN_BUILD_TEST)
    message(FATAL_ERROR "Route scenario is permitted only with upstream tests enabled")
  endif()
  target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/tests/RouteProgressScenario.cpp"
    "${OPENNAV_ROOT}/tests/NavigationObjectScenario.cpp")
  target_include_directories(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/tests")
  target_compile_definitions(${PACKAGE_NAME} PRIVATE OPENNAV_ROUTE_TESTS=1)
endif()
if(WIN32 AND OCPN_BUILD_TEST)
    # Non-installed disposable desktop helper. A fixture-free application must
    # also be tested at real Windows DPI without enabling synthetic vessel data.
    add_executable(opennav-test-dpi "${OPENNAV_ROOT}/tests/WindowsDpi.cpp")
    target_compile_features(opennav-test-dpi PRIVATE cxx_std_17)
    target_compile_definitions(opennav-test-dpi PRIVATE _WIN32_WINNT=0x0A00)
    target_link_libraries(opennav-test-dpi PRIVATE user32)
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
    target_sources(tests PRIVATE "${OPENNAV_ROOT}/tests/anchor_view_tests.cpp"
      "${OPENNAV_ROOT}/tests/anchor_geometry_upstream_tests.cpp"
      "${OPENNAV_ROOT}/src/integration/AnchorGeometry.cpp")
    target_sources(tests PRIVATE "${OPENNAV_ROOT}/tests/passage_view_tests.cpp")
    target_compile_definitions(tests PRIVATE OPENNAV_PASSAGE_GTEST=1)
    target_sources(tests PRIVATE "${OPENNAV_ROOT}/tests/instrument_view_tests.cpp")
    target_compile_definitions(tests PRIVATE OPENNAV_INSTRUMENT_GTEST=1)
    add_executable(instrument_panel_test "${OPENNAV_ROOT}/tests/instrument_panel_test.cpp")
    target_link_libraries(instrument_panel_test PRIVATE opennav_ui)
    target_compile_features(instrument_panel_test PRIVATE cxx_std_17)
    add_executable(settings_drawer_test "${OPENNAV_ROOT}/tests/settings_drawer_test.cpp")
    target_link_libraries(settings_drawer_test PRIVATE opennav_ui)
    target_compile_features(settings_drawer_test PRIVATE cxx_std_17)
    add_executable(energy_panel_test "${OPENNAV_ROOT}/tests/energy_panel_test.cpp")
    target_link_libraries(energy_panel_test PRIVATE opennav_ui opennav_integration)
    target_compile_features(energy_panel_test PRIVATE cxx_std_17)
    add_executable(passage_drawer_test "${OPENNAV_ROOT}/tests/passage_drawer_test.cpp")
    target_link_libraries(passage_drawer_test PRIVATE opennav_ui opennav_integration)
    target_compile_features(passage_drawer_test PRIVATE cxx_std_17)
    add_executable(anchor_drawer_test "${OPENNAV_ROOT}/tests/anchor_drawer_test.cpp")
    target_link_libraries(anchor_drawer_test PRIVATE opennav_ui)
    target_compile_features(anchor_drawer_test PRIVATE cxx_std_17)
    # Dedicated component process; never installed, linked into OpenCPN, or
    # enabled by a product flag. It cannot access charts, profiles or hardware.
    add_executable(ais_drawer_test "${OPENNAV_ROOT}/tests/ais_drawer_test.cpp")
    target_link_libraries(ais_drawer_test PRIVATE opennav_ui)
    target_compile_features(ais_drawer_test PRIVATE cxx_std_17)
    if(LINUX)
      find_package(PkgConfig REQUIRED)
      pkg_check_modules(OPENNAV_UI_TEST_GTK REQUIRED IMPORTED_TARGET gtk+-3.0)
      target_link_libraries(ais_drawer_test PRIVATE PkgConfig::OPENNAV_UI_TEST_GTK)
      target_link_libraries(passage_drawer_test PRIVATE PkgConfig::OPENNAV_UI_TEST_GTK)
      target_link_libraries(anchor_drawer_test PRIVATE PkgConfig::OPENNAV_UI_TEST_GTK)
      target_link_libraries(instrument_panel_test PRIVATE PkgConfig::OPENNAV_UI_TEST_GTK)
      target_link_libraries(energy_panel_test PRIVATE PkgConfig::OPENNAV_UI_TEST_GTK)
      target_link_libraries(settings_drawer_test PRIVATE PkgConfig::OPENNAV_UI_TEST_GTK)
    endif()
    target_sources(tests PRIVATE
      "${OPENNAV_ROOT}/tests/route_progress_upstream_tests.cpp"
      "${OPENNAV_ROOT}/tests/marine_decoder_upstream_tests.cpp"
      "${OPENNAV_ROOT}/tests/ais_clock_upstream_tests.cpp"
      "${OPENNAV_ROOT}/tests/settings_store_upstream_tests.cpp"
      "${OPENNAV_ROOT}/tests/online_ais_settings_upstream_tests.cpp"
      "${OPENNAV_ROOT}/src/integration/OnlineAis.cpp"
      "${OPENNAV_ROOT}/tests/recovery_store_upstream_tests.cpp"
      "${OPENNAV_ROOT}/src/integration/RecoveryStore.cpp"
      "${OPENNAV_ROOT}/src/integration/SettingsStore.cpp"
      "${OPENNAV_ROOT}/src/integration/OpenCPNRouteReader.cpp")
    target_include_directories(tests PRIVATE "${OPENNAV_ROOT}/src")
    target_link_libraries(tests PRIVATE opennav_integration opennav_marine opennav_application)
    target_link_libraries(tests PRIVATE opennav_ais opennav_ais_credentials)
    if(LINUX AND TARGET ocpn::libudev)
      target_sources(tests PRIVATE "${OPENNAV_ROOT}/tests/serial_discovery_upstream_tests.cpp")
      foreach(symbol udev_new udev_unref udev_enumerate_new udev_enumerate_unref
                     udev_device_new_from_syspath udev_device_unref)
        target_link_options(tests PRIVATE "LINKER:--wrap=${symbol}")
      endforeach()
    endif()
    if(WIN32)
      target_sources(tests PRIVATE "${OPENNAV_ROOT}/tests/windows_serial_discovery_upstream_tests.cpp")
      target_include_directories(tests PRIVATE "${CMAKE_SOURCE_DIR}")
    endif()
  endif()
endfunction()
cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL opennav_attach_route_tests)

# Local adversarial internet-client tests use the actual patched bundled library.
# This driver is never installed or included in product packages.
if(OCPN_BUILD_TEST)
  add_executable(ais_transport_test_client "${OPENNAV_ROOT}/tests/ais_transport/client.cpp")
  target_link_libraries(ais_transport_test_client PRIVATE ocpn::ixwebsocket)
  target_compile_features(ais_transport_test_client PRIVATE cxx_std_17)
endif()

add_library(opennav_ais_runtime "${OPENNAV_ROOT}/src/ais/AisStreamProvider.cpp")
target_link_libraries(opennav_ais_runtime PUBLIC opennav_ais_codec opennav_ais_credentials PRIVATE ocpn::ixwebsocket)
target_compile_features(opennav_ais_runtime PUBLIC cxx_std_17)
target_link_libraries(${PACKAGE_NAME} PRIVATE opennav_ais_runtime)
if(OCPN_BUILD_TEST)
  # Explicit read-only internet commissioning. No chart, profile, plugins or
  # marine output; never installed or launched by the product.
  add_executable(aisstream_live_probe "${OPENNAV_ROOT}/tools/aisstream-live-probe.cpp")
  target_link_libraries(aisstream_live_probe PRIVATE opennav_ais_runtime)
  add_executable(ais_provider_test_client "${OPENNAV_ROOT}/tests/ais_transport/provider_client.cpp"
    "${OPENNAV_ROOT}/src/ais/AisStreamProvider.cpp")
  target_compile_definitions(ais_provider_test_client PRIVATE OPENNAV_AIS_TEST_TRANSPORT=1)
  target_link_libraries(ais_provider_test_client PRIVATE opennav_ais_codec opennav_ais_credentials ocpn::ixwebsocket)
  target_compile_features(ais_provider_test_client PRIVATE cxx_std_17)
endif()

# Separately owned, deterministically derived presentation resources. Verify
# source bytes before generation; never overwrite the stock s57data directory.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(xnav_chart_style "${CMAKE_BINARY_DIR}/opennav-chart-style/v1")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${OPENNAV_ROOT}/tools/generate-xnav-chart-style.py"
  --source "${CMAKE_SOURCE_DIR}/data/s57data" --output "${xnav_chart_style}"
  RESULT_VARIABLE xnav_style_result)
if(NOT xnav_style_result EQUAL 0)
  message(FATAL_ERROR "XNav presentation resource verification/generation failed")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${OPENNAV_ROOT}/resources/chart-style/v1/definition.json"
  "${OPENNAV_ROOT}/resources/chart-style/v1/source-lock.json"
  "${OPENNAV_ROOT}/docs/design/prototype-tokens.json"
  "${OPENNAV_ROOT}/tools/generate-xnav-chart-style.py")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${OPENNAV_ROOT}/tools/chart_raster_ink.py")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/ChartPresentation.cpp")
target_include_directories(${PACKAGE_NAME} PRIVATE "${xnav_chart_style}")
install(FILES "${xnav_chart_style}/chartsymbols.xml" "${xnav_chart_style}/S52RAZDS.RLE"
  "${xnav_chart_style}/rastersymbols-day.png" "${xnav_chart_style}/rastersymbols-dusk.png"
  "${xnav_chart_style}/rastersymbols-dark.png" "${xnav_chart_style}/manifest.json"
  DESTINATION "${PREFIX_PKGDATA}/opennav/chart-style/v1")
