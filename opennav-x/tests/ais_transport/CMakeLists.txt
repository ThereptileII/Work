cmake_minimum_required(VERSION 3.24)
project(OpenNavAisTransportTests LANGUAGES C CXX)
if(NOT OPENNAV_PINNED_SOURCE)
  message(FATAL_ERROR "Specify an inspected/patched pinned source worktree")
endif()
set(USE_TLS ON CACHE BOOL "" FORCE)
set(USE_OPEN_SSL ON CACHE BOOL "" FORCE)
set(USE_ZLIB ON CACHE BOOL "" FORCE)
add_subdirectory("${OPENNAV_PINNED_SOURCE}/libs/IXWebSocket" ixwebsocket)
add_executable(ais_transport_test_client client.cpp)
target_link_libraries(ais_transport_test_client PRIVATE ocpn::ixwebsocket)
target_compile_features(ais_transport_test_client PRIVATE cxx_std_17)

set(OPENNAV_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../.." opennav)
add_executable(ais_provider_test_client provider_client.cpp ../../src/ais/AisStreamProvider.cpp)
target_compile_definitions(ais_provider_test_client PRIVATE OPENNAV_AIS_TEST_TRANSPORT=1)
target_link_libraries(ais_provider_test_client PRIVATE opennav_ais_codec opennav_ais_credentials ocpn::ixwebsocket)
target_compile_features(ais_provider_test_client PRIVATE cxx_std_17)

# Explicit commissioning binary uses the production endpoint/credential factory.
# It is not installed and is never executed against the live service by CI.
add_executable(aisstream_live_probe ../../tools/aisstream-live-probe.cpp ../../src/ais/AisStreamProvider.cpp)
target_link_libraries(aisstream_live_probe PRIVATE opennav_ais_codec opennav_ais_credentials ocpn::ixwebsocket)
target_compile_features(aisstream_live_probe PRIVATE cxx_std_17)

# Explicitly selected real-clock >120s report-outage proof, excluded from normal
# builds/suites. This is a loopback fixture, not a firewall or live-AIS switch.
add_executable(ais_outage_test_client EXCLUDE_FROM_ALL outage_client.cpp ../../src/ais/AisStreamProvider.cpp)
target_compile_definitions(ais_outage_test_client PRIVATE OPENNAV_AIS_TEST_TRANSPORT=1)
target_link_libraries(ais_outage_test_client PRIVATE opennav_ais_codec opennav_ais_credentials ocpn::ixwebsocket)
target_compile_features(ais_outage_test_client PRIVATE cxx_std_17)
