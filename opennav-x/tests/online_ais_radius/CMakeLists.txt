cmake_minimum_required(VERSION 3.24)
project(OnlineAisRadiusTests LANGUAGES CXX)
find_package(wxWidgets 3.2 REQUIRED COMPONENTS base)
include(${wxWidgets_USE_FILE})
# Point this at the same pinned RapidJSON headers used by the product build.
find_path(RAPIDJSON_INCLUDE_DIR rapidjson/document.h REQUIRED)
add_library(online_ais_radius_model STATIC
  ../../src/integration/OnlineAis.cpp ../../src/integration/AisViewport.cpp
  ../../src/ais/Subscription.cpp ../../src/ais/AisStreamSession.cpp
  ../../src/ais/AisStreamCodec.cpp ../../src/ais/TargetCache.cpp
  ../../src/ais/Provider.cpp ../../src/ais/ChartTargets.cpp
  ../../src/ais/Credentials.cpp ../../src/vessel/VesselState.cpp
  ../../src/vessel/AisSelection.cpp)
target_include_directories(online_ais_radius_model PUBLIC ../../src)
target_include_directories(online_ais_radius_model SYSTEM PRIVATE "${RAPIDJSON_INCLUDE_DIR}")
target_compile_features(online_ais_radius_model PUBLIC cxx_std_17)
target_link_libraries(online_ais_radius_model PUBLIC ${wxWidgets_LIBRARIES})
if(MSVC)
  target_compile_options(online_ais_radius_model PRIVATE /utf-8 /W4)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16)
  target_compile_options(online_ais_radius_model PRIVATE -Wno-template-body)
endif()
add_executable(online_ais_radius_test main.cpp)
target_link_libraries(online_ais_radius_test PRIVATE online_ais_radius_model)
enable_testing()
add_test(NAME online_ais_radius COMMAND online_ais_radius_test)
set_tests_properties(online_ais_radius PROPERTIES TIMEOUT 20)

# Existing contracts verify the default viewport policy and parser stay intact.
foreach(test online_ais_tests ais_session_tests aisstream_codec_tests)
  add_executable(${test} ../${test}.cpp)
  target_link_libraries(${test} PRIVATE online_ais_radius_model)
  add_test(NAME ${test} COMMAND ${test})
endforeach()
