cmake_minimum_required(VERSION 3.24)
project(PilotComponentTests LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
  add_compile_options(/utf-8 /W4)
endif()

# Real GoogleTest release selected by pinned OpenCPN libs/gtest/CMakeLists.txt.
# Download occurs at configure time only; tests need no network or equipment.
include(FetchContent)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(googletest
  URL https://codeload.github.com/google/googletest/tar.gz/58d77fa8070e8cec2dc1ed015d66b454c8d78850
  URL_HASH SHA256=c6ab3b6b33f51ef7465921f8f8c10c15d7cbc510761a15a18ad85babf6d73278
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(googletest)

add_library(pilot_component_model STATIC
  ../../src/application/PilotPresentation.cpp
  ../../src/adapters/Autopilot.cpp ../../src/vessel/VesselState.cpp)
target_include_directories(pilot_component_model PUBLIC ../../src)
target_compile_definitions(pilot_component_model PUBLIC
  XNAV_ENABLE_TEST_FIXTURES=0 XNAV_ENABLE_PILOT_LOOPBACK_TESTS=0)
add_executable(pilot_presentation_tests ../pilot_presentation_tests.cpp)
target_link_libraries(pilot_presentation_tests PRIVATE pilot_component_model GTest::gtest_main)
add_test(NAME pilot_presentation COMMAND pilot_presentation_tests)
set_tests_properties(pilot_presentation PROPERTIES TIMEOUT 15)

find_package(wxWidgets 3.2 REQUIRED COMPONENTS core base)
include(${wxWidgets_USE_FILE})
add_executable(pilot_drawer_test ../pilot_drawer_test.cpp
  ../../src/ui/PilotDrawer.cpp ../../src/ui/Drawer.cpp
  ../../src/ui/Controls.cpp ../../src/ui/ChoiceField.cpp
  ../../src/ui/Sheet.cpp ../../src/ui/Range.cpp)
target_link_libraries(pilot_drawer_test PRIVATE pilot_component_model ${wxWidgets_LIBRARIES})
if(MSVC)
  target_link_libraries(pilot_drawer_test PRIVATE dwrite)
else()
  target_compile_options(pilot_drawer_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(UNIX AND NOT APPLE)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GTK REQUIRED IMPORTED_TARGET gtk+-3.0)
  target_link_libraries(pilot_drawer_test PRIVATE PkgConfig::GTK)
endif()
# The native runner supplies a fresh capture directory and verifies result.json.
# Do not register a reusable fixed-path UI test: the fixture rejects old output.
