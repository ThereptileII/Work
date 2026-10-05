cmake_minimum_required(VERSION 3.24)
project(AisDrawerScrollTests LANGUAGES CXX)
find_package(wxWidgets 3.2 REQUIRED COMPONENTS core base)
include(${wxWidgets_USE_FILE})
add_executable(ais_drawer_scroll_test main.cpp
  ../../src/ui/AisDrawer.cpp ../../src/ui/Drawer.cpp
  ../../src/ui/Controls.cpp ../../src/ui/ListView.cpp
  ../../src/ui/ChoiceField.cpp ../../src/ui/Sheet.cpp ../../src/ui/Range.cpp
  ../../src/vessel/VesselState.cpp ../../src/vessel/AisSelection.cpp
  ../../src/ais/ChartTargets.cpp ../../src/ais/TargetCache.cpp
  ../../src/ais/Credentials.cpp)
target_include_directories(ais_drawer_scroll_test PRIVATE ../../src)
target_compile_features(ais_drawer_scroll_test PRIVATE cxx_std_17)
target_link_libraries(ais_drawer_scroll_test PRIVATE ${wxWidgets_LIBRARIES})
if(MSVC)
  target_compile_options(ais_drawer_scroll_test PRIVATE /utf-8 /W4)
  target_link_libraries(ais_drawer_scroll_test PRIVATE dwrite)
else()
  target_compile_options(ais_drawer_scroll_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(UNIX AND NOT APPLE)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GTK REQUIRED IMPORTED_TARGET gtk+-3.0)
  target_link_libraries(ais_drawer_scroll_test PRIVATE PkgConfig::GTK)
endif()
enable_testing()
add_test(NAME ais_drawer_scroll COMMAND ais_drawer_scroll_test)
set_tests_properties(ais_drawer_scroll PROPERTIES TIMEOUT 30)
