cmake_minimum_required(VERSION 3.24)
project(AboutLicensesTests LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
  add_compile_options(/utf-8)
endif()
add_executable(about_notices_tests ../about_notices_tests.cpp)
target_include_directories(about_notices_tests PRIVATE ../../src)
add_test(NAME about_notice_package_read COMMAND about_notices_tests)
find_package(wxWidgets 3.2 REQUIRED COMPONENTS core base aui)
include(${wxWidgets_USE_FILE})
add_executable(about_licenses_test ../about_licenses_test.cpp
  ../../src/ui/SettingsDrawer.cpp ../../src/ui/Drawer.cpp
  ../../src/ui/Controls.cpp ../../src/ui/ChoiceField.cpp
  ../../src/ui/Sheet.cpp ../../src/application/Settings.cpp
  ../../src/vessel/VesselState.cpp)
target_include_directories(about_licenses_test PRIVATE ../../src)
target_link_libraries(about_licenses_test PRIVATE ${wxWidgets_LIBRARIES})
# Compile the actual callback wiring as well as the component, without linking
# or starting the full OpenCPN application.
add_library(about_shell_compile OBJECT ../../src/ui/Shell.cpp)
target_include_directories(about_shell_compile PRIVATE ../../src)
if(MSVC)
  target_link_libraries(about_licenses_test PRIVATE dwrite)
  target_compile_options(about_licenses_test PRIVATE /Gy)
  target_link_options(about_licenses_test PRIVATE /OPT:REF)
else()
  # The drawer uses the production number parser, not the unrelated settings
  # persistence/protocol functions in the same translation unit.
  target_compile_options(about_licenses_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(about_licenses_test PRIVATE -Wl,--gc-sections)
endif()
if(UNIX AND NOT APPLE)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GTK REQUIRED IMPORTED_TARGET gtk+-3.0)
  target_link_libraries(about_licenses_test PRIVATE PkgConfig::GTK)
endif()
