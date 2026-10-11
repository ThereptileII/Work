cmake_minimum_required(VERSION 3.24)
project(BoatSetupComponentTests LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
  add_compile_options(/utf-8)
endif()
find_package(wxWidgets 3.2 REQUIRED COMPONENTS core base aui)
include(${wxWidgets_USE_FILE})
add_executable(boat_setup_dialog_test ../boat_setup_dialog_test.cpp
  ../../src/ui/BoatSetupDialog.cpp ../../src/ui/Controls.cpp
  ../../src/ui/ChoiceField.cpp ../../src/ui/Sheet.cpp ../../src/ui/Drawer.cpp
  ../../src/application/BoatSetup.cpp ../../src/application/Settings.cpp
  ../../src/application/MarineMapping.cpp ../../src/smartnav/EnergyConfiguration.cpp
  ../../src/adapters/St4000Pilot.cpp ../../src/adapters/BoatN2k.cpp
  ../../src/vessel/VesselState.cpp ../../src/vessel/SensorRegistry.cpp)
target_include_directories(boat_setup_dialog_test PRIVATE ../../src)
target_link_libraries(boat_setup_dialog_test PRIVATE ${wxWidgets_LIBRARIES})
add_executable(about_notices_tests ../about_notices_tests.cpp)
target_include_directories(about_notices_tests PRIVATE ../../src)
add_test(NAME setup_packaged_document_read COMMAND about_notices_tests)
add_library(setup_shell_compile OBJECT ../../src/ui/Shell.cpp)
target_include_directories(setup_shell_compile PRIVATE ../../src)
if(MSVC)
  target_link_libraries(boat_setup_dialog_test PRIVATE dwrite)
  target_compile_options(boat_setup_dialog_test PRIVATE /Gy)
  target_link_options(boat_setup_dialog_test PRIVATE /OPT:REF)
else()
  # Keep the real validation routines; omit unrelated transport implementations
  # sharing their translation units from this offline component executable.
  target_compile_options(boat_setup_dialog_test PRIVATE -ffunction-sections -fdata-sections)
  target_link_options(boat_setup_dialog_test PRIVATE -Wl,--gc-sections)
endif()
if(UNIX AND NOT APPLE)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GTK REQUIRED IMPORTED_TARGET gtk+-3.0)
  target_link_libraries(boat_setup_dialog_test PRIVATE PkgConfig::GTK)
endif()
