cmake_minimum_required(VERSION 3.24)
project(ChartAnchorWatchRendererTest LANGUAGES CXX)
find_package(wxWidgets 3.2 REQUIRED COMPONENTS core base)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
include(${wxWidgets_USE_FILE})
get_filename_component(XNAV_REPOSITORY "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(XNAV_PREPARED_SOURCE "${XNAV_REPOSITORY}/build/integration-source" CACHE PATH
    "Prepared pinned OpenCPN source containing the candidate anchor paint hooks")
set(XNAV_PINNED_SOURCE "${XNAV_REPOSITORY}/upstream/OpenCPN" CACHE PATH "Original pinned OpenCPN source")
set(painters "${CMAKE_CURRENT_BINARY_DIR}/anchor_ring_painters.inc")
add_custom_command(OUTPUT "${painters}"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/prepare.py"
    --prepared "${XNAV_PREPARED_SOURCE}" --pinned "${XNAV_PINNED_SOURCE}" --output "${painters}"
  DEPENDS prepare.py "${XNAV_PREPARED_SOURCE}/gui/src/chcanv.cpp"
    "${XNAV_PREPARED_SOURCE}/gui/src/waypointman_gui.cpp"
    "${XNAV_PINNED_SOURCE}/gui/src/chcanv.cpp" "${XNAV_REPOSITORY}/src/ui/Controls.cpp"
  VERBATIM)
add_executable(chart_anchor_watch_renderer_test main.cpp "${painters}"
  "${XNAV_REPOSITORY}/src/integration/ChartAnchorWatch.cpp")
target_include_directories(chart_anchor_watch_renderer_test PRIVATE stubs
  "${XNAV_REPOSITORY}/src" "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(chart_anchor_watch_renderer_test PRIVATE OPENNAV_X)
target_compile_features(chart_anchor_watch_renderer_test PRIVATE cxx_std_17)
target_link_libraries(chart_anchor_watch_renderer_test PRIVATE ${wxWidgets_LIBRARIES})
if(MSVC)
  target_compile_options(chart_anchor_watch_renderer_test PRIVATE /utf-8 /W4)
else()
  target_compile_options(chart_anchor_watch_renderer_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
enable_testing()
add_test(NAME chart_anchor_watch_renderer COMMAND chart_anchor_watch_renderer_test)
set_tests_properties(chart_anchor_watch_renderer PROPERTIES TIMEOUT 30)
