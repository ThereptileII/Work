include_guard(GLOBAL)
function(opennav_attach_boat_feedback_tests)
  if(TARGET boat_feedback_tests)
    return()
  endif()
  if(NOT OPENNAV_ROOT)
    get_filename_component(OPENNAV_ROOT "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/.." ABSOLUTE)
  endif()
  set(boat_feedback_commit "${OPENNAV_BUILD_COMMIT}")
  if(NOT boat_feedback_commit)
    execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY "${OPENNAV_ROOT}"
      OUTPUT_VARIABLE boat_feedback_commit OUTPUT_STRIP_TRAILING_WHITESPACE
      COMMAND_ERROR_IS_FATAL ANY)
  endif()
  # Focused boat-feedback regressions use the production component libraries.
  # Keep these offline mains out of the upstream gtest binary and product;
  # tools/test-boat-feedback-widgets.py runs each once on Linux and Windows.
  set(boat_feedback_models chart_info_tests pilot_status_tests
    anchor_route_transition_tests navigation_naming_tests route_context_tests)
  foreach(name IN LISTS boat_feedback_models)
    if(NOT TARGET ${name})
      add_executable(${name} "${OPENNAV_ROOT}/tests/${name}.cpp")
      target_link_libraries(${name} PRIVATE opennav_application)
    endif()
  endforeach()
  set(boat_feedback_widgets chart_info_drawer_test navigation_name_editor_test
    route_context_card_test)
  foreach(name IN LISTS boat_feedback_widgets)
    if(NOT TARGET ${name})
      add_executable(${name} "${OPENNAV_ROOT}/tests/${name}.cpp")
      target_link_libraries(${name} PRIVATE opennav_ui)
    endif()
  endforeach()
  add_executable(chart_light_hover_tests "${OPENNAV_ROOT}/tests/chart_light_hover_tests.cpp")
  target_link_libraries(chart_light_hover_tests PRIVATE opennav_ui)
  add_executable(ais_drawer_scroll_test "${OPENNAV_ROOT}/tests/ais_drawer_scroll/main.cpp")
  target_link_libraries(ais_drawer_scroll_test PRIVATE opennav_ui)
  add_executable(online_ais_radius_test "${OPENNAV_ROOT}/tests/online_ais_radius/main.cpp"
    "${OPENNAV_ROOT}/src/integration/OnlineAis.cpp")
  target_link_libraries(online_ais_radius_test PRIVATE opennav_integration
    opennav_ais_codec opennav_ais_credentials ${wxWidgets_LIBRARIES})
  # Compile the actual anchor painter against an offline chart/DC observation
  # boundary. The generated functions come from the same prepared/pinned core
  # sources as the product; no fixture headers may reach a production target.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(OPENNAV_BOAT_FEEDBACK_PREPARED_SOURCE "${OPENNAV_ROOT}/build/integration-source"
    CACHE PATH "Prepared pinned OpenCPN sources for boat-feedback renderer tests")
  set(OPENNAV_BOAT_FEEDBACK_PINNED_SOURCE "${OPENNAV_ROOT}/upstream/OpenCPN"
    CACHE PATH "Original pinned OpenCPN sources for boat-feedback renderer tests")
  set(anchor_fixture "${OPENNAV_ROOT}/tests/chart_anchor_watch")
  set(anchor_generated "${CMAKE_CURRENT_BINARY_DIR}/boat-feedback-anchor")
  set(anchor_painters "${anchor_generated}/anchor_ring_painters.inc")
  add_custom_command(OUTPUT "${anchor_painters}"
    COMMAND "${Python3_EXECUTABLE}" "${anchor_fixture}/prepare.py"
      --prepared "${OPENNAV_BOAT_FEEDBACK_PREPARED_SOURCE}"
      --pinned "${OPENNAV_BOAT_FEEDBACK_PINNED_SOURCE}" --output "${anchor_painters}"
    DEPENDS "${anchor_fixture}/prepare.py"
      "${OPENNAV_BOAT_FEEDBACK_PREPARED_SOURCE}/gui/src/chcanv.cpp"
      "${OPENNAV_BOAT_FEEDBACK_PREPARED_SOURCE}/gui/src/waypointman_gui.cpp"
      "${OPENNAV_BOAT_FEEDBACK_PINNED_SOURCE}/gui/src/chcanv.cpp"
      "${OPENNAV_ROOT}/src/ui/Controls.cpp"
    VERBATIM)
  add_executable(chart_anchor_watch_renderer_test "${anchor_fixture}/main.cpp"
    "${anchor_painters}" "${OPENNAV_ROOT}/src/integration/ChartAnchorWatch.cpp")
  target_include_directories(chart_anchor_watch_renderer_test BEFORE PRIVATE
    "${anchor_fixture}/stubs" "${OPENNAV_ROOT}/src" "${anchor_generated}")
  target_include_directories(chart_anchor_watch_renderer_test SYSTEM PRIVATE ${wxWidgets_INCLUDE_DIRS})
  target_compile_definitions(chart_anchor_watch_renderer_test PRIVATE OPENNAV_X
    ${wxWidgets_DEFINITIONS} "$<$<CONFIG:Debug>:${wxWidgets_DEFINITIONS_DEBUG}>")
  separate_arguments(anchor_wx_flags NATIVE_COMMAND "${wxWidgets_CXX_FLAGS}")
  target_compile_options(chart_anchor_watch_renderer_test PRIVATE ${anchor_wx_flags})
  target_link_libraries(chart_anchor_watch_renderer_test PRIVATE ${wxWidgets_LIBRARIES})
  set(boat_feedback_targets ${boat_feedback_models} ${boat_feedback_widgets}
    chart_light_hover_tests ais_drawer_scroll_test online_ais_radius_test
    chart_anchor_watch_renderer_test)
  set(boat_feedback_manifest "{\n  \"schema\": 1,\n  \"commit\": \"${boat_feedback_commit}\",\n  \"tests\": [")
  set(boat_feedback_separator "")
  foreach(name IN LISTS boat_feedback_targets)
    target_compile_features(${name} PRIVATE cxx_std_17)
    if(MSVC)
      target_compile_options(${name} PRIVATE /utf-8)
    endif()
    string(APPEND boat_feedback_manifest "${boat_feedback_separator}\n    {\"name\": \"${name}\", \"path\": \"$<TARGET_FILE:${name}>\"}")
    set(boat_feedback_separator ",")
  endforeach()
  string(APPEND boat_feedback_manifest "\n  ]\n}\n")
  file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/$<CONFIG>/boat-feedback-tests.json"
    CONTENT "${boat_feedback_manifest}")
  add_custom_target(boat_feedback_tests DEPENDS ${boat_feedback_targets})
  if(UNIX AND NOT APPLE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OPENNAV_FEEDBACK_GTK REQUIRED IMPORTED_TARGET gtk+-3.0)
    foreach(name IN LISTS boat_feedback_widgets)
      target_link_libraries(${name} PRIVATE PkgConfig::OPENNAV_FEEDBACK_GTK)
    endforeach()
    target_link_libraries(ais_drawer_scroll_test PRIVATE PkgConfig::OPENNAV_FEEDBACK_GTK)
  endif()
endfunction()
