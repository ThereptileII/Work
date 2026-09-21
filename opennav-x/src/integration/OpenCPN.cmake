# Included only by the explicit OpenNav source hook. Upstream defaults unchanged.
if(NOT EXISTS "${OPENNAV_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "OPENNAV_ROOT must point to the OpenNav X source root")
endif()
set(OPENNAV_BUILD_UI_COMPONENTS ON CACHE BOOL "" FORCE)
set(OPENNAV_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${OPENNAV_ROOT}" "${CMAKE_BINARY_DIR}/opennav")
target_sources(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src/integration/OpenCPNIntegration.cpp")
target_include_directories(${PACKAGE_NAME} PRIVATE "${OPENNAV_ROOT}/src")
target_compile_definitions(${PACKAGE_NAME} PRIVATE OPENNAV_X=1)
# Preserve normal plugin preferences while upstream Safe Mode blocks loading.
# Limit this additional definition to the one affected model translation unit.
set_property(SOURCE "${CMAKE_SOURCE_DIR}/model/src/plugin_loader.cpp"
  DIRECTORY "${CMAKE_SOURCE_DIR}/model" APPEND PROPERTY COMPILE_DEFINITIONS OPENNAV_X=1)
target_link_libraries(${PACKAGE_NAME} PRIVATE opennav_integration opennav_platform opennav_ui)
if(WIN32)
  install(TARGETS opennav-restart RUNTIME DESTINATION .)
  add_custom_command(TARGET ${PACKAGE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:opennav-restart> $<TARGET_FILE_DIR:${PACKAGE_NAME}>)
  add_dependencies(${PACKAGE_NAME} opennav-restart)
endif()
