# Included after the product's opennav_ui/opennav_application targets. Their
# single source lists own StartupUpdate.cpp and StartupUpdateDialog.cpp.
if(WIN32)
  add_executable(skager-update-prompt WIN32
    "${CMAKE_CURRENT_LIST_DIR}/../src/platform/windows/StartupUpdatePrompt.cpp")
  target_link_libraries(skager-update-prompt PRIVATE opennav_ui opennav_application)
  target_compile_features(skager-update-prompt PRIVATE cxx_std_17)
  if(MSVC)
    target_compile_options(skager-update-prompt PRIVATE /utf-8 /W4)
  endif()
endif()
