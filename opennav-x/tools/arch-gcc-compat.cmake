# GCC 16 diagnoses intentional bounded copies followed by explicit NUL writes
# in pristine 5.12.4 TCDS_Binary_Harmonic.cpp:473-476. Keep the diagnostics but
# do not turn this one category into an error for this one translation unit.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16)
  set_property(SOURCE "${CMAKE_SOURCE_DIR}/gui/src/TCDS_Binary_Harmonic.cpp"
    APPEND PROPERTY COMPILE_OPTIONS -Wno-error=stringop-truncation)
endif()
