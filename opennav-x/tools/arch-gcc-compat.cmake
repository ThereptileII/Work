# GCC 16 diagnoses intentional bounded copies followed by explicit NUL writes
# in pristine 5.12.4 TCDS_Binary_Harmonic.cpp:473-476. Keep the diagnostics but
# do not turn this one category into an error for this one translation unit.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16)
  set_property(SOURCE "${CMAKE_SOURCE_DIR}/gui/src/TCDS_Binary_Harmonic.cpp"
    APPEND PROPERTY COMPILE_OPTIONS -Wno-error=stringop-truncation)
  if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    # RapidJSON 1.1.0 contains an unused invalid template assignment operator.
    # GCC 16 diagnoses it eagerly; defer that diagnostic as older compilers do.
    # Instantiating the invalid operator would still be an error. Restrict this
    # compatibility flag to the three translation units including the header.
    function(opennav_rapidjson_gcc16_compat)
      foreach(unit ais_decoder comm_decoder comm_drv_signalk_net)
        set_property(SOURCE "${CMAKE_SOURCE_DIR}/model/src/${unit}.cpp"
          DIRECTORY "${CMAKE_SOURCE_DIR}/model"
          APPEND PROPERTY COMPILE_OPTIONS -Wno-template-body)
      endforeach()
    endfunction()
    cmake_language(DEFER CALL opennav_rapidjson_gcc16_compat)
  endif()
endif()
