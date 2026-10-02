# Observe the actual configured curl target before compilation or its long suite.
# IMPORT_LIB_SUFFIX is a supported upstream option, set by the producer caller.
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  function(_xnav_check_curl_import_layout)
    if(NOT MSVC OR NOT WIN32 OR NOT CMAKE_SIZEOF_VOID_P EQUAL 4 OR
        NOT BUILD_SHARED_LIBS OR BUILD_STATIC_LIBS OR
        NOT DEFINED IMPORT_LIB_SUFFIX OR NOT "${IMPORT_LIB_SUFFIX}" STREQUAL "")
      message(FATAL_ERROR "curl import layout requires native Win32 shared-only and explicit empty IMPORT_LIB_SUFFIX")
    endif()
    if(NOT TARGET libcurl_shared)
      message(FATAL_ERROR "Configured curl shared target missing")
    endif()
    get_target_property(_type libcurl_shared TYPE)
    get_target_property(_name libcurl_shared OUTPUT_NAME)
    get_target_property(_prefix libcurl_shared IMPORT_PREFIX)
    get_target_property(_suffix libcurl_shared IMPORT_SUFFIX)
    if(NOT _type STREQUAL "SHARED_LIBRARY" OR NOT _name STREQUAL "libcurl" OR
        NOT _prefix STREQUAL "" OR NOT _suffix STREQUAL ".lib")
      message(FATAL_ERROR "Configured curl import target differs from libcurl.lib")
    endif()
    file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/xnav-curl-import-$<CONFIG>.txt"
      CONTENT "$<TARGET_LINKER_FILE:libcurl_shared>\n")
  endfunction()
  cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL _xnav_check_curl_import_layout)
endif()
