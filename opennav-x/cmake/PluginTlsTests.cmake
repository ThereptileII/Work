# Reuse the exact SHA256 header pin used by the native loader fixtures. A local
# pinned header supports offline configuration; ordinary standalone CI retains
# only that same small header instead of fetching another upstream source tree.
set(PICOSHA2_DIR "" CACHE PATH "Directory containing the exact pinned picosha2.h")
file(READ "${CMAKE_CURRENT_LIST_DIR}/../tests/windows_ocharts_loader/inputs.lock.json"
  plugin_tls_inputs)
string(JSON plugin_tls_pico_hash GET "${plugin_tls_inputs}" picosha2 sha256)
string(JSON plugin_tls_pico_size GET "${plugin_tls_inputs}" picosha2 bytes)
if(NOT PICOSHA2_DIR)
  set(PICOSHA2_DIR "${CMAKE_CURRENT_BINARY_DIR}/plugin-tls-picosha2")
  if(NOT EXISTS "${PICOSHA2_DIR}/picosha2.h")
    string(JSON plugin_tls_pico_url GET "${plugin_tls_inputs}" picosha2 url)
    file(MAKE_DIRECTORY "${PICOSHA2_DIR}")
    file(DOWNLOAD "${plugin_tls_pico_url}" "${PICOSHA2_DIR}/picosha2.h"
      EXPECTED_HASH "SHA256=${plugin_tls_pico_hash}" TLS_VERIFY ON TIMEOUT 30
      STATUS plugin_tls_download_status)
    list(GET plugin_tls_download_status 0 plugin_tls_download_result)
    if(NOT plugin_tls_download_result EQUAL 0)
      message(FATAL_ERROR "Pinned picosha2 download failed: ${plugin_tls_download_status}")
    endif()
  endif()
endif()
if(NOT EXISTS "${PICOSHA2_DIR}/picosha2.h")
  message(FATAL_ERROR "Exact pinned picosha2 header required")
endif()
file(SHA256 "${PICOSHA2_DIR}/picosha2.h" plugin_tls_pico_actual_hash)
file(SIZE "${PICOSHA2_DIR}/picosha2.h" plugin_tls_pico_actual_size)
if(NOT plugin_tls_pico_actual_hash STREQUAL plugin_tls_pico_hash OR
   NOT plugin_tls_pico_actual_size EQUAL plugin_tls_pico_size)
  message(FATAL_ERROR "picosha2 header differs from the native fixture pin")
endif()
add_executable(plugin_tls_byte_policy_test
  "${CMAKE_CURRENT_LIST_DIR}/../tests/plugin_tls_byte_policy_test.cpp")
target_include_directories(plugin_tls_byte_policy_test PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/../src" "${PICOSHA2_DIR}")
target_compile_features(plugin_tls_byte_policy_test PRIVATE cxx_std_17)
add_test(NAME plugin_tls_known_obsolete_bytes COMMAND plugin_tls_byte_policy_test)
set_tests_properties(plugin_tls_known_obsolete_bytes PROPERTIES TIMEOUT 30)
