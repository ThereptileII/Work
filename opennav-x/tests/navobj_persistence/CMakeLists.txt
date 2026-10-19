cmake_minimum_required(VERSION 3.20)
project(navobj_persistence_fixture LANGUAGES C CXX)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)

set(OPENCPN_SOURCE_DIR "" CACHE PATH "Prepared pinned OpenCPN source")
set(sqlite_root "${OPENCPN_SOURCE_DIR}/libs/sqlite")
foreach(source model/src/navobj_db.cpp gui/src/chcanv.cpp
               libs/sqlite/src/sqlite3.c libs/sqlite/include/sqlite3.h)
  if(NOT EXISTS "${OPENCPN_SOURCE_DIR}/${source}")
    message(FATAL_ERROR "Prepared OpenCPN input is missing: ${source}")
  endif()
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${OPENCPN_SOURCE_DIR}/model/src/navobj_db.cpp"
  "${OPENCPN_SOURCE_DIR}/gui/src/chcanv.cpp"
  "${sqlite_root}/src/sqlite3.c"
  "${sqlite_root}/include/sqlite3.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/extract.py")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/extract.py"
          --source "${OPENCPN_SOURCE_DIR}" --output "${CMAKE_CURRENT_BINARY_DIR}"
  COMMAND_ERROR_IS_FATAL ANY)

# Use the same pinned amalgamation on Windows and Linux. Deliberately avoid
# FindSQLite3/the upstream system-library fallback, so fixture bytes are clear.
add_library(navobj_fixture_sqlite STATIC "${sqlite_root}/src/sqlite3.c")
target_include_directories(navobj_fixture_sqlite PUBLIC "${sqlite_root}/include")
target_compile_definitions(navobj_fixture_sqlite PRIVATE SQLITE_THREADSAFE=1)
target_link_libraries(navobj_fixture_sqlite PUBLIC Threads::Threads)
if(UNIX AND NOT APPLE)
  target_compile_definitions(navobj_fixture_sqlite PRIVATE SQLITE_OMIT_LOAD_EXTENSION)
endif()

add_executable(navobj_persistence persistence.cpp)
target_link_libraries(navobj_persistence PRIVATE navobj_fixture_sqlite)
add_executable(navobj_finish finish.cpp)
foreach(target navobj_persistence navobj_finish)
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
  target_compile_features(${target} PRIVATE cxx_std_17)
  if(MSVC)
    # Keep the native excerpts unchanged, including upstream conversions from
    # wx-style sizes/time values to SQLite's int arguments. Show MSVC warnings;
    # the fixture's database/behavior assertions remain unconditional.
    target_compile_options(${target} PRIVATE /W4 /utf-8)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror)
  endif()
endforeach()
enable_testing()
add_test(NAME navobj_persistence COMMAND navobj_persistence)
add_test(NAME navobj_finish COMMAND navobj_finish)
set_tests_properties(navobj_persistence navobj_finish PROPERTIES TIMEOUT 15)
