cmake_minimum_required(VERSION 3.20)
project(skager_ocharts_object_probe VERSION 2.0.0.49 LANGUAGES C CXX)
# This entry is intentionally not a production preparation or package. Its
# independent locked headers suffice only for ClCompile, never a DLL link.
get_filename_component(SKAGER_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${SKAGER_ROOT}/cmake/ocharts-adapter/PreparedPath.cmake")
include("${SKAGER_ROOT}/cmake/ocharts-adapter/Targets.cmake")
