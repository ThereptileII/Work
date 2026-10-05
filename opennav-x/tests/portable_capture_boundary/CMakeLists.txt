cmake_minimum_required(VERSION 3.21)
project(PortableCaptureBoundary LANGUAGES CXX)
# Compile the unchanged production guard; no wx/OpenCPN/application rebuild.
add_executable(portable-capture-probe probe.cpp ../../src/platform/PortableProfile.cpp)
target_include_directories(portable-capture-probe PRIVATE ../../src)
target_compile_features(portable-capture-probe PRIVATE cxx_std_17)
