#include "platform/PortableProfile.h"
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  try {
    const auto paths = opennav::platform::PreviewProfile(
        opennav::platform::PathFromUtf8(argv[1]), argv[2]);
    if (!paths) return 3;
    std::cout << opennav::platform::PathUtf8(paths->root) << '\n'
              << opennav::platform::PathUtf8(paths->profile) << '\n'
              << opennav::platform::PathUtf8(paths->logs) << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
