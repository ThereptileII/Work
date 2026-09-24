#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
g++ -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Iinclude src/BridgeCore.cpp tests/core_tests.cpp -o "$test_dir/core_tests"
"$test_dir/core_tests"
g++ -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Itests/hardware_stubs -Iinclude src/BridgeCore.cpp src/SeaTalkBus.cpp tests/seatalk_bus_tests.cpp \
  -o "$test_dir/seatalk_bus_tests"
"$test_dir/seatalk_bus_tests"
library_dir=".pio/libdeps/esp32dev/NMEA2000-library/src"
if [[ ! -d "$library_dir" ]]; then
  echo 'Run pio pkg install first to install the pinned NMEA2000 dependency.' >&2
  exit 1
fi
g++ -std=c++17 -O1 -g -pthread -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Iinclude -I"$library_dir" src/BridgeCore.cpp tests/nmea_tests.cpp "$library_dir"/*.cpp \
  -o "$test_dir/nmea_tests"
"$test_dir/nmea_tests"
