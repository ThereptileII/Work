# Offline AIS radius and target-path regression (SCRUM-306)

This executable uses production configuration, radius geometry, subscription
policy, session, JSON decoder, cache, aggregation and chart-target projection.
Only credential/transport boundaries are fakes. It opens no service connection,
reads no real credential and launches no OpenCPN process. The three existing
cache, session and codec contract executables are also built to preserve their
prior rejection/precedence/lifecycle assertions.

```sh
source /home/standard/Projects/X-nav/tools/local-env.sh
cmake -S tests/online_ais_radius -B build/scrum306-ais-radius -G Ninja \
  -DwxWidgets_CONFIG_EXECUTABLE=/home/standard/Projects/X-nav/tools/wx-config-local \
  -DRAPIDJSON_INCLUDE_DIR="$PWD/build/scrum301-ais/_deps/opennav_rapidjson-src/include" \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/scrum306-ais-radius -j2
ctest --test-dir build/scrum306-ais-radius --output-on-failure
```

The RapidJSON path must identify the pinned 1.1.0 headers from the qualified
product/source dependency. This test does not download or choose a new parser.
Native Windows uses the same directory with the qualified Win32 wxWidgets and
pinned parser inputs; run CTest with `-C Release --output-on-failure`.

The production slider is exercised by the separate `ais_drawer_scroll` fixture.
Its Linux OS pointer test requires an isolated X11 backend; follow its README.
The model harness proves valid Class A/B traffic reaches the copied panel data
and chart marks. It cannot prove live AISStream coverage or actual chart paint.
