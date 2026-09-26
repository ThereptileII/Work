# Product and test build separation

Beta 2 removes synthetic inputs from the installed application and portable
recovery application. The production default is:

```text
XNAV_ENABLE_TEST_FIXTURES=OFF
OPENNAV_ENABLE_ROUTE_SCENARIO=OFF
```

The flag is a numeric compile-time definition, not an environment variable,
saved preference or runtime permission. `BuildFeatures.h` exposes the build
purpose to diagnostics and the read-only installer loader self-test. A product
self-test reports `test_fixtures: false` and `build_purpose: INSTALLED PRODUCT`.
Packaging must check these fields before publishing or installing a product.

The live Vessel Data, SmartNav and adapter libraries contain no definitions for
`DemoFixture`, `DemoAis`, `DemoSource`, `SimulatorFixture`, `PreviewEnergyModel`
or `SimulatedAutopilot`. Their definitions are in `opennav_test_fixtures`, linked
only by dedicated contract executables and, when explicitly requested, the
developer UI. Adapter simulator declarations also have separate headers so
compiler devirtualization cannot emit inline simulator methods into live
libraries. A product target that accidentally calls a generator fails at link
time.

The installed command-line interface does not register synthetic startup
switches. Unknown switches fail in the normal OpenCPN command-line parser.
The shared startup policy also rejects every synthetic request in a product
build. Developer builds require explicit XNav, exactly one test input, and the
existing disposable-profile checks for mutable route/object scenarios. Safe and
Legacy do not gain synthetic modules.

Deterministic regression infrastructure remains available:

```text
Contract tests: OPENNAV_BUILD_TESTS=ON (product runtime flag may remain OFF)
Integrated CI: XNAV_ENABLE_TEST_FIXTURES=ON
               OPENNAV_ENABLE_ROUTE_SCENARIO=ON
               OCPN_BUILD_TEST=ON
```

Integrated fixture executables identify as `DEVELOPER TEST BUILD`. They are
not installable release payloads. CI must separately build, smoke-test and
package the product executable with fixture flags OFF. The product and test
executables must come from the same source commit.

Explicit historical replay remains an advanced diagnostic operation. It cannot
change the real route/ownship or enable physical control. Provenance labels in
recording parsers and diagnostics remain necessary to reject mixed sources and
identify older recordings; they are not synthetic generators or product modes.

`build_policy_product` and `build_policy_fixtures` compile the startup policy
independently and exercise all source/mode combinations, environment bypass
attempts and invalid persisted modes. Existing simulator, recording, energy and
adapter contracts continue to link their dedicated fixtures.
`production_runtime_generator_absence` inspects the actual compiled public
symbol tables with native MSVC or GNU tools. All six generators must be absent
from live runtime archives and present in the separate test archive, which
provides a positive control for the symbol reader.
