# SCRUM-308 focused light-support tower proof

[Receipt](receipt.json) records the source/recipe hashes, generated resource
hashes, exact extracted core/private methods and focused results:

- 216 tower resource and refusal checks.
- 53,258 existing seamark/whole-atlas checks, including eight negative whole-resource cases.
- 50,373 actual pinned symbol-loader, lookup-selector and RenderSY checks.
- All private adapter patches reproduce exactly against hash-verified pinned source blobs.

The [implementation boundary](../../design/reviews/scrum308-light-support-tower.md)
retains ordinary/conspicuous physical tower shapes. Only the precisely
classified light-support tower inks change. This is not supplied new tower art,
and the original boat object's classification remains unproven.

Reproduction from the repository root, with the prepared core source and derived
public private-source copies (no application build):

```sh
python3 tools/generate-xnav-chart-style.py \
  --source build/integration-source/data/s57data \
  --output build/scrum308-light-tower/resources
python3 tests/chart_light_tower_resources_tests.py \
  --source build/integration-source/data/s57data \
  --generated build/scrum308-light-tower/resources
python3 tools/test-seamark-resources.py \
  --source build/integration-source/data/s57data \
  --generated build/scrum308-light-tower/resources
source /home/standard/Projects/X-nav/tools/local-env.sh
env -u WAYLAND_DISPLAY GDK_BACKEND=x11 xvfb-run -a \
  python3 tools/verify-anchor-loader.py --seamarks \
  --source build/integration-source \
  --generated build/scrum308-light-tower/resources \
  --output build/scrum308-light-tower/loader \
  --wx-config /home/standard/Projects/X-nav/.local/sysroot/usr/bin/wx-config \
  --wx-prefix /home/standard/Projects/X-nav/.local/sysroot/usr \
  --private-source build/scrum308-light-tower/private-original \
  --private-render-source build/scrum308-light-tower/private
```

The retained loader receipt compares actual methods, records terminal painter
calls and verifies PNG crops/GL atlas rectangle metadata. It does not execute a
GL draw or load the private DLL. Native Windows, physical display, actual
scale/zoom recognition and boat acceptance remain open. No proprietary chart
content or boat interaction was used.
