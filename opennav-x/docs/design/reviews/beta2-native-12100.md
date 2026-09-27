# Native development review — 12100a74

Exact commit `12100a74ff619b7268a6e20902bd9d0de3b53b39`; downloaded artifact
and upload-log hash verified in [evidence](../../evidence/beta2-windows-12100a74.json).
This candidate failed two interactions and is not a distributable product.

Reference intent: chart dominance, four readable rail values, stable alerts,
touch-sized actions, coherent dark surfaces and restrained night lighting.

Reviewed at native 1280×800:

- `dpi-150-01-navigation-day.png`: coastline clearly visible; SOG, depth, wind
  and heading all fit beside the chart with a critical alert. Bottom destination
  uses deliberate ellipsis. DEMO belongs to this isolated fixture build only.
- `dpi-150-system-page.png`: System is a full page. Its actions do not overlap
  the header alert and all eight actions fit. A native hover tooltip extends
  outside the frame in this capture; primary content is not clipped. Avoid
  interpreting that OS surface as part of the styled sheet.
- `chart-opengl-01-loaded.png`: public NOAA ENC has soundings, coastline,
  route points and ownship. The requested GL backend was rejected by the host;
  actual rendering is upstream software fallback, not hardware GL acceptance.
- `beta-night-autopilot.png`: dim text/surfaces and large course/mode actions.
  Control is OFF in this capture. Subsequent simulator interaction failed and
  remains a functional blocker, regardless of this visual result.

The current diagnostic/native-bound synchronization closes the earlier false
150% overflow assertion without relaxing containment or touch-size checks.
Six mode/coastline checks and twelve primary night surfaces pass per scale.
The boat display, installed fixture-free product and physical GPU still require
their own evidence. No physical equipment commands were sent.
