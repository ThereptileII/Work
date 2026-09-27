# Instruments and energy: viewport review

Reference intent: group related readings, emphasize values, keep units and data
state readable, and use restrained surfaces instead of a grid of gauges.

Three additional native 1280×800 images from the verified, failed development
candidate `12100a74` were reviewed. These are visual observations, not candidate
acceptance:

| Image | Observation | SHA-256 |
| --- | --- | --- |
| `dpi-100-instruments.png` | Navigation and wind are clear, but excessive row whitespace pushes the conditions units below the first viewport. | `2fa6935efadbae2d64322b071351ccaf5c80eb93216c95e16980cce10bbe35ae` |
| `dpi-150-energy.png` | Battery, propulsion and destination hierarchy is readable; supporting range/state content requires scrolling. | `61c290a4d951fa2d6d56ae32a9ef264f5e6b5859d333785a17b8fb0246395c42` |
| `dpi-125-night-energy.png` | Dim surfaces and values retain hierarchy; the lower range card is only partially visible before scrolling. | `7de0b863659f40dff316fe4cba1b934eec0d6c2df34c954ea563f9438667b87e` |

Instrument rows now use 168 DIP instead of 188 DIP. Fonts, value size, units and
quality annotations are unchanged; the existing minimum 120 DIP numeric-region
regression remains enforced. Extra configured values still use another row and
the existing touch scrolling. This is a whitespace refinement, not an attempt to
compress every configured sensor onto one screen.

The updated Linux 1280×800 image shows complete navigation, wind and the first
conditions group, including units and the pressure `NO DATA` annotation.
`alpha-instruments-linux.png` SHA-256:
`f27dcc7bffa590639bc777af29effb398d0146cf58296f67775069e0b117e49b`.
The full fixture UI/scenario smoke passed after the refinement. Its Demo labels
belong to the separate automated-test executable, not the installed product.

The exact replacement still needs native DPI review and actual boat-display
iteration. Energy's supporting-content placement remains a boat review item;
no new energy calculation or prediction behavior was introduced here.
