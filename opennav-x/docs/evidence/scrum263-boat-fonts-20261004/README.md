# SCRUM-263 boat font prerequisite

Read-only observation at **2026-10-04 07:00:40 UTC**: Windows 11 Pro,
version `10.0.22631`, 64-bit. Managed GDI font-family enumeration reports all
three prototype choices installed: **Segoe UI Variable Display**, **Segoe UI**
and **Arial**. This agrees with the prior 2026-10-03 inventory.

[receipt.json](receipt.json) retains the nonpersonal result, query hash and
exact policy-source hashes at `615118f351abdea7b79a042db63e58fe0a635e0a`.
[query.ps1](query.ps1) was sent as one encoded command through `ssh boat`;
it only reads installed font families and four operating-system fields. No
application, helper, font installation or configuration change was performed.

| Current policy role | First available requested face on the boat |
| --- | --- |
| UI, geographic water names, soundings | Segoe UI Variable Display |
| Ordinary chart text, geographic land names, generated LIGHTS descriptions | Segoe UI |

The immutable HTML root uses Variable Display → Segoe UI → Arial; explicit
`.chart-label` uses Segoe UI. The role-specific native selectors are
`Controls.cpp:157`, `ChartTextFace.h:10`, `ChartPresentation.cpp:73` and
`ChartSoundingFont.h:10`. No lower fallback is required by this inventory.
Standard chart presentation retains its existing preferences.

The [retained same-boat HTML reference](../scrum263-boat-html-reference-5884701/README.md)
already observed actual browser glyphs on 2026-10-03: Variable Display for UI,
water names and depth, and Segoe UI for ordinary chart labels. That is historical
browser evidence. Today's inventory and policy comparison do not observe the
new native application's selected HDC face, glyph substitution, weight/italic
realization or DPI rendering; those remain part of the candidate's boat review.
