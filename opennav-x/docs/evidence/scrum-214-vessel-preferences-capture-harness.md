# SCRUM-214 Vessel Preferences capture harness update

The Vessel tab now presents five editable fields and a `Save vessel profile`
action inline. The old `Vessel dimensions`, `Chart safety depth`, and
`Battery & reserve` summary links no longer describe that screen. The harnesses
now assert the five safe diagnostic field names and Save identity, then retain
`Advanced vessel model` and `Advanced battery model` as the actual lower-page
navigation endpoints.

The Linux layout capture and native Windows DPI/capture flows keep their
existing viewport, pointer/touch, scroll, settled HWND/diagnostic geometry, and
endpoint-stability assertions. They inspect field labels and action state only:
no text values are filled, and the Save action is never activated. The native
DPI run remains necessary for authoritative Windows and real-application
acceptance; these harness edits alone do not establish it.
