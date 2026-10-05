# SCRUM-296 — chart scale legend

The immutable prototype defines one non-interactive `.scale` legend beside
Follow boat: a 65px reference bracket above an 8px label, separated by 5px.
There is no separate scale action. SKAGER already has no scale button, so the
useful distance legend is retained rather than removing the user's map reference.

The native legend had reversed the bracket and label. It now uses the prototype
ordering, bottom inset and spacing. OpenCPN still selects the actual distance in
the user's units and projects its length at the bracket's actual screen row.
No fixed decorative distance, new gesture handler or navigation calculation is
introduced. A small neutral backing prevents ENC soundings reading as scale text.
Legacy and Standard paths are unchanged.

The changed chart-presentation translation unit passes its integrated Linux
compile check. The shared UI component build passes. Native Windows and boat
visual acceptance remain pending for the coordinated staging candidate.
