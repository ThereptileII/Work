# SKAGER resource-log expectation repair

The exact `9fcd3db` candidate's [Linux integrated job](https://github.com/ThereptileII/Work/actions/runs/37080314681/job/111079164722)
built successfully and passed all 147 integrated CTest cases, the 17-check
floating-surface fixture, peer-command refusals and loader/resource check.
The subsequent installed-resource scenario failed because its assertion still
expected the former `OpenNav installed resource defaults` log prefix. The actual
application log contains the new `SKAGER` prefix and the unchanged
`original supported OpenCPN; configured selections preserved` message.

Downloaded artifact 11259141261 is 105,416 bytes, SHA-256
`627310aea57c60bed249825ede9ce5026569a59fb0b47858f28bd825697e3fb6`.
All ZIP CRCs pass. Selected original logs/results are retained unchanged under
`ci-failure`; the receipt hashes their bytes. This was an assertion mismatch,
not an observed application crash or lost resource.

The correction changes exactly that expected prefix. A focused rerun against
the same frozen Linux executable passes SKAGER, Legacy and Safe, with three
clean exits. Existing assertions still verify real harmonic sources, removal
of preceding application generations, preservation of the configured one-file
harmonic selection, and the stock basemap/sound paths. No product code or
behavioral check is removed. The original test had already failed in CI, so no
extra negative rerun was necessary. The test uses a synthetic stock locator,
not an accepted Windows installer or any boat profile.

The native Windows candidate remains independently in progress. This focused
correction does not turn the failed full Linux job into a passing run, and no
replacement full run is dispatched solely for this one-line test change.
