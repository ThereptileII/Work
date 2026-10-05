# SCRUM-229 — retained portable wrapper-test binding

Full run [37057756273](https://github.com/ThereptileII/Work/actions/runs/37057756273)
on `cf44e938a3539af6e97a03346542f04f3e7895a4` failed one contract test on both
platforms: `native_diagnostic_geometry_observation`. The Windows job passed
88/89 CTest entries; Linux passed 91/92. Ten Python cases raised `NameError:
preferences_touch is not defined` before reaching their behavioral assertions.
The independent native Settings component proof had already passed on that
source. This failure is in the retained portable test's AST execution scope,
not evidence of an application crash or a native touch failure.

The DPI harness now delegates to the shared `preferences-touch.py`. Its old
portable test extracted the wrapper but still provided only the previous
inline body's dependencies. Bind the real shared helper into the isolated
test namespace, provide its report and observed native class query, and replace
only its sleep dependency with the test clock. The real movement, hit testing,
clipping, bounded retries and tap/refusal assertions remain unchanged. Do not
stub the delegated function: that would hide this integration error.

The exact retained Python suite passes all 24 cases locally in 0.454 seconds.
Native Windows execution uses the existing short Python-only workflow; its
result is recorded separately. No application code or release gate is removed.
The running full candidate and its failures remain retained; no full retry is
authorized by this local pass alone.
