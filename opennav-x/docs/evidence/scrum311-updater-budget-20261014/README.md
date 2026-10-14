# Native updater job budget correction

Original candidate `e538bbc857d3eb207ebeef62ec9954577c2b2d16`,
[run 38633971333 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38633971333).
Windows updater job `116092940606` was cancelled with GitHub annotation
`The job has exceeded the maximum execution time of 20m0s`.
All executed steps succeeded, including both artifact uploads. This is a job-budget
failure, not a passing required job or an observed SKAGER assertion/crash.
Windows integration was skipped before SDK verification or application compilation.

The original uploaded operator kit is independently authenticated against its
original API metadata and upload log. Full outer/inner bytes, digests, CRC,
exact inventory and all nine manifest payload hashes pass;
[bounded observation](operator-kit-observation.json). Integrity of this component
does not qualify the cancelled job or supply the missing Windows application.

Correction `38fca523` preserves every test/platform and gives Windows 35 minutes
for the existing build/test/package/upload sequence; Linux retains 20 minutes.
Matrix fail-fast is disabled so one platform cannot discard the other's result.
No SDK rebuild is needed: all 30 authenticated producer input paths are unchanged.
The existing retained-package routes cannot resume a Windows integration that
never started, so no separate verifier run or fabricated continuation is used.

One replacement candidate is frozen at
`89854110b41286cab0c6bf9748f37882a595a366` in
[run 38638855800](https://github.com/ThereptileII/Work/actions/runs/38638855800).
Its options are private Staging trust enabled, extended tests disabled and design
validation disabled. The old run's remaining Linux compilation was cancelled to
avoid continuing an undeliverable candidate alongside the replacement. Original
successful component results remain evidence with their original identity;
replacement application, installed journey and boat acceptance remain pending.
