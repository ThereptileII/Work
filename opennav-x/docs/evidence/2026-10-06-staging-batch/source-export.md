# Exact source export correction

Initial remote candidate `39fd80a185c68f1e06f14ccd83aeea3f3e84997f`
failed distribution preflight in [run 37416635422](https://github.com/ThereptileII/Work/actions/runs/37416635422)
before the Windows application build. It retained an old fixed-version assertion.
The local preflight passed because prior local version-resource changes had not
all been exported into the presumed remote source ancestor.

A complete comparison of every mapped local tracked blob against the recursive
remote tree found eleven differences: ten version-resource/installer/evidence
files and one missing earlier dependency evidence record. Those exact source
files are included in the correction. No validation assertion was weakened to
accept the incomplete export. The corrected tree must match every mapped local
blob and mode before moving the Staging ref; unrelated monorepo content remains
untouched. Local commit ancestry alone is not proof of this mapping.

The failed run remains evidence. Its successor supersedes it before a native
application can be accepted. The separate version-aware boat deployment helper
is included so a valid `.1` package is compared to its exact expected version,
instead of a historical constant, without changing hash or profile gates.
