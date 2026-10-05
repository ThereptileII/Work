# SCRUM-225: caller-relative dependency paths

[Native run 37283380247](https://github.com/ThereptileII/Work/actions/runs/37283380247)
at frozen commit `7397e057137303cb9371d16d6133d319361bf84e` stopped during
dependency verification, before full Windows application compilation. Artifact
`11333945843` was downloaded and its SHA-256 verified as
`cadc7f5e316279f06ab5b8878bbe61d256102390d1e01f00d4765060370d86b4`.

The retained `ais-native-runtime/dependency-native-reprobe.log` reports the
missing directory `D:\a\Work\Work\opennav-x\dependency-bundle`. The workflow
downloaded the bundle beside `opennav-x`; its AIS caller initially authenticated
those workspace-relative inputs, then forwarded the same relative spellings to
a child launched with the project directory as its working directory. The
child therefore looked in a different location. Earlier curl source checks and
zlib negative controls passed; this failure does not establish a dependency
compiler or TLS regression. The earlier repeated-native evidence in
`dependency-reprobe-native.json` retains its original, narrower scope.

The helper now makes both arguments absolute in the caller's directory before
verification or child launch. It deliberately leaves links unresolved so the
existing link/reparse, authenticated provenance, inventory and native-toolchain
checks remain authoritative. Same-job mode and the native reprobe are unchanged.

The focused offline regression invokes the real argument/reprobe path and an
inert child from the project directory, using workspace-relative,
project-relative and absolute inputs. Different files at the incorrect location
ensure the test checks the selected input bytes. Existing tamper/fail-closed
cases remain required. Native rerun and application/boat qualification remain
pending; this helper repair neither modifies nor relabels frozen `7397`.
