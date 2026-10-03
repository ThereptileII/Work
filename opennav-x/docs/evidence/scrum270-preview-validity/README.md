# SCRUM-270: coherent preview progress snapshot

The native failure was a harness timing error. In [run 37120549213](https://github.com/ThereptileII/Work/actions/runs/37120549213), step 15 reached the real Demo waypoint transition at second 115. `smoke-preview.py` waited only for decreased SOC, then indexed an intentionally absent route distance. The application correctly reported `ActivePointChanged` / `UNAVAILABLE` and withheld arrival SOC too. The retained screenshot reads “Route unavailable.” All preceding preview page checks completed; later preview checks did not run.

The only functional change is the `later=data(...)` predicate: require one `Valid` route snapshot containing both `remaining_nm` and `arrival_soc`, alongside the original decreased-SOC condition. The 12-second timeout, latitude/distance/arrival comparisons, scenario timeline and application sources are unchanged. No reset, pause, default distance or fixture extension is introduced. A persistently invalid/ended route still fails the existing timeout.

Failure provenance:

- Frozen application remote commit `9d98a500916e8a7f59dac9735427dde6d3c7d2e5`; tooling repair base `48be04c35de20c150e885e797eff16e25db208cb`.
- Artifact `11275626437`, `windows-integration-9d98a500916e8a7f59dac9735427dde6d3c7d2e5`, 53,118,284 bytes; SHA256 `eb11a4ab014832e3304a387ada3b3f8afe8195f2d3850ee24c1ccaed1f064adc`. The independent artifact auditor verified the archive SHA and all 13,361 entry CRCs.
- Exact entry `evidence/local/preview-logs/opennav-diagnostics.json`, 65,677 bytes; SHA256 `d2d2a125309198258edb53c19b2099a38169bffd796e1ccb75d94578df4eb48c`. Raw artifact remains retained in `scrum259-full-9d98-watch/.local/windows-integration-9d98.zip`; no rewritten JSON substitutes for it.
- Snapshot: waypoint index 2 / `DEMO-Sheltered-bay`, observation 6138453 ms, latitude 59.1145 and SOC 47.65103358. Window found at 6023359 ms; failure cleanup at 6138640 ms. These agree with the genuine second-115 production fixture output.

`prove.py` freshly compiles only the six small unchanged fixture/route/energy implementation units with `emit.cpp`; no cached binary/object identity is assumed. It verifies those implementation bytes against the base commit and records the compiler-discovered local header closure. The emitter calls actual `DemoFixture`, `AssessRoute`, `PreviewEnergyModel` and `PredictVesselEnergy` at seconds 0, 114, 115 and 116. It only serializes the fields consumed by the harness; it contains no copied trip equations and does not execute `PreviewDiagnostics` or an application.

The proof extracts the old and new predicates, `item` helper and three original comparisons from the actual Python AST. The retained native failure and real second-115 output reproduce the original `KeyError('remaining_nm')`, but the new predicate rejects both. Seconds 114 and 116 are accepted and pass all three unchanged comparisons against the initial production sample. The initial sample remains rejected because SOC has not progressed. The AST check verifies the unchanged 12-second default and unchanged comparisons. Python syntax checks also passed.

Reproduce from the repository root with the exact extracted artifact entry:

```sh
python docs/evidence/scrum270-preview-validity/prove.py \
  --failed-json /path/to/extracted/evidence/local/preview-logs/opennav-diagnostics.json \
  --output .local/preview-validity-proof
```

[proof.json](proof.json) records every consumed source/header hash, the emitter and output hashes, compiler command and five results; [samples.json](samples.json) contains actual emitted values. The emitter binary remains private in `.local/preview-validity-proof/emit`. No application source, full build, broad suite, native execution, CI dispatch or boat operation occurred. A native run of the existing complete preview scenario remains required; this bounded proof is not native UI acceptance.
