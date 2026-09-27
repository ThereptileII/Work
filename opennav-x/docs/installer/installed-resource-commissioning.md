# Observed installed coastline-resource default

The first fixture-free boat launch of `8e780edc34f68abd693a5d5f6aecdb3ba05a75c4`
filled the existing empty `Directories/BaseShapefileDir` with the original
supported OpenCPN application's `basemap_shp` directory. It did not point to the
XNav generation, change chart directories or copy the chart database.

`OpenCPNIntegration.cpp::ApplyInstalledResourceDefaults` and
`InstalledResources.cpp` deliberately supply this default only when no resource
was selected. Their managed `app/OPENNAV_INSTALLED_STOCK` marker keeps it stable
through integration updates/uninstall. Pinned `navutil.cpp::UpdateSettings`
persists the value with wxFileConfig's Windows backslash escaping. The existing
native installer resource tests already check that it identifies the same stock
filesystem object. The boat runtime guard previously refused every directory
change, including this intentional default; no acknowledgement was sent after
that refusal.

The installed warning/runtime review now recognizes only this exact transition:

* An existing empty BaseShapefileDir becomes the exact escaped stock default.
* The current owned generation's marker is hash-bound and names its exact
  supported stock executable; the executable hash is independently checked.
* All six resource prerequisites used by the product selector exist, are
  nonempty, and do not traverse redirected paths.
* Custom/missing baseline values, generation paths, alternative spellings,
  traversal, other directories, chart paths and data connections still refuse.

The common cold restoration and new-launch checks remain strict. Recognizing a
running application's resource default does not approve baseline adoption or
refresh a launch audit. Final closed-session changes still require their own
exact per-key review and preserved recovery lineage.

Portable filesystem tests exercise the complete installed runtime proof with
an inert owned marker and stock hash oracle, successful filling of the empty
preference, every missing selector prerequisite, changed marker/stock identity,
alternative paths and protected custom selections. They also verify that cold
restoration still rejects the same delta. Native execution and actual boat
warning review are separate gates; no test starts equipment or vendor helpers.

Native tooling `3bc275e88eb831fae0d11cc8dd6597cb28658808` passed all nine jobs
in [run36296325100](https://github.com/ThereptileII/Work/actions/runs/36296325100).
The installed suite passed114 checks, including29 complete runtime checks.
All nine artifact API/upload/download hashes, sizes and ZIP CRCs match.
[Qualification evidence](../evidence/beta2-installed-resource-3bc275.json).
This validates the tooling, not the pending boat warning or product screens.
