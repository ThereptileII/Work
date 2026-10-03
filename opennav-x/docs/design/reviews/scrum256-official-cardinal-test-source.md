# SCRUM-256: official cardinal presentation-test source

Read-only research, 2026-10-03. No application launch, chart modification, build, CI or boat action.

## Recommended bounded fixture

Use **IHO S-64 Edition 3.0(.3), `3.3 Settings/ENC_ROOT/GB4X0001.000`**, explicitly labelled **official presentation-test data; not an operational nautical chart**. It contains actual S-57 BOYCAR and BCNCAR records for all four CATCAM values. These objects were supplied by IHO; none were generated or injected by this research.

The [official IHO download document](https://iho.int/uploads/user/pubs/standards/s-64/S-64_Download_Links_Document.pdf) links its unencrypted package to [this download](https://drive.google.com/file/d/1iJl0SyymUxfzFMqFZNK0t-qUwNvw61m4/view). The [instruction manual](https://iho.int/uploads/user/pubs/standards/s-64/S-64%20Ed%203.0.3_EN_Clean_Final.pdf), section 3.3.1, page 111, specifies this cell for comparing paper-chart and simplified symbols. Its reference view is 32°37.280′ S, 61°21.000′ E at 1:10,000, with Other display category and 10 m safety depth/contour. This is a later fixture configuration, not a change to product defaults.

## Actual decoded contents

Read directly with pyogrio 0.13.0 / GDAL 3.12.4 S-57 driver, using raw feature arrays and decoded point WKB. Pinned OpenCPN `s57expectedinput.csv` independently maps CATCAM 1/2/3/4 to north/east/south/west.

| Cell | BOYCAR N/E/S/W | BCNCAR N/E/S/W | Coverage longitude | Coverage latitude |
|---|---|---|---|---|
| GB4X0001, Settings | 2 / 1 / 1 / 1 | 1 / 1 / 1 / 1 | 61.3333333 to 61.5 | -32.6333333 to -32.3166667 |
| GB4X0000, Power Up | 1 / 4 / 4 / 2 | 2 / 1 / 2 / 1 | 60.7666667 to 61.3333333 | -32.6333333 to -32.3166667 |

Both cells specify compilation scale 1:52,000. The Settings cell has one additional north buoy outside the symbol cluster with DATSTA=20010816 and DATEND=20120816. Do not change the clock or count that expired example as a current visible mark.

The eight undated Settings symbols have no DATSTA/DATEND/PERSTA/PEREND or SCAMIN value. Their actual positions are:

| Feature | CATCAM | Latitude | Longitude |
|---|---|---|---|
| BCNCAR | 1 | -32.6215085 | 61.3459877 |
| BCNCAR | 3 | -32.6215332 | 61.3477097 |
| BCNCAR | 4 | -32.6215326 | 61.3493894 |
| BCNCAR | 2 | -32.6214853 | 61.3511253 |
| BOYCAR | 1 | -32.6199859 | 61.3459936 |
| BOYCAR | 3 | -32.6198969 | 61.3477206 |
| BOYCAR | 4 | -32.6199096 | 61.3493575 |
| BOYCAR | 2 | -32.6198881 | 61.3511069 |

This compact cluster spans longitude 61.3459877–61.3511253 and latitude -32.6215332–-32.6198881. A view centred near latitude -32.6207, longitude 61.3486 is an evidence-derived framing option. Preserve chart geometry, date rules, topmark relations and navigation semantics; inspect actual renderer output before choosing final capture scale.

## Provenance and rights boundary

IHO provides the unencrypted package for ECDIS visualization/operation testing. The manual is copyrighted by IHO (2020), with third-party rights possible. Page ii contains limited reproduction/distribution conditions and requires prior permission for commercial exploitation; it is **not an open-source or public-domain licence**. The current proposal is local evaluation from the official download. Keep original datasets outside the repository/product; do not infer permission to redistribute cells or reference artwork, or imply IHO endorsement. If public redistribution becomes necessary, resolve that permission separately.

## Reproducibility

Private cache: `/home/standard/Projects/X-nav-worktrees/scrum257-final-captures/.local/cardinal-research/`. `inspect-features.py` produces `feature-inventory.json`, retaining DSID metadata, feature IDs, attributes and coordinates. No application renderer was used.

- Unencrypted ZIP SHA-256: `01fe16330e1a704f1e65de43ca97724623fd8e619d31c6e49ea7f960ecac6057`.
- `2.1.1 Power Up/ENC_ROOT/GB4X0000.000`: 945697 bytes; SHA-256 `c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
- `3.3 Settings/ENC_ROOT/GB4X0001.000`: 33989 bytes; SHA-256 `8f3733335b4fb0b2dbbc8a13bb3ca3b6ee296525d483a4d01056fe1ab201a3bd`.
- Manual PDF SHA-256: `ff7c19f510f95be30964e8c5ae9601f25a71785d774a44fe91469976b2846e12`.

This establishes official test-source suitability and actual categories only. Actual pinned OpenCPN decode, SKAGER/Standard portrayal, Day/Dusk/Night recognition, software/GL, Windows/DPI and physical-display acceptance remain unproved by this research. No operational-chart cardinal recognition claim follows.
