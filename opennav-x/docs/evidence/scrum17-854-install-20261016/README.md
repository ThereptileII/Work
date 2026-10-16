# SCRUM-17: 854 installation and stock restoration

The actual boat Update to `854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc`
(`0.4.0-beta2.4`) and subsequent exact stock RTL-SDR restoration passed.
The final identity was recorded at **2026-10-16 11:16:41 UTC**. Fresh passive
commissioning is separate and was in progress at this snapshot; this receipt
does **not** establish boat functional acceptance.

The active generation changed from `6a6bb7454555418aa8dcce31a0a89d12`
(product `f1a1e68c81fb60324f47cc20a7c4479e1c60c0c8`) to
`d597466b96c84153975ad7169c2f3b92`. The previous generation remains selected
as the rollback predecessor, and its complete tree was rechecked unchanged
after Update. The final executable SHA-256 is
`8ad205a452683316330549bb93a5d0c039b2a2acb4d59eaa6aa4c5d3b85bc511`.
The original stock executable remains
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`.

The [original failed Update and diagnostic](../scrum17-rtlsdr-854-20261016/README.md)
remain preserved. That Update refused the retained stock `rtlsdr_pi.dll` before
commit. The read-only 103-PE scan found only its two missing wx 3.1.2 imports.
For the successful Update, the separately [qualified maintenance helper](../scrum22-rtlsdr-hold-20261016/README.md)
held only that exact 208,384-byte stock DLL, ran no application, and subsequently
moved it back with its original access ACL. The original installer/package and
strict installer gates were unchanged; Update ran separately between Hold and
Restore. The stock DLL SHA-256 is
`404e69931034140fbda0179964d0a29077a46b78a441c05fc68a4ee0616a64e5`.

Inspect, Hold and Restore bind the same private plan SHA-256
`de0ee95ab4f5458b0393614f1c5d740588ae4a0f600d9f71aaea90ba4e0cefb9`.
Successful Restore verifies the complete profile, managed tree, old generation,
and stock tree except the independently pinned DLL. The DLL bytes and original
access ACL are checked separately before and after the move. This preserves the
original executable and profile state while allowing the new installation state
to select the new generation. Inventory digests are recorded in
[receipt.json](receipt.json); they are the maintained helper's inventory hashes,
not filesystem-image or runtime-behavior claims.

The installer report records **Update passed**, mandatory self-test and commit.
The wrapper records `launchPending=true`; the final identity records
`applicationProcessCount=0`. No ordinary navigation/commissioning launch or
hardware command was performed in this installation sequence. The mandatory
loader/resource self-test and maintenance subprocesses did execute: this is
**not** a zero-code-execution claim.

The private installer report is bound by SHA-256
`2f52e515c28c25fadb9251ebc934e2957a460f8b4224344217f9f58fe072036d`,
and final identity by
`ab627d0eb4e0778c76c370b3925871fb353f608de9adcc390314858ca6b92bf4`.
The receipt records exact sizes and hashes for all nine retained source records,
including the historical failure and prior installed identity. Private user paths,
account/SID and ACL text, profile/navigation contents, charts and credentials are
omitted. Only these two evidence documents were added; no product source or
package was changed for this record.

Stock restoration does not authorize that incompatible plugin for the new
generation. Fresh installed/plugin inventory and the commissioning plan remain
required before the separately guarded ordinary launch. Plugin initialization,
private-chart outcomes, AIS/zoom behavior and physical output are not qualified
by this installation receipt.
