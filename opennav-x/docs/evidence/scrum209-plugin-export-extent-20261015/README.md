# Bundled-plugin export-directory refusal

Application producer: `89854110b41286cab0c6bf9748f37882a595a366`,
[38638855800 attempt 2](https://github.com/ThereptileII/Work/actions/runs/38638855800/attempts/2),
successful native integration job `116128956769`. The original downstream
qualification fails Day → Dusk. That original failure is preserved.

Checked-pointer harness `c1dd02682d2037af10961baaf6266f66e922c7d4` passes
five focused portable helper cases and retests the original sealed application
and package inputs in
[38684111006](https://github.com/ThereptileII/Work/actions/runs/38684111006),
job `116254949622`. It builds no application and grants no release acceptance.
The retest's authenticated receipt binds the original input archive SHA-256
`4d23efa558deab6ca8b8a0f7ca177e2af55b74465e7550cd9c8dd92452e4c792`.

The retest fails at the first Day pointer action. Native window inspection shows
a `Message` dialog reporting startup errors for `chartdldr_pi.dll`,
`dashboard_pi.dll`, `grib_pi.dll` and `wmm_pi.dll`. The visible Day control exists
but the main frame is blocked by that dialog. Each original plugin log entry
reports `Cannot inspect plugin imports: Invalid export directory extent` before
`LoadLibrary`. Do not dismiss this product error or weaken the pointer check to
make the mode test pass. Earlier fixture/resource checks do not prove plugin
factory acceptance.

The result archive is artifact `11949317833`, 12,011,226 bytes, SHA-256
`3e9fcb83e7bf4564ba96d27d74942720cbc22dda5ab6d521c4c871ef122098bc`.
Original Actions metadata, run/source identity, downloaded length/hash and all
1,906 ZIP member CRCs verify. [Sanitized receipt](retest-receipt.json).

Separate original plugin capture artifact `11934702133` is 131,376,585 bytes,
SHA-256 `67ce2859b66423015f8c9cbb3577b91409547625c5b8d7483402e48d6f177c63`;
inventory SHA-256 is
`d0390e889be821e277c5962292366c4d129354003ccb5dfeadc539d926aea811`.
The maintained authentication, bounded unpacking, capture and TLS-binding checks
pass for its 1,585 inventory files and unchanged dependency SDK. This is input
custody, not a native plugin/download acceptance result.

Its `runtime/wxmsw32u_core_vc14x.dll` has:

- PE32/I386; 7,467,008 file bytes.
- SHA-256 `beadbd9824a90d54e25f899afb9cf45611e7f74c9e0a0ceb6531e0bf6d21bdc3`.
- Export RVA `0x55d9b0`, size `0x11af00` (1,158,912 bytes), fully backed in
  `.rdata` at raw offset 5,619,632, with 19,522 export address-table entries.

Compiled recovery artifact `11933564059` authenticates 427,698,288 outer bytes,
SHA-256 `237bd4d75d00b08069b067bc778434186f20068d3366fdb2d393da5d9a3ee9b4`.
The inner archive SHA-256 is
`29c726a2efd105f7464a1377f360a4391eb64a959d60e821e15854e94f814480`;
its original-log-bound manifest is
`a1187c39a1f886fcf156a0aebef5caa8c219aa34086e9acc57bfb58507f4f816`.
All 16,984 members and 894,235,565 unpacked bytes verify. Both actual install
variants contain the same wx core hash above. Each of the four actual bundled
plugins directly imports it; their own export directories are only 7,080 or
7,564 bytes. This establishes the same-package common dependency rather than
inferring it solely from the captured SDK.

The current reader rejects this backed directory solely because its extent
exceeds 1 MiB. That extent is not the amount of data the parser scans: work is
already bounded by the file/raw/RVA checks, 65,536 EAT entries, 4,096 forwarders,
per-string limits and the shared 1 MiB declaration-name budget. Independent
source review supports removing only the redundant extent ceiling and retaining
all those bounds. Malformed/unbacked large-directory fixtures must still refuse;
valid large directories and late legacy-TLS forwarders need positive/negative
coverage. The corrected native/product gates remain pending. No bundled-plugin,
installed lifecycle or boat acceptance follows
from the byte-reader diagnosis.
