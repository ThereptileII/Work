# Native IHO review input preparation

`tools/prototype/fetch-iho-review-cell.py` retrieves only the source-locked
official unencrypted presentation-test archive already inspected in
[the source/rights review](../design/reviews/scrum256-official-cardinal-test-source.md).
The exact 2,676,974-byte archive and 945,697-byte `GB4X0000.000` member are checked
against their independently recorded SHA-256 values before writing a fresh
fixture directory. No other member is extracted and existing output is refused.

On 2026-10-03 both the retained-cache path and an actual download from the
officially linked Google Drive file produced archive hash
`01fe16330e1a704f1e65de43ca97724623fd8e619d31c6e49ea7f960ecac6057` and cell hash
`c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
This verifies acquisition for the collector; it is not a Windows rendering test.

Only code and this receipt are committed. Keep the original archive, chart cell
and generated SENC outside source, product and review artifacts. It is official
test geography, not an operational nautical chart or an unrestricted licence.
The future native run must download and verify the same pins again; no chart
data is bundled with SKAGER by this change.
