# Immutable prototype tooling

Install `requirements.txt` in a dedicated Python environment, then:

```
python -m playwright install chromium
python tools/prototype/test_contract.py
python tools/prototype/render.py --output evidence/local/prototype-reference
```

The supplied 113 files are verified byte-for-byte before/after every render.
No source rewriting, network requests, device connections or application launch
occurs. Each state uses a new browser context and real prototype interactions.
The fixed browser clock/reduced-motion setting removes temporal variation.

`capture.json` contains the input hash, browser/Playwright versions, geometry,
computed styles, actual platform fonts, visible actions and PNG hashes. The
Linux set uses Liberation Sans via the prototype fallback stack on this host;
it is not Windows typography acceptance. Windows must render its own reference
with the same installed font environment used by native XNav.

The prototype contains intentionally illustrative data. These files are design
and CI resources only, not installed product resources or a new Demo mode.
Do not execute `prototype/build.mjs` in the immutable evidence directory.
