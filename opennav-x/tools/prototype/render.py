#!/usr/bin/env python3
"""Render the immutable supplied prototype offline, using its real interactions.

Browser clock/motion are controlled through Playwright, never by editing HTML.
The output records platform fonts: a Linux fallback is not Windows acceptance.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import stat
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
ORIGINAL = ROOT / "docs/design/prototype"
MANIFEST = ROOT / "docs/design/prototype-original.json"

# Each state starts in a new context. Only existing prototype UI is exercised.
STATES = {
    "navigation": [],
    "passage": ['.nav-btn[data-panel="route"]'],
    "traffic": ['.nav-btn[data-panel="ais"]'],
    "ais-target": ['.nav-btn[data-panel="ais"]', '[data-select-target="0"]'],
    "energy": ['.nav-btn[data-view="energy"]'],
    "instruments": ['.nav-btn[data-view="instruments"]'],
    "anchor": ['.nav-btn[data-panel="anchor"]'],
    "radar": ['.nav-btn[data-view="radar"]'],
    "autopilot": ['.autopilot-summary'],
    "settings": ['.nav-btn[data-panel="settings"]'],
    "sensors": ['.nav-btn[data-panel="settings"]', '[data-settings-tab="Sensors"]'],
    "display": ['.nav-btn[data-panel="settings"]', '[data-settings-tab="Display"]'],
    "system": ['.nav-btn[data-panel="settings"]', '[data-settings-tab="System"]'],
    "health": ['.health-button'],
    "health-gps": ['.health-button', '.sensor-details summary'],
    "alerts": ['.alert-button'],
    "search": ['.header-end [data-panel="search"]'],
    "layers": ['.map-top-controls [data-panel="layers"]'],
    "rail": ['.rail-title button'],
    "diagnostics": ['.nav-btn[data-panel="settings"]', '[data-settings-tab="System"]', '[data-action="showDiagnostics"]'],
    "waypoint": ['[data-map-waypoint="0"]'],
    "navigation-loss": ['.health-button', '[data-action="gpsLoss"]', '#drawerBack'],
}

SELECTORS = [".topbar", ".sidebar", "#workspace", "#chartView", ".data-rail",
             ".timeline", ".timeline-heading", ".timeline-heading .eyebrow", ".timeline-heading .eyebrow svg",
             ".timeline-heading .advisory-label", ".timeline-heading .text-button", ".timeline-heading .text-button span",
             ".timeline-events", ".timeline-event", ".event-dot", ".event-dot svg", ".event-time", ".event-time span",
             ".timeline-event>b", ".timeline-event>small", ".timeline-event>small span", ".timeline-event>small em",
             ".statusbar", ".statusbar>span:first-child", ".footer-middle", ".statusbar>button", ".nav-btn", ".icon-btn", ".metric",
             ".metric-value", ".metric-label", ".next-turn", ".turn-main",
             ".turn-sub", ".map-tools", ".follow-btn", ".autopilot-summary",
             ".drawer", ".drawer-head", ".drawer-head h2", ".drawer-body",
             ".btn", ".segment button", ".toggle", ".list-card", ".row",
             '#chartSearch', '#searchResults .list-card b', '#searchResults .list-card small',
             ".dashboard-card", ".big-stat", ".view-header h1", ".tag",
             ".chart-route", ".chart-land", ".chart-depth", ".ais-ship",
             ".pill-row", ".stats-grid", ".section-label", ".route-waypoint",
             ".waypoint-number", ".waypoint-info", ".waypoint-info b",
             ".waypoint-info small", ".waypoint-info span", ".callout",
             "#fullView", ".view-header", ".view-header .eyebrow",
             ".view-header p", ".dashboard-grid", ".dashboard-card h3",
             ".wind-rose", ".instrument-grid", ".instrument-tile",
             ".instrument-tile label", ".instrument-tile b",
             ".instrument-tile>small", ".action-row", ".energy-gauge",
             ".battery-visual", ".battery-visual i", ".energy-chart",
             ".energy-chart svg", ".stat-label", ".power-row", ".power-row>div",
             ".power-row label", ".power-row b", ".settings-tabs",
             ".settings-tabs button", ".settings-intro", ".settings-intro .eyebrow", ".settings-intro h3",
             ".settings-intro p", ".suite-link", ".suite-link b", ".suite-link small",
             ".suite-link-icon", ".field", ".field input", ".profile-btn", ".pill-row", ".anchor-graphic", ".anchor-distance",
             ".anchor-distance small", "#anchorRadius", ".field b", ".stats-grid",
             ".stats-grid label", ".stats-grid strong", ".note",
             ".heading-dial", ".heading-dial svg", ".dial-value", ".dial-value span",
             ".dial-value small", ".heading-controls", ".radar-layout", ".radar-display",
             ".radar-scope", ".radar-crosshair", ".radar-north", ".radar-scope-caption",
             ".radar-scope-caption b", ".radar-scope-caption span", ".radar-legend",
             ".radar-control-panel", ".radar-mode-label", ".range-field", ".range-field input",
             ".radar-layout .row", ".radar-layout .field", ".radar-layout select",
             ".health-button", ".sensor-details", ".sensor-details summary", ".sensor-dot"]

# Observers only: these do not modify the immutable chart or its supplied data.
CHART_TEXT_SELECTORS = [".chart-label", ".chart-water-label", ".chart-symbol-label", ".chart-landmark-label",
                        ".chart-depth", ".map-pop-label", ".waypoint-number", ".map-waypoint text",
                        ".light-sector-readout b", ".light-sector-readout small"]
SELECTORS += CHART_TEXT_SELECTORS
FONT_SELECTORS = [".brand", ".metric-value", ".metric-label", ".drawer-head h2", ".view-header h1",
                  ".settings-tabs button", "#chartSearch", "#searchResults .list-card b",
                  "#searchResults .list-card small", ".timeline-heading .eyebrow",
                  ".timeline-heading .text-button", ".event-time", ".timeline-event>b",
                  ".timeline-event>small"] + CHART_TEXT_SELECTORS

MEASURE = """selectors => {
 const app = document.querySelector('#app');
 const cs = getComputedStyle(app);
 const vars = Object.fromEntries([...cs].filter(x=>x.startsWith('--'))
     .sort().map(x=>[x,cs.getPropertyValue(x).trim()]));
 for (const k of Object.keys(vars)) if (vars[k].startsWith('url(')) delete vars[k];
 const properties = ['fontFamily','fontSize','fontWeight','fontStyle','lineHeight',
   'letterSpacing','fontVariantNumeric','textTransform','color','backgroundColor',
   'borderRadius','borderWidth','borderColor','padding','margin','gap','boxShadow',
   'opacity','transition','minHeight','minWidth','stroke','strokeWidth','fill',
   'display','alignItems','justifyContent','gridTemplateColumns','rowGap','columnGap',
   'paddingTop','paddingRight','paddingBottom','paddingLeft','borderLeftWidth',
   'borderLeftColor','textAlign','cursor'];
 const components = {};
 for (const sel of selectors) {
   components[sel] = [...document.querySelectorAll(sel)].filter(e=>e.getClientRects().length)
    .map(e=>{const s=getComputedStyle(e),r=e.getBoundingClientRect();return {
     rect:{x:r.x,y:r.y,width:r.width,height:r.height},
     style:Object.fromEntries(properties.map(k=>[k,s[k]]))};});
 }
 const interactive = [...document.querySelectorAll('button,[role=button],input,select')]
   .filter(e=>e.getClientRects().length).map(e=>({tag:e.tagName,id:e.id,
      label:e.getAttribute('aria-label')||e.textContent.trim(),
      disabled:!!e.disabled,attributes:Object.fromEntries([...e.attributes]
        .filter(a=>a.name.startsWith('data-')).map(a=>[a.name,a.value]))}));
 return {theme:app.dataset.theme,variables:vars,components,interactive};
}"""


def verify_original():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for item in manifest["files"]:
        path = ORIGINAL / item["path"]
        data = path.read_bytes()
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise RuntimeError(f"Immutable prototype differs: {item['path']}")
    return manifest


def plain_file(path):
    path = Path(path).absolute()
    for part in (path, *path.parents):
        info = part.lstat()
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise ValueError("Linked/reparse renderer input refused")
    if not path.is_file():
        raise ValueError("Renderer input is not a regular file")
    return path.resolve(strict=True)


@contextmanager
def owned_browser(args, record):
    """Keep the Playwright driver and browser temporary profiles in owned output.

    No persistent context or existing browser connection is used. TEMP is set
    before the driver starts because it, rather than the browser, creates the
    temporary user-data directory. This does not claim OS network isolation.
    """
    from playwright.sync_api import sync_playwright
    options = {"headless": True}
    if args.browser_channel:
        if platform.system() != "Windows" or not args.browser_executable or not args.expected_browser_sha256:
            raise ValueError("Installed Edge requires Windows and exact executable/hash inputs")
        executable = plain_file(args.browser_executable)
        digest = hashlib.sha256(executable.read_bytes()).hexdigest()
        if executable.name.lower() != "msedge.exe" or digest != args.expected_browser_sha256:
            raise ValueError("Installed Edge executable identity differs")
        options.update(channel=args.browser_channel, executable_path=str(executable))
        record["installedBrowser"] = {"channel": args.browser_channel, "executable": str(executable),
                                      "sha256": digest, "scope": "launcher bytes; browser reports runtime version"}
    elif args.browser_executable or args.expected_browser_sha256:
        raise ValueError("Executable identity options require explicit installed browser channel")
    if args.runtime_receipt:
        receipt = plain_file(args.runtime_receipt)
        record["runtimeReceipt"] = {"sha256": hashlib.sha256(receipt.read_bytes()).hexdigest(),
                                    "record": json.loads(receipt.read_text(encoding="utf-8"))}
    record["rendererSha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    record["python"] = {"executable": sys.executable, "version": sys.version}
    saved = {key: os.environ.get(key) for key in ("TEMP", "TMP", "TMPDIR")}
    try:
        with tempfile.TemporaryDirectory(prefix="owned-browser-", dir=args.output.resolve()) as temporary:
            for key in saved:
                os.environ[key] = temporary
            record["profilePolicy"] = "Fresh Playwright temporary profile within output; never uses normal profile"
            with sync_playwright() as pw:
                browser = pw.chromium.launch(**options)
                try:
                    yield browser
                finally:
                    browser.close()
        record["ownedTemporaryDirectoryRemoved"] = not Path(temporary).exists()
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--states", nargs="+", choices=list(STATES), default=list(STATES))
    parser.add_argument("--themes", nargs="+", choices=["day", "dusk", "night"], default=["day", "dusk", "night"])
    parser.add_argument("--scale", type=float, choices=[1, 1.25, 1.5], default=1,
                        help="Additional physical 1280x800 DPI reference; default remains canonical DPR1")
    parser.add_argument("--browser-channel", choices=["msedge"], help="Explicit installed Edge; default Chromium unchanged")
    parser.add_argument("--browser-executable", type=Path)
    parser.add_argument("--expected-browser-sha256")
    parser.add_argument("--runtime-receipt", type=Path)
    args = parser.parse_args()
    manifest = verify_original()
    args.output.mkdir(parents=True, exist_ok=True)
    record = {"schema": 1, "htmlSha256": manifest["htmlSha256"],
              "platform": platform.system(), "playwright": importlib.metadata.version("playwright"),
              "viewport": {"width": round(1280 / args.scale), "height": round(800 / args.scale)}, "deviceScaleFactor": args.scale,
              "reducedMotion": True, "clockUtc": "2026-09-28T11:49:00Z", "states": {}}
    with owned_browser(args, record) as browser:
        record["browser"] = browser.version
        for theme in args.themes:
            for name in args.states:
                context = browser.new_context(viewport=record["viewport"], device_scale_factor=args.scale,
                    reduced_motion="reduce", locale="en-GB", timezone_id="UTC", color_scheme="dark",
                    offline=True, service_workers="block")
                requests, errors, local_requests = [], [], []
                def reject(route):
                    requests.append(route.request.url)
                    route.abort()
                context.route("http://**/*", reject)
                context.route("https://**/*", reject)
                page = context.new_page()
                page.on("request", lambda request: local_requests.append(request.url)
                        if request.url.startswith("file:") else None)
                page.on("pageerror", lambda error: errors.append(str(error)))
                page.clock.install(time=datetime(2026, 9, 28, 11, 49, tzinfo=timezone.utc))
                page.goto((ORIGINAL / manifest["html"]).as_uri())
                page.evaluate("document.fonts.ready")
                for _ in range(["day", "dusk", "night"].index(theme)):
                    page.locator("#themeButton").click()
                for selector in STATES[name]:
                    page.locator(selector).filter(visible=True).first.click()
                page.clock.run_for(3500)  # let prototype toasts finish through their own timer
                page.mouse.move(record["viewport"]["width"]-1, record["viewport"]["height"]-1)
                data = page.evaluate(MEASURE, SELECTORS)
                data["localResourceRequests"] = local_requests
                cdp = context.new_cdp_session(page)
                cdp.send("DOM.enable")
                cdp.send("CSS.enable")
                doc = cdp.send("DOM.getDocument")
                data["platformFonts"] = {}
                data["fontObservations"] = {}
                for font_selector in FONT_SELECTORS:
                    node = cdp.send("DOM.querySelector", {"nodeId": doc["root"]["nodeId"], "selector": font_selector})
                    fonts = []
                    if node["nodeId"]:
                        fonts = cdp.send("CSS.getPlatformFontsForNode", {"nodeId": node["nodeId"]})["fonts"]
                        data["platformFonts"][font_selector] = fonts
                    data["fontObservations"][font_selector] = {
                        "nodePresent": bool(node["nodeId"]), "sample": "first matching DOM node",
                        "glyphCount": sum(font["glyphCount"] for font in fonts),
                        "scope": "Zero glyphs or missing node establish no rendered face"}
                unexpected_local = [url for url in local_requests
                                    if not url.startswith(ORIGINAL.resolve().as_uri() + "/")]
                if data["theme"] != theme or errors or requests or unexpected_local:
                    raise RuntimeError(f"{name}/{theme}: wrong state or offline render failed: {errors}, {requests}")
                if name == "navigation" and args.scale == 1:
                    expected = {".topbar": (0, 0, 1280, 68), ".sidebar": (0, 68, 80, 698),
                                ".data-rail": (1094, 68, 186, 698), ".statusbar": (0, 766, 1280, 34),
                                "#chartView": (80, 68, 1014, 566)}
                    for selector, bounds in expected.items():
                        actual = data["components"][selector][0]["rect"]
                        if tuple(actual[k] for k in ("x", "y", "width", "height")) != bounds:
                            raise RuntimeError(f"Canonical geometry changed: {selector}: {actual}")
                stem = f"{name}-{theme}"
                png = args.output / f"{stem}.png"
                page.screenshot(path=str(png), animations="disabled")
                data["screenshotSha256"] = hashlib.sha256(png.read_bytes()).hexdigest()
                cdp.detach()
                record["states"][stem] = data
                print(stem, flush=True)
                context.close()
    verify_original()
    (args.output / "capture.json").write_text(json.dumps(record, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
