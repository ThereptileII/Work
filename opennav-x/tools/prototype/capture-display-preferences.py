#!/usr/bin/env python3
"""Capture derived Display preference states from the immutable prototype."""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import re
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import render  # noqa: E402: reuse the canonical offline renderer contract

OUTPUT = ROOT / "evidence/local/scrum216-display-reference"
UI_SCALES = ("100%", "125%", "150%")
LAYOUTS = ("Balanced", "Chart focus", "Instrument focus")
RESPONSIVE_VIEWPORTS = ((1024, 640), (760, 800))
MEASURE_SELECTORS = list(dict.fromkeys(render.SELECTORS + [
    "#displayScale", "#displayLayout", ".field select", ".settings-tabs",
    ".settings-tabs button", ".drawer-body > .btn", ".data-rail",
    ".timeline", ".metric-value", "#app",
]))


def viewport_arg(value):
    match = re.fullmatch(r"([1-9][0-9]*)x([1-9][0-9]*)", value)
    if not match:
        raise argparse.ArgumentTypeError("viewport must be WIDTHxHEIGHT")
    return int(match.group(1)), int(match.group(2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--device-scale-factor", type=float, default=1.0,
                        help="Browser deviceScaleFactor; independent of prototype UI scale")
    parser.add_argument("--ui-scales", nargs="+", choices=UI_SCALES, default=list(UI_SCALES))
    parser.add_argument("--layouts", nargs="+", choices=LAYOUTS, default=list(LAYOUTS))
    parser.add_argument("--responsive-viewports", nargs="*", type=viewport_arg,
                        default=list(RESPONSIVE_VIEWPORTS),
                        help="Additional CSS viewport sizes, e.g. 1024x640 760x800")
    args = parser.parse_args()
    if args.width < 1 or args.height < 1 or args.device_scale_factor <= 0:
        parser.error("width, height, and device-scale-factor must be positive")

    manifest = render.verify_original()
    args.output.mkdir(parents=True, exist_ok=True)
    from playwright.sync_api import sync_playwright

    viewports = [(args.width, args.height)]
    for item in args.responsive_viewports:
        if item not in viewports:
            viewports.append(item)
    record = {
        "schema": 1,
        "purpose": "Derived measurements after actual Display select and Apply interactions; not a replacement canonical reference.",
        "htmlSha256": manifest["htmlSha256"],
        "platform": platform.system(),
        "playwright": importlib.metadata.version("playwright"),
        "deviceScaleFactor": args.device_scale_factor,
        "uiScaleMeaning": "The prototype's Interface scale preference (CSS sizing), independent of browser deviceScaleFactor or operating-system DPI.",
        "responsiveViewports": [
            {"width": width, "height": height} for width, height in viewports
        ],
        "reducedMotion": True,
        "clockUtc": "2026-09-28T11:49:00Z",
        "states": {},
    }

    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=True)
        record["browser"] = browser.version
        for width, height in viewports:
            for scale in args.ui_scales:
                for layout in args.layouts:
                    state_key = f"{width}x{height}-dpr{args.device_scale_factor:g}-ui{scale}-layout{layout.lower().replace(' ', '-') }"
                    context = browser.new_context(
                        viewport={"width": width, "height": height},
                        device_scale_factor=args.device_scale_factor,
                        reduced_motion="reduce", locale="en-GB", timezone_id="UTC",
                        color_scheme="dark",
                    )
                    requests, errors = [], []

                    def route_request(route):
                        url = route.request.url
                        if url.startswith(("http://", "https://")):
                            requests.append(url)
                            route.abort()
                        else:
                            route.continue_()

                    context.route("**/*", route_request)
                    page = context.new_page()
                    page.on("pageerror", lambda error: errors.append(str(error)))
                    page.clock.install(time=datetime(2026, 9, 28, 11, 49, tzinfo=timezone.utc))
                    page.goto((render.ORIGINAL / manifest["html"]).as_uri())
                    page.evaluate("document.fonts.ready")
                    for selector in render.STATES["display"]:
                        page.locator(selector).filter(visible=True).first.click()

                    app = page.locator("#app")
                    if page.locator("#displayScale").input_value() != "100%" or page.locator("#displayLayout").input_value() != "Balanced":
                        raise RuntimeError("Prototype Display defaults changed unexpectedly")
                    if app.get_attribute("data-ui-scale") is not None or app.get_attribute("data-layout") is not None:
                        raise RuntimeError("Fresh context unexpectedly has committed display preferences")

                    scale_select = page.locator("#displayScale")
                    layout_select = page.locator("#displayLayout")
                    apply_button = page.locator('[data-action="applyDisplay"]')
                    for control in (scale_select, layout_select, apply_button):
                        if not control.is_visible() or not control.is_enabled():
                            raise RuntimeError(f"Display control unavailable: {control}")
                    scale_select.select_option(label=scale)
                    layout_select.select_option(label=layout)
                    if app.get_attribute("data-ui-scale") is not None or app.get_attribute("data-layout") is not None:
                        raise RuntimeError("Select changes committed before Apply")
                    apply_button.click()
                    if app.get_attribute("data-ui-scale") != scale or app.get_attribute("data-layout") != layout:
                        raise RuntimeError("Apply did not commit the selected display preferences")
                    if scale_select.input_value() != scale or layout_select.input_value() != layout:
                        raise RuntimeError("Applied select values do not match requested state")
                    page.clock.run_for(3500)  # match render.py's settled toast timing
                    page.mouse.move(width - 1, height - 1)
                    data = page.evaluate(render.MEASURE, MEASURE_SELECTORS)
                    data["drawerBodyScrollTop"] = page.locator(".drawer-body").evaluate("element => element.scrollTop")
                    png = args.output / f"{state_key}.png"
                    page.screenshot(path=str(png), animations="disabled")
                    data["screenshotSha256"] = hashlib.sha256(png.read_bytes()).hexdigest()
                    data["uiScale"] = scale
                    data["layout"] = layout
                    data["viewport"] = {"width": width, "height": height}
                    data["deviceScaleFactor"] = args.device_scale_factor
                    data["interaction"] = {
                        "controlIds": ["displayScale", "displayLayout"],
                        "applyAction": "applyDisplay",
                        "selectsVisibleEnabled": True,
                        "notCommittedBeforeApply": True,
                        "committedValues": {"data-ui-scale": scale, "data-layout": layout},
                    }
                    if errors or requests:
                        raise RuntimeError(f"{state_key}: offline render failed: pageErrors={errors}, externalRequests={requests}")
                    record["states"][state_key] = data
                    print(state_key, flush=True)
                    context.close()
        browser.close()

    render.verify_original()
    (args.output / "capture.json").write_text(
        json.dumps(record, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
