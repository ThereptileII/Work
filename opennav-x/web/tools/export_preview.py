#!/usr/bin/env python3
"""Export allowlisted informational Django pages, never account/backend source.

The output is a closed, owner-private design preview, not production commerce.
Use a fresh output directory; existing content is never recursively removed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

WEB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WEB))
os.environ["DJANGO_SETTINGS_MODULE"] = "skager_web.settings.preview"

import django
django.setup()
from django.test import Client
from skager_web.marketing.content import ARTICLES

PAGES = ("/", "/features/", "/how-it-works/", "/pricing/", "/apps/", "/install/", "/support/", "/support/faq/",
         "/support/articles/", "/open-source/", "/legal/", "/legal/terms/", "/legal/privacy/",
         "/legal/refunds/", "/legal/beta-terms/", "/legal/cookies/", "/account/", "/downloads/",
         "/checkout/", "/welcome/")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    args = parser.parse_args()
    if len(args.source_commit) != 40 or any(c not in "0123456789abcdef" for c in args.source_commit):
        parser.error("source-commit must be the exact committed source revision")
    output = args.output.resolve()
    if output.exists():
        parser.error("output must not exist; retain previous previews separately")
    output.mkdir(parents=True)
    client = Client()
    for path in PAGES + tuple("/support/articles/" + a["slug"] + "/" for a in ARTICLES):
        response = client.get(path)
        if response.status_code != 200:
            raise RuntimeError(f"Preview page refused: {path}")
        destination = output / path.lstrip("/") / "index.html"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(response.content)
    shutil.copytree(WEB / "skager_web/marketing/static/marketing", output / "static/marketing")
    (output / "robots.txt").write_text("User-agent: *\nDisallow: /\n", encoding="utf-8")
    # These headers mirror the Django boundary on the static preview host.
    (output / "_headers").write_text(
        "/*\n  X-Robots-Tag: noindex, nofollow\n  X-Content-Type-Options: nosniff\n"
        "  Referrer-Policy: strict-origin-when-cross-origin\n"
        "  Content-Security-Policy: default-src 'self'; script-src 'self'; style-src 'self'; "
        "img-src 'self' data:; font-src 'self'; connect-src 'self'; object-src 'none'; "
        "base-uri 'self'; form-action 'self'; frame-ancestors 'none'\n", encoding="utf-8")
    files = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(output.rglob("*")) if p.is_file()}
    (output / "preview-provenance.json").write_text(json.dumps({
        "schema": 1, "sourceCommit": args.source_commit, "audience": "owner-private",
        "publicBetaOpen": False, "backendIncluded": False, "files": files,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"Exported {len(PAGES) + len(ARTICLES)} informational pages; public beta remains closed.")


if __name__ == "__main__":
    main()
