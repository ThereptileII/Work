"""Version owned static URLs so updates cannot retain an earlier visual bundle."""
from functools import lru_cache
from hashlib import sha256
from pathlib import Path


@lru_cache(maxsize=1)
def _versions():
    static = Path(__file__).parent / "static" / "marketing"
    names = {"css": "site.css", "js": "site.js", "home_css": "home.css", "home_js": "home.js"}
    return {f"marketing_{kind}_version": sha256((static / name).read_bytes()).hexdigest()[:16]
            for kind, name in names.items()}


def marketing_asset_versions(request):
    # Application inputs are immutable while deployed; a new version restarts
    # Django. Static exports use these same templates and current owned assets.
    return _versions()
