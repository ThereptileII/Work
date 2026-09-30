from .base import *  # noqa: F403


SECRET_KEY = "development-only-not-for-deployment"
DEBUG = True
ALLOWED_HOSTS = ["localhost", "127.0.0.1"]

# SQLite exists only so schema/model checks can run on a developer machine.
# Integrity acceptance must use PostgreSQL; see core/tests/test_integrity.py.
DATABASES = {
    "default": {
        "ENGINE": "django.db.backends.sqlite3",
        "NAME": BASE_DIR / ".development.sqlite3",  # noqa: F405
    }
}
