"""Loopback review and static export only, never production or paid service."""

from .development import *  # noqa: F403

DEBUG = False
ALLOWED_HOSTS = ["localhost", "127.0.0.1", "testserver"]
DATABASES = {"default": {"ENGINE": "django.db.backends.sqlite3", "NAME": ":memory:"}}
OIDC_AUTH_ENABLED = False
