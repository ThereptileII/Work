import os

from django.core.exceptions import ImproperlyConfigured

from .base import *  # noqa: F403


def required(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise ImproperlyConfigured(f"Required PostgreSQL test setting {name} is missing")
    return value


SECRET_KEY = "isolated-postgresql-integrity-tests-only"
DEBUG = False
ALLOWED_HOSTS = []
database = {
    "ENGINE": "django.db.backends.postgresql",
    "NAME": required("TEST_POSTGRES_DB"),
    "USER": required("TEST_POSTGRES_USER"),
    "HOST": os.environ.get("TEST_POSTGRES_SOCKET", "").strip()
    or os.environ.get("TEST_POSTGRES_HOST", "").strip(),
    "PORT": required("TEST_POSTGRES_PORT"),
}
password = os.environ.get("TEST_POSTGRES_PASSWORD", "")
if password:
    database["PASSWORD"] = password
if not database["HOST"]:
    raise ImproperlyConfigured(
        "Set TEST_POSTGRES_SOCKET for a local Unix socket or "
        "TEST_POSTGRES_HOST for a CI/service host"
    )
DATABASES = {"default": database}
