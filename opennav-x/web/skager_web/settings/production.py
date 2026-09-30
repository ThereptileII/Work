import os

from django.core.exceptions import ImproperlyConfigured

from .base import *  # noqa: F403


def required(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise ImproperlyConfigured(f"Required production setting {name} is missing")
    return value


SECRET_KEY = required("DJANGO_SECRET_KEY")
if len(SECRET_KEY) < 50 or SECRET_KEY.startswith("django-insecure-") or SECRET_KEY == "development-only-not-for-deployment":
    raise ImproperlyConfigured("DJANGO_SECRET_KEY must be a strong production-only value")

DEBUG = False
ALLOWED_HOSTS = [host.strip() for host in required("DJANGO_ALLOWED_HOSTS").split(",") if host.strip()]
if not ALLOWED_HOSTS:
    raise ImproperlyConfigured("DJANGO_ALLOWED_HOSTS must contain at least one host")
if any(host == "*" or host.startswith(".") for host in ALLOWED_HOSTS):
    raise ImproperlyConfigured("Production hosts must be explicit; wildcards are forbidden")

if required("POSTGRES_SSLMODE") != "verify-full":
    raise ImproperlyConfigured("Production PostgreSQL must use sslmode=verify-full")

DATABASES = {
    "default": {
        "ENGINE": "django.db.backends.postgresql",
        "NAME": required("POSTGRES_DB"),
        "USER": required("POSTGRES_USER"),
        "PASSWORD": required("POSTGRES_PASSWORD"),
        "HOST": required("POSTGRES_HOST"),
        "PORT": required("POSTGRES_PORT"),
        "CONN_MAX_AGE": 60,
        "CONN_HEALTH_CHECKS": True,
        "OPTIONS": {
            "sslmode": "verify-full",
            "sslrootcert": required("POSTGRES_SSLROOTCERT"),
        },
    }
}

oidc_enabled_value = os.environ.get("OIDC_AUTH_ENABLED", "0")
if oidc_enabled_value not in ("0", "1"):
    raise ImproperlyConfigured("OIDC_AUTH_ENABLED must be exactly 0 or 1")
OIDC_AUTH_ENABLED = oidc_enabled_value == "1"
if OIDC_AUTH_ENABLED:
    OIDC_ISSUER = required("OIDC_ISSUER")
    OIDC_ALLOWED_ISSUERS = tuple(
        issuer.strip() for issuer in required("OIDC_ALLOWED_ISSUERS").split(",") if issuer.strip()
    )
    OIDC_CLIENT_ID = required("OIDC_CLIENT_ID")
    OIDC_CLIENT_SECRET = required("OIDC_CLIENT_SECRET")
    OIDC_AUTHORIZATION_ENDPOINT = required("OIDC_AUTHORIZATION_ENDPOINT")
    OIDC_TOKEN_ENDPOINT = required("OIDC_TOKEN_ENDPOINT")
    OIDC_JWKS_URI = required("OIDC_JWKS_URI")
    OIDC_CALLBACK_URL = required("OIDC_CALLBACK_URL")

    from skager_web.accounts.config import validate_oidc_configuration

    validate_oidc_configuration(globals())

SESSION_COOKIE_SECURE = True
SESSION_COOKIE_HTTPONLY = True
SESSION_COOKIE_SAMESITE = "Lax"
CSRF_COOKIE_SECURE = True
SECURE_SSL_REDIRECT = True
SECURE_HSTS_SECONDS = 31_536_000
SECURE_HSTS_INCLUDE_SUBDOMAINS = True
SECURE_HSTS_PRELOAD = True
SECURE_CONTENT_TYPE_NOSNIFF = True
