from urllib.parse import urlsplit

from django.conf import settings
from django.core.exceptions import ImproperlyConfigured


def _value(config, name: str):
    if isinstance(config, dict):
        return config.get(name, "")
    return getattr(config, name, "")


def _https_url(config, name: str) -> str:
    value = _value(config, name)
    parsed = urlsplit(value)
    if (
        parsed.scheme != "https"
        or not parsed.hostname
        or parsed.username
        or parsed.password
        or parsed.query
        or parsed.fragment
    ):
        raise ImproperlyConfigured(
            f"{name} must be a fixed HTTPS URL without credentials, a query, or a fragment"
        )
    return value


def validate_oidc_configuration(config=None) -> None:
    config = config or settings
    if not _value(config, "OIDC_AUTH_ENABLED"):
        return

    issuer = _https_url(config, "OIDC_ISSUER")
    allowed = tuple(_value(config, "OIDC_ALLOWED_ISSUERS"))
    if not allowed or issuer not in allowed:
        raise ImproperlyConfigured("OIDC_ISSUER must exactly match the configured issuer allow-list")
    if any(_https_origin(item) != _https_origin(issuer) for item in allowed):
        raise ImproperlyConfigured("All allowed OIDC issuers must share the configured Auth0 origin")

    issuer_origin = _https_origin(issuer)
    for name in ("OIDC_AUTHORIZATION_ENDPOINT", "OIDC_TOKEN_ENDPOINT", "OIDC_JWKS_URI"):
        if _https_origin(_https_url(config, name)) != issuer_origin:
            raise ImproperlyConfigured(f"{name} must use the configured issuer origin")

    _https_url(config, "OIDC_CALLBACK_URL")
    if not _value(config, "OIDC_CLIENT_ID") or not _value(config, "OIDC_CLIENT_SECRET"):
        raise ImproperlyConfigured("OIDC client credentials are required when authentication is enabled")
    success_url = _value(config, "OIDC_SUCCESS_URL")
    logout_url = _value(config, "OIDC_LOGOUT_URL")
    if not _is_fixed_local_path(success_url):
        raise ImproperlyConfigured("OIDC_SUCCESS_URL must be a fixed local path")
    if not _is_fixed_local_path(logout_url):
        raise ImproperlyConfigured("OIDC_LOGOUT_URL must be a fixed local path")


def _https_origin(value: str) -> tuple[str, str, int]:
    parsed = urlsplit(value)
    if parsed.scheme != "https" or not parsed.hostname:
        raise ImproperlyConfigured("OIDC issuer allow-list entries must be HTTPS URLs")
    return parsed.scheme, parsed.hostname, parsed.port or 443


def _is_fixed_local_path(value: str) -> bool:
    return (
        isinstance(value, str)
        and value.startswith("/")
        and not value.startswith("//")
        and "\\" not in value
        and not any(ord(character) < 32 or ord(character) == 127 for character in value)
    )
