"""Server-only Supabase identity verification; not a login/session integration.

The configured project's Auth server validates the bearer token on each call.
No locally decoded token, email address or user-editable metadata grants identity.
"""

import json
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable
from uuid import UUID

import requests
from django.core.exceptions import ValidationError
from django.core.validators import validate_email


MAX_TOKEN_BYTES = 16 << 10
MAX_RESPONSE_BYTES = 64 << 10
HTTP_TIMEOUT_SECONDS = (3.05, 5)
_FAILURE = "Supabase identity verification failed"
_PROJECT_URL = re.compile(r"https://[a-z0-9]{20}\.supabase\.co\Z", re.ASCII)
_PUBLISHABLE_KEY = re.compile(r"sb_publishable_[A-Za-z0-9_-]{16,512}\Z", re.ASCII)
_BEARER_TOKEN = re.compile(r"[A-Za-z0-9_-]+\.[A-Za-z0-9_-]+\.[A-Za-z0-9_-]+\Z", re.ASCII)
_TIMESTAMP = re.compile(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d{1,6})?(?:Z|[+-]\d{2}:\d{2})\Z", re.ASCII)


class SupabaseConfigurationError(ValueError):
    """The explicitly supplied server configuration is unsupported."""


class SupabaseAuthenticationError(Exception):
    """Generic refusal without provider bodies, tokens or request objects."""


@dataclass(frozen=True)
class SupabaseIdentity:
    issuer: str
    subject: str
    verified_email: str | None


@dataclass(frozen=True)
class SupabaseIdentityVerifier:
    """Construct from reviewed server configuration, never request parameters.

    Only canonical hosted-project URLs and modern publishable keys are accepted.
    Legacy anon/service-role JWT keys, secret keys, custom domains and local Auth
    services are deliberately unsupported until separately reviewed.
    """

    project_url: str
    publishable_key: str = field(repr=False)
    enabled: bool = False
    session_factory: Callable = field(default=requests.Session, repr=False, compare=False)

    def __post_init__(self):
        if (
            not isinstance(self.project_url, str)
            or not _PROJECT_URL.fullmatch(self.project_url)
            or not isinstance(self.publishable_key, str)
            or not _PUBLISHABLE_KEY.fullmatch(self.publishable_key)
            or type(self.enabled) is not bool
            or not callable(self.session_factory)
        ):
            raise SupabaseConfigurationError("A fixed hosted Supabase project and publishable key are required")

    @property
    def issuer(self) -> str:
        return self.project_url + "/auth/v1"

    def verify(self, access_token: str) -> SupabaseIdentity:
        if (
            not self.enabled
            or not isinstance(access_token, str)
            or len(access_token) > MAX_TOKEN_BYTES
            or not _BEARER_TOKEN.fullmatch(access_token)
        ):
            raise SupabaseAuthenticationError(_FAILURE)
        try:
            # Fresh transport: no saved cookies, credentials, retries or proxy/netrc
            # credentials inherited from the host. TLS verification stays enabled.
            with self.session_factory() as session:
                session.trust_env = False
                response = session.get(
                    self.issuer + "/user",
                    headers={
                        "apikey": self.publishable_key,
                        "Authorization": "Bearer " + access_token,
                        "Accept": "application/json",
                        "Accept-Encoding": "identity",
                    },
                    timeout=HTTP_TIMEOUT_SECONDS,
                    allow_redirects=False,
                    stream=True,
                    verify=True,
                )
                payload = _read_user(response, self.issuer + "/user")
            return _identity(payload, self.issuer)
        except Exception:
            # Leave the handler before raising so even __context__ cannot retain
            # a transport exception's Authorization header or provider body.
            pass
        raise SupabaseAuthenticationError(_FAILURE) from None


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError("Invalid JSON constant")


def _read_user(response, expected_url: str) -> dict:
    try:
        if response.status_code != 200 or response.url != expected_url:
            raise ValueError("Unexpected Auth response")
        if response.headers.get("Content-Encoding", "identity").lower() != "identity":
            raise ValueError("Encoded response refused")
        if response.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
            raise ValueError("JSON response required")
        length = response.headers.get("Content-Length")
        if length is not None and (
            not re.fullmatch(r"[0-9]{1,10}", length, re.ASCII) or int(length) > MAX_RESPONSE_BYTES
        ):
            raise ValueError("Invalid response length")
        raw = response.raw.read(MAX_RESPONSE_BYTES + 1, decode_content=False)
        if not raw or len(raw) > MAX_RESPONSE_BYTES or (length is not None and len(raw) != int(length)):
            raise ValueError("Invalid response size")
        payload = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_unique_object, parse_constant=_invalid_constant
        )
        if not isinstance(payload, dict):
            raise ValueError("User object required")
        return payload
    finally:
        response.close()


def _timestamp(value):
    if not isinstance(value, str) or not _TIMESTAMP.fullmatch(value):
        raise ValueError("Invalid timestamp")
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def _identity(payload: dict, issuer: str) -> SupabaseIdentity:
    subject = payload.get("id")
    if not isinstance(subject, str):
        raise ValueError("User ID required")
    identifier = UUID(subject)
    if str(identifier) != subject or identifier.int == 0:
        raise ValueError("Canonical nonzero UUID required")
    if (
        payload.get("aud") != "authenticated"
        or payload.get("role") != "authenticated"
        or payload.get("is_anonymous") is not False
        or payload.get("deleted_at") is not None
    ):
        raise ValueError("Ordinary authenticated user required")
    now = datetime.now(timezone.utc)
    if payload.get("banned_until") is not None and _timestamp(payload["banned_until"]) > now:
        raise ValueError("Banned user refused")
    email = None
    if payload.get("email_confirmed_at") is not None:
        confirmed = _timestamp(payload["email_confirmed_at"])
        candidate = payload.get("email")
        if confirmed > now or not isinstance(candidate, str) or len(candidate) > 254:
            raise ValueError("Invalid confirmed email")
        # Keep provider spelling; this is contact data, never an account-join key.
        if any(ord(character) < 33 or ord(character) == 127 for character in candidate):
            raise ValueError("Invalid confirmed email")
        try:
            validate_email(candidate)
        except ValidationError:
            raise ValueError("Invalid confirmed email") from None
        email = candidate
    return SupabaseIdentity(issuer=issuer, subject=subject, verified_email=email)
