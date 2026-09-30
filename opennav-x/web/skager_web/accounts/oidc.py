import base64
import hashlib
import json
import secrets
from dataclasses import dataclass
from datetime import timedelta
from urllib.parse import urlencode

import requests
from authlib.oidc.core import CodeIDToken
from django.conf import settings
from django.db import transaction
from django.utils import timezone
from joserfc import jwt
from joserfc.jwk import KeySet

from .config import validate_oidc_configuration
from .models import OidcAuthorizationAttempt


class OidcAuthenticationError(Exception):
    pass


@dataclass(frozen=True)
class PendingAuthorization:
    state: str
    nonce: str
    code_verifier: str


def begin_authorization(session_key: str) -> tuple[PendingAuthorization, str]:
    validate_oidc_configuration()
    pending = PendingAuthorization(
        state=secrets.token_urlsafe(32),
        nonce=secrets.token_urlsafe(32),
        code_verifier=secrets.token_urlsafe(64),
    )
    challenge = _base64url(hashlib.sha256(pending.code_verifier.encode("ascii")).digest())
    OidcAuthorizationAttempt.objects.create(
        state_digest=_digest(pending.state),
        session_key_digest=_digest(session_key),
        nonce=pending.nonce,
        code_verifier=pending.code_verifier,
        expires_at=timezone.now() + timedelta(seconds=settings.OIDC_ATTEMPT_AGE_SECONDS),
    )
    query = urlencode(
        {
            "response_type": "code",
            "client_id": settings.OIDC_CLIENT_ID,
            "redirect_uri": settings.OIDC_CALLBACK_URL,
            "scope": "openid profile email",
            "state": pending.state,
            "nonce": pending.nonce,
            "code_challenge": challenge,
            "code_challenge_method": "S256",
        }
    )
    return pending, f"{settings.OIDC_AUTHORIZATION_ENDPOINT}?{query}"


def consume_authorization(session_key: str, state: str) -> PendingAuthorization:
    now = timezone.now()
    try:
        with transaction.atomic():
            attempt = OidcAuthorizationAttempt.objects.select_for_update().get(
                state_digest=_digest(state),
                session_key_digest=_digest(session_key),
                consumed_at__isnull=True,
                expires_at__gt=now,
            )
            pending = PendingAuthorization(
                state=state,
                nonce=attempt.nonce,
                code_verifier=attempt.code_verifier,
            )
            attempt.consumed_at = now
            attempt.nonce = ""
            attempt.code_verifier = ""
            attempt.save(update_fields=("consumed_at", "nonce", "code_verifier"))
    except OidcAuthorizationAttempt.DoesNotExist as exc:
        raise OidcAuthenticationError("OIDC authorization attempt is invalid") from exc
    return pending


def exchange_and_validate(code: str, pending: PendingAuthorization) -> dict:
    validate_oidc_configuration()
    try:
        token_response = requests.post(
            settings.OIDC_TOKEN_ENDPOINT,
            data={
                "grant_type": "authorization_code",
                "client_id": settings.OIDC_CLIENT_ID,
                "client_secret": settings.OIDC_CLIENT_SECRET,
                "code": code,
                "redirect_uri": settings.OIDC_CALLBACK_URL,
                "code_verifier": pending.code_verifier,
            },
            timeout=settings.OIDC_HTTP_TIMEOUT_SECONDS,
            allow_redirects=False,
            stream=True,
        )
        token_payload = _read_json(token_response, settings.OIDC_TOKEN_RESPONSE_MAX_BYTES)
        id_token = token_payload["id_token"]
        if not isinstance(id_token, str) or len(id_token.encode("ascii")) > settings.OIDC_ID_TOKEN_MAX_BYTES:
            raise ValueError("ID token is invalid or too large")

        jwks_response = requests.get(
            settings.OIDC_JWKS_URI,
            timeout=settings.OIDC_HTTP_TIMEOUT_SECONDS,
            allow_redirects=False,
            stream=True,
        )
        jwks = _read_json(jwks_response, settings.OIDC_JWKS_RESPONSE_MAX_BYTES)
        keys = jwks.get("keys")
        if not isinstance(keys, list) or not keys or len(keys) > settings.OIDC_JWKS_MAX_KEYS:
            raise ValueError("JWKS key count is invalid")

        token = jwt.decode(
            id_token,
            KeySet.import_key_set(jwks),
            algorithms=["RS256"],
        )
        claims = CodeIDToken(
            token.claims,
            token.header,
            options={
                "iss": {"essential": True, "value": settings.OIDC_ISSUER},
                "sub": {"essential": True},
                "aud": {"essential": True, "value": settings.OIDC_CLIENT_ID},
                "exp": {"essential": True},
                "iat": {"essential": True},
                "nonce": {"essential": True, "value": pending.nonce},
            },
            params={
                "client_id": settings.OIDC_CLIENT_ID,
                "nonce": pending.nonce,
                "access_token": token_payload.get("access_token"),
            },
        )
        claims.validate(leeway=settings.OIDC_CLOCK_SKEW_SECONDS)
        return dict(claims)
    except Exception as exc:
        raise OidcAuthenticationError("OIDC response validation failed") from exc


def _base64url(value: bytes) -> str:
    return base64.urlsafe_b64encode(value).rstrip(b"=").decode("ascii")


def _digest(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _read_json(response, limit: int) -> dict:
    try:
        if 300 <= response.status_code < 400:
            raise OidcAuthenticationError("OIDC provider redirects are forbidden")
        response.raise_for_status()
        content_length = response.headers.get("Content-Length")
        if content_length:
            try:
                if int(content_length) > limit:
                    raise OidcAuthenticationError("OIDC provider response is too large")
            except ValueError as exc:
                raise OidcAuthenticationError("OIDC provider Content-Length is invalid") from exc
        body = response.raw.read(limit + 1, decode_content=True)
        if len(body) > limit:
            raise OidcAuthenticationError("OIDC provider response is too large")
        payload = json.loads(body)
        if not isinstance(payload, dict):
            raise OidcAuthenticationError("OIDC provider response must be a JSON object")
        return payload
    finally:
        response.close()
