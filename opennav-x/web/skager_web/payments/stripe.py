"""Stripe TESTONLY boundary. Verified events are evidence, never authorization."""

import hashlib
import hmac
import json
import re
import time
from dataclasses import dataclass, field
from urllib.parse import urlsplit
from uuid import UUID

import requests

API_VERSION = "2026-09-30.endive"
API_URL = "https://api.stripe.com/v1/checkout/sessions"
MAX_RESPONSE_BYTES = 64 * 1024
MAX_WEBHOOK_BYTES = 256 * 1024
MAX_SIGNATURE_HEADER = 2048
MAX_SIGNATURES = 8
SIGNATURE_TOLERANCE_SECONDS = 300
HTTP_TIMEOUT = (3.05, 10)
EVENT_TYPES = frozenset({
    "checkout.session.completed", "checkout.session.expired",
    "checkout.session.async_payment_succeeded", "checkout.session.async_payment_failed",
})


class StripeBoundaryError(Exception):
    """Sanitized failure; provider bodies and credentials must never be logged."""


def _matches(pattern, value):
    return isinstance(value, str) and re.fullmatch(pattern, value, flags=re.ASCII) is not None


def _order_id(value):
    if not isinstance(value, UUID) or value.int == 0:
        raise StripeBoundaryError("A nonzero immutable order UUID is required")
    return str(value)


@dataclass(frozen=True)
class TestStripeConfig:
    # Provisioned server-side only, after confirming one-time USD 20 in sandbox.
    secret_key: str = field(repr=False)
    price_id: str
    return_origin: str = ""
    allow_promotion_codes: bool = False

    def __post_init__(self):
        if not _matches(r"sk_test_[A-Za-z0-9]{16,256}", self.secret_key):
            raise StripeBoundaryError("A Stripe test secret key is required")
        if not _matches(r"price_[A-Za-z0-9]{8,128}", self.price_id):
            raise StripeBoundaryError("An approved test price ID is required")
        # A reviewed server configuration value, never a browser/request parameter.
        # Require a bare lowercase HTTPS DNS origin; exclude IPs and every URL component.
        if not _matches(
            r"https://(?=.{1,253}\Z)(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?\.)+"
            r"[a-z](?:[a-z0-9-]{0,61}[a-z0-9])?", self.return_origin
        ):
            raise StripeBoundaryError("A reviewed HTTPS server return origin is required")
        if type(self.allow_promotion_codes) is not bool:
            raise StripeBoundaryError("Promotion configuration is invalid")


@dataclass(frozen=True)
class HostedTestCheckout:
    session_id: str
    url: str
    order_id: str


@dataclass(frozen=True)
class VerifiedTestEvent:
    # Values identify signed evidence; no claim of account ownership or payment effect.
    event_id: str
    event_type: str
    session_id: str
    order_id: str
    created: int
    signed_at: int
    payload_digest: str
    amount_total: int
    currency: str
    payment_status: str


def _strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("Duplicate JSON key")
            result[key] = value
        return result

    def invalid_constant(value):
        raise ValueError("Non-finite JSON number")

    value = json.loads(raw.decode("utf-8"), object_pairs_hook=pairs,
                       parse_constant=invalid_constant)
    if not isinstance(value, dict):
        raise ValueError("Expected object")
    return value


def _session_identity(payload, expected_order=None):
    if (payload.get("object") != "checkout.session"
            or payload.get("livemode") is not False
            or payload.get("mode") != "payment"
            or not _matches(r"cs_test_[A-Za-z0-9]{8,200}", payload.get("id"))):
        raise ValueError("Invalid test session")
    reference = payload.get("client_reference_id")
    if not isinstance(reference, str) or str(UUID(reference)) != reference or UUID(reference).int == 0:
        raise ValueError("Invalid order reference")
    metadata = payload.get("metadata")
    if not isinstance(metadata, dict) or metadata.get("order_id") != reference:
        raise ValueError("Order references differ")
    if expected_order is not None and reference != expected_order:
        raise ValueError("Order identity differs")
    if (payload.get("currency") != "usd" or type(payload.get("amount_subtotal")) is not int
            or payload["amount_subtotal"] != 2000
            or type(payload.get("amount_total")) is not int
            or not 0 <= payload["amount_total"] <= 2000
            or payload.get("payment_status") not in {"paid", "unpaid", "no_payment_required"}):
        raise ValueError("Invalid test amount")
    return reference


def _checkout_url(value, session_id):
    if not isinstance(value, str) or not 1 <= len(value) <= 4096 or any(ord(c) <= 32 for c in value):
        raise ValueError("Invalid checkout URL")
    parsed = urlsplit(value)
    if (parsed.scheme != "https" or parsed.netloc != "checkout.stripe.com"
            or parsed.username or parsed.password or parsed.port
            or parsed.path != f"/c/pay/{session_id}" or parsed.query):
        raise ValueError("Unexpected checkout destination")
    return value


def _read_response(response):
    try:
        if response.status_code != 200 or response.url != API_URL or response.history:
            raise ValueError("Unexpected response origin or status")
        if response.headers.get("Content-Encoding", "identity").lower() != "identity":
            raise ValueError("Encoded response forbidden")
        if response.headers.get("Content-Type", "").split(";", 1)[0].lower() != "application/json":
            raise ValueError("Unexpected content type")
        length = response.headers.get("Content-Length")
        if length is not None and (not re.fullmatch(r"[0-9]{1,8}", length)
                                   or int(length) > MAX_RESPONSE_BYTES):
            raise ValueError("Response too large")
        raw = response.raw.read(MAX_RESPONSE_BYTES + 1, decode_content=False)
        if not isinstance(raw, bytes) or len(raw) > MAX_RESPONSE_BYTES:
            raise ValueError("Response too large")
        if length is not None and len(raw) != int(length):
            raise ValueError("Truncated response")
        return _strict_json(raw)
    finally:
        response.close()


def create_hosted_test_checkout(config: TestStripeConfig, order_id: UUID) -> HostedTestCheckout:
    """Create one fixed-price hosted sandbox session; caller persists the binding."""
    if type(config) is not TestStripeConfig:
        raise StripeBoundaryError("Test configuration is required")
    reference = _order_id(order_id)
    try:
        # A fresh session excludes ambient proxies, .netrc credentials and retry adapters.
        with requests.Session() as transport:
            transport.trust_env = False
            response = transport.post(
                API_URL,
                auth=(config.secret_key, ""),
                headers={"Stripe-Version": API_VERSION,
                         "Idempotency-Key": f"skager-test-checkout-v1-{reference}",
                         "Accept": "application/json", "Accept-Encoding": "identity"},
                data={"mode": "payment", "ui_mode": "hosted", "currency": "usd",
                      "line_items[0][price]": config.price_id, "line_items[0][quantity]": "1",
                      "client_reference_id": reference, "metadata[order_id]": reference,
                      "success_url": config.return_origin + "/checkout/test/success/",
                      "cancel_url": config.return_origin + "/checkout/test/cancel/",
                      "allow_promotion_codes": str(config.allow_promotion_codes).lower()},
                allow_redirects=False, timeout=HTTP_TIMEOUT, stream=True,
            )
            payload = _read_response(response)
        _session_identity(payload, reference)
        if payload.get("ui_mode") != "hosted" or payload.get("status") != "open":
            raise ValueError("Expected open hosted session")
        return HostedTestCheckout(payload["id"], _checkout_url(payload.get("url"), payload["id"]), reference)
    except Exception:
        pass
    # Raise outside the handler: even __context__ must not retain provider secrets.
    raise StripeBoundaryError("Stripe test checkout failed")


def verify_test_webhook(raw: bytes, signature_header: str, signing_secret: str,
                        *, now: int | None = None) -> VerifiedTestEvent:
    """Verify bounded unmodified bytes, then normalize. No IO or replay storage."""
    try:
        if type(raw) is not bytes or not 1 <= len(raw) <= MAX_WEBHOOK_BYTES:
            raise ValueError("Invalid body")
        if (not isinstance(signature_header, str) or not signature_header.isascii()
                or not 1 <= len(signature_header) <= MAX_SIGNATURE_HEADER
                or any(ord(c) < 32 for c in signature_header)):
            raise ValueError("Invalid signature header")
        if not _matches(r"whsec_[A-Za-z0-9]{16,256}", signing_secret):
            raise ValueError("Invalid signing secret")
        timestamps, signatures = [], []
        parts = signature_header.split(",")
        if len(parts) > 32:
            raise ValueError("Too many header fields")
        for part in parts:
            name, separator, value = part.strip().partition("=")
            if not separator:
                raise ValueError("Malformed signature header")
            if name == "t":
                if not re.fullmatch(r"[1-9][0-9]{0,11}", value):
                    raise ValueError("Invalid timestamp")
                timestamps.append(value)
            elif name == "v1":
                if not re.fullmatch(r"[0-9a-f]{64}", value):
                    raise ValueError("Invalid signature")
                signatures.append(value)
        if len(timestamps) != 1 or not 1 <= len(signatures) <= MAX_SIGNATURES:
            raise ValueError("Ambiguous signature header")
        received_at = int(time.time()) if now is None else now
        if type(received_at) is not int or abs(received_at - int(timestamps[0])) > SIGNATURE_TOLERANCE_SECONDS:
            raise ValueError("Stale or future signature")
        expected = hmac.new(signing_secret.encode("ascii"),
                            timestamps[0].encode("ascii") + b"." + raw, hashlib.sha256).hexdigest()
        matched = False
        for candidate in signatures:
            matched |= hmac.compare_digest(expected, candidate)
        if not matched:
            raise ValueError("Signature mismatch")
        # JSON parsing is deliberately after authentication and freshness checks.
        payload = _strict_json(raw)
        if (payload.get("object") != "event" or payload.get("livemode") is not False
                or payload.get("api_version") != API_VERSION
                or "account" in payload or "context" in payload
                or not _matches(r"evt_[A-Za-z0-9]{8,200}", payload.get("id"))
                or payload.get("type") not in EVENT_TYPES
                or type(payload.get("created")) is not int or not 0 < payload["created"] < 10**12):
            raise ValueError("Unsupported test event")
        data = payload.get("data")
        session = data.get("object") if isinstance(data, dict) else None
        if not isinstance(session, dict):
            raise ValueError("Missing session")
        reference = _session_identity(session)
        return VerifiedTestEvent(payload["id"], payload["type"], session["id"], reference,
                                 payload["created"], int(timestamps[0]), hashlib.sha256(raw).hexdigest(),
                                 session["amount_total"], session["currency"], session["payment_status"])
    except Exception:
        pass
    raise StripeBoundaryError("Stripe test webhook rejected")
