import hashlib
import hmac
import json
import traceback
import unittest
from dataclasses import FrozenInstanceError, asdict
from unittest.mock import MagicMock, patch
from uuid import UUID

import requests

from skager_web.payments import stripe

ORDER = UUID("d38c40ca-0588-4e96-a4cd-e116b9a1c6e2")
OTHER = UUID("436cb1e2-9540-449a-9450-92d7e0865fc5")
KEY = "sk_test_syntheticKeyNeverARealCredential1234"
SECRET = "whsec_syntheticSigningSecretNeverReal1234"
ORIGIN = "https://reviewed-backend.example.test"
NOW = 1791880000


def session_payload():
    return {"id": "cs_test_syntheticSession123", "object": "checkout.session", "livemode": False,
            "mode": "payment", "ui_mode": "hosted", "status": "open",
            "client_reference_id": str(ORDER), "metadata": {"order_id": str(ORDER)},
            "currency": "usd", "amount_subtotal": 2000, "amount_total": 2000,
            "payment_status": "unpaid",
            "url": "https://checkout.stripe.com/c/pay/cs_test_syntheticSession123#hosted"}


def event_payload():
    return {"id": "evt_syntheticEvent123", "object": "event", "livemode": False,
            "api_version": stripe.API_VERSION, "type": "checkout.session.completed",
            "created": NOW - 20, "data": {"object": session_payload()}}


def body(payload):
    return json.dumps(payload, separators=(",", ":")).encode()


def signature(raw, timestamp=NOW, secret=SECRET):
    digest = hmac.new(secret.encode(), str(timestamp).encode() + b"." + raw, hashlib.sha256).hexdigest()
    return f"t={timestamp},v1={digest}"


class FakeResponse:
    def __init__(self, payload=None, *, raw=None, status=200, url=stripe.API_URL, headers=None):
        self.status_code, self.url, self.history = status, url, []
        self.headers = {"Content-Type": "application/json"} if headers is None else headers
        encoded = body(session_payload() if payload is None else payload) if raw is None else raw
        self.raw = MagicMock()
        self.raw.read.side_effect = lambda count, **kwargs: encoded[:count]
        self.closed = False

    def close(self):
        self.closed = True


class CheckoutTests(unittest.TestCase):
    def setUp(self):
        self.config = stripe.TestStripeConfig(KEY, "price_syntheticApprovedUSD20", ORIGIN)
        self.transport = MagicMock()
        self.transport.__enter__.return_value = self.transport
        patcher = patch.object(stripe.requests, "Session", return_value=self.transport)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.response = FakeResponse()
        self.transport.post.return_value = self.response

    def checkout(self, config=None, order=ORDER):
        return stripe.create_hosted_test_checkout(config or self.config, order)

    def test_fixed_host_price_identity_urls_version_and_transport(self):
        result = self.checkout()
        self.assertEqual(result.order_id, str(ORDER))
        args, kwargs = self.transport.post.call_args
        self.assertEqual(args, (stripe.API_URL,))
        self.assertEqual(kwargs["auth"], (KEY, ""))
        self.assertFalse(kwargs["allow_redirects"])
        self.assertFalse(self.transport.trust_env)
        self.assertTrue(kwargs["stream"])
        self.assertEqual(kwargs["timeout"], (3.05, 10))
        self.assertEqual(kwargs["headers"]["Stripe-Version"], stripe.API_VERSION)
        self.assertEqual(kwargs["data"], {
            "mode": "payment", "ui_mode": "hosted", "currency": "usd",
            "line_items[0][price]": self.config.price_id, "line_items[0][quantity]": "1",
            "client_reference_id": str(ORDER), "metadata[order_id]": str(ORDER),
            "success_url": ORIGIN + "/checkout/test/success/",
            "cancel_url": ORIGIN + "/checkout/test/cancel/",
            "allow_promotion_codes": "false"})
        self.assertTrue(self.response.closed)
        self.response.raw.read.assert_called_once_with(stripe.MAX_RESPONSE_BYTES + 1, decode_content=False)

    def test_retry_reuses_order_idempotency_and_other_order_changes_it(self):
        self.checkout()
        first = self.transport.post.call_args.kwargs["headers"]["Idempotency-Key"]
        self.checkout()
        self.assertEqual(self.transport.post.call_args.kwargs["headers"]["Idempotency-Key"], first)
        payload = session_payload()
        payload.update(client_reference_id=str(OTHER), metadata={"order_id": str(OTHER)})
        self.transport.post.return_value = FakeResponse(payload)
        self.checkout(order=OTHER)
        self.assertNotEqual(self.transport.post.call_args.kwargs["headers"]["Idempotency-Key"], first)

    def test_promotions_are_server_configuration(self):
        self.checkout(stripe.TestStripeConfig(KEY, self.config.price_id, ORIGIN, True))
        self.assertEqual(self.transport.post.call_args.kwargs["data"]["allow_promotion_codes"], "true")

    def test_configuration_rejects_live_keys_and_hides_secrets(self):
        # Construct deliberately fake rejected prefixes; a literal live-shaped
        # string is needlessly flagged by repository secret scanning.
        for key in ["sk_" + "live_" + "syntheticCredential123456",
                    "rk_" + "test_" + "syntheticCredential123456", "", KEY + "\n"]:
            with self.subTest(key_type=key[:7]), self.assertRaises(stripe.StripeBoundaryError):
                stripe.TestStripeConfig(key, self.config.price_id, ORIGIN)
        for price in ["", "price_bad\n", "https://attacker.test", 123]:
            with self.subTest(price=price), self.assertRaises(stripe.StripeBoundaryError):
                stripe.TestStripeConfig(KEY, price, ORIGIN)
        with self.assertRaises(stripe.StripeBoundaryError):
            stripe.TestStripeConfig(KEY, self.config.price_id, ORIGIN, 1)
        self.assertNotIn(KEY, repr(self.config))
        with self.assertRaises(FrozenInstanceError):
            self.config.price_id = "price_different"

    def test_return_origin_required_and_rejects_url_injection_before_network(self):
        with self.assertRaises(stripe.StripeBoundaryError):
            stripe.TestStripeConfig(KEY, self.config.price_id)
        for origin in ["", None, "http://backend.example.test", "https://localhost",
                       "https://127.0.0.1", "https://[::1]", "https://backend.example.test/",
                       "https://backend.example.test/path", "https://backend.example.test:443",
                       "https://user@backend.example.test", "https://backend.example.test?x=y",
                       "https://backend.example.test#fragment", "https://backend.example.test\\@evil.test",
                       "https://backend.example.test\n", "https://-bad.example.test",
                       "https://" + "a" * 64 + ".example.test"]:
            with self.subTest(origin=origin), self.assertRaises(stripe.StripeBoundaryError):
                stripe.TestStripeConfig(KEY, self.config.price_id, origin)
        self.transport.post.assert_not_called()

    def test_immutable_uuid_required_before_network(self):
        for order in [str(ORDER), "email@example.test", None, UUID(int=0)]:
            with self.subTest(order=order), self.assertRaises(stripe.StripeBoundaryError):
                self.checkout(order=order)
        self.transport.post.assert_not_called()

    def test_response_session_identity_and_amount_fail_closed(self):
        for change in [{"livemode": True}, {"livemode": 0}, {"id": "cs_live_synthetic123"},
                       {"client_reference_id": str(OTHER)}, {"metadata": {"order_id": str(OTHER)}},
                       {"currency": "eur"}, {"amount_subtotal": 1999}, {"amount_total": True},
                       {"amount_total": -1}, {"amount_total": 2001}, {"mode": "subscription"},
                       {"ui_mode": "embedded"}, {"status": "complete"}, {"object": "payment_intent"},
                       {"payment_status": "unknown"}, {"client_reference_id": ""}]:
            payload = session_payload()
            payload.update(change)
            self.transport.post.return_value = FakeResponse(payload)
            with self.subTest(change=change), self.assertRaises(stripe.StripeBoundaryError):
                self.checkout()
            self.assertTrue(self.transport.post.return_value.closed)

    def test_destination_rejects_host_confusion_and_arbitrary_paths(self):
        for url in ["http://checkout.stripe.com/c/pay/cs_test_syntheticSession123",
                    "https://checkout.stripe.com.attacker.test/c/pay/cs_test_syntheticSession123",
                    "https://checkout.stripe.com@attacker.test/c/pay/cs_test_syntheticSession123",
                    "https://checkout.stripe.com:443/c/pay/cs_test_syntheticSession123",
                    "https://checkout.stripe.com/c/pay/cs_test_someOtherSession",
                    "https://checkout.stripe.com/c/pay/cs_test_syntheticSession123?next=evil",
                    "https://checkout.stripe.com\\@evil.test/c/pay/cs_test_syntheticSession123",
                    "https://checkout.stripe.com/c/pay/cs_test_syntheticSession123\n", "x" * 4097]:
            payload = session_payload()
            payload["url"] = url
            self.transport.post.return_value = FakeResponse(payload)
            with self.subTest(url=url[:100]), self.assertRaises(stripe.StripeBoundaryError):
                self.checkout()

    def test_response_status_origin_redirect_content_bounds_and_encoding(self):
        responses = [FakeResponse(status=code) for code in [201, 302, 307, 400, 429, 500]]
        responses += [FakeResponse(url="https://attacker.test"),
                      FakeResponse(raw=b"x" * (stripe.MAX_RESPONSE_BYTES + 1)),
                      FakeResponse(raw=b"{}", headers={"Content-Type": "application/json", "Content-Length": "100"}),
                      FakeResponse(headers={"Content-Type": "application/json", "Content-Length": "-1"}),
                      FakeResponse(headers={"Content-Type": "application/json", "Content-Length": "65537"}),
                      FakeResponse(headers={"Content-Type": "application/json", "Content-Encoding": "gzip"}),
                      FakeResponse(headers={"Content-Type": "text/html"}),
                      FakeResponse(raw=b'{"id":"one","id":"two"}'),
                      FakeResponse(raw=b'{"x":NaN}'), FakeResponse(raw=b'\xff'), FakeResponse(raw=b'[]')]
        redirected = FakeResponse()
        redirected.history = [object()]
        responses.append(redirected)
        for response in responses:
            self.transport.post.return_value = response
            with self.subTest(status=response.status_code, url=response.url), self.assertRaises(stripe.StripeBoundaryError):
                self.checkout()
            self.assertTrue(response.closed)

    def test_timeouts_errors_are_sanitized_without_retries(self):
        for error in [requests.Timeout(KEY), requests.ConnectionError(SECRET), RuntimeError(KEY)]:
            self.transport.post.reset_mock()
            self.transport.post.side_effect = error
            try:
                self.checkout()
            except stripe.StripeBoundaryError as exc:
                trace = "".join(traceback.format_exception(exc))
                self.assertNotIn(KEY, trace)
                self.assertNotIn(SECRET, trace)
                self.assertIsNone(exc.__context__)
                self.assertIsNone(exc.__cause__)
            else:
                self.fail("Expected sanitized failure")
            self.transport.post.assert_called_once()


class WebhookTests(unittest.TestCase):
    def verify(self, payload=None, raw=None, header=None, now=NOW):
        raw = body(payload or event_payload()) if raw is None else raw
        return stripe.verify_test_webhook(raw, signature(raw) if header is None else header, SECRET, now=now)

    def test_minimal_immutable_envelope_excludes_unowned_fields(self):
        payload = event_payload()
        payload["data"]["object"]["customer_details"] = {"email": "private@example.test"}
        payload["data"]["object"]["metadata"]["account_id"] = "attacker-supplied"
        raw = body(payload)
        event = self.verify(raw=raw)
        self.assertEqual(event.event_id, payload["id"])
        self.assertEqual(event.order_id, str(ORDER))
        self.assertEqual(event.payload_digest, hashlib.sha256(raw).hexdigest())
        self.assertNotIn("private@example.test", repr(event))
        self.assertNotIn("account_id", asdict(event))
        with self.assertRaises(FrozenInstanceError):
            event.event_id = "evt_replaced"

    def test_raw_body_authenticated_before_json(self):
        raw = body(event_payload())
        with patch.object(stripe, "_strict_json") as parse:
            for modified in [raw + b" ", raw.replace(b"completed", b"expired")]:
                with self.assertRaises(stripe.StripeBoundaryError):
                    self.verify(raw=modified, header=signature(raw))
            parse.assert_not_called()

    def test_multiple_v1_signatures_supported_v0_cannot_authenticate(self):
        raw = body(event_payload())
        good = signature(raw).split("v1=")[1]
        self.verify(raw=raw, header=f"t={NOW},v0={'f' * 64},v1={'0' * 64},v1={good}")
        with self.assertRaises(stripe.StripeBoundaryError):
            self.verify(raw=raw, header=f"t={NOW},v0={good}")

    def test_constant_time_comparison_of_every_v1(self):
        raw = body(event_payload())
        with patch.object(stripe.hmac, "compare_digest", wraps=hmac.compare_digest) as compare:
            self.verify(raw=raw, header=signature(raw) + ",v1=" + "0" * 64)
            self.assertEqual(compare.call_count, 2)

    def test_stale_future_and_duplicate_timestamps(self):
        raw = body(event_payload())
        for timestamp in [NOW - 301, NOW + 301]:
            with self.subTest(timestamp=timestamp), self.assertRaises(stripe.StripeBoundaryError):
                self.verify(raw=raw, header=signature(raw, timestamp))
        for timestamp in [NOW - 300, NOW + 300]:
            self.verify(raw=raw, header=signature(raw, timestamp))
        with self.assertRaises(stripe.StripeBoundaryError):
            self.verify(raw=raw, header=signature(raw) + f",t={NOW}")

    def test_retries_retain_identity_and_digest_for_durable_deduplication(self):
        raw = body(event_payload())
        first = self.verify(raw=raw)
        retry = self.verify(raw=raw, header=signature(raw, NOW + 60), now=NOW + 60)
        self.assertEqual((first.event_id, first.payload_digest), (retry.event_id, retry.payload_digest))
        self.assertNotEqual(first.signed_at, retry.signed_at)

    def test_header_and_body_bounds(self):
        raw = body(event_payload())
        for header in ["", "x" * 2049, signature(raw) + "\n", f"t={NOW},v1=short",
                       f"t={NOW},v1={'g' * 64}", f"t=-1,v1={'0' * 64}",
                       f"t={NOW},v1={'0' * 64},oops", f"t={NOW},v1={'0' * 64},v1=é",
                       f"t={NOW}," + ",".join(["v1=" + "0" * 64] * 9)]:
            with self.subTest(header=header[:60]), self.assertRaises(stripe.StripeBoundaryError):
                self.verify(raw=raw, header=header)
        for invalid in [b"", b"x" * (stripe.MAX_WEBHOOK_BYTES + 1), bytearray(raw), raw.decode()]:
            with self.assertRaises(stripe.StripeBoundaryError):
                self.verify(raw=invalid, header=signature(raw))

    def test_signed_json_must_be_strict_object(self):
        for raw in [b"[]", b"null", b'\xff', b'{"id":1,"id":2}', b'{"x":NaN}',
                    b"{" * 2000, b'{} trailing']:
            with self.subTest(raw=raw[:30]), self.assertRaises(stripe.StripeBoundaryError):
                self.verify(raw=raw)

    def test_signed_event_environment_identity_version_schema(self):
        for change in [{"livemode": True}, {"livemode": 0}, {"object": "not.event"},
                       {"id": "evt_bad\n"}, {"id": ""}, {"type": "customer.created"}, {"type": []},
                       {"api_version": "2025-06-30.basil"}, {"account": "acct_other"},
                       {"context": "org_other"}, {"created": True}, {"created": -1}, {"data": []}]:
            payload = event_payload()
            payload.update(change)
            with self.subTest(change=change), self.assertRaises(stripe.StripeBoundaryError):
                self.verify(payload)
        for change in [{"livemode": True}, {"id": "cs_live_synthetic123"}, {"currency": "eur"},
                       {"metadata": {"order_id": str(OTHER)}}, {"amount_subtotal": True},
                       {"amount_total": 2001}, {"amount_total": False},
                       {"client_reference_id": str(ORDER).upper()}, {"mode": "setup"}]:
            payload = event_payload()
            payload["data"]["object"].update(change)
            with self.subTest(change=change), self.assertRaises(stripe.StripeBoundaryError):
                self.verify(payload)

    def test_webhook_failure_retains_no_secret_exception_context(self):
        raw = body(event_payload())
        with patch.object(stripe, "_strict_json", side_effect=RuntimeError(SECRET)):
            try:
                self.verify(raw=raw)
            except stripe.StripeBoundaryError as exc:
                self.assertIsNone(exc.__context__)
                self.assertIsNone(exc.__cause__)
                self.assertNotIn(SECRET, "".join(traceback.format_exception(exc)))
            else:
                self.fail("Expected sanitized failure")

    def test_discount_total_is_evidence_without_network_or_grants(self):
        payload = event_payload()
        payload["data"]["object"].update(amount_total=0, payment_status="no_payment_required")
        with patch.object(stripe.requests, "Session", side_effect=AssertionError("No IO")):
            event = self.verify(payload)
        self.assertEqual(event.amount_total, 0)
        self.assertFalse(hasattr(event, "entitlement"))


if __name__ == "__main__":
    unittest.main()
