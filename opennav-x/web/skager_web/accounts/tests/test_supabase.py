import io
import json
from dataclasses import FrozenInstanceError
from unittest.mock import Mock, patch

import requests
from django.test import SimpleTestCase

from skager_web.accounts.supabase import (
    HTTP_TIMEOUT_SECONDS,
    MAX_RESPONSE_BYTES,
    MAX_TOKEN_BYTES,
    SupabaseAuthenticationError,
    SupabaseConfigurationError,
    SupabaseIdentity,
    SupabaseIdentityVerifier,
)


PROJECT_URL = "https://kelwgubkuvditalsbqpr.supabase.co"
PUBLISHABLE_KEY = "sb_publishable_" + "test_only_" * 4
# Deliberately not decodable claims: only the Auth server grants identity.
ACCESS_TOKEN = "header.payload.signature"
SUBJECT = "7aeb1798-8b0c-4b5a-a5a6-f6bf819bb7a2"
USER = {
    "id": SUBJECT,
    "aud": "authenticated",
    "role": "authenticated",
    "is_anonymous": False,
    "email": "Owner@example.test",
    "email_confirmed_at": "2025-01-02T03:04:05.123456Z",
}


class FakeRaw:
    def __init__(self, body):
        self.stream = io.BytesIO(body)
        self.reads = []

    def read(self, amount, decode_content=False):
        self.reads.append((amount, decode_content))
        return self.stream.read(amount)


class FakeResponse:
    def __init__(self, payload=USER, *, body=None, status=200, headers=None, url=None):
        self.status_code = status
        self.url = url or PROJECT_URL + "/auth/v1/user"
        self.headers = {"Content-Type": "application/json", **(headers or {})}
        self.raw = FakeRaw(json.dumps(payload).encode("utf-8") if body is None else body)
        self.closed = False

    def close(self):
        self.closed = True


class SupabaseVerificationTests(SimpleTestCase):
    def setUp(self):
        # Every network entry point fails unless a test supplies its own transport.
        # The adapter suite must never reach the real configured project.
        self.network_guard = patch.object(
            requests.sessions.Session, "request", side_effect=AssertionError("External HTTP forbidden")
        )
        self.network_guard.start()
        self.addCleanup(self.network_guard.stop)

    def verifier(self, response=None, *, enabled=True):
        session = Mock()
        session.__enter__ = Mock(return_value=session)
        session.__exit__ = Mock(return_value=False)
        session.get.return_value = response or FakeResponse()
        factory = Mock(return_value=session)
        verifier = SupabaseIdentityVerifier(
            PROJECT_URL, PUBLISHABLE_KEY, enabled=enabled, session_factory=factory
        )
        return verifier, session, factory

    def refused(self, response):
        verifier, _, _ = self.verifier(response)
        with self.assertRaisesMessage(
            SupabaseAuthenticationError, "Supabase identity verification failed"
        ) as raised:
            verifier.verify(ACCESS_TOKEN)
        self.assertTrue(response.closed)
        self.assertIsNone(raised.exception.__cause__)
        self.assertIsNone(raised.exception.__context__)
        self.assertTrue(raised.exception.__suppress_context__)

    def test_valid_user_uses_one_fixed_tls_request_and_returns_only_owned_values(self):
        response = FakeResponse()
        verifier, session, factory = self.verifier(response)
        identity = verifier.verify(ACCESS_TOKEN)
        self.assertEqual(identity, SupabaseIdentity(PROJECT_URL + "/auth/v1", SUBJECT, USER["email"]))
        self.assertFalse(session.trust_env)
        factory.assert_called_once_with()
        session.get.assert_called_once_with(
            PROJECT_URL + "/auth/v1/user",
            headers={
                "apikey": PUBLISHABLE_KEY,
                "Authorization": "Bearer " + ACCESS_TOKEN,
                "Accept": "application/json",
                "Accept-Encoding": "identity",
            },
            timeout=HTTP_TIMEOUT_SECONDS,
            allow_redirects=False,
            stream=True,
            verify=True,
        )
        self.assertEqual(response.raw.reads, [(MAX_RESPONSE_BYTES + 1, False)])
        self.assertTrue(response.closed)
        session.__exit__.assert_called_once()
        self.assertEqual(set(vars(identity)), {"issuer", "subject", "verified_email"})
        with self.assertRaises(FrozenInstanceError):
            identity.subject = "substitution"
        with self.assertRaises(FrozenInstanceError):
            verifier.project_url = "https://attacker.example"
        self.assertNotIn(PUBLISHABLE_KEY, repr(verifier))

    def test_default_disabled_verifier_never_creates_transport(self):
        factory = Mock()
        verifier = SupabaseIdentityVerifier(PROJECT_URL, PUBLISHABLE_KEY, session_factory=factory)
        with self.assertRaises(SupabaseAuthenticationError):
            verifier.verify(ACCESS_TOKEN)
        factory.assert_not_called()

    def test_non_boolean_enablement_is_configuration_error(self):
        for value in (None, 0, 1, "false", "true"):
            with self.subTest(value=value), self.assertRaises(SupabaseConfigurationError):
                SupabaseIdentityVerifier(PROJECT_URL, PUBLISHABLE_KEY, enabled=value)

    def test_url_rejects_insecure_dynamic_custom_and_ambiguous_origins(self):
        for value in (
            None, "", "http://kelwgubkuvditalsbqpr.supabase.co", PROJECT_URL + "/",
            PROJECT_URL + "/auth/v1", PROJECT_URL + "?next=x", PROJECT_URL + "#x",
            PROJECT_URL + ":443", PROJECT_URL + ".attacker.example",
            "https://user:password@kelwgubkuvditalsbqpr.supabase.co",
            "https://127.0.0.1", "https://localhost", "https://auth.skager.app",
            PROJECT_URL.upper(), PROJECT_URL + "\n", " " + PROJECT_URL,
            "https://kelwgubkuvditalsbqpr%2esupabase.co",
        ):
            with self.subTest(value=value), self.assertRaises(SupabaseConfigurationError):
                SupabaseIdentityVerifier(value, PUBLISHABLE_KEY)

    def test_only_modern_publishable_key_shape_is_allowed(self):
        for value in (
            None, "", ACCESS_TOKEN, "sb_secret_" + "x" * 32,
            "sb_publishable_short", PUBLISHABLE_KEY + "\r\nInjected: true",
            PUBLISHABLE_KEY + "\n", PUBLISHABLE_KEY + " ", "sb_publishable_" + "a" * 513,
        ):
            with self.subTest(value=value), self.assertRaises(SupabaseConfigurationError):
                SupabaseIdentityVerifier(PROJECT_URL, value)
        with self.assertRaises(SupabaseConfigurationError):
            SupabaseIdentityVerifier(PROJECT_URL, PUBLISHABLE_KEY, session_factory=None)

    def test_invalid_or_oversized_token_is_refused_before_transport(self):
        for value in (
            None, b"a.b.c", 123, "", "opaque-token", "a.b", "a.b.c.d", "a..c",
            "Bearer a.b.c", " a.b.c", "a.b.c\n", "a.b.c\r\nX: injected", "å.b.c",
            "a.b.c=", "a.b." + "c" * MAX_TOKEN_BYTES,
        ):
            verifier, _, factory = self.verifier()
            with self.subTest(value_type=type(value).__name__), self.assertRaises(SupabaseAuthenticationError):
                verifier.verify(value)
            factory.assert_not_called()

    def test_remote_expiry_revocation_not_found_rate_limit_and_failure_are_refused(self):
        for status in (204, 301, 302, 303, 307, 308, 400, 401, 403, 404, 429, 500, 503):
            with self.subTest(status=status):
                self.refused(FakeResponse(status=status, payload={"message": "private provider detail"}))

    def test_success_status_with_redirected_or_changed_url_is_refused(self):
        for url in ("https://attacker.example/user", PROJECT_URL + "/auth/v1/user?x=y"):
            with self.subTest(url=url):
                self.refused(FakeResponse(url=url))

    def test_transport_and_stream_failures_are_generic_and_close_resources(self):
        for error in (
            requests.Timeout("token=secret"), requests.ConnectionError("token=secret"),
            requests.exceptions.SSLError("token=secret"), RuntimeError("private provider body"),
        ):
            verifier, session, _ = self.verifier()
            session.get.side_effect = error
            with self.subTest(error=type(error).__name__), self.assertRaises(SupabaseAuthenticationError) as raised:
                verifier.verify(ACCESS_TOKEN)
            self.assertEqual(str(raised.exception), "Supabase identity verification failed")
            self.assertIsNone(raised.exception.__cause__)
            self.assertIsNone(raised.exception.__context__)
            self.assertTrue(raised.exception.__suppress_context__)
            session.__exit__.assert_called_once()
        response = FakeResponse()
        response.raw.read = Mock(side_effect=requests.Timeout("private stream detail"))
        self.refused(response)

    def test_compressed_non_json_and_invalid_length_responses_are_refused(self):
        for headers in (
            {"Content-Encoding": "gzip"}, {"Content-Encoding": "br"},
            {"Content-Encoding": "identity,gzip"}, {"Content-Type": "text/html"},
            {"Content-Type": ""}, {"Content-Length": "-1"}, {"Content-Length": "+1"},
            {"Content-Length": " 2"}, {"Content-Length": "1.0"},
            {"Content-Length": "99999999999"}, {"Content-Length": str(MAX_RESPONSE_BYTES + 1)},
            {"Content-Length": "1"}, {"Content-Length": "0"},
        ):
            with self.subTest(headers=headers):
                self.refused(FakeResponse(headers=headers))

    def test_correct_length_and_json_charset_are_accepted(self):
        body = json.dumps(USER).encode("utf-8")
        response = FakeResponse(body=body, headers={
            "Content-Length": str(len(body)), "Content-Type": "application/json; charset=utf-8",
        })
        verifier, _, _ = self.verifier(response)
        self.assertEqual(verifier.verify(ACCESS_TOKEN).subject, SUBJECT)

    def test_oversized_body_is_refused_with_and_without_declared_length(self):
        for headers in ({}, {"Content-Length": str(MAX_RESPONSE_BYTES + 1)}):
            response = FakeResponse(body=b" " * (MAX_RESPONSE_BYTES + 1), headers=headers)
            self.refused(response)
            self.assertLessEqual(len(response.raw.reads), 1)

    def test_malformed_duplicate_nonfinite_and_non_object_json_is_refused(self):
        for body in (
            b"", b"{", b"null", b"[]", b"1", b'"user"', b"\xff",
            b'{"id":"first","id":"second"}', b'{"metadata":{"role":1,"role":2}}',
            b'{"value":NaN}', b'{"value":Infinity}', b'{"value":-Infinity}',
            b'{} trailing', b"[" * 1100 + b"]" * 1100,
        ):
            with self.subTest(body_prefix=body[:45]):
                self.refused(FakeResponse(body=body))

    def test_missing_malformed_noncanonical_or_nil_uuid_is_refused(self):
        for subject in (
            None, 123, "", "other-user", SUBJECT.upper(), SUBJECT.replace("-", ""),
            "{" + SUBJECT + "}", "urn:uuid:" + SUBJECT,
            "00000000-0000-0000-0000-000000000000", SUBJECT + "\n",
        ):
            with self.subTest(subject=subject):
                self.refused(FakeResponse(payload={**USER, "id": subject}))
        self.refused(FakeResponse(payload={key: value for key, value in USER.items() if key != "id"}))

    def test_non_ordinary_users_missing_security_fields_and_deletion_are_refused(self):
        for field, values in (
            ("aud", (None, "anon", "service_role", ["authenticated"])),
            ("role", (None, "anon", "service_role", "administrator")),
            ("is_anonymous", (None, True, 0, "false")),
            ("deleted_at", ("2025-01-01T00:00:00Z", "", False)),
            ("banned_until", ("2999-01-01T00:00:00Z", "invalid", "2025-01-01", "")),
        ):
            for value in values:
                with self.subTest(field=field, value=value):
                    self.refused(FakeResponse(payload={**USER, field: value}))
        for field in ("aud", "role", "is_anonymous"):
            with self.subTest(missing=field):
                self.refused(FakeResponse(payload={key: value for key, value in USER.items() if key != field}))

    def test_elapsed_ban_and_timezone_confirmations_are_accepted(self):
        response = FakeResponse(payload={
            **USER, "banned_until": "2025-01-01T00:00:00Z",
            "email_confirmed_at": "2025-01-01T01:02:03+02:00",
        })
        verifier, _, _ = self.verifier(response)
        self.assertEqual(verifier.verify(ACCESS_TOKEN).verified_email, USER["email"])

    def test_unconfirmed_contact_and_metadata_never_grant_verified_email_or_identity(self):
        response = FakeResponse(payload={
            **USER, "email_confirmed_at": None, "confirmed_at": "2025-01-01T00:00:00Z",
            "sub": "forged-subject", "iss": "https://attacker.example",
            "user_metadata": {"id": "forged", "email_verified": True, "role": "service_role"},
            "app_metadata": {"email_verified": True, "issuer": "https://attacker.example"},
        })
        verifier, _, _ = self.verifier(response)
        self.assertEqual(verifier.verify(ACCESS_TOKEN), SupabaseIdentity(PROJECT_URL + "/auth/v1", SUBJECT, None))

    def test_bad_confirmed_email_and_confirmation_timestamp_are_refused(self):
        for value in (None, 42, "", "not-email", "a@example.test\n", " a@example.test", "a" * 255):
            with self.subTest(email=value):
                self.refused(FakeResponse(payload={**USER, "email": value}))
        for value in (False, "", "invalid", "2025-01-02", "2025-01-02T03:04:05",
                      "2025-02-30T00:00:00Z", "2999-01-01T00:00:00Z"):
            with self.subTest(confirmation=value):
                self.refused(FakeResponse(payload={**USER, "email_confirmed_at": value}))

    def test_same_email_with_different_provider_subjects_remains_distinct(self):
        other_subject = "70da6179-8a55-4df9-9115-c3a6bd643bb7"
        first, _, _ = self.verifier()
        second, _, _ = self.verifier(FakeResponse(payload={**USER, "id": other_subject}))
        self.assertNotEqual(first.verify(ACCESS_TOKEN), second.verify(ACCESS_TOKEN))

    def test_each_call_reverifies_and_does_not_cache_a_previous_success(self):
        verifier, session, factory = self.verifier()
        revoked = FakeResponse(status=403, payload={"code": "session_not_found"})
        session.get.side_effect = [FakeResponse(), revoked]
        self.assertEqual(verifier.verify(ACCESS_TOKEN).subject, SUBJECT)
        with self.assertRaises(SupabaseAuthenticationError):
            verifier.verify(ACCESS_TOKEN)
        self.assertEqual(factory.call_count, 2)
        self.assertTrue(revoked.closed)
