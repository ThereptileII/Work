import hashlib
import io
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from unittest import skipUnless
from unittest.mock import patch
from urllib.parse import parse_qs, urlsplit

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from django.core.exceptions import ImproperlyConfigured
from django.core.management import call_command
from django.db import close_old_connections, connection
from django.middleware.csrf import _get_new_csrf_string, _mask_cipher_secret
from django.test import Client, SimpleTestCase, TestCase, TransactionTestCase, override_settings
from django.urls import reverse
from joserfc import jwt
from joserfc.jwk import RSAKey

from skager_web.accounts.config import validate_oidc_configuration
from skager_web.accounts.models import OidcAuthorizationAttempt
from skager_web.accounts.oidc import (
    OidcAuthenticationError,
    begin_authorization,
    consume_authorization,
)
from skager_web.accounts.views import ACCOUNT_SESSION_KEY
from skager_web.core.models import Account


OIDC_SETTINGS = {
    "OIDC_AUTH_ENABLED": True,
    "OIDC_ISSUER": "https://tenant.example/",
    "OIDC_ALLOWED_ISSUERS": ("https://tenant.example/",),
    "OIDC_CLIENT_ID": "skager-web-test",
    "OIDC_CLIENT_SECRET": "test-secret-never-production",
    "OIDC_AUTHORIZATION_ENDPOINT": "https://tenant.example/authorize",
    "OIDC_TOKEN_ENDPOINT": "https://tenant.example/oauth/token",
    "OIDC_JWKS_URI": "https://tenant.example/.well-known/jwks.json",
    "OIDC_CALLBACK_URL": "https://skager.test/auth/callback/",
    "OIDC_SUCCESS_URL": "/signed-in",
    "OIDC_LOGOUT_URL": "/signed-out",
    "OIDC_HTTP_TIMEOUT_SECONDS": 2,
    "OIDC_CLOCK_SKEW_SECONDS": 30,
}


class FakeRaw:
    def __init__(self, body):
        self.stream = io.BytesIO(body)

    def read(self, amount, decode_content=False):
        return self.stream.read(amount)


class FakeResponse:
    def __init__(self, payload=None, *, status_code=200, body=None, headers=None):
        if body is None:
            body = json.dumps(payload).encode("utf-8")
        self.status_code = status_code
        self.headers = headers or {}
        self.raw = FakeRaw(body)
        self.closed = False

    def raise_for_status(self):
        if self.status_code >= 400:
            raise RuntimeError(f"HTTP {self.status_code}")

    def close(self):
        self.closed = True


@override_settings(**OIDC_SETTINGS)
class OidcFlowTests(TestCase):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.private_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        cls.other_private_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        cls.private_pem = cls.private_key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption(),
        )
        cls.other_private_pem = cls.other_private_key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption(),
        )
        public_pem = cls.private_key.public_key().public_bytes(
            serialization.Encoding.PEM,
            serialization.PublicFormat.SubjectPublicKeyInfo,
        )
        cls.jwk = RSAKey.import_key(public_pem).as_dict(kid="test-key", use="sig")

    def begin(self):
        response = self.client.get(reverse("accounts:login"))
        self.assertEqual(response.status_code, 302)
        query = parse_qs(urlsplit(response.url).query)
        state = query["state"][0]
        attempt = OidcAuthorizationAttempt.objects.get(
            state_digest=hashlib.sha256(state.encode("utf-8")).hexdigest()
        )
        pending = {
            "state": state,
            "nonce": attempt.nonce,
            "code_verifier": attempt.code_verifier,
        }
        return response, pending

    def signed_token(self, pending, **overrides):
        now = int(time.time())
        claims = {
            "iss": OIDC_SETTINGS["OIDC_ISSUER"],
            "sub": "auth0|customer-1",
            "aud": OIDC_SETTINGS["OIDC_CLIENT_ID"],
            "iat": now,
            "exp": now + 300,
            "nonce": pending["nonce"],
            "email": "customer@example.test",
            "name": "Test Customer",
        }
        claims.update(overrides)
        return jwt.encode(
            {"alg": "RS256", "kid": "test-key"},
            claims,
            RSAKey.import_key(self.private_pem),
            algorithms=["RS256"],
        )

    def callback(self, pending, token, state=None):
        with (
            patch(
                "skager_web.accounts.oidc.requests.post",
                return_value=FakeResponse({"id_token": token, "access_token": "discarded-access-token"}),
            ) as token_post,
            patch(
                "skager_web.accounts.oidc.requests.get",
                return_value=FakeResponse({"keys": [self.jwk]}),
            ),
        ):
            response = self.client.get(
                reverse("accounts:callback"),
                {"code": "one-time-code", "state": state or pending["state"]},
            )
        return response, token_post

    def test_login_uses_fixed_callback_state_nonce_and_s256_pkce(self):
        response, pending = self.begin()
        query = parse_qs(urlsplit(response.url).query)
        self.assertEqual(urlsplit(response.url)._replace(query="").geturl(), OIDC_SETTINGS["OIDC_AUTHORIZATION_ENDPOINT"])
        self.assertEqual(query["redirect_uri"], [OIDC_SETTINGS["OIDC_CALLBACK_URL"]])
        self.assertEqual(query["state"], [pending["state"]])
        self.assertEqual(query["nonce"], [pending["nonce"]])
        self.assertEqual(query["code_challenge_method"], ["S256"])
        self.assertNotIn(pending["code_verifier"], response.url)
        self.assertNotIn("offline_access", query["scope"][0])

    def test_valid_callback_rotates_session_and_keeps_tokens_out(self):
        _, pending = self.begin()
        pre_callback_session = self.client.cookies["sessionid"].value
        response, token_post = self.callback(pending, self.signed_token(pending))
        self.assertRedirects(response, OIDC_SETTINGS["OIDC_SUCCESS_URL"], fetch_redirect_response=False)
        self.assertNotEqual(pre_callback_session, self.client.cookies["sessionid"].value)
        account = Account.objects.get(issuer=OIDC_SETTINGS["OIDC_ISSUER"], subject="auth0|customer-1")
        self.assertEqual(self.client.session[ACCOUNT_SESSION_KEY], str(account.id))
        self.assertEqual(set(self.client.session.keys()), {ACCOUNT_SESSION_KEY, "_session_expiry"})
        request_data = token_post.call_args.kwargs["data"]
        self.assertEqual(request_data["code_verifier"], pending["code_verifier"])
        self.assertEqual(request_data["client_id"], OIDC_SETTINGS["OIDC_CLIENT_ID"])
        self.assertEqual(request_data["client_secret"], OIDC_SETTINGS["OIDC_CLIENT_SECRET"])
        self.assertNotIn("id_token", self.client.session)
        self.assertNotIn("access_token", self.client.session)
        self.assertNotIn("refresh_token", self.client.session)
        attempt = OidcAuthorizationAttempt.objects.get(
            state_digest=hashlib.sha256(pending["state"].encode("utf-8")).hexdigest()
        )
        self.assertEqual(attempt.nonce, "")
        self.assertEqual(attempt.code_verifier, "")

    def test_same_email_does_not_join_distinct_subjects(self):
        Account.objects.create(
            issuer=OIDC_SETTINGS["OIDC_ISSUER"],
            subject="auth0|other-subject",
            contact_email="customer@example.test",
        )
        _, pending = self.begin()
        response, _ = self.callback(pending, self.signed_token(pending))
        self.assertEqual(response.status_code, 302)
        self.assertEqual(Account.objects.filter(contact_email="customer@example.test").count(), 2)

    def test_wrong_signature_is_rejected(self):
        _, pending = self.begin()
        now = int(time.time())
        token = jwt.encode(
            {"alg": "RS256", "kid": "test-key"},
            {
                "iss": OIDC_SETTINGS["OIDC_ISSUER"],
                "sub": "auth0|customer-1",
                "aud": OIDC_SETTINGS["OIDC_CLIENT_ID"],
                "iat": now,
                "exp": now + 300,
                "nonce": pending["nonce"],
            },
            RSAKey.import_key(self.other_private_pem),
            algorithms=["RS256"],
        )
        response, _ = self.callback(pending, token)
        self.assertEqual(response.status_code, 400)
        self.assertFalse(Account.objects.exists())

    def test_wrong_issuer_audience_nonce_and_expiry_are_rejected(self):
        cases = (
            {"iss": "https://attacker.example/"},
            {"aud": "other-client"},
            {"nonce": "wrong-nonce"},
            {"exp": int(time.time()) - 120},
        )
        for overrides in cases:
            with self.subTest(overrides=overrides):
                self.client = Client()
                _, pending = self.begin()
                response, _ = self.callback(pending, self.signed_token(pending, **overrides))
                self.assertEqual(response.status_code, 400)
        self.assertFalse(Account.objects.exists())

    def test_state_mismatch_is_consumed_without_token_exchange(self):
        _, pending = self.begin()
        with patch("skager_web.accounts.oidc.requests.post") as token_post:
            response = self.client.get(
                reverse("accounts:callback"), {"code": "code", "state": "wrong-state"}
            )
        self.assertEqual(response.status_code, 400)
        token_post.assert_not_called()

    def test_oversized_callback_parameters_are_rejected_before_database_or_network_work(self):
        _, pending = self.begin()
        for params in (
            {"code": "c" * 2_049, "state": pending["state"]},
            {"code": "code", "state": "s" * 129},
        ):
            with self.subTest(parameter=max(params, key=lambda key: len(params[key]))):
                with patch("skager_web.accounts.oidc.requests.post") as token_post:
                    response = self.client.get(reverse("accounts:callback"), params)
                self.assertEqual(response.status_code, 400)
                token_post.assert_not_called()

    def test_expired_authorization_attempt_is_rejected_before_exchange(self):
        with override_settings(OIDC_ATTEMPT_AGE_SECONDS=-1):
            _, pending = self.begin()
        with patch("skager_web.accounts.oidc.requests.post") as token_post:
            response = self.client.get(
                reverse("accounts:callback"),
                {"code": "code", "state": pending["state"]},
            )
        self.assertEqual(response.status_code, 400)
        token_post.assert_not_called()

    def test_expired_attempt_cleanup_keeps_live_attempts(self):
        with override_settings(OIDC_ATTEMPT_AGE_SECONDS=-1):
            _, expired = self.begin()
        _, live = self.begin()
        call_command("purge_oidc_attempts", verbosity=0)
        digests = set(OidcAuthorizationAttempt.objects.values_list("state_digest", flat=True))
        self.assertNotIn(hashlib.sha256(expired["state"].encode("utf-8")).hexdigest(), digests)
        self.assertIn(hashlib.sha256(live["state"].encode("utf-8")).hexdigest(), digests)

    def test_callback_replay_is_rejected_before_second_exchange(self):
        _, pending = self.begin()
        token = self.signed_token(pending)
        first, first_post = self.callback(pending, token)
        self.assertEqual(first.status_code, 302)
        first_post.assert_called_once()
        with patch("skager_web.accounts.oidc.requests.post") as second_post:
            second = self.client.get(
                reverse("accounts:callback"),
                {"code": "one-time-code", "state": pending["state"]},
            )
        self.assertEqual(second.status_code, 400)
        second_post.assert_not_called()

    def test_provider_redirect_is_rejected_without_following_it(self):
        _, pending = self.begin()
        redirect_response = FakeResponse(
            status_code=307,
            body=b"",
            headers={"Location": "https://attacker.example/token"},
        )
        with (
            patch(
                "skager_web.accounts.oidc.requests.post",
                return_value=redirect_response,
            ) as token_post,
            patch("skager_web.accounts.oidc.requests.get") as jwks_get,
        ):
            response = self.client.get(
                reverse("accounts:callback"),
                {"code": "code", "state": pending["state"]},
            )
        self.assertEqual(response.status_code, 400)
        self.assertFalse(token_post.call_args.kwargs["allow_redirects"])
        jwks_get.assert_not_called()
        self.assertTrue(redirect_response.closed)

    def test_oversized_token_response_is_rejected_before_jwks_fetch(self):
        _, pending = self.begin()
        oversized = b"x" * (OIDC_SETTINGS.get("OIDC_TOKEN_RESPONSE_MAX_BYTES", 32_768) + 1)
        oversized_response = FakeResponse(body=oversized)
        with (
            patch(
                "skager_web.accounts.oidc.requests.post",
                return_value=oversized_response,
            ),
            patch("skager_web.accounts.oidc.requests.get") as jwks_get,
        ):
            response = self.client.get(
                reverse("accounts:callback"),
                {"code": "code", "state": pending["state"]},
            )
        self.assertEqual(response.status_code, 400)
        jwks_get.assert_not_called()
        self.assertTrue(oversized_response.closed)

    def test_jwks_redirect_is_rejected_without_following_it(self):
        _, pending = self.begin()
        token = self.signed_token(pending)
        with (
            patch(
                "skager_web.accounts.oidc.requests.post",
                return_value=FakeResponse({"id_token": token}),
            ),
            patch(
                "skager_web.accounts.oidc.requests.get",
                return_value=FakeResponse(
                    status_code=308,
                    body=b"",
                    headers={"Location": "https://attacker.example/jwks.json"},
                ),
            ) as jwks_get,
        ):
            response = self.client.get(
                reverse("accounts:callback"),
                {"code": "code", "state": pending["state"]},
            )
        self.assertEqual(response.status_code, 400)
        self.assertFalse(jwks_get.call_args.kwargs["allow_redirects"])

    def test_logout_requires_csrf_protected_post_and_flushes_session(self):
        client = Client(enforce_csrf_checks=True)
        session = client.session
        session[ACCOUNT_SESSION_KEY] = "00000000-0000-0000-0000-000000000001"
        session.save()
        self.assertEqual(client.get(reverse("accounts:logout")).status_code, 405)
        self.assertEqual(client.post(reverse("accounts:logout")).status_code, 403)

        csrf_secret = _get_new_csrf_string()
        client.cookies["csrftoken"] = csrf_secret
        response = client.post(
            reverse("accounts:logout"), HTTP_X_CSRFTOKEN=_mask_cipher_secret(csrf_secret)
        )
        self.assertRedirects(response, OIDC_SETTINGS["OIDC_LOGOUT_URL"], fetch_redirect_response=False)
        self.assertNotIn(ACCOUNT_SESSION_KEY, client.session)


class DisabledOidcTests(TestCase):
    def test_authentication_routes_are_disabled_by_default(self):
        for route in ("accounts:login", "accounts:callback"):
            self.assertEqual(self.client.get(reverse(route)).status_code, 404)
        self.assertEqual(self.client.post(reverse("accounts:logout")).status_code, 404)


@override_settings(**OIDC_SETTINGS)
class OidcConfigurationTests(SimpleTestCase):
    def test_complete_fixed_configuration_is_accepted(self):
        validate_oidc_configuration()

    def test_issuer_must_be_allowlisted(self):
        with override_settings(OIDC_ALLOWED_ISSUERS=("https://other.example/",)):
            with self.assertRaises(ImproperlyConfigured):
                validate_oidc_configuration()

    def test_provider_endpoints_must_share_https_issuer_origin(self):
        for setting_name, value in (
            ("OIDC_TOKEN_ENDPOINT", "https://attacker.example/oauth/token"),
            ("OIDC_JWKS_URI", "http://tenant.example/.well-known/jwks.json"),
            ("OIDC_CALLBACK_URL", "http://skager.test/auth/callback/"),
        ):
            with self.subTest(setting_name=setting_name):
                with override_settings(**{setting_name: value}):
                    with self.assertRaises(ImproperlyConfigured):
                        validate_oidc_configuration()

    def test_local_redirect_paths_reject_network_like_and_control_values(self):
        for value in ("//evil.example", "/\\evil.example", "/good\nLocation: https://evil.example"):
            with self.subTest(value=value):
                with override_settings(OIDC_SUCCESS_URL=value):
                    with self.assertRaises(ImproperlyConfigured):
                        validate_oidc_configuration()


@override_settings(**OIDC_SETTINGS)
class OidcAttemptConcurrencyTests(TransactionTestCase):
    @skipUnless(connection.vendor == "postgresql", "row-lock acceptance requires PostgreSQL")
    def test_only_one_concurrent_callback_consumes_an_attempt(self):
        pending, _ = begin_authorization("concurrent-session-key")
        start = threading.Barrier(2)

        def consume():
            close_old_connections()
            try:
                start.wait(timeout=5)
                consume_authorization("concurrent-session-key", pending.state)
                return True
            except OidcAuthenticationError:
                return False
            finally:
                close_old_connections()

        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(executor.map(lambda _: consume(), range(2)))
        self.assertEqual(sorted(results), [False, True])
