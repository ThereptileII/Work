from django.db import IntegrityError, connection, transaction
from django.test import TransactionTestCase
from django.utils import timezone

from skager_web.core.models import (
    Account,
    AuditEvent,
    DownloadAudit,
    Entitlement,
    EntitlementEffect,
    Environment,
    Order,
    ProviderTransactionBinding,
    Release,
    ReleaseArtifact,
    VerifiedEventInbox,
)


class PostgreSQLIntegrityTests(TransactionTestCase):
    def setUp(self):
        self.assertEqual(
            connection.vendor,
            "postgresql",
            "Integrity acceptance must run against PostgreSQL; skipped SQLite is not acceptable",
        )
        self.account = Account.objects.create(issuer="https://tenant.auth0.com/", subject="auth0|customer-1")
        self.order = Order.objects.create(
            account=self.account,
            environment=Environment.STAGING,
            product_code="public-beta-access",
            amount_minor=2000,
            currency="USD",
        )

    def assert_integrity_error(self, operation):
        with self.assertRaises(IntegrityError), transaction.atomic():
            operation()

    def test_auth0_pair_is_unique_and_immutable(self):
        self.assert_integrity_error(
            lambda: Account.objects.create(issuer=self.account.issuer, subject=self.account.subject)
        )
        self.account.subject = "auth0|changed"
        self.assert_integrity_error(self.account.save)

    def test_money_and_order_terms_are_database_enforced(self):
        self.assert_integrity_error(
            lambda: Order.objects.create(
                account=self.account,
                environment=Environment.STAGING,
                product_code="public-beta-access",
                amount_minor=-1,
                currency="USD",
            )
        )
        free_order = Order.objects.create(
            account=self.account,
            environment=Environment.STAGING,
            product_code="approved-fully-discounted-access",
            amount_minor=0,
            currency="USD",
        )
        self.assertEqual(free_order.amount_minor, 0)
        self.assert_integrity_error(
            lambda: Order.objects.create(
                account=self.account,
                environment=Environment.STAGING,
                product_code="public-beta-access",
                amount_minor=2000,
                currency="usd",
            )
        )
        self.order.amount_minor = 1
        self.assert_integrity_error(self.order.save)

    def test_provider_transaction_is_unique_immutable_and_environment_bound(self):
        binding = ProviderTransactionBinding.objects.create(
            order=self.order,
            environment=Environment.STAGING,
            provider="paddle",
            provider_transaction_id="txn_1",
        )
        other_order = Order.objects.create(
            account=self.account,
            environment=Environment.STAGING,
            product_code="other",
            amount_minor=100,
            currency="USD",
        )
        self.assert_integrity_error(
            lambda: ProviderTransactionBinding.objects.create(
                order=other_order,
                environment=Environment.STAGING,
                provider="paddle",
                provider_transaction_id="txn_1",
            )
        )
        binding.provider_transaction_id = "txn_changed"
        self.assert_integrity_error(binding.save)
        self.assert_integrity_error(
            lambda: ProviderTransactionBinding.objects.create(
                order=other_order,
                environment=Environment.PRODUCTION,
                provider="paddle",
                provider_transaction_id="txn_2",
            )
        )

    def test_verified_event_is_idempotent_and_receipt_is_immutable(self):
        event = VerifiedEventInbox.objects.create(
            environment=Environment.STAGING,
            provider="paddle",
            provider_event_id="evt_1",
            payload_digest="a" * 64,
            normalized_payload={"event_type": "transaction.completed"},
            signature_verified_at=timezone.now(),
        )
        self.assert_integrity_error(
            lambda: VerifiedEventInbox.objects.create(
                environment=Environment.STAGING,
                provider="paddle",
                provider_event_id="evt_1",
                payload_digest="b" * 64,
                normalized_payload={},
                signature_verified_at=timezone.now(),
            )
        )
        event.payload_digest = "c" * 64
        self.assert_integrity_error(event.save)

    def test_verified_event_processing_can_advance_but_receipt_cannot_be_deleted(self):
        event = VerifiedEventInbox.objects.create(
            environment=Environment.STAGING,
            provider="paddle",
            provider_event_id="evt_processing",
            payload_digest="d" * 64,
            normalized_payload={"event_type": "transaction.completed"},
            signature_verified_at=timezone.now(),
        )
        event.processing_state = VerifiedEventInbox.ProcessingState.APPLIED
        event.processed_at = timezone.now()
        event.save()
        event.refresh_from_db()
        self.assertEqual(event.processing_state, VerifiedEventInbox.ProcessingState.APPLIED)
        self.assert_integrity_error(event.delete)

    def test_entitlement_effect_event_environment_must_match(self):
        entitlement = Entitlement.objects.create(
            account=self.account,
            order=self.order,
            environment=Environment.STAGING,
            product_code="public-beta-access",
            status=Entitlement.Status.ACTIVE,
        )
        production_event = VerifiedEventInbox.objects.create(
            environment=Environment.PRODUCTION,
            provider="paddle",
            provider_event_id="evt_wrong_environment",
            payload_digest="e" * 64,
            normalized_payload={"event_type": "transaction.completed"},
            signature_verified_at=timezone.now(),
        )
        self.assert_integrity_error(
            lambda: EntitlementEffect.objects.create(
                entitlement=entitlement,
                event=production_event,
                effect_kind="activate",
                to_status=Entitlement.Status.ACTIVE,
                resulting_version=1,
            )
        )

    def test_entitlement_and_download_are_bound_to_owner_and_order(self):
        other = Account.objects.create(issuer="https://tenant.auth0.com/", subject="auth0|customer-2")
        self.assert_integrity_error(
            lambda: Entitlement.objects.create(
                account=other,
                order=self.order,
                environment=Environment.STAGING,
                product_code="public-beta-access",
                status=Entitlement.Status.ACTIVE,
            )
        )
        entitlement = Entitlement.objects.create(
            account=self.account,
            order=self.order,
            environment=Environment.STAGING,
            product_code="public-beta-access",
            status=Entitlement.Status.ACTIVE,
        )
        artifact = ReleaseArtifact.objects.create(
            application_commit="1" * 40,
            binary_object_key="private/release.exe",
            binary_sha256="2" * 64,
            source_public_url="https://example.test/source",
            license_public_url="https://example.test/licenses",
        )
        release = Release.objects.create(
            artifact=artifact,
            version="0.1.0",
            channel="public-beta",
            manifest_object_key="private/public-beta/manifest.json",
            signature_object_key="private/public-beta/manifest.sig",
        )
        denied = DownloadAudit.objects.create(
            account=other,
            entitlement=None,
            release=release,
            authorized=False,
            denial_reason="no_entitlement",
        )
        self.assertFalse(denied.authorized)
        self.assert_integrity_error(
            lambda: DownloadAudit.objects.create(
                account=other, entitlement=None, release=release, authorized=True
            )
        )
        self.assert_integrity_error(
            lambda: DownloadAudit.objects.create(
                account=other, entitlement=entitlement, release=release, authorized=True
            )
        )

    def test_refunded_order_does_not_block_a_new_entitlement(self):
        first = Entitlement.objects.create(
            account=self.account,
            order=self.order,
            environment=Environment.STAGING,
            product_code="public-beta-access",
            status=Entitlement.Status.REFUNDED,
        )
        replacement_order = Order.objects.create(
            account=self.account,
            environment=Environment.STAGING,
            product_code="public-beta-access",
            amount_minor=2000,
            currency="USD",
        )
        replacement = Entitlement.objects.create(
            account=self.account,
            order=replacement_order,
            environment=Environment.STAGING,
            product_code="public-beta-access",
            status=Entitlement.Status.ACTIVE,
        )
        self.assertNotEqual(first.id, replacement.id)

    def test_one_artifact_can_be_promoted_across_channels(self):
        artifact = ReleaseArtifact.objects.create(
            application_commit="3" * 40,
            binary_object_key="private/release.exe",
            binary_sha256="4" * 64,
            source_public_url="https://example.test/source",
            license_public_url="https://example.test/licenses",
        )
        beta = Release.objects.create(
            artifact=artifact,
            version="0.2.0-beta.1",
            channel="public-beta",
            manifest_object_key="private/public-beta/manifest.json",
            signature_object_key="private/public-beta/manifest.sig",
        )
        stable = Release.objects.create(
            artifact=artifact,
            version="0.2.0",
            channel="stable",
            manifest_object_key="private/stable/manifest.json",
            signature_object_key="private/stable/manifest.sig",
        )
        self.assertEqual(artifact.publications.count(), 2)
        self.assertNotEqual(beta.manifest_object_key, stable.manifest_object_key)
        self.assertNotEqual(beta.signature_object_key, stable.signature_object_key)

    def test_release_signed_metadata_references_are_nonempty_and_not_reused(self):
        artifact = ReleaseArtifact.objects.create(
            application_commit="5" * 40,
            binary_object_key="private/release.exe",
            binary_sha256="6" * 64,
            source_public_url="https://example.test/source",
            license_public_url="https://example.test/licenses",
        )
        Release.objects.create(
            artifact=artifact,
            version="0.3.0-beta.1",
            channel="public-beta",
            manifest_object_key="private/public-beta/0.3.0/manifest.json",
            signature_object_key="private/public-beta/0.3.0/manifest.sig",
        )
        self.assert_integrity_error(
            lambda: Release.objects.create(
                artifact=artifact,
                version="0.3.0",
                channel="stable",
                manifest_object_key="private/public-beta/0.3.0/manifest.json",
                signature_object_key="private/stable/0.3.0/manifest.sig",
            )
        )
        self.assert_integrity_error(
            lambda: Release.objects.create(
                artifact=artifact,
                version="0.3.1",
                channel="stable",
                manifest_object_key="private/stable/0.3.1/manifest.json",
                signature_object_key="private/public-beta/0.3.0/manifest.sig",
            )
        )
        self.assert_integrity_error(
            lambda: Release.objects.create(
                artifact=artifact,
                version="0.3.2",
                channel="stable",
                manifest_object_key="",
                signature_object_key="private/stable/0.3.2/manifest.sig",
            )
        )
        self.assert_integrity_error(
            lambda: Release.objects.create(
                artifact=artifact,
                version="0.3.3",
                channel="stable",
                manifest_object_key="private/stable/0.3.1/manifest.json",
                signature_object_key="",
            )
        )

    def test_audit_rows_are_append_only(self):
        audit = AuditEvent.objects.create(
            account=self.account,
            event_kind="order.created",
            object_type="order",
            object_id=self.order.id,
            details={},
        )
        audit.details = {"changed": True}
        self.assert_integrity_error(audit.save)
        self.assert_integrity_error(audit.delete)
