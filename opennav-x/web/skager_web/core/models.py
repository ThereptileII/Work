import uuid

from django.core.validators import RegexValidator
from django.db import models
from django.db.models import Q


hex64 = RegexValidator(r"\A[0-9a-f]{64}\Z", "Use a lowercase SHA-256 hex digest.")
currency_code = RegexValidator(r"\A[A-Z]{3}\Z", "Use an ISO 4217-style uppercase code.")
commit_hash = RegexValidator(r"\A[0-9a-f]{40}\Z", "Use a lowercase Git commit hash.")


class Environment(models.TextChoices):
    DEVELOPMENT = "development", "Development"
    STAGING = "staging", "Staging"
    PRODUCTION = "production", "Production"


class Account(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    issuer = models.URLField(max_length=512)
    subject = models.CharField(max_length=255)
    contact_email = models.EmailField(blank=True)
    display_name = models.CharField(max_length=200, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        constraints = [
            models.UniqueConstraint(fields=("issuer", "subject"), name="account_auth0_identity_uniq"),
            models.CheckConstraint(condition=~Q(issuer=""), name="account_issuer_nonempty"),
            models.CheckConstraint(condition=~Q(subject=""), name="account_subject_nonempty"),
        ]


class Order(models.Model):
    class Status(models.TextChoices):
        PENDING = "pending", "Pending"
        PAID = "paid", "Paid"
        REFUNDED = "refunded", "Refunded"
        SUSPENDED = "suspended", "Suspended"

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    account = models.ForeignKey(Account, on_delete=models.PROTECT, related_name="orders")
    environment = models.CharField(max_length=16, choices=Environment.choices)
    product_code = models.CharField(max_length=100)
    # Zero supports an explicitly authorized 100% provider discount. The
    # service layer must still verify the provider-computed total.
    amount_minor = models.PositiveBigIntegerField()
    currency = models.CharField(max_length=3, validators=[currency_code])
    status = models.CharField(max_length=16, choices=Status.choices, default=Status.PENDING)
    state_version = models.PositiveBigIntegerField(default=0)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        constraints = [
            models.CheckConstraint(condition=~Q(product_code=""), name="order_product_nonempty"),
            models.CheckConstraint(condition=Q(currency__regex=r"^[A-Z]{3}$"), name="order_currency_format"),
            models.CheckConstraint(
                condition=Q(environment__in=Environment.values), name="order_environment_valid"
            ),
        ]


class ProviderTransactionBinding(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    order = models.OneToOneField(Order, on_delete=models.PROTECT, related_name="provider_binding")
    environment = models.CharField(max_length=16, choices=Environment.choices)
    provider = models.CharField(max_length=32)
    provider_transaction_id = models.CharField(max_length=255)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        constraints = [
            models.UniqueConstraint(
                fields=("environment", "provider", "provider_transaction_id"),
                name="provider_transaction_env_ref_uniq",
            ),
            models.CheckConstraint(condition=~Q(provider=""), name="provider_binding_provider_nonempty"),
            models.CheckConstraint(
                condition=~Q(provider_transaction_id=""), name="provider_binding_transaction_nonempty"
            ),
            models.CheckConstraint(
                condition=Q(environment__in=Environment.values), name="provider_binding_environment_valid"
            ),
        ]


class VerifiedEventInbox(models.Model):
    class ProcessingState(models.TextChoices):
        RECEIVED = "received", "Received"
        APPLIED = "applied", "Applied"
        REJECTED = "rejected", "Rejected"

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    environment = models.CharField(max_length=16, choices=Environment.choices)
    provider = models.CharField(max_length=32)
    provider_event_id = models.CharField(max_length=255)
    payload_digest = models.CharField(max_length=64, validators=[hex64])
    normalized_payload = models.JSONField(default=dict)
    signature_verified_at = models.DateTimeField()
    received_at = models.DateTimeField(auto_now_add=True)
    processing_state = models.CharField(
        max_length=16, choices=ProcessingState.choices, default=ProcessingState.RECEIVED
    )
    processed_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        constraints = [
            models.UniqueConstraint(
                fields=("environment", "provider", "provider_event_id"), name="verified_event_env_ref_uniq"
            ),
            models.CheckConstraint(condition=~Q(provider_event_id=""), name="verified_event_ref_nonempty"),
            models.CheckConstraint(
                condition=Q(payload_digest__regex=r"^[0-9a-f]{64}$"), name="verified_event_digest_format"
            ),
            models.CheckConstraint(
                condition=Q(environment__in=Environment.values), name="verified_event_environment_valid"
            ),
        ]


class Entitlement(models.Model):
    class Status(models.TextChoices):
        ACTIVE = "active", "Active"
        REFUNDED = "refunded", "Refunded"
        SUSPENDED = "suspended", "Suspended"

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    account = models.ForeignKey(Account, on_delete=models.PROTECT, related_name="entitlements")
    order = models.OneToOneField(Order, on_delete=models.PROTECT, related_name="entitlement")
    environment = models.CharField(max_length=16, choices=Environment.choices)
    product_code = models.CharField(max_length=100)
    status = models.CharField(max_length=16, choices=Status.choices)
    state_version = models.PositiveBigIntegerField(default=1)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        constraints = [
            models.CheckConstraint(condition=Q(state_version__gte=1), name="entitlement_version_positive"),
            models.CheckConstraint(
                condition=Q(environment__in=Environment.values), name="entitlement_environment_valid"
            ),
        ]


class EntitlementEffect(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    entitlement = models.ForeignKey(Entitlement, on_delete=models.PROTECT, related_name="effects")
    event = models.ForeignKey(VerifiedEventInbox, on_delete=models.PROTECT, related_name="entitlement_effects")
    effect_kind = models.CharField(max_length=32)
    from_status = models.CharField(max_length=16, blank=True)
    to_status = models.CharField(max_length=16, choices=Entitlement.Status.choices)
    resulting_version = models.PositiveBigIntegerField()
    applied_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        constraints = [
            models.UniqueConstraint(fields=("event", "effect_kind"), name="entitlement_event_effect_uniq"),
            models.CheckConstraint(condition=Q(resulting_version__gte=1), name="effect_version_positive"),
        ]


class ReleaseArtifact(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    application_commit = models.CharField(max_length=40, validators=[commit_hash])
    binary_object_key = models.CharField(max_length=512)
    binary_sha256 = models.CharField(max_length=64, validators=[hex64])
    source_public_url = models.URLField(max_length=1024)
    license_public_url = models.URLField(max_length=1024)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        constraints = [
            models.UniqueConstraint(
                fields=("application_commit", "binary_sha256"), name="release_artifact_identity_uniq"
            ),
            models.CheckConstraint(
                condition=Q(application_commit__regex=r"^[0-9a-f]{40}$"), name="release_commit_format"
            ),
            models.CheckConstraint(
                condition=Q(binary_sha256__regex=r"^[0-9a-f]{64}$"), name="release_digest_format"
            ),
        ]


class Release(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    artifact = models.ForeignKey(ReleaseArtifact, on_delete=models.PROTECT, related_name="publications")
    version = models.CharField(max_length=64)
    channel = models.CharField(max_length=32)
    manifest_object_key = models.CharField(max_length=512)
    signature_object_key = models.CharField(max_length=512)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        constraints = [
            models.UniqueConstraint(fields=("channel", "version"), name="release_channel_version_uniq"),
            models.UniqueConstraint(fields=("manifest_object_key",), name="release_manifest_key_uniq"),
            models.UniqueConstraint(fields=("signature_object_key",), name="release_signature_key_uniq"),
            models.CheckConstraint(condition=~Q(manifest_object_key=""), name="release_manifest_key_nonempty"),
            models.CheckConstraint(condition=~Q(signature_object_key=""), name="release_signature_key_nonempty"),
        ]


class DownloadAudit(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    account = models.ForeignKey(Account, on_delete=models.PROTECT, related_name="download_audits")
    entitlement = models.ForeignKey(
        Entitlement, null=True, blank=True, on_delete=models.PROTECT, related_name="download_audits"
    )
    release = models.ForeignKey(Release, on_delete=models.PROTECT, related_name="download_audits")
    authorized = models.BooleanField()
    denial_reason = models.CharField(max_length=64, blank=True)
    requested_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        constraints = [
            models.CheckConstraint(
                condition=Q(authorized=False) | Q(entitlement__isnull=False),
                name="authorized_download_has_entitlement",
            )
        ]


class SupportSubmission(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    account = models.ForeignKey(Account, on_delete=models.PROTECT, related_name="support_submissions")
    category = models.CharField(max_length=32)
    subject = models.CharField(max_length=200)
    body = models.TextField()
    customer_status = models.CharField(max_length=32, default="received")
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)


class JiraMapping(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    submission = models.OneToOneField(SupportSubmission, on_delete=models.PROTECT, related_name="jira_mapping")
    jira_project_key = models.CharField(max_length=32)
    jira_issue_id = models.CharField(max_length=64, unique=True)
    created_at = models.DateTimeField(auto_now_add=True)


class JiraOutbox(models.Model):
    class Status(models.TextChoices):
        PENDING = "pending", "Pending"
        PROCESSING = "processing", "Processing"
        SENT = "sent", "Sent"
        FAILED = "failed", "Failed"

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    submission = models.ForeignKey(SupportSubmission, on_delete=models.PROTECT, related_name="jira_commands")
    command_kind = models.CharField(max_length=32)
    sanitized_payload = models.JSONField(default=dict)
    status = models.CharField(max_length=16, choices=Status.choices, default=Status.PENDING)
    attempts = models.PositiveIntegerField(default=0)
    available_at = models.DateTimeField()
    last_error_code = models.CharField(max_length=64, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)


class AuditEvent(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    account = models.ForeignKey(Account, null=True, blank=True, on_delete=models.PROTECT, related_name="audit_events")
    source_event = models.ForeignKey(
        VerifiedEventInbox, null=True, blank=True, on_delete=models.PROTECT, related_name="audit_events"
    )
    event_kind = models.CharField(max_length=64)
    object_type = models.CharField(max_length=64)
    object_id = models.UUIDField()
    details = models.JSONField(default=dict)
    occurred_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        indexes = [models.Index(fields=("object_type", "object_id"), name="audit_object_idx")]
