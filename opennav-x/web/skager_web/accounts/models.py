import uuid

from django.db import models


class OidcAuthorizationAttempt(models.Model):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    state_digest = models.CharField(max_length=64, unique=True)
    session_key_digest = models.CharField(max_length=64)
    nonce = models.CharField(max_length=255)
    code_verifier = models.CharField(max_length=255)
    expires_at = models.DateTimeField()
    consumed_at = models.DateTimeField(null=True, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        indexes = [models.Index(fields=("expires_at",), name="oidc_attempt_expiry_idx")]
