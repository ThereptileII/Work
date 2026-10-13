import uuid

from django.db import models
from django.db.models import Q
from django.db.models.functions import Length
from django.db.models.lookups import LessThanOrEqual


class Boat(models.Model):
    """Ownership foundation only; no connected-vessel data is stored here."""

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    owner = models.ForeignKey("core.Account", on_delete=models.PROTECT, related_name="boats")
    name = models.CharField(max_length=80)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ("created_at", "id")
        constraints = [
            models.CheckConstraint(condition=Q(name__regex=r"\S"), name="boat_name_nonblank"),
            models.CheckConstraint(
                condition=LessThanOrEqual(Length("name"), 80), name="boat_name_length_valid"
            ),
        ]
