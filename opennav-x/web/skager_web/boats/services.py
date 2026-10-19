"""Owner-scoped operations for an Account resolved by trusted authentication.

The caller must supply the authenticated Account, never a request's owner ID.
These internal services do not authenticate a browser session or expose an API.
"""

from django.core.exceptions import PermissionDenied, ValidationError
from django.http import Http404

from skager_web.core.models import Account

from .models import Boat


def _require_account(account: Account) -> None:
    if (
        not isinstance(account, Account)
        or account.pk is None
        or account._state.adding
        or not Account.objects.filter(pk=account.pk).exists()
    ):
        raise PermissionDenied("Authentication required.")


def list_boats(*, account: Account):
    _require_account(account)
    return Boat.objects.filter(owner=account)


def get_boat(*, account: Account, boat_id) -> Boat:
    _require_account(account)
    try:
        return Boat.objects.get(pk=boat_id, owner=account)
    except (Boat.DoesNotExist, ValidationError, TypeError, ValueError):
        raise Http404("Boat not found.") from None


def create_boat(*, account: Account, name: str) -> Boat:
    _require_account(account)
    if not isinstance(name, str):
        raise ValidationError({"name": "Enter a boat name."})
    boat = Boat(owner=account, name=name.strip())
    boat.full_clean()
    boat.save(force_insert=True)
    return boat
