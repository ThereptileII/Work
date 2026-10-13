import uuid

from django.core.exceptions import PermissionDenied, ValidationError
from django.db import DataError, IntegrityError, connection, transaction
from django.db.models.deletion import ProtectedError
from django.http import Http404
from django.test import TestCase

from skager_web.core.models import Account

from ..models import Boat
from ..services import create_boat, get_boat, list_boats


class OwnershipServiceTests(TestCase):
    def setUp(self):
        self.owner = Account.objects.create(issuer="https://tenant.auth0.com/", subject="auth0|owner")
        self.other = Account.objects.create(issuer=self.owner.issuer, subject="auth0|other")

    def test_creation_assigns_authenticated_account_and_strips_name(self):
        boat = create_boat(account=self.owner, name="  S/Y Aurora  ")
        self.assertIsInstance(boat.id, uuid.UUID)
        self.assertEqual(boat.owner_id, self.owner.id)
        self.assertEqual(boat.name, "S/Y Aurora")
        self.assertIsNotNone(boat.created_at)
        self.assertIsNotNone(boat.updated_at)
        self.assertEqual(get_boat(account=self.owner, boat_id=boat.id), boat)

    def test_list_and_lookup_never_expose_another_owners_boat(self):
        mine = create_boat(account=self.owner, name="Mine")
        theirs = create_boat(account=self.other, name="Private boat name")
        self.assertEqual(list(list_boats(account=self.owner)), [mine])
        self.assertEqual(list(list_boats(account=self.other)), [theirs])
        messages = []
        for boat_id in (theirs.id, uuid.uuid4(), "invalid-id", None):
            with self.assertRaises(Http404) as error:
                get_boat(account=self.owner, boat_id=boat_id)
            messages.append(str(error.exception))
        self.assertEqual(messages, ["Boat not found."] * 4)

    def test_services_reject_ids_missing_and_unsaved_accounts(self):
        for account in (None, self.owner.id, str(self.owner.id), Account(id=self.owner.id)):
            for operation in (
                lambda: create_boat(account=account, name="Aurora"),
                lambda: list_boats(account=account),
                lambda: get_boat(account=account, boat_id=uuid.uuid4()),
            ):
                with self.assertRaises(PermissionDenied):
                    operation()

    def test_deleting_an_owner_is_protected(self):
        boat = create_boat(account=self.owner, name="Aurora")
        with self.assertRaises(ProtectedError):
            self.owner.delete()
        self.assertTrue(Account.objects.filter(pk=self.owner.pk).exists())
        self.assertTrue(Boat.objects.filter(pk=boat.pk, owner=self.owner).exists())

    def test_names_are_nonblank_and_at_most_eighty_characters(self):
        for name in ("", "   ", "\t\n", "x" * 81, None, 12):
            with self.subTest(name=name), self.assertRaises(ValidationError):
                create_boat(account=self.owner, name=name)
        boat = create_boat(account=self.owner, name="Å" * 80)
        self.assertEqual(len(boat.name), 80)
        self.assertEqual(Boat.objects.count(), 1)

    def test_name_constraints_also_protect_direct_database_writes(self):
        for name in ("", "   ", "x" * 81):
            # PostgreSQL rejects overlong varchar before the CHECK constraint;
            # SQLite reaches the explicit length check. Both must reject it.
            expected = DataError if connection.vendor == "postgresql" and len(name) > 80 else IntegrityError
            with self.subTest(name=name), self.assertRaises(expected), transaction.atomic():
                Boat.objects.create(owner=self.owner, name=name)
        boat = create_boat(account=self.owner, name="Aurora")
        with self.assertRaises(IntegrityError), transaction.atomic():
            Boat.objects.filter(pk=boat.pk).update(name="")
        boat.refresh_from_db()
        self.assertEqual(boat.name, "Aurora")


class PostgreSQLOwnershipTests(TestCase):
    """Real PostgreSQL is mandatory for owner-immutability acceptance."""

    def setUp(self):
        self.assertEqual(connection.vendor, "postgresql", "Owner trigger acceptance requires PostgreSQL")
        self.owner = Account.objects.create(issuer="https://tenant.auth0.com/", subject="auth0|owner")
        self.other = Account.objects.create(issuer=self.owner.issuer, subject="auth0|other")
        self.boat = create_boat(account=self.owner, name="Aurora")

    def test_owner_cannot_change_through_model_save(self):
        self.boat.owner = self.other
        with self.assertRaises(IntegrityError), transaction.atomic():
            self.boat.save()
        self.boat.refresh_from_db()
        self.assertEqual(self.boat.owner_id, self.owner.pk)

    def test_owner_cannot_change_through_bulk_updates(self):
        with self.assertRaises(IntegrityError), transaction.atomic():
            Boat.objects.filter(pk=self.boat.pk).update(owner=self.other)
        self.boat.owner = self.other
        with self.assertRaises(IntegrityError), transaction.atomic():
            Boat.objects.bulk_update([self.boat], ["owner"])
        self.assertEqual(get_boat(account=self.owner, boat_id=self.boat.id).owner_id, self.owner.pk)
        self.assertFalse(list_boats(account=self.other).exists())

    def test_same_owner_can_rename_without_transferring_ownership(self):
        self.boat.name = "Renamed Aurora"
        self.boat.save()
        self.boat.refresh_from_db()
        self.assertEqual(self.boat.name, "Renamed Aurora")
        self.assertEqual(self.boat.owner_id, self.owner.pk)
