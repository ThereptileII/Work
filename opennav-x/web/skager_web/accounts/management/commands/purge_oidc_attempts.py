from django.core.management.base import BaseCommand
from django.utils import timezone

from skager_web.accounts.models import OidcAuthorizationAttempt


class Command(BaseCommand):
    help = "Delete expired OIDC authorization attempts."

    def handle(self, *args, **options):
        deleted, _ = OidcAuthorizationAttempt.objects.filter(expires_at__lte=timezone.now()).delete()
        self.stdout.write(f"Deleted {deleted} expired OIDC authorization attempt(s).")
