from django.conf import settings
from django.http import Http404, HttpResponseBadRequest
from django.shortcuts import redirect
from django.views.decorators.http import require_GET, require_POST

from skager_web.core.models import Account

from .oidc import (
    OidcAuthenticationError,
    begin_authorization,
    consume_authorization,
    exchange_and_validate,
)


ACCOUNT_SESSION_KEY = "account_id"


def _require_enabled() -> None:
    if not settings.OIDC_AUTH_ENABLED:
        raise Http404


@require_GET
def login(request):
    _require_enabled()
    request.session.cycle_key()
    request.session.set_expiry(settings.SESSION_COOKIE_AGE)
    request.session.save()
    _, authorization_url = begin_authorization(request.session.session_key)
    return redirect(authorization_url)


@require_GET
def callback(request):
    _require_enabled()
    returned_state = request.GET.get("state", "")
    code = request.GET.get("code", "")
    if not request.session.session_key or not returned_state or not code:
        return HttpResponseBadRequest("Authentication failed.")
    if len(returned_state) > settings.OIDC_STATE_MAX_CHARS or len(code) > settings.OIDC_CODE_MAX_CHARS:
        return HttpResponseBadRequest("Authentication failed.")

    try:
        pending = consume_authorization(request.session.session_key, returned_state)
        claims = exchange_and_validate(code, pending)
        issuer = claims["iss"]
        subject = claims["sub"]
        if not isinstance(issuer, str) or len(issuer) > 512:
            raise ValueError("invalid issuer")
        if not isinstance(subject, str) or not subject or len(subject) > 255:
            raise ValueError("invalid subject")
        contact_email = claims.get("email", "")
        display_name = claims.get("name", "")
        if not isinstance(contact_email, str) or len(contact_email) > 254:
            contact_email = ""
        if not isinstance(display_name, str) or len(display_name) > 200:
            display_name = ""
        account, _ = Account.objects.get_or_create(
            issuer=issuer,
            subject=subject,
            defaults={
                "contact_email": contact_email,
                "display_name": display_name,
            },
        )
    except (KeyError, TypeError, ValueError, OidcAuthenticationError):
        return HttpResponseBadRequest("Authentication failed.")

    request.session.flush()
    request.session[ACCOUNT_SESSION_KEY] = str(account.id)
    request.session.set_expiry(settings.SESSION_COOKIE_AGE)
    return redirect(settings.OIDC_SUCCESS_URL)


@require_POST
def logout(request):
    _require_enabled()
    request.session.flush()
    return redirect(settings.OIDC_LOGOUT_URL)
