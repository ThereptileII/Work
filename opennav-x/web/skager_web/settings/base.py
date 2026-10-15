from pathlib import Path


BASE_DIR = Path(__file__).resolve().parents[2]

INSTALLED_APPS = [
    "django.contrib.contenttypes",
    "django.contrib.sessions",
    "django.contrib.staticfiles",
    "skager_web.accounts",
    "skager_web.core",
    "skager_web.marketing",
    "skager_web.boats",
]
MIDDLEWARE = [
    "django.middleware.security.SecurityMiddleware",
    "django.contrib.sessions.middleware.SessionMiddleware",
    "django.middleware.csrf.CsrfViewMiddleware",
    "django.middleware.clickjacking.XFrameOptionsMiddleware",
    "skager_web.marketing.middleware.MarketingSecurityHeaders",
]
ROOT_URLCONF = "skager_web.urls"
TEMPLATES = [{
    "BACKEND": "django.template.backends.django.DjangoTemplates",
    "DIRS": [],
    "APP_DIRS": True,
    "OPTIONS": {"context_processors": [
        "django.template.context_processors.request",
        "skager_web.marketing.assets.marketing_asset_versions",
        "skager_web.marketing.assets.site_status",
    ]},
}]
# Public checkout, accounts and downloads open together at launch.
MARKETING_PUBLIC_ACCESS_OPEN = True
# The seller named in the legal documents. Fill in before payments open (SCRUM-61/62/63).
MARKETING_OPERATOR = {
    "name": "[Operator legal name]",
    "org_number": "[organisation number]",
    "address": "[registered address], Sweden",
    "email": "[contact email]",
}
MARKETING_LEGAL_VERSION = "Draft 0.1 · 15 October 2026 · for legal review"
STATIC_URL = "/static/"
USE_TZ = True
TIME_ZONE = "UTC"
DEFAULT_AUTO_FIELD = "django.db.models.BigAutoField"
SESSION_ENGINE = "django.contrib.sessions.backends.db"
SESSION_COOKIE_AGE = 1_800
SESSION_COOKIE_HTTPONLY = True
SESSION_COOKIE_SAMESITE = "Lax"

# Auth routes exist but return 404 until an environment explicitly enables and
# fully configures the provider. Redirect destinations are fixed server config.
OIDC_AUTH_ENABLED = False
OIDC_ISSUER = ""
OIDC_ALLOWED_ISSUERS = ()
OIDC_CLIENT_ID = ""
OIDC_CLIENT_SECRET = ""
OIDC_AUTHORIZATION_ENDPOINT = ""
OIDC_TOKEN_ENDPOINT = ""
OIDC_JWKS_URI = ""
OIDC_CALLBACK_URL = ""
OIDC_SUCCESS_URL = "/"
OIDC_LOGOUT_URL = "/"
OIDC_HTTP_TIMEOUT_SECONDS = 5
OIDC_CLOCK_SKEW_SECONDS = 30
OIDC_ATTEMPT_AGE_SECONDS = 300
OIDC_TOKEN_RESPONSE_MAX_BYTES = 32_768
OIDC_JWKS_RESPONSE_MAX_BYTES = 262_144
OIDC_ID_TOKEN_MAX_BYTES = 16_384
OIDC_JWKS_MAX_KEYS = 10
OIDC_STATE_MAX_CHARS = 128
OIDC_CODE_MAX_CHARS = 2_048

# Auth0 is the identity provider. There is no local password model.
