from pathlib import Path


BASE_DIR = Path(__file__).resolve().parents[2]

INSTALLED_APPS = [
    "django.contrib.contenttypes",
    "django.contrib.sessions",
    "skager_web.accounts",
    "skager_web.core",
]
MIDDLEWARE = [
    "django.middleware.security.SecurityMiddleware",
    "django.contrib.sessions.middleware.SessionMiddleware",
    "django.middleware.csrf.CsrfViewMiddleware",
    "django.middleware.clickjacking.XFrameOptionsMiddleware",
]
ROOT_URLCONF = "skager_web.urls"
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
