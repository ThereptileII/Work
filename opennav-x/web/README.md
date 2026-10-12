# SKAGER web application and closed beta-site preview

This is the Django modular monolith selected in
`docs/architecture/public-beta-web-commerce.md`: the database foundation,
default-disabled Auth0/OIDC integration, and informational marketing/support
pages now share one application. There is no enabled checkout, entitlement
provisioning, public download service, ticket intake or upload endpoint.

The owner's 2026-10-12 instruction authorizes website work in parallel with
software qualification. Jira remains the backlog. Public access stays closed
until the complete readiness review and explicit GO; this preview is not that
review. Legal and privacy pages are visibly review drafts.

## Focused page checks and private preview

```sh
DJANGO_SETTINGS_MODULE=skager_web.settings.preview python manage.py test \
  skager_web.marketing.tests \
  skager_web.accounts.tests.test_oidc_flow.OidcConfigurationTests --noinput
python manage.py runserver 127.0.0.1:8788 --settings=skager_web.settings.development --noreload
```

These page/configuration tests need no database or external service. Support
search uses bounded, escaped GET input and progressively enhanced local
filtering. Page responses use a restrictive content policy; no analytics,
remote fonts, vendor scripts, payment forms or credentials are included.

`tools/export_preview.py --output /absolute/fresh/dist --source-commit FULL_SHA`
exports only 20 allowlisted informational pages and owned static assets from
these same Django templates. It never exports account/backend source or secrets.
The separately hosted owner-private Sites preview is a static design review,
not a replacement for the selected Django/PostgreSQL/Render production backend.
Its account/download pages explain that access is closed. The chart illustration
is explicitly labelled and is not a product screenshot or navigable chart.
Provenance records the exact website source revision and asset hashes.

Dependencies are pinned to Django 5.2.17 LTS and psycopg 3.3.6. The versions
were checked against the official Django 5.2 release notes and PyPI project
records on 2026-09-30.

Run model and migration drift checks from this directory:

```sh
python -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python manage.py check
.venv/bin/python manage.py makemigrations --check --dry-run
```

The integrity suite must use a disposable PostgreSQL database. It deliberately
fails instead of skipping on another backend:

```sh
export DJANGO_SETTINGS_MODULE=skager_web.settings.test_postgres
export TEST_POSTGRES_DB=scrum94_schema
export TEST_POSTGRES_USER=scrum94
export TEST_POSTGRES_SOCKET=/path/to/private/postgres/socket
export TEST_POSTGRES_PORT=55494
.venv/bin/python manage.py test skager_web.core.tests
```

For a disposable CI PostgreSQL service, set `TEST_POSTGRES_HOST=127.0.0.1` and
`TEST_POSTGRES_PASSWORD` instead; `TEST_POSTGRES_SOCKET` remains the preferred
local Unix-socket option. These variables are read only by the test settings.
SQLite in the development settings supports local model and migration checks
only. It is not integrity acceptance evidence. Keep PostgreSQL integrity and
authentication CI independent of native navigation builds.
