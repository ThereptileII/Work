# SKAGER web database foundation

This directory contains the SCRUM-94 schema foundation for the Django modular
monolith selected in `docs/architecture/public-beta-web-commerce.md`. It has no
public routes, login, checkout, webhook endpoint, download service, support
workflow, worker, or vendor adapter.

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
only. It is not integrity acceptance evidence.
