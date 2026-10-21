import uuid

import django.db.models.deletion
from django.db import migrations, models
from django.db.models.functions import Length
from django.db.models.lookups import LessThanOrEqual


def protect_owner(apps, schema_editor):
    if schema_editor.connection.vendor == "postgresql":
        # This function is installed by core.0002_postgresql_integrity.
        schema_editor.execute("""
            CREATE TRIGGER boat_owner_immutable
            BEFORE UPDATE ON boats_boat
            FOR EACH ROW EXECUTE FUNCTION skager_reject_changed_columns('owner_id');
        """)


def unprotect_owner(apps, schema_editor):
    if schema_editor.connection.vendor == "postgresql":
        schema_editor.execute("DROP TRIGGER IF EXISTS boat_owner_immutable ON boats_boat;")


class Migration(migrations.Migration):
    initial = True
    dependencies = [("core", "0003_close_reviewed_integrity_gaps")]
    operations = [
        migrations.CreateModel(
            name="Boat",
            fields=[
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("name", models.CharField(max_length=80)),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("owner", models.ForeignKey(
                    on_delete=django.db.models.deletion.PROTECT,
                    related_name="boats", to="core.account",
                )),
            ],
            options={
                "ordering": ("created_at", "id"),
                "constraints": [
                    models.CheckConstraint(condition=models.Q(name__regex=r"\S"), name="boat_name_nonblank"),
                    models.CheckConstraint(
                        condition=LessThanOrEqual(Length("name"), 80), name="boat_name_length_valid"
                    ),
                ],
            },
        ),
        migrations.RunPython(protect_owner, unprotect_owner),
    ]
