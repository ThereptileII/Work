from django.db import migrations, models
from django.db.models import Q


CREATE_SQL = r"""
CREATE OR REPLACE FUNCTION skager_validate_effect_environment()
RETURNS trigger LANGUAGE plpgsql AS $$
DECLARE
    entitlement_environment varchar(16);
    event_environment varchar(16);
BEGIN
    SELECT environment INTO entitlement_environment
      FROM core_entitlement WHERE id = NEW.entitlement_id;
    SELECT environment INTO event_environment
      FROM core_verifiedeventinbox WHERE id = NEW.event_id;
    IF entitlement_environment IS DISTINCT FROM event_environment THEN
        RAISE EXCEPTION 'entitlement effect event environment must match entitlement environment'
            USING ERRCODE = '23514';
    END IF;
    RETURN NEW;
END;
$$;

CREATE TRIGGER entitlement_effect_environment_matches
BEFORE INSERT ON core_entitlementeffect
FOR EACH ROW EXECUTE FUNCTION skager_validate_effect_environment();

CREATE TRIGGER verified_event_delete_rejected
BEFORE DELETE ON core_verifiedeventinbox
FOR EACH ROW EXECUTE FUNCTION skager_reject_mutation();
"""

DROP_SQL = r"""
DROP TRIGGER IF EXISTS verified_event_delete_rejected ON core_verifiedeventinbox;
DROP TRIGGER IF EXISTS entitlement_effect_environment_matches ON core_entitlementeffect;
DROP FUNCTION IF EXISTS skager_validate_effect_environment();
"""


def create_postgresql_integrity(apps, schema_editor):
    if schema_editor.connection.vendor == "postgresql":
        schema_editor.execute(CREATE_SQL)


def drop_postgresql_integrity(apps, schema_editor):
    if schema_editor.connection.vendor == "postgresql":
        schema_editor.execute(DROP_SQL)


class Migration(migrations.Migration):
    dependencies = [("core", "0002_postgresql_integrity")]
    operations = [
        migrations.AddConstraint(
            model_name="release",
            constraint=models.UniqueConstraint(fields=("manifest_object_key",), name="release_manifest_key_uniq"),
        ),
        migrations.AddConstraint(
            model_name="release",
            constraint=models.UniqueConstraint(fields=("signature_object_key",), name="release_signature_key_uniq"),
        ),
        migrations.AddConstraint(
            model_name="release",
            constraint=models.CheckConstraint(condition=~Q(manifest_object_key=""), name="release_manifest_key_nonempty"),
        ),
        migrations.AddConstraint(
            model_name="release",
            constraint=models.CheckConstraint(condition=~Q(signature_object_key=""), name="release_signature_key_nonempty"),
        ),
        migrations.RunPython(create_postgresql_integrity, drop_postgresql_integrity),
    ]
