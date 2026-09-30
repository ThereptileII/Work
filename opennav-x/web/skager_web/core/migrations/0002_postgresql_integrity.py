from django.db import migrations


CREATE_SQL = r"""
CREATE OR REPLACE FUNCTION skager_reject_changed_columns()
RETURNS trigger LANGUAGE plpgsql AS $$
DECLARE
    column_name text;
BEGIN
    FOREACH column_name IN ARRAY TG_ARGV LOOP
        IF to_jsonb(NEW) -> column_name IS DISTINCT FROM to_jsonb(OLD) -> column_name THEN
            RAISE EXCEPTION '%% is immutable on %%', column_name, TG_TABLE_NAME
                USING ERRCODE = '23514';
        END IF;
    END LOOP;
    RETURN NEW;
END;
$$;

CREATE OR REPLACE FUNCTION skager_reject_mutation()
RETURNS trigger LANGUAGE plpgsql AS $$
BEGIN
    RAISE EXCEPTION '%% records are append-only', TG_TABLE_NAME
        USING ERRCODE = '23514';
END;
$$;

CREATE OR REPLACE FUNCTION skager_validate_binding_order()
RETURNS trigger LANGUAGE plpgsql AS $$
DECLARE
    order_environment varchar(16);
BEGIN
    SELECT environment INTO order_environment FROM core_order WHERE id = NEW.order_id;
    IF order_environment IS DISTINCT FROM NEW.environment THEN
        RAISE EXCEPTION 'provider binding environment must match its order'
            USING ERRCODE = '23514';
    END IF;
    RETURN NEW;
END;
$$;

CREATE OR REPLACE FUNCTION skager_validate_entitlement_order()
RETURNS trigger LANGUAGE plpgsql AS $$
DECLARE
    bound_account uuid;
    bound_environment varchar(16);
    bound_product varchar(100);
BEGIN
    SELECT account_id, environment, product_code
      INTO bound_account, bound_environment, bound_product
      FROM core_order WHERE id = NEW.order_id;
    IF (bound_account, bound_environment, bound_product)
       IS DISTINCT FROM (NEW.account_id, NEW.environment, NEW.product_code) THEN
        RAISE EXCEPTION 'entitlement owner, environment, and product must match its order'
            USING ERRCODE = '23514';
    END IF;
    RETURN NEW;
END;
$$;

CREATE OR REPLACE FUNCTION skager_validate_download_owner()
RETURNS trigger LANGUAGE plpgsql AS $$
DECLARE
    entitlement_account uuid;
BEGIN
    IF NEW.entitlement_id IS NULL THEN
        RETURN NEW;
    END IF;
    SELECT account_id INTO entitlement_account FROM core_entitlement WHERE id = NEW.entitlement_id;
    IF entitlement_account IS DISTINCT FROM NEW.account_id THEN
        RAISE EXCEPTION 'download account must own its entitlement'
            USING ERRCODE = '23514';
    END IF;
    RETURN NEW;
END;
$$;

CREATE TRIGGER account_identity_immutable
BEFORE UPDATE ON core_account
FOR EACH ROW EXECUTE FUNCTION skager_reject_changed_columns('issuer', 'subject');

CREATE TRIGGER order_terms_immutable
BEFORE UPDATE ON core_order
FOR EACH ROW EXECUTE FUNCTION skager_reject_changed_columns(
    'account_id', 'environment', 'product_code', 'amount_minor', 'currency', 'created_at'
);

CREATE TRIGGER binding_matches_order
BEFORE INSERT OR UPDATE ON core_providertransactionbinding
FOR EACH ROW EXECUTE FUNCTION skager_validate_binding_order();

CREATE TRIGGER binding_immutable
BEFORE UPDATE OR DELETE ON core_providertransactionbinding
FOR EACH ROW EXECUTE FUNCTION skager_reject_mutation();

CREATE TRIGGER verified_event_receipt_immutable
BEFORE UPDATE ON core_verifiedeventinbox
FOR EACH ROW EXECUTE FUNCTION skager_reject_changed_columns(
    'environment', 'provider', 'provider_event_id', 'payload_digest',
    'normalized_payload', 'signature_verified_at', 'received_at'
);

CREATE TRIGGER entitlement_matches_order
BEFORE INSERT OR UPDATE ON core_entitlement
FOR EACH ROW EXECUTE FUNCTION skager_validate_entitlement_order();

CREATE TRIGGER entitlement_binding_immutable
BEFORE UPDATE ON core_entitlement
FOR EACH ROW EXECUTE FUNCTION skager_reject_changed_columns(
    'account_id', 'order_id', 'environment', 'product_code', 'created_at'
);

CREATE TRIGGER entitlement_effect_append_only
BEFORE UPDATE OR DELETE ON core_entitlementeffect
FOR EACH ROW EXECUTE FUNCTION skager_reject_mutation();

CREATE TRIGGER release_artifact_append_only
BEFORE UPDATE OR DELETE ON core_releaseartifact
FOR EACH ROW EXECUTE FUNCTION skager_reject_mutation();

CREATE TRIGGER release_publication_append_only
BEFORE UPDATE OR DELETE ON core_release
FOR EACH ROW EXECUTE FUNCTION skager_reject_mutation();

CREATE TRIGGER download_owner_matches
BEFORE INSERT OR UPDATE ON core_downloadaudit
FOR EACH ROW EXECUTE FUNCTION skager_validate_download_owner();

CREATE TRIGGER download_audit_append_only
BEFORE UPDATE OR DELETE ON core_downloadaudit
FOR EACH ROW EXECUTE FUNCTION skager_reject_mutation();

CREATE TRIGGER audit_event_append_only
BEFORE UPDATE OR DELETE ON core_auditevent
FOR EACH ROW EXECUTE FUNCTION skager_reject_mutation();
"""

DROP_SQL = r"""
DROP TRIGGER IF EXISTS audit_event_append_only ON core_auditevent;
DROP TRIGGER IF EXISTS download_audit_append_only ON core_downloadaudit;
DROP TRIGGER IF EXISTS download_owner_matches ON core_downloadaudit;
DROP TRIGGER IF EXISTS release_publication_append_only ON core_release;
DROP TRIGGER IF EXISTS release_artifact_append_only ON core_releaseartifact;
DROP TRIGGER IF EXISTS entitlement_effect_append_only ON core_entitlementeffect;
DROP TRIGGER IF EXISTS entitlement_binding_immutable ON core_entitlement;
DROP TRIGGER IF EXISTS entitlement_matches_order ON core_entitlement;
DROP TRIGGER IF EXISTS verified_event_receipt_immutable ON core_verifiedeventinbox;
DROP TRIGGER IF EXISTS binding_immutable ON core_providertransactionbinding;
DROP TRIGGER IF EXISTS binding_matches_order ON core_providertransactionbinding;
DROP TRIGGER IF EXISTS order_terms_immutable ON core_order;
DROP TRIGGER IF EXISTS account_identity_immutable ON core_account;
DROP FUNCTION IF EXISTS skager_validate_download_owner();
DROP FUNCTION IF EXISTS skager_validate_entitlement_order();
DROP FUNCTION IF EXISTS skager_validate_binding_order();
DROP FUNCTION IF EXISTS skager_reject_mutation();
DROP FUNCTION IF EXISTS skager_reject_changed_columns();
"""


def create_postgresql_integrity(apps, schema_editor):
    if schema_editor.connection.vendor == "postgresql":
        schema_editor.execute(CREATE_SQL)


def drop_postgresql_integrity(apps, schema_editor):
    if schema_editor.connection.vendor == "postgresql":
        schema_editor.execute(DROP_SQL)


class Migration(migrations.Migration):
    dependencies = [("core", "0001_initial_schema")]
    operations = [migrations.RunPython(create_postgresql_integrity, drop_postgresql_integrity)]
