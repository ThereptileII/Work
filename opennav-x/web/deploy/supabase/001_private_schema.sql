-- SKAGER server-only schema bootstrap. Inspect the selected project first.
-- This creates no users, credentials, vessel records, tables or public policies.
-- Existing schemas are refused rather than adopted. Retain this transaction's
-- result before any later migrations; it is not the Django migration ledger.
BEGIN;
DO $$
BEGIN
  IF EXISTS (SELECT 1 FROM pg_namespace WHERE nspname = 'skager_private') THEN
    RAISE EXCEPTION 'skager_private already exists; inspect its ownership and grants before continuing';
  END IF;
END;
$$;
CREATE SCHEMA skager_private;
REVOKE ALL ON SCHEMA skager_private FROM PUBLIC, anon, authenticated;
ALTER DEFAULT PRIVILEGES IN SCHEMA skager_private
  REVOKE ALL ON TABLES FROM PUBLIC, anon, authenticated;
ALTER DEFAULT PRIVILEGES IN SCHEMA skager_private
  REVOKE ALL ON SEQUENCES FROM PUBLIC, anon, authenticated;
ALTER DEFAULT PRIVILEGES IN SCHEMA skager_private
  REVOKE ALL ON FUNCTIONS FROM PUBLIC, anon, authenticated;
COMMIT;

-- Verify no client-facing role can use the namespace.
SELECT n.nspname, pg_get_userbyid(n.nspowner) AS owner,
       has_schema_privilege('anon', n.oid, 'USAGE') AS anonymous_usage,
       has_schema_privilege('authenticated', n.oid, 'USAGE') AS authenticated_usage
FROM pg_namespace AS n WHERE n.nspname = 'skager_private';
