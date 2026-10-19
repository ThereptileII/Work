-- Apply after auth.sql. No browser role can read/write private boat records.
BEGIN;
CREATE SCHEMA IF NOT EXISTS skager_private;
REVOKE ALL ON SCHEMA skager_private FROM PUBLIC, anon, authenticated;
CREATE TABLE IF NOT EXISTS skager_private.portal_boats (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  account_id uuid NOT NULL REFERENCES auth.users(id) ON DELETE RESTRICT,
  name text NOT NULL CHECK (
    char_length(name) BETWEEN 1 AND 80 AND name = btrim(name)
    AND name ~ '[^[:space:]]' AND name !~ '[[:cntrl:]]'
  ),
  created_at timestamptz NOT NULL DEFAULT now()
);
ALTER TABLE skager_private.portal_boats ENABLE ROW LEVEL SECURITY;
REVOKE ALL ON skager_private.portal_boats FROM PUBLIC, anon, authenticated, service_role;
CREATE INDEX IF NOT EXISTS portal_boats_owner ON skager_private.portal_boats(account_id, created_at, id);

CREATE OR REPLACE FUNCTION public.portal_list_boats(p_account_id uuid)
RETURNS jsonb LANGUAGE sql SECURITY DEFINER SET search_path = pg_catalog AS $$
  SELECT coalesce(jsonb_agg(jsonb_build_object(
    'id', b.id, 'name', b.name, 'created_at', b.created_at
  ) ORDER BY b.created_at, b.id), '[]'::jsonb)
  FROM skager_private.portal_boats AS b WHERE b.account_id = p_account_id;
$$;

CREATE OR REPLACE FUNCTION public.portal_create_boat(p_account_id uuid, p_name text)
RETURNS jsonb LANGUAGE plpgsql SECURITY DEFINER SET search_path = pg_catalog AS $$
DECLARE b skager_private.portal_boats;
BEGIN
  INSERT INTO skager_private.portal_boats(account_id, name) VALUES (p_account_id, btrim(p_name)) RETURNING * INTO b;
  RETURN jsonb_build_object('id', b.id, 'name', b.name, 'created_at', b.created_at);
END;
$$;
REVOKE ALL ON FUNCTION public.portal_list_boats(uuid) FROM PUBLIC, anon, authenticated;
REVOKE ALL ON FUNCTION public.portal_create_boat(uuid, text) FROM PUBLIC, anon, authenticated;
GRANT EXECUTE ON FUNCTION public.portal_list_boats(uuid) TO service_role;
GRANT EXECUTE ON FUNCTION public.portal_create_boat(uuid, text) TO service_role;
COMMIT;
