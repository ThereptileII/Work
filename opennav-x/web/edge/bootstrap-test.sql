-- Disposable CI database ONLY. Never apply this Supabase Auth stand-in remotely.
DO $$ BEGIN
  IF current_database() <> 'scrum94_schema' THEN RAISE EXCEPTION 'Disposable database required'; END IF;
END $$;
CREATE ROLE anon NOLOGIN;
CREATE ROLE authenticated NOLOGIN;
CREATE ROLE service_role NOLOGIN;
CREATE SCHEMA auth;
CREATE TABLE auth.users (
  id uuid PRIMARY KEY,
  email text,
  email_confirmed_at timestamptz,
  banned_until timestamptz,
  deleted_at timestamptz
);
