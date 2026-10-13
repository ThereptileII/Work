-- SCRUM-40. Run as the database owner. Private tables are never exposed by PostgREST.
begin;
create schema if not exists skager_private;
revoke all on schema skager_private from public, anon, authenticated;
create table if not exists skager_private.auth_challenges (
  token_hash text primary key check (token_hash ~ '^[a-f0-9]{64}$'),
  email text not null check (length(email) <= 254),
  expires_at timestamptz not null default (now() + interval '10 minutes')
);
create table if not exists skager_private.auth_sessions (
  token_hash text primary key check (token_hash ~ '^[a-f0-9]{64}$'),
  subject uuid not null references auth.users(id) on delete cascade,
  created_at timestamptz not null default now(),
  expires_at timestamptz not null default (now() + interval '20 minutes')
);
create index if not exists auth_sessions_subject on skager_private.auth_sessions(subject);
create table if not exists skager_private.auth_throttles (
  bucket text primary key check (length(bucket) between 1 and 100),
  count integer not null,
  expires_at timestamptz not null
);
revoke all on all tables in schema skager_private from public, anon, authenticated;
alter table skager_private.auth_challenges enable row level security;
alter table skager_private.auth_sessions enable row level security;
alter table skager_private.auth_throttles enable row level security;

create or replace function public.skager_auth_throttle(p_keys text[], p_limit integer, p_seconds integer)
returns boolean language plpgsql security definer set search_path = '' as $$
begin
  if p_keys is null or p_limit is null or p_seconds is null or cardinality(p_keys) not between 1 and 3 or p_limit not between 1 and 100 or p_seconds not between 1 and 3600
    or exists(select 1 from unnest(p_keys) k where k is null or length(k) not between 1 and 100) then
    raise exception 'Invalid throttle configuration';
  end if;
  -- Serialize the short check/update to make limits atomic, including new buckets.
  perform pg_advisory_xact_lock(723481002);
  delete from skager_private.auth_throttles where expires_at <= now();
  if exists(select 1 from skager_private.auth_throttles where bucket = any(p_keys) and count >= p_limit) then return false; end if;
  insert into skager_private.auth_throttles as t(bucket, count, expires_at)
    select distinct k, 1, now() + make_interval(secs => p_seconds) from unnest(p_keys) k
    on conflict(bucket) do update set count = t.count + 1;
  return true;
end $$;

create or replace function public.skager_auth_challenge_create(p_hash text, p_email text)
returns boolean language plpgsql security definer set search_path = '' as $$
begin
  delete from skager_private.auth_challenges where expires_at <= now();
  insert into skager_private.auth_challenges(token_hash,email) values(p_hash,p_email);
  return true;
end $$;
create or replace function public.skager_auth_challenge_take(p_hash text)
returns table(email text) language sql security definer set search_path = '' as $$
  delete from skager_private.auth_challenges c where c.token_hash = p_hash and c.expires_at > now() returning c.email;
$$;
create or replace function public.skager_auth_session_create(p_hash text, p_subject uuid)
returns boolean language plpgsql security definer set search_path = '' as $$
begin
  if not exists(select 1 from auth.users u where u.id = p_subject and u.email_confirmed_at is not null
      and (u.banned_until is null or u.banned_until <= now()) and u.deleted_at is null) then
    raise exception 'Identity is unavailable';
  end if;
  delete from skager_private.auth_sessions where expires_at <= now();
  perform pg_advisory_xact_lock(hashtextextended(p_subject::text, 0));
  delete from skager_private.auth_sessions where token_hash in (
    select s.token_hash from skager_private.auth_sessions s where s.subject = p_subject
    order by s.created_at desc offset 4);
  insert into skager_private.auth_sessions(token_hash,subject) values(p_hash,p_subject);
  return true;
end $$;
create or replace function public.skager_auth_session_get(p_hash text)
returns table(id uuid,email text) language sql security definer set search_path = '' as $$
  select u.id,u.email::text from skager_private.auth_sessions s join auth.users u on u.id = s.subject
    where s.token_hash = p_hash and s.expires_at > now() and u.email_confirmed_at is not null
    and (u.banned_until is null or u.banned_until <= now()) and u.deleted_at is null;
$$;
create or replace function public.skager_auth_session_delete(p_hash text)
returns boolean language plpgsql security definer set search_path = '' as $$
begin
  delete from skager_private.auth_sessions where token_hash = p_hash;
  return true;
end $$;

revoke all on function public.skager_auth_throttle(text[],integer,integer),
  public.skager_auth_challenge_create(text,text), public.skager_auth_challenge_take(text),
  public.skager_auth_session_create(text,uuid), public.skager_auth_session_get(text),
  public.skager_auth_session_delete(text) from public,anon,authenticated;
grant execute on function public.skager_auth_throttle(text[],integer,integer),
  public.skager_auth_challenge_create(text,text), public.skager_auth_challenge_take(text),
  public.skager_auth_session_create(text,uuid), public.skager_auth_session_get(text),
  public.skager_auth_session_delete(text) to service_role;
commit;
