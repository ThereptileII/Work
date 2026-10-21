-- SKAGER Cloud: pairing the SKAGER app with an account, device tokens, the update-feed
-- entitlement and boat telemetry (latest snapshot plus a one-minute history kept 30 days).
-- Apply after store.sql (and store-steps.sql / store-guide.sql), as the database owner,
-- in one transaction. Additive: existing tables and functions are unchanged.
-- Private tables are never exposed; only service_role may execute the RPCs.
begin;

-- ---------------------------------------------------------------- tables
-- A pairing request: the app holds the secret device code (only its hash is stored)
-- and shows the short user code, which the customer approves while logged in.
create table skager_private.cloud_links (
  device_hash text primary key check (device_hash ~ '^[a-f0-9]{64}$'),
  user_code text not null unique check (user_code ~ '^[BCDFGHJKLMNPQRSTVWXZ2-9]{8}$'),
  device_name text not null check (char_length(device_name) between 1 and 60 and device_name !~ '[[:cntrl:]]'),
  platform text not null check (platform in ('windows','linux')),
  app_version text not null check (app_version ~ '^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$'),
  status text not null default 'pending' check (status in ('pending','approved','denied')),
  account_id uuid references auth.users(id) on delete cascade,
  created_at timestamptz not null default now(),
  expires_at timestamptz not null default (now() + interval '10 minutes'),
  check ((status = 'pending') = (account_id is null))
);
-- One row per paired SKAGER installation. The token is stored only as its SHA-256.
create table skager_private.cloud_devices (
  id uuid primary key default gen_random_uuid(),
  account_id uuid not null references auth.users(id) on delete cascade,
  token_hash text not null unique check (token_hash ~ '^[a-f0-9]{64}$'),
  name text not null check (char_length(name) between 1 and 60 and name !~ '[[:cntrl:]]'),
  platform text not null check (platform in ('windows','linux')),
  app_version text not null check (app_version ~ '^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$'),
  telemetry_enabled boolean not null default false,
  created_at timestamptz not null default now(),
  last_seen_at timestamptz not null default now(),
  revoked_at timestamptz
);
create index cloud_devices_account on skager_private.cloud_devices(account_id, created_at);
-- items: {"navigation.sog": {"v": 5.2, "age": 0.4, "q": "live", "src": "..."}, ...}
-- The boat clock is never trusted: observation time = received_at - age.
create table skager_private.cloud_telemetry_latest (
  device_id uuid primary key references skager_private.cloud_devices(id) on delete cascade,
  received_at timestamptz not null default now(),
  items jsonb not null check (jsonb_typeof(items) = 'object' and pg_column_size(items) <= 16384)
);
-- First usable sample in each minute; values only ({"navigation.sog": 5.2, ...}).
create table skager_private.cloud_telemetry_history (
  device_id uuid not null references skager_private.cloud_devices(id) on delete cascade,
  minute timestamptz not null check (minute = date_trunc('minute', minute)),
  items jsonb not null check (jsonb_typeof(items) = 'object'),
  primary key (device_id, minute)
);

revoke all on skager_private.cloud_links, skager_private.cloud_devices, skager_private.cloud_telemetry_latest,
  skager_private.cloud_telemetry_history from public, anon, authenticated, service_role;
alter table skager_private.cloud_links enable row level security;
alter table skager_private.cloud_devices enable row level security;
alter table skager_private.cloud_telemetry_latest enable row level security;
alter table skager_private.cloud_telemetry_history enable row level security;

-- ---------------------------------------------------------------- helpers (private)
-- SKAGER Cloud is active while Stripe keeps the subscription live. A failed renewal
-- (past_due) keeps access for 7 days after the paid period so a card update is enough.
create function skager_private.cloud_status(p_email text) returns jsonb
language sql stable set search_path = '' as $$
  select coalesce((select jsonb_build_object(
      'active', s.status in ('active','trialing')
        or (s.status = 'past_due' and coalesce(s.current_period_end, now()) > now() - interval '7 days'),
      'status', s.status, 'current_period_end', s.current_period_end, 'cancel_at_period_end', s.cancel_at_period_end)
    from skager_private.store_subscriptions s where s.email = p_email
    order by (s.status in ('active','trialing','past_due')) desc, s.updated_at desc limit 1),
    jsonb_build_object('active', false, 'status', null, 'current_period_end', null, 'cancel_at_period_end', false));
$$;
create function skager_private.cloud_device_json(d skager_private.cloud_devices) returns jsonb
language sql stable set search_path = '' as $$
  select jsonb_build_object('id', d.id, 'name', d.name, 'platform', d.platform, 'app_version', d.app_version,
    'telemetry_enabled', d.telemetry_enabled, 'created_at', d.created_at, 'last_seen_at', d.last_seen_at,
    'last_data_at', (select l.received_at from skager_private.cloud_telemetry_latest l where l.device_id = d.id));
$$;

-- ---------------------------------------------------------------- pairing
create function public.cloud_link_start(p_device_hash text, p_user_code text, p_name text, p_platform text,
  p_version text) returns jsonb
language plpgsql security definer set search_path = '' as $$
begin
  delete from skager_private.cloud_links where expires_at <= now();
  insert into skager_private.cloud_links(device_hash, user_code, device_name, platform, app_version)
    values (p_device_hash, p_user_code, btrim(p_name), p_platform, p_version);
  return jsonb_build_object('state', 'pending',
    'expires_at', (select expires_at from skager_private.cloud_links where device_hash = p_device_hash));
exception when unique_violation then
  -- A colliding user code (or a replayed device code) is simply retried by the caller.
  return jsonb_build_object('state', 'retry');
end $$;

-- What the logged-in customer sees before approving: never the device secret.
create function public.cloud_link_find(p_account_id uuid, p_user_code text) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select jsonb_build_object('state', 'pending', 'name', l.device_name, 'platform', l.platform,
      'app_version', l.app_version, 'expires_at', l.expires_at)
    from skager_private.cloud_links l
    join skager_private.store_customers c on c.account_id = p_account_id and c.status = 'active'
    where l.user_code = p_user_code and l.status = 'pending' and l.expires_at > now()),
    jsonb_build_object('state', 'unknown'));
$$;

create function public.cloud_link_decide(p_account_id uuid, p_user_code text, p_approve boolean) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.cloud_links;
begin
  if p_approve is null or not exists(select 1 from skager_private.store_customers c
      where c.account_id = p_account_id and c.status = 'active') then
    return jsonb_build_object('state', 'unknown');
  end if;
  update skager_private.cloud_links set status = case when p_approve then 'approved' else 'denied' end,
      account_id = p_account_id, expires_at = now() + interval '10 minutes'
    where user_code = p_user_code and status = 'pending' and expires_at > now() returning * into v;
  if v.device_hash is null then return jsonb_build_object('state', 'unknown'); end if;
  return jsonb_build_object('state', v.status, 'name', v.device_name);
end $$;

-- The app polls with its device code. Approval is exchanged exactly once for a device;
-- the caller generated the token and passes only its hash.
create function public.cloud_link_claim(p_device_hash text, p_token_hash text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.cloud_links; d skager_private.cloud_devices;
begin
  select * into v from skager_private.cloud_links where device_hash = p_device_hash for update;
  if v.device_hash is null or v.expires_at <= now() then
    delete from skager_private.cloud_links where device_hash = p_device_hash;
    return jsonb_build_object('state', 'expired');
  end if;
  if v.status = 'pending' then return jsonb_build_object('state', 'pending'); end if;
  delete from skager_private.cloud_links where device_hash = p_device_hash;
  if v.status = 'denied' or not exists(select 1 from skager_private.store_customers c
      where c.account_id = v.account_id and c.status = 'active') then
    return jsonb_build_object('state', 'denied');
  end if;
  insert into skager_private.cloud_devices(account_id, token_hash, name, platform, app_version)
    values (v.account_id, p_token_hash, v.device_name, v.platform, v.app_version) returning * into d;
  return jsonb_build_object('state', 'approved', 'device_id', d.id);
end $$;

-- ---------------------------------------------------------------- device requests
-- Resolves a device token. Revoked devices, revoked customers and deleted/banned
-- accounts never resolve. last_seen_at is written at most once a minute.
create function public.cloud_device_auth(p_token_hash text, p_version text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare d skager_private.cloud_devices; v_email text;
begin
  select dv.* into d from skager_private.cloud_devices dv
    join auth.users u on u.id = dv.account_id
    join skager_private.store_customers c on c.account_id = u.id and c.status = 'active'
    where dv.token_hash = p_token_hash and dv.revoked_at is null
      and u.deleted_at is null and (u.banned_until is null or u.banned_until <= now());
  if d.id is null then return jsonb_build_object('state', 'unknown'); end if;
  select c.email into v_email from skager_private.store_customers c where c.account_id = d.account_id;
  if d.last_seen_at < now() - interval '1 minute'
      or (p_version is not null and p_version ~ '^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$' and p_version <> d.app_version) then
    update skager_private.cloud_devices set last_seen_at = now(),
        app_version = case when p_version ~ '^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$' then p_version else app_version end
      where id = d.id;
  end if;
  return jsonb_build_object('state', 'active', 'device_id', d.id, 'account_id', d.account_id, 'email', v_email,
    'name', d.name, 'cloud', skager_private.cloud_status(v_email));
end $$;

-- The app removes its own pairing (Unlink in SKAGER).
create function public.cloud_device_forget(p_device_id uuid) returns void
language sql security definer set search_path = '' as $$
  update skager_private.cloud_devices set revoked_at = now() where id = p_device_id and revoked_at is null;
  delete from skager_private.cloud_telemetry_latest where device_id = p_device_id;
  delete from skager_private.cloud_telemetry_history where device_id = p_device_id;
$$;

-- Called only after cloud_device_auth reported an active subscription. One upload per
-- 5 s per device; the first usable sample of each minute also enters the history.
create function public.cloud_telemetry_put(p_device_id uuid, p_items jsonb) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_last timestamptz; v_values jsonb;
begin
  if p_items is null or jsonb_typeof(p_items) <> 'object' then raise exception 'Invalid telemetry'; end if;
  select received_at into v_last from skager_private.cloud_telemetry_latest where device_id = p_device_id for update;
  if v_last is not null and v_last > now() - interval '5 seconds' then
    return jsonb_build_object('state', 'throttled');
  end if;
  insert into skager_private.cloud_telemetry_latest(device_id, received_at, items) values (p_device_id, now(), p_items)
    on conflict (device_id) do update set received_at = excluded.received_at, items = excluded.items;
  update skager_private.cloud_devices set telemetry_enabled = true where id = p_device_id and not telemetry_enabled;
  select jsonb_object_agg(e.key, e.value->'v') into v_values from jsonb_each(p_items) e
    where e.value->>'q' in ('live','aging','estimated') and jsonb_typeof(e.value->'v') = 'number';
  if v_values is not null then
    insert into skager_private.cloud_telemetry_history(device_id, minute, items)
      values (p_device_id, date_trunc('minute', now()), v_values) on conflict do nothing;
  end if;
  return jsonb_build_object('state', 'stored');
end $$;

-- The app reports that sharing was switched off; the latest snapshot is removed so
-- the web never shows a boat as sharing when it is not. History ages out normally.
create function public.cloud_telemetry_stop(p_device_id uuid) returns void
language sql security definer set search_path = '' as $$
  update skager_private.cloud_devices set telemetry_enabled = false where id = p_device_id;
  delete from skager_private.cloud_telemetry_latest where device_id = p_device_id;
$$;

-- ---------------------------------------------------------------- portal
create function public.cloud_overview(p_account_id uuid) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select jsonb_build_object('cloud', skager_private.cloud_status(c.email),
      'devices', coalesce((select jsonb_agg(skager_private.cloud_device_json(d) order by d.created_at)
        from skager_private.cloud_devices d where d.account_id = p_account_id and d.revoked_at is null), '[]'::jsonb))
    from skager_private.store_customers c where c.account_id = p_account_id and c.status = 'active'),
    jsonb_build_object('cloud', null, 'devices', '[]'::jsonb));
$$;

create function public.cloud_device_revoke(p_account_id uuid, p_device_id uuid) returns jsonb
language plpgsql security definer set search_path = '' as $$
begin
  if not exists(select 1 from skager_private.cloud_devices where id = p_device_id and account_id = p_account_id
      and revoked_at is null) then
    return jsonb_build_object('state', 'unknown');
  end if;
  perform public.cloud_device_forget(p_device_id);
  return jsonb_build_object('state', 'revoked');
end $$;

-- Latest snapshot of one of the customer's own devices.
create function public.cloud_boat_latest(p_account_id uuid, p_device_id uuid) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select jsonb_build_object('state', 'ok', 'device', skager_private.cloud_device_json(d),
      'received_at', l.received_at, 'items', l.items)
    from skager_private.cloud_devices d
    join skager_private.store_customers c on c.account_id = d.account_id and c.status = 'active'
    left join skager_private.cloud_telemetry_latest l on l.device_id = d.id
    where d.id = p_device_id and d.account_id = p_account_id and d.revoked_at is null),
    jsonb_build_object('state', 'unknown'));
$$;

-- History for up to four keys, averaged into at most 240 buckets per key so a
-- 30-day window still fits one bounded response: {"key": [[epoch_s, value], ...]}.
create function public.cloud_boat_series(p_account_id uuid, p_device_id uuid, p_keys text[], p_hours integer)
returns jsonb
language plpgsql stable security definer set search_path = '' as $$
declare v_bucket double precision; v_since timestamptz; v_result jsonb := '{}'::jsonb; v_key text; v_points jsonb;
begin
  if p_keys is null or cardinality(p_keys) not between 1 and 4 or p_hours is null or p_hours not between 1 and 720
      or exists(select 1 from unnest(p_keys) k where k is null or k !~ '^[a-z_]{2,20}\.[a-z_]{2,30}$') then
    raise exception 'Invalid series request';
  end if;
  if not exists(select 1 from skager_private.cloud_devices d
      join skager_private.store_customers c on c.account_id = d.account_id and c.status = 'active'
      where d.id = p_device_id and d.account_id = p_account_id and d.revoked_at is null) then
    return jsonb_build_object('state', 'unknown');
  end if;
  v_since := now() - make_interval(hours => p_hours);
  v_bucket := greatest(60, p_hours * 3600 / 240.0);
  foreach v_key in array p_keys loop
    select coalesce(jsonb_agg(jsonb_build_array(b.t, round(b.v::numeric, 4)) order by b.t), '[]'::jsonb) into v_points
      from (select (floor(extract(epoch from h.minute) / v_bucket) * v_bucket)::bigint as t,
                   avg((h.items->>v_key)::double precision) as v
              from skager_private.cloud_telemetry_history h
              where h.device_id = p_device_id and h.minute >= v_since and jsonb_typeof(h.items->v_key) = 'number'
              group by 1) b;
    v_result := v_result || jsonb_build_object(v_key, v_points);
  end loop;
  return jsonb_build_object('state', 'ok', 'bucket_seconds', v_bucket, 'series', v_result);
end $$;

-- ---------------------------------------------------------------- maintenance
create function public.cloud_job_maintenance() returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_links integer; v_history integer;
begin
  delete from skager_private.cloud_links where expires_at <= now();
  get diagnostics v_links = row_count;
  delete from skager_private.cloud_telemetry_history where minute < now() - interval '30 days';
  get diagnostics v_history = row_count;
  return jsonb_build_object('expired_links', v_links, 'pruned_history', v_history);
end $$;

-- A refunded/closed account (store_revoke) loses its devices with its sessions.
create function skager_private.cloud_customer_revoked() returns trigger
language plpgsql set search_path = '' as $$
begin
  if new.status = 'revoked' and old.status <> 'revoked' and new.account_id is not null then
    update skager_private.cloud_devices set revoked_at = now() where account_id = new.account_id and revoked_at is null;
    delete from skager_private.cloud_telemetry_latest l using skager_private.cloud_devices d
      where d.id = l.device_id and d.account_id = new.account_id;
  end if;
  return new;
end $$;
create trigger cloud_customer_revoked after update of status on skager_private.store_customers
  for each row execute function skager_private.cloud_customer_revoked();

-- ---------------------------------------------------------------- catalog copy
-- Automatic updates are a SKAGER Cloud service; Beta Access keeps every update as a
-- download. Only seed wording is replaced; an admin's own edits are kept.
update skager_private.store_products set summary = 'SKAGER for Windows with every beta update to download, your account and support.',
    updated_at = now()
  where slug = 'skager-beta-access' and summary = 'SKAGER for Windows with every beta update, your account and support.';
update skager_private.store_products set summary = 'Automatic updates, boat data and your boat online.', updated_at = now()
  where slug = 'skager-cloud' and summary = 'Boat data, online dashboard and web route planning.';

-- ---------------------------------------------------------------- grants
do $$
declare f record;
begin
  for f in select p.oid::regprocedure as sig from pg_proc p join pg_namespace n on n.oid = p.pronamespace
           where n.nspname = 'public' and p.proname like 'cloud\_%' loop
    execute format('revoke all on function %s from public, anon, authenticated', f.sig);
    execute format('grant execute on function %s to service_role', f.sig);
  end loop;
  for f in select p.oid::regprocedure as sig from pg_proc p join pg_namespace n on n.oid = p.pronamespace
           where n.nspname = 'skager_private' and p.proname like 'cloud\_%' loop
    execute format('revoke all on function %s from public, anon, authenticated, service_role', f.sig);
  end loop;
end $$;
commit;
