-- SKAGER Companion (SCRUM-385): phone sign-in, anchor watch state, boat events with a
-- push queue, the boat's track and the "lost contact" check.
-- Apply after cloud.sql, as the database owner, in one transaction. Additive: existing
-- tables and functions are unchanged. Only service_role may execute the RPCs.
--
-- The anchor alarm itself is decided by OpenCPN on the boat computer and keeps working
-- without internet. This schema only stores what the boat reported and queues
-- notifications; it never decides that a boat is dragging.
begin;

-- ---------------------------------------------------------------- tables
-- A one-time authorization code from the signed-in web page to the app (PKCE, RFC 7636).
-- Only the code's SHA-256 and the S256 challenge are stored.
create table skager_private.companion_grants (
  code_hash text primary key check (code_hash ~ '^[a-f0-9]{64}$'),
  account_id uuid not null references auth.users(id) on delete cascade,
  challenge text not null check (challenge ~ '^[A-Za-z0-9_-]{43}$'),
  created_at timestamptz not null default now(),
  expires_at timestamptz not null default (now() + interval '2 minutes')
);
create index companion_grants_account on skager_private.companion_grants(account_id);
-- One row per signed-in phone. The token is stored only as its SHA-256. A session
-- expires after 90 days without use; the push token belongs to exactly one session.
create table skager_private.companion_sessions (
  id uuid primary key default gen_random_uuid(),
  account_id uuid not null references auth.users(id) on delete cascade,
  token_hash text not null unique check (token_hash ~ '^[a-f0-9]{64}$'),
  name text not null check (char_length(name) between 1 and 60 and name !~ '[[:cntrl:]]'),
  platform text not null check (platform in ('android','ios')),
  app_version text not null check (app_version ~ '^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$'),
  created_at timestamptz not null default now(),
  last_seen_at timestamptz not null default now(),
  expires_at timestamptz not null default (now() + interval '90 days'),
  push_service text check (push_service in ('fcm','apns')),
  push_token text unique check (char_length(push_token) between 20 and 4096 and push_token ~ '^[A-Za-z0-9_:.-]+$'),
  push_updated_at timestamptz,
  check ((push_service is null) = (push_token is null))
);
create index companion_sessions_account on skager_private.companion_sessions(account_id, created_at);
-- Where the boat has been: one point per accepted upload (at most every 5 s, normally
-- every 15 s), kept 7 days. Only live/aging positions with a source enter the track.
create table skager_private.cloud_track (
  device_id uuid not null references skager_private.cloud_devices(id) on delete cascade,
  at timestamptz not null,
  lat double precision not null check (lat between -90 and 90),
  lon double precision not null check (lon between -180 and 180),
  sog real check (sog between 0 and 100),
  primary key (device_id, at)
);
-- Something that happened on board, or a gap the service noticed. Events reported by
-- the boat carry its own id so a retried upload is stored once.
create table skager_private.cloud_events (
  id bigint generated always as identity primary key,
  device_id uuid not null references skager_private.cloud_devices(id) on delete cascade,
  account_id uuid not null references auth.users(id) on delete cascade,
  client_id text check (client_id ~ '^[A-Za-z0-9-]{8,64}$'),
  kind text not null check (kind in ('anchor_alarm','anchor_alarm_cleared','anchor_armed','anchor_disarmed',
    'alert_critical','alert_cleared','connection_lost','connection_restored')),
  severity text not null check (severity in ('critical','warning','info')),
  title text not null check (char_length(title) between 1 and 120 and title !~ '[[:cntrl:]]'),
  detail text not null default '' check (char_length(detail) <= 300 and detail !~ '[[:cntrl:]]'),
  data jsonb not null default '{}'::jsonb check (jsonb_typeof(data) = 'object' and pg_column_size(data) <= 1024),
  occurred_at timestamptz not null,
  received_at timestamptz not null default now(),
  push_state text not null default 'none'
    check (push_state in ('none','pending','sending','sent','failed','unconfigured','expired')),
  push_attempts smallint not null default 0,
  push_claimed_at timestamptz,
  unique (device_id, client_id)
);
create index cloud_events_account on skager_private.cloud_events(account_id, id desc);
create index cloud_events_push on skager_private.cloud_events(push_state) where push_state in ('pending','sending');
-- The anchor watch as last reported with the snapshot (null: the boat did not say).
alter table skager_private.cloud_telemetry_latest add column anchor jsonb
  check (anchor is null or (jsonb_typeof(anchor) = 'object' and pg_column_size(anchor) <= 1024));

revoke all on skager_private.companion_grants, skager_private.companion_sessions, skager_private.cloud_track,
  skager_private.cloud_events from public, anon, authenticated, service_role;
alter table skager_private.companion_grants enable row level security;
alter table skager_private.companion_sessions enable row level security;
alter table skager_private.cloud_track enable row level security;
alter table skager_private.cloud_events enable row level security;

-- ---------------------------------------------------------------- helpers (private)
-- Same lock order as cloud_telemetry_put (user -> customer -> subscriptions -> device),
-- so refunds, cancellations and unlinking are serialized with every write.
create function skager_private.cloud_device_writable(p_device_id uuid) returns text
language plpgsql set search_path = '' as $$
declare v_account uuid; v_email text;
begin
  select account_id into v_account from skager_private.cloud_devices where id = p_device_id;
  perform 1 from auth.users where id = v_account and deleted_at is null
    and (banned_until is null or banned_until <= now()) for share;
  if not found then return 'revoked'; end if;
  select email into v_email from skager_private.store_customers
    where account_id = v_account and status = 'active' for share;
  if v_email is null then return 'revoked'; end if;
  perform 1 from skager_private.store_subscriptions where email = v_email order by subscription_id for share;
  if not coalesce((skager_private.cloud_status(v_email)->>'active')::boolean, false) then return 'inactive'; end if;
  perform 1 from skager_private.cloud_devices where id = p_device_id and revoked_at is null for update;
  if not found then return 'revoked'; end if;
  return 'ok';
end $$;

-- Fixed wording per kind: the boat supplies facts, never the notification text.
create function skager_private.cloud_event_text(p_kind text, p_alert text, out severity text, out title text,
  out push boolean)
language sql immutable set search_path = '' as $$
  select case p_kind when 'anchor_alarm' then 'critical' when 'alert_critical' then 'critical'
           when 'connection_lost' then 'critical' else 'info' end,
         case p_kind
           when 'anchor_alarm' then 'Anchor alarm'
           when 'anchor_alarm_cleared' then 'Anchor alarm ended'
           when 'anchor_armed' then 'Anchor watch on'
           when 'anchor_disarmed' then 'Anchor watch off'
           when 'alert_critical' then coalesce(nullif(p_alert, ''), 'Critical alert')
           when 'alert_cleared' then coalesce(nullif(p_alert, ''), 'Alert') || ' cleared'
           when 'connection_lost' then 'No contact with the boat'
           when 'connection_restored' then 'Contact with the boat restored' end,
         p_kind in ('anchor_alarm','anchor_alarm_cleared','alert_critical','connection_lost','connection_restored');
$$;

create function skager_private.cloud_event_json(e skager_private.cloud_events, p_boat text) returns jsonb
language sql stable set search_path = '' as $$
  select jsonb_build_object('id', e.id, 'device_id', e.device_id, 'boat', p_boat, 'kind', e.kind,
    'severity', e.severity, 'title', e.title, 'detail', e.detail, 'data', e.data,
    'occurred_at', e.occurred_at, 'received_at', e.received_at);
$$;

-- Active phone session lookups never resolve for a closed account or a deleted user.
create function skager_private.companion_account_active(p_account_id uuid) returns text
language sql stable set search_path = '' as $$
  select c.email from skager_private.store_customers c join auth.users u on u.id = c.account_id
    where c.account_id = p_account_id and c.status = 'active' and u.deleted_at is null
      and (u.banned_until is null or u.banned_until <= now());
$$;

-- ---------------------------------------------------------------- phone sign-in
-- The signed-in web page asks for a code bound to the app's PKCE challenge.
create function public.companion_grant_create(p_account_id uuid, p_code_hash text, p_challenge text) returns jsonb
language plpgsql security definer set search_path = '' as $$
begin
  if skager_private.companion_account_active(p_account_id) is null then return jsonb_build_object('state', 'unknown'); end if;
  delete from skager_private.companion_grants where expires_at <= now();
  if (select count(*) from skager_private.companion_grants where account_id = p_account_id) >= 5 then
    return jsonb_build_object('state', 'throttled');
  end if;
  insert into skager_private.companion_grants(code_hash, account_id, challenge) values (p_code_hash, p_account_id, p_challenge);
  return jsonb_build_object('state', 'ok');
end $$;

-- The app exchanges the code with its verifier (the caller passes S256(verifier)).
-- The code is consumed whatever the outcome, so it cannot be guessed against.
create function public.companion_grant_redeem(p_code_hash text, p_challenge text, p_token_hash text, p_name text,
  p_platform text, p_version text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare g skager_private.companion_grants; s skager_private.companion_sessions;
begin
  delete from skager_private.companion_grants where code_hash = p_code_hash returning * into g;
  if g.code_hash is null or g.expires_at <= now() or g.challenge <> p_challenge
      or skager_private.companion_account_active(g.account_id) is null then
    return jsonb_build_object('state', 'invalid');
  end if;
  insert into skager_private.companion_sessions(account_id, token_hash, name, platform, app_version)
    values (g.account_id, p_token_hash, btrim(p_name), p_platform, p_version) returning * into s;
  -- At most ten phones: the longest unused session makes room.
  delete from skager_private.companion_sessions where id in (select id from skager_private.companion_sessions
    where account_id = g.account_id order by last_seen_at desc offset 10);
  return jsonb_build_object('state', 'ok', 'session_id', s.id);
end $$;

-- Resolves a phone token; extends the 90-day window at most once a minute.
create function public.companion_session_auth(p_token_hash text, p_version text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare s skager_private.companion_sessions; v_email text;
begin
  select * into s from skager_private.companion_sessions where token_hash = p_token_hash and expires_at > now();
  if s.id is null then return jsonb_build_object('state', 'unknown'); end if;
  v_email := skager_private.companion_account_active(s.account_id);
  if v_email is null then return jsonb_build_object('state', 'unknown'); end if;
  if s.last_seen_at < now() - interval '1 minute'
      or (p_version ~ '^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$' and p_version <> s.app_version) then
    update skager_private.companion_sessions set last_seen_at = now(), expires_at = now() + interval '90 days',
        app_version = case when p_version ~ '^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$' then p_version else app_version end
      where id = s.id;
  end if;
  return jsonb_build_object('state', 'active', 'session_id', s.id, 'account_id', s.account_id, 'email', v_email,
    'name', s.name, 'push', s.push_token is not null, 'cloud', skager_private.cloud_status(v_email));
end $$;

create function public.companion_signout(p_session_id uuid) returns void
language sql security definer set search_path = '' as $$
  delete from skager_private.companion_sessions where id = p_session_id;
$$;

-- Registers (or with null, removes) this phone's push token. A token moves with the
-- phone: registering it here removes it from any older session.
create function public.companion_push_set(p_session_id uuid, p_service text, p_token text) returns jsonb
language plpgsql security definer set search_path = '' as $$
begin
  if p_token is not null then
    update skager_private.companion_sessions set push_service = null, push_token = null, push_updated_at = now()
      where push_token = p_token and id <> p_session_id;
  end if;
  update skager_private.companion_sessions set push_service = case when p_token is null then null else p_service end,
      push_token = p_token, push_updated_at = now()
    where id = p_session_id;
  return jsonb_build_object('state', case when found then 'ok' else 'unknown' end);
end $$;

-- ---------------------------------------------------------------- boat writes
-- Called right after cloud_telemetry_put stored a snapshot. Records the anchor watch
-- and notices that a boat which had gone quiet is back.
create function public.cloud_anchor_put(p_device_id uuid, p_anchor jsonb) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_device skager_private.cloud_devices; v_lost timestamptz; t record;
begin
  if p_anchor is not null and jsonb_typeof(p_anchor) <> 'object' then raise exception 'Invalid anchor'; end if;
  update skager_private.cloud_telemetry_latest set anchor = p_anchor where device_id = p_device_id;
  if not found then return jsonb_build_object('state', 'gone'); end if;
  select * into v_device from skager_private.cloud_devices where id = p_device_id and revoked_at is null;
  if v_device.id is null then return jsonb_build_object('state', 'gone'); end if;
  select max(l.occurred_at) into v_lost from skager_private.cloud_events l
    where l.device_id = p_device_id and l.kind = 'connection_lost'
      and not exists(select 1 from skager_private.cloud_events r where r.device_id = p_device_id
        and r.kind = 'connection_restored' and r.id > l.id);
  if v_lost is not null then
    select * into t from skager_private.cloud_event_text('connection_restored', null);
    insert into skager_private.cloud_events(device_id, account_id, kind, severity, title, detail, occurred_at, push_state)
      values (p_device_id, v_device.account_id, 'connection_restored', t.severity, t.title,
        'SKAGER is sending boat data again.', now(), 'pending');
    return jsonb_build_object('state', 'stored', 'restored', true);
  end if;
  return jsonb_build_object('state', 'stored', 'restored', false);
end $$;

-- Events reported by the boat (validated by the caller). Retries are stored once.
-- p_events: [{"id": "...", "kind": "...", "age": seconds, "alert": "GPS position lost",
--             "data": {"distance_m": 61.2, "radius_m": 50}}]
create function public.cloud_event_put(p_device_id uuid, p_events jsonb) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_state text; v_account uuid; e jsonb; t record; v_new integer := 0; v_detail text;
begin
  if p_events is null or jsonb_typeof(p_events) <> 'array' or jsonb_array_length(p_events) not between 1 and 20 then
    raise exception 'Invalid events';
  end if;
  v_state := skager_private.cloud_device_writable(p_device_id);
  if v_state <> 'ok' then return jsonb_build_object('state', v_state); end if;
  select account_id into v_account from skager_private.cloud_devices where id = p_device_id;
  for e in select value from jsonb_array_elements(p_events) loop
    select * into t from skager_private.cloud_event_text(e->>'kind', e->>'alert');
    if t.title is null then raise exception using errcode = 'check_violation', message = 'Invalid event kind'; end if;
    v_detail := case e->>'kind'
      when 'anchor_alarm' then 'The boat is outside the anchor watch circle.'
      when 'anchor_alarm_cleared' then 'The boat is back within the anchor watch circle.'
      else '' end;
    insert into skager_private.cloud_events(device_id, account_id, client_id, kind, severity, title, detail, data,
        occurred_at, push_state)
      values (p_device_id, v_account, e->>'id', e->>'kind', t.severity, t.title, v_detail,
        coalesce(e->'data', '{}'::jsonb), now() - make_interval(secs => (e->>'age')::double precision),
        case when t.push then 'pending' else 'none' end)
      on conflict (device_id, client_id) do nothing;
    if found then v_new := v_new + 1; end if;
  end loop;
  update skager_private.cloud_devices set telemetry_enabled = true where id = p_device_id and not telemetry_enabled;
  return jsonb_build_object('state', 'stored', 'new', v_new);
end $$;

-- Every stored snapshot with a current position adds one track point.
create function skager_private.cloud_track_point() returns trigger
language plpgsql set search_path = '' as $$
declare v_lat jsonb := new.items->'navigation.latitude'; v_lon jsonb := new.items->'navigation.longitude';
  v_sog jsonb := new.items->'navigation.sog';
begin
  if v_lat->>'q' in ('live','aging') and v_lon->>'q' in ('live','aging')
      and jsonb_typeof(v_lat->'v') = 'number' and jsonb_typeof(v_lon->'v') = 'number'
      and coalesce((v_lat->>'age')::double precision, 1e9) <= 30 and coalesce((v_lon->>'age')::double precision, 1e9) <= 30 then
    insert into skager_private.cloud_track(device_id, at, lat, lon, sog)
      values (new.device_id, new.received_at - make_interval(secs => (v_lat->>'age')::double precision),
        (v_lat->>'v')::double precision, (v_lon->>'v')::double precision,
        case when v_sog->>'q' in ('live','aging') and jsonb_typeof(v_sog->'v') = 'number'
          then (v_sog->>'v')::real end)
      on conflict do nothing;
  end if;
  return null;
end $$;
create trigger cloud_track_point after insert or update of items on skager_private.cloud_telemetry_latest
  for each row execute function skager_private.cloud_track_point();

-- Unlinking or revoking a boat computer removes everything it reported.
create function skager_private.cloud_device_removed() returns trigger
language plpgsql set search_path = '' as $$
begin
  if new.revoked_at is not null and old.revoked_at is null then
    delete from skager_private.cloud_track where device_id = new.id;
    delete from skager_private.cloud_events where device_id = new.id;
  end if;
  return new;
end $$;
create trigger cloud_device_removed after update of revoked_at on skager_private.cloud_devices
  for each row execute function skager_private.cloud_device_removed();

-- A refunded/closed account loses its phone sessions too.
create function skager_private.companion_customer_revoked() returns trigger
language plpgsql set search_path = '' as $$
begin
  if new.status = 'revoked' and old.status <> 'revoked' and new.account_id is not null then
    delete from skager_private.companion_sessions where account_id = new.account_id;
    delete from skager_private.companion_grants where account_id = new.account_id;
  end if;
  return new;
end $$;
create trigger companion_customer_revoked after update of status on skager_private.store_customers
  for each row execute function skager_private.companion_customer_revoked();

-- ---------------------------------------------------------------- push queue
-- Claims due notifications with the phones to send them to. A claim that was not
-- finished within two minutes is retried, at most three attempts; anything received
-- more than an hour ago is no longer pushed (it stays in the alarm log).
create function public.cloud_push_claim(p_limit integer) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_result jsonb;
begin
  update skager_private.cloud_events set push_state = 'expired'
    where push_state in ('pending','sending') and received_at < now() - interval '1 hour';
  update skager_private.cloud_events set push_state = 'failed'
    where push_state = 'sending' and push_claimed_at < now() - interval '2 minutes' and push_attempts >= 3;
  with due as (
    select id from skager_private.cloud_events
      where push_state = 'pending' or (push_state = 'sending' and push_claimed_at < now() - interval '2 minutes')
      order by id limit greatest(1, least(coalesce(p_limit, 20), 50)) for update skip locked),
  claimed as (
    update skager_private.cloud_events e set push_state = 'sending', push_claimed_at = now(),
        push_attempts = e.push_attempts + 1
      from due where e.id = due.id returning e.*)
  select coalesce(jsonb_agg(skager_private.cloud_event_json(c, d.name) || jsonb_build_object(
      'targets', coalesce((select jsonb_agg(jsonb_build_object('session_id', s.id, 'service', s.push_service,
          'token', s.push_token))
        from skager_private.companion_sessions s
        where s.account_id = c.account_id and s.push_token is not null and s.expires_at > now()), '[]'::jsonb))
      order by c.id), '[]'::jsonb) into v_result
    from claimed c join skager_private.cloud_devices d on d.id = c.device_id;
  return v_result;
end $$;

-- p_state: sent | failed | unconfigured. Tokens the push service rejected as no longer
-- valid are removed from their sessions.
create function public.cloud_push_done(p_event_id bigint, p_state text, p_invalid text[]) returns void
language sql security definer set search_path = '' as $$
  update skager_private.cloud_events set push_state = case
      when p_state in ('sent','unconfigured') then p_state
      when push_attempts >= 3 then 'failed' else 'pending' end
    where id = p_event_id and push_state = 'sending';
  update skager_private.companion_sessions set push_service = null, push_token = null, push_updated_at = now()
    where push_token = any(coalesce(p_invalid, '{}'::text[]));
$$;

-- A boat whose anchor watch was on and which has not reported for three minutes.
-- One "no contact" event per gap; it is not raised for a boat that stopped sharing
-- (sharing off removes the snapshot) or whose Cloud ended.
create function public.cloud_contact_check() returns integer
language plpgsql security definer set search_path = '' as $$
declare r record; v_count integer := 0;
begin
  for r in select d.id, d.account_id, l.received_at from skager_private.cloud_telemetry_latest l
      join skager_private.cloud_devices d on d.id = l.device_id and d.revoked_at is null and d.telemetry_enabled
      join skager_private.store_customers c on c.account_id = d.account_id and c.status = 'active'
      where l.anchor->>'armed' = 'true' and l.received_at < now() - interval '3 minutes'
        and l.received_at > now() - interval '1 day'
        and coalesce((skager_private.cloud_status(c.email)->>'active')::boolean, false)
        and not exists(select 1 from skager_private.cloud_events e where e.device_id = d.id
          and e.kind = 'connection_lost' and e.occurred_at >= l.received_at) loop
    insert into skager_private.cloud_events(device_id, account_id, kind, severity, title, detail, data, occurred_at,
        push_state)
      values (r.id, r.account_id, 'connection_lost', 'critical', 'No contact with the boat',
        'The anchor watch was on. SKAGER on board keeps watching and sounds its alarm there, but this phone '
          || 'cannot see it until the boat is back online.',
        jsonb_build_object('last_data_at', r.received_at), now(), 'pending');
    v_count := v_count + 1;
  end loop;
  return v_count;
end $$;

-- ---------------------------------------------------------------- phone reads
create function public.companion_boats(p_account_id uuid) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select jsonb_build_object('cloud', skager_private.cloud_status(c.email),
      'devices', coalesce((select jsonb_agg(skager_private.cloud_device_json(d) || jsonb_build_object(
          'anchor', l.anchor, 'last_event_id', (select max(e.id) from skager_private.cloud_events e where e.device_id = d.id))
          order by d.created_at)
        from skager_private.cloud_devices d left join skager_private.cloud_telemetry_latest l on l.device_id = d.id
        where d.account_id = p_account_id and d.revoked_at is null), '[]'::jsonb))
    from skager_private.store_customers c where c.account_id = p_account_id and c.status = 'active'),
    jsonb_build_object('cloud', null, 'devices', '[]'::jsonb));
$$;

create function public.companion_boat(p_account_id uuid, p_device_id uuid) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select jsonb_build_object('state', 'ok', 'device', skager_private.cloud_device_json(d),
      'received_at', l.received_at, 'items', l.items, 'anchor', l.anchor)
    from skager_private.cloud_devices d
    join skager_private.store_customers c on c.account_id = d.account_id and c.status = 'active'
    left join skager_private.cloud_telemetry_latest l on l.device_id = d.id
    where d.id = p_device_id and d.account_id = p_account_id and d.revoked_at is null),
    jsonb_build_object('state', 'unknown'));
$$;

-- Track for 1 hour to 7 days, thinned evenly to at most 1500 points:
-- {"points": [[epoch_s, lat, lon, sog|null], ...]}
create function public.companion_track(p_account_id uuid, p_device_id uuid, p_hours integer) returns jsonb
language plpgsql stable security definer set search_path = '' as $$
declare v_count bigint; v_step bigint; v_points jsonb;
begin
  if p_hours is null or p_hours not between 1 and 168 then raise exception 'Invalid track request'; end if;
  if not exists(select 1 from skager_private.cloud_devices d
      join skager_private.store_customers c on c.account_id = d.account_id and c.status = 'active'
      where d.id = p_device_id and d.account_id = p_account_id and d.revoked_at is null) then
    return jsonb_build_object('state', 'unknown');
  end if;
  select count(*) into v_count from skager_private.cloud_track
    where device_id = p_device_id and at >= now() - make_interval(hours => p_hours);
  v_step := greatest(1, ceil(v_count / 1500.0)::bigint);
  select coalesce(jsonb_agg(jsonb_build_array(extract(epoch from t.at)::bigint, round(t.lat::numeric, 6),
      round(t.lon::numeric, 6), case when t.sog is null then null else round(t.sog::numeric, 1) end) order by t.at),
      '[]'::jsonb) into v_points
    from (select at, lat, lon, sog, row_number() over (order by at desc) - 1 as n from skager_private.cloud_track
          where device_id = p_device_id and at >= now() - make_interval(hours => p_hours)) t
    where t.n % v_step = 0;
  return jsonb_build_object('state', 'ok', 'points', v_points);
end $$;

-- Alarm log, newest first, paged by id. p_device_id null: all of the account's boats.
create function public.companion_events(p_account_id uuid, p_device_id uuid, p_before bigint, p_limit integer)
returns jsonb
language sql stable security definer set search_path = '' as $$
  select jsonb_build_object('events', coalesce((select jsonb_agg(skager_private.cloud_event_json(e, d.name) order by e.id desc)
    from (select * from skager_private.cloud_events e
          where e.account_id = p_account_id and (p_device_id is null or e.device_id = p_device_id)
            and (p_before is null or e.id < p_before)
          order by e.id desc limit greatest(1, least(coalesce(p_limit, 50), 100))) e
    join skager_private.cloud_devices d on d.id = e.device_id and d.revoked_at is null
    where skager_private.companion_account_active(p_account_id) is not null), '[]'::jsonb));
$$;

-- ---------------------------------------------------------------- portal
create function public.companion_phones(p_account_id uuid) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce(jsonb_agg(jsonb_build_object('id', s.id, 'name', s.name, 'platform', s.platform,
      'app_version', s.app_version, 'created_at', s.created_at, 'last_seen_at', s.last_seen_at,
      'push', s.push_token is not null) order by s.created_at), '[]'::jsonb)
    from skager_private.companion_sessions s where s.account_id = p_account_id and s.expires_at > now();
$$;

create function public.companion_phone_revoke(p_account_id uuid, p_session_id uuid) returns jsonb
language plpgsql security definer set search_path = '' as $$
begin
  delete from skager_private.companion_sessions where id = p_session_id and account_id = p_account_id;
  return jsonb_build_object('state', case when found then 'revoked' else 'unknown' end);
end $$;

-- ---------------------------------------------------------------- maintenance
create function public.companion_job_maintenance() returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_track integer; v_events integer; v_sessions integer; v_lost integer;
begin
  delete from skager_private.companion_grants where expires_at <= now();
  delete from skager_private.companion_sessions where expires_at <= now();
  get diagnostics v_sessions = row_count;
  delete from skager_private.cloud_track where at < now() - interval '7 days';
  get diagnostics v_track = row_count;
  delete from skager_private.cloud_events where received_at < now() - interval '90 days';
  get diagnostics v_events = row_count;
  v_lost := public.cloud_contact_check();
  return jsonb_build_object('expired_sessions', v_sessions, 'pruned_track', v_track, 'pruned_events', v_events,
    'contact_lost', v_lost);
end $$;

-- ---------------------------------------------------------------- grants
do $$
declare f record;
begin
  for f in select p.oid::regprocedure as sig from pg_proc p join pg_namespace n on n.oid = p.pronamespace
           where n.nspname = 'public' and (p.proname like 'companion\_%' or p.proname like 'cloud\_%') loop
    execute format('revoke all on function %s from public, anon, authenticated', f.sig);
    execute format('grant execute on function %s to service_role', f.sig);
  end loop;
  for f in select p.oid::regprocedure as sig from pg_proc p join pg_namespace n on n.oid = p.pronamespace
           where n.nspname = 'skager_private' and (p.proname like 'companion\_%' or p.proname like 'cloud\_%') loop
    execute format('revoke all on function %s from public, anon, authenticated, service_role', f.sig);
  end loop;
end $$;
commit;
