-- SKAGER Companion: alarms the owner sets on boat data. Apply after companion.sql,
-- as owner, in one transaction. A rule is one data type, above or below, and a value;
-- the service checks it whenever the boat uploads (at most every 5 s, normally 15 s).
begin;

create table skager_private.cloud_alarm_rules (
  id bigint generated always as identity primary key,
  account_id uuid not null references auth.users(id) on delete cascade,
  device_id uuid not null references skager_private.cloud_devices(id) on delete cascade,
  key text not null check (key ~ '^[a-z_]{1,24}\.[a-z_]{1,32}$'),
  -- Wording and display unit come from the service's fixed telemetry list, never the phone.
  label text not null check (char_length(label) between 1 and 60 and label !~ '[[:cntrl:]]'),
  unit text not null check (char_length(unit) <= 8 and unit !~ '[[:cntrl:]]'),
  direction text not null check (direction in ('above', 'below')),
  threshold double precision not null check (threshold between -1000000 and 1000000),
  -- How far back the value must come before the alarm clears, so it never flaps.
  margin double precision not null check (margin between 0 and 1000000),
  enabled boolean not null default true,
  state text not null default 'unknown' check (state in ('unknown', 'ok', 'alarm')),
  value double precision,
  changed_at timestamptz,
  created_at timestamptz not null default now()
);
create index cloud_alarm_rules_device on skager_private.cloud_alarm_rules(device_id);
create index cloud_alarm_rules_account on skager_private.cloud_alarm_rules(account_id);
revoke all on skager_private.cloud_alarm_rules from public, anon, authenticated, service_role;
alter table skager_private.cloud_alarm_rules enable row level security;

alter table skager_private.cloud_events drop constraint cloud_events_kind_check;
alter table skager_private.cloud_events add constraint cloud_events_kind_check check (kind in ('anchor_alarm',
  'anchor_alarm_cleared', 'anchor_armed', 'anchor_disarmed', 'alert_critical', 'alert_cleared', 'connection_lost',
  'connection_restored', 'threshold_alarm', 'threshold_cleared'));

create function skager_private.cloud_alarm_number(p_value double precision) returns text
language sql immutable set search_path = '' as $$
  select regexp_replace(round(p_value::numeric, 1)::text, '\.0$', '');
$$;

create function skager_private.cloud_alarm_json(r skager_private.cloud_alarm_rules) returns jsonb
language sql stable set search_path = '' as $$
  select jsonb_build_object('id', r.id, 'device_id', r.device_id, 'key', r.key, 'label', r.label, 'unit', r.unit,
    'direction', r.direction, 'threshold', r.threshold, 'enabled', r.enabled, 'state', r.state, 'value', r.value,
    'changed_at', r.changed_at);
$$;

-- Called after each accepted upload. Only current readings (live, aging or estimated)
-- count; a missing or stale reading never raises or clears an alarm.
create function public.cloud_alarm_check(p_device_id uuid) returns integer
language plpgsql security definer set search_path = '' as $$
declare v_items jsonb; r skager_private.cloud_alarm_rules; v double precision; v_item jsonb;
  v_breach boolean; v_clear boolean; v_count integer := 0; v_limit text;
begin
  select items into v_items from skager_private.cloud_telemetry_latest where device_id = p_device_id;
  if v_items is null then return 0; end if;
  for r in select * from skager_private.cloud_alarm_rules a where a.device_id = p_device_id and a.enabled
      order by a.id for update loop
    v_item := v_items -> r.key;
    if v_item is null or jsonb_typeof(v_item -> 'v') <> 'number'
        or coalesce(v_item ->> 'q', '') not in ('live', 'aging', 'estimated') then
      continue;
    end if;
    v := (v_item ->> 'v')::double precision;
    v_breach := case r.direction when 'above' then v > r.threshold else v < r.threshold end;
    v_clear := case r.direction when 'above' then v <= r.threshold - r.margin else v >= r.threshold + r.margin end;
    v_limit := r.label || ' ' || r.direction || ' ' || skager_private.cloud_alarm_number(r.threshold)
      || case when r.unit = '' then '' else ' ' || r.unit end;
    if v_breach and r.state <> 'alarm' then
      update skager_private.cloud_alarm_rules set state = 'alarm', value = v, changed_at = now() where id = r.id;
      insert into skager_private.cloud_events(device_id, account_id, kind, severity, title, detail, data,
          occurred_at, push_state)
        values (r.device_id, r.account_id, 'threshold_alarm', 'critical', left(v_limit, 120),
          left('Now ' || skager_private.cloud_alarm_number(v) || case when r.unit = '' then '' else ' ' || r.unit end
            || '. Your alarm is set at ' || skager_private.cloud_alarm_number(r.threshold) || '.', 300),
          jsonb_build_object('rule_id', r.id, 'key', r.key, 'value', v, 'threshold', r.threshold,
            'direction', r.direction),
          now(), 'pending');
      v_count := v_count + 1;
    elsif v_clear and r.state = 'alarm' then
      update skager_private.cloud_alarm_rules set state = 'ok', value = v, changed_at = now() where id = r.id;
      insert into skager_private.cloud_events(device_id, account_id, kind, severity, title, detail, data,
          occurred_at, push_state)
        values (r.device_id, r.account_id, 'threshold_cleared', 'info', left(r.label || ' back to normal', 120),
          left('Now ' || skager_private.cloud_alarm_number(v) || case when r.unit = '' then '' else ' ' || r.unit end
            || '.', 300),
          jsonb_build_object('rule_id', r.id, 'key', r.key, 'value', v, 'threshold', r.threshold,
            'direction', r.direction),
          now(), 'pending');
      v_count := v_count + 1;
    elsif not v_breach and r.state = 'unknown' then
      update skager_private.cloud_alarm_rules set state = 'ok', value = v, changed_at = now() where id = r.id;
    else
      update skager_private.cloud_alarm_rules set value = v where id = r.id;
    end if;
  end loop;
  return v_count;
end $$;

-- ---------------------------------------------------------------- phone
create function skager_private.companion_owns_boat(p_account_id uuid, p_device_id uuid) returns boolean
language sql stable set search_path = '' as $$
  select exists(select 1 from skager_private.cloud_devices d
    join skager_private.store_customers c on c.account_id = d.account_id and c.status = 'active'
    where d.id = p_device_id and d.account_id = p_account_id and d.revoked_at is null);
$$;

create function public.companion_alarms(p_account_id uuid, p_device_id uuid) returns jsonb
language sql stable security definer set search_path = '' as $$
  select case when skager_private.companion_owns_boat(p_account_id, p_device_id) then
    jsonb_build_object('state', 'ok', 'alarms', coalesce((select jsonb_agg(skager_private.cloud_alarm_json(a)
      order by a.id) from skager_private.cloud_alarm_rules a where a.device_id = p_device_id), '[]'::jsonb))
    else jsonb_build_object('state', 'unknown') end;
$$;

-- Create (p_id null) or change a rule. A change starts it afresh: the next upload
-- decides whether it is in alarm, so a new limit is never silently ignored.
create function public.companion_alarm_put(p_account_id uuid, p_device_id uuid, p_id bigint, p_key text,
  p_label text, p_unit text, p_direction text, p_threshold double precision, p_margin double precision,
  p_enabled boolean) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare r skager_private.cloud_alarm_rules;
begin
  if not skager_private.companion_owns_boat(p_account_id, p_device_id) then
    return jsonb_build_object('state', 'unknown');
  end if;
  perform 1 from skager_private.cloud_devices where id = p_device_id for update;
  if p_id is null then
    if (select count(*) from skager_private.cloud_alarm_rules where device_id = p_device_id) >= 20 then
      return jsonb_build_object('state', 'limit');
    end if;
    insert into skager_private.cloud_alarm_rules(account_id, device_id, key, label, unit, direction, threshold,
        margin, enabled)
      values (p_account_id, p_device_id, p_key, p_label, p_unit, p_direction, p_threshold, p_margin,
        coalesce(p_enabled, true))
      returning * into r;
  else
    update skager_private.cloud_alarm_rules set key = p_key, label = p_label, unit = p_unit,
        direction = p_direction, threshold = p_threshold, margin = p_margin, enabled = coalesce(p_enabled, true),
        state = 'unknown', value = null, changed_at = null
      where id = p_id and device_id = p_device_id and account_id = p_account_id
      returning * into r;
    if r.id is null then return jsonb_build_object('state', 'unknown'); end if;
  end if;
  return jsonb_build_object('state', 'ok', 'alarm', skager_private.cloud_alarm_json(r));
end $$;

create function public.companion_alarm_delete(p_account_id uuid, p_id bigint) returns jsonb
language plpgsql security definer set search_path = '' as $$
begin
  delete from skager_private.cloud_alarm_rules where id = p_id and account_id = p_account_id;
  return jsonb_build_object('state', case when found then 'ok' else 'unknown' end);
end $$;

-- ---------------------------------------------------------------- grants
do $$
declare f record;
begin
  for f in select p.oid::regprocedure as sig from pg_proc p join pg_namespace n on n.oid = p.pronamespace
           where n.nspname = 'public' and p.proname in ('cloud_alarm_check', 'companion_alarms',
             'companion_alarm_put', 'companion_alarm_delete') loop
    execute format('revoke all on function %s from public, anon, authenticated', f.sig);
    execute format('grant execute on function %s to service_role', f.sig);
  end loop;
  for f in select p.oid::regprocedure as sig from pg_proc p join pg_namespace n on n.oid = p.pronamespace
           where n.nspname = 'skager_private' and p.proname in ('cloud_alarm_number', 'cloud_alarm_json',
             'companion_owns_boat') loop
    execute format('revoke all on function %s from public, anon, authenticated, service_role', f.sig);
  end loop;
end $$;
commit;
