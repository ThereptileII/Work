-- Apply once after cloud.sql, as owner, in one transaction.
-- History of directions (heading, course, wind angles) is averaged on the circle:
-- the mean of sine and cosine, turned back into an angle. A plain average is wrong
-- across north or dead astern (350 and 10 degrees would give 180, not 0).
-- Heading and course come back as 0..360; wind angles keep the boat's sign
-- (-180..180, port negative). A bucket whose directions cancel out has no point.
begin;
create or replace function public.cloud_boat_series(p_account_id uuid, p_device_id uuid, p_keys text[], p_hours integer)
returns jsonb
language plpgsql stable security definer set search_path = '' as $$
declare v_bucket double precision; v_since timestamptz; v_result jsonb := '{}'::jsonb; v_key text; v_points jsonb;
  v_circular boolean; v_signed boolean;
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
    v_circular := v_key in ('navigation.heading', 'navigation.cog', 'wind.apparent_angle', 'wind.true_angle');
    v_signed := v_key in ('wind.apparent_angle', 'wind.true_angle');
    select coalesce(jsonb_agg(jsonb_build_array(b.t, round(b.v::numeric, 4)) order by b.t), '[]'::jsonb) into v_points
      from (select a.t,
                   case when not v_circular then a.mean
                        when sqrt(a.s * a.s + a.c * a.c) < 1e-6 then null
                        when v_signed then degrees(atan2(a.s, a.c))
                        else mod((degrees(atan2(a.s, a.c)) + 360)::numeric, 360)::double precision end as v
              from (select (floor(extract(epoch from h.minute) / v_bucket) * v_bucket)::bigint as t,
                           avg((h.items->>v_key)::double precision) as mean,
                           avg(sin(radians((h.items->>v_key)::double precision))) as s,
                           avg(cos(radians((h.items->>v_key)::double precision))) as c
                      from skager_private.cloud_telemetry_history h
                      where h.device_id = p_device_id and h.minute >= v_since and jsonb_typeof(h.items->v_key) = 'number'
                      group by 1) a) b
      where b.v is not null;
    v_result := v_result || jsonb_build_object(v_key, v_points);
  end loop;
  return jsonb_build_object('state', 'ok', 'bucket_seconds', v_bucket, 'series', v_result);
end $$;
commit;
