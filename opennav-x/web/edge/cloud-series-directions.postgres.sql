-- Disposable CI database ONLY: rollback-only assertions for cloud-series-directions.sql.
BEGIN;
DO $test$
DECLARE owner uuid := 'bbbbbbbb-1111-4bbb-8bbb-bbbbbbbbbbbb';
  device uuid; series jsonb; m timestamptz := date_trunc('hour', now()) - interval '2 hours';
  point jsonb;
BEGIN
  IF current_database() <> 'scrum94_schema' THEN RAISE EXCEPTION 'Disposable database required'; END IF;
  INSERT INTO auth.users(id,email,email_confirmed_at) VALUES (owner,'directions@example.test',now());
  INSERT INTO skager_private.store_customers(email,account_id,status,password_set)
    VALUES ('directions@example.test',owner,'active',true);
  INSERT INTO skager_private.cloud_devices(account_id,token_hash,name,platform,app_version)
    VALUES (owner,repeat('d',64),'Vega','windows','0.4.0') RETURNING id INTO device;
  -- One 6-minute bucket (24 h window): across north, dead astern, opposite (cancels), and a plain value.
  INSERT INTO skager_private.cloud_telemetry_history(device_id,minute,items) VALUES
    (device, m, '{"navigation.heading": 350, "wind.apparent_angle": 170, "navigation.cog": 90, "navigation.sog": 4}'),
    (device, m + interval '1 minute', '{"navigation.heading": 10, "wind.apparent_angle": -170, "navigation.cog": 270, "navigation.sog": 6}'),
    (device, m + interval '2 minutes', '{"navigation.heading": 0, "wind.apparent_angle": 180, "navigation.sog": 5}');
  series := public.cloud_boat_series(owner, device, ARRAY['navigation.heading','wind.apparent_angle','navigation.cog','navigation.sog'], 24);

  point := series->'series'->'navigation.heading'->0;
  IF jsonb_array_length(series->'series'->'navigation.heading') <> 1
      OR NOT ((point->>1)::numeric < 0.01 OR (point->>1)::numeric > 359.99) THEN
    RAISE EXCEPTION 'Heading across north must average to 0, not 120: %', series;
  END IF;
  IF (series->'series'->'navigation.heading'->0->>1)::numeric < 0 THEN RAISE EXCEPTION 'Heading must be 0..360'; END IF;
  IF abs(abs((series->'series'->'wind.apparent_angle'->0->>1)::numeric) - 180) > 0.01 THEN
    RAISE EXCEPTION 'Wind dead astern must average to 180: %', series;
  END IF;
  -- 90 and 270 cancel out: no direction in that bucket, so no point.
  IF jsonb_array_length(series->'series'->'navigation.cog') <> 0 THEN RAISE EXCEPTION 'Cancelled direction plotted: %', series; END IF;
  IF (series->'series'->'navigation.sog'->0->>1)::numeric <> 5 THEN RAISE EXCEPTION 'Plain values still averaged: %', series; END IF;
END $test$;
ROLLBACK;
