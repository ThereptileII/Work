-- Disposable CI database ONLY: rollback-only assertions for cloud.sql.
BEGIN;
DO $test$
DECLARE owner uuid := '88888888-8888-4888-8888-888888888888';
  stranger uuid := '99999999-9999-4999-8999-999999999999';
  dev_a text := repeat('a', 64); dev_b text := repeat('b', 64); dev_c text := repeat('c', 64);
  tok_a text := repeat('1', 64); tok_b text := repeat('2', 64);
  result jsonb; device uuid; series jsonb;
BEGIN
  INSERT INTO auth.users(id,email,email_confirmed_at) VALUES (owner,'skipper@example.com',now()),
    (stranger,'stranger@example.com',now());
  INSERT INTO skager_private.store_customers(email,account_id,status,password_set)
    VALUES ('skipper@example.com',owner,'active',true), ('stranger@example.com',stranger,'active',true);

  -- Pairing: start, a duplicate user code is retried, the customer approves, the app claims once.
  IF public.cloud_link_start(dev_a,'BCDF2345','Aurora helm','windows','0.4.0-beta2.6')->>'state' <> 'pending' THEN
    RAISE EXCEPTION 'Link start failed';
  END IF;
  IF public.cloud_link_start(dev_b,'BCDF2345','Other','windows','0.4.0')->>'state' <> 'retry' THEN
    RAISE EXCEPTION 'Duplicate user code accepted';
  END IF;
  BEGIN
    PERFORM public.cloud_link_start(dev_b,'bcdf2345','Other','windows','0.4.0');
    RAISE EXCEPTION 'Lower-case user code accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;
  BEGIN
    PERFORM public.cloud_link_start(dev_b,'BCDF2346',E'Bad\nname','windows','0.4.0');
    RAISE EXCEPTION 'Control character in device name accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;
  IF public.cloud_link_claim(dev_a,tok_a)->>'state' <> 'pending' THEN RAISE EXCEPTION 'Unapproved link claimed'; END IF;
  result := public.cloud_link_find(owner,'BCDF2345');
  IF result->>'state' <> 'pending' OR result->>'name' <> 'Aurora helm' OR result ? 'device_hash' THEN
    RAISE EXCEPTION 'Link lookup wrong: %', result;
  END IF;
  IF public.cloud_link_find(owner,'ZZZZ9999')->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Unknown code found'; END IF;
  IF public.cloud_link_decide(owner,'BCDF2345',true)->>'state' <> 'approved' THEN RAISE EXCEPTION 'Approve failed'; END IF;
  IF public.cloud_link_decide(stranger,'BCDF2345',true)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Decided twice'; END IF;
  result := public.cloud_link_claim(dev_a,tok_a);
  device := (result->>'device_id')::uuid;
  IF result->>'state' <> 'approved' OR device IS NULL THEN RAISE EXCEPTION 'Claim failed: %', result; END IF;
  IF public.cloud_link_claim(dev_a,tok_b)->>'state' <> 'expired' THEN RAISE EXCEPTION 'Link claimed twice'; END IF;

  -- Denied and expired links never become devices.
  PERFORM public.cloud_link_start(dev_b,'BCDF2346','Spare','linux','0.4.0');
  PERFORM public.cloud_link_decide(owner,'BCDF2346',false);
  IF public.cloud_link_claim(dev_b,tok_b)->>'state' <> 'denied' THEN RAISE EXCEPTION 'Denied link claimed'; END IF;
  PERFORM public.cloud_link_start(dev_c,'BCDF2347','Late','linux','0.4.0');
  UPDATE skager_private.cloud_links SET expires_at = now() - interval '1 second' WHERE device_hash = dev_c;
  IF public.cloud_link_decide(owner,'BCDF2347',true)->>'state' <> 'unknown'
    OR public.cloud_link_claim(dev_c,tok_b)->>'state' <> 'expired' THEN
    RAISE EXCEPTION 'Expired link usable';
  END IF;
  IF (SELECT count(*) FROM skager_private.cloud_devices) <> 1 THEN RAISE EXCEPTION 'Unexpected devices'; END IF;

  -- Device auth: account known, no Cloud yet, so no update feed or telemetry entitlement.
  result := public.cloud_device_auth(tok_a,'0.4.0-beta2.7');
  IF result->>'state' <> 'active' OR result->>'email' <> 'skipper@example.com'
    OR (result->'cloud'->>'active')::boolean THEN
    RAISE EXCEPTION 'Device auth without Cloud wrong: %', result;
  END IF;
  IF (SELECT app_version FROM skager_private.cloud_devices WHERE id = device) <> '0.4.0-beta2.7' THEN
    RAISE EXCEPTION 'Device version not recorded';
  END IF;
  IF public.cloud_device_auth(tok_b,NULL)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Unknown token resolved'; END IF;

  INSERT INTO skager_private.store_subscriptions(subscription_id,email,status,current_period_end)
    VALUES ('sub_cloudtest0001','skipper@example.com','active',now() + interval '20 days');
  IF NOT (public.cloud_device_auth(tok_a,NULL)->'cloud'->>'active')::boolean THEN RAISE EXCEPTION 'Active Cloud not seen'; END IF;
  UPDATE skager_private.store_subscriptions SET status = 'past_due', current_period_end = now() - interval '2 days';
  IF NOT (public.cloud_device_auth(tok_a,NULL)->'cloud'->>'active')::boolean THEN RAISE EXCEPTION 'Past-due grace missing'; END IF;
  UPDATE skager_private.store_subscriptions SET current_period_end = now() - interval '8 days';
  IF (public.cloud_device_auth(tok_a,NULL)->'cloud'->>'active')::boolean THEN RAISE EXCEPTION 'Grace too long'; END IF;
  UPDATE skager_private.store_subscriptions SET status = 'canceled';
  IF (public.cloud_device_auth(tok_a,NULL)->'cloud'->>'active')::boolean THEN RAISE EXCEPTION 'Canceled Cloud active'; END IF;
  UPDATE skager_private.store_subscriptions SET status = 'active', current_period_end = now() + interval '20 days';

  -- Telemetry: latest snapshot, throttled within 5 s, history keeps fresh numeric values only.
  result := public.cloud_telemetry_put(device, jsonb_build_object(
    'navigation.sog', jsonb_build_object('v',5.2,'age',0.4,'q','live','src','GPS'),
    'environment.depth', jsonb_build_object('v',12.5,'age',30,'q','stale','src','DST')));
  IF result->>'state' <> 'stored' THEN RAISE EXCEPTION 'Telemetry not stored'; END IF;
  IF public.cloud_telemetry_put(device, '{}'::jsonb)->>'state' <> 'throttled' THEN RAISE EXCEPTION 'Not throttled'; END IF;
  IF (SELECT items FROM skager_private.cloud_telemetry_history WHERE device_id = device) <> '{"navigation.sog": 5.2}'::jsonb THEN
    RAISE EXCEPTION 'History wrong';
  END IF;
  IF NOT (SELECT telemetry_enabled FROM skager_private.cloud_devices WHERE id = device) THEN
    RAISE EXCEPTION 'Telemetry flag not set';
  END IF;
  INSERT INTO skager_private.cloud_telemetry_history(device_id,minute,items) VALUES
    (device, date_trunc('minute', now()) - interval '10 minutes', '{"navigation.sog": 4.8}'),
    (device, date_trunc('minute', now()) - interval '40 days', '{"navigation.sog": 1.0}');

  result := public.cloud_boat_latest(owner, device);
  IF result->>'state' <> 'ok' OR (result->'items'->'navigation.sog'->>'v')::numeric <> 5.2 THEN
    RAISE EXCEPTION 'Latest wrong: %', result;
  END IF;
  IF public.cloud_boat_latest(stranger, device)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Other account sees boat'; END IF;
  series := public.cloud_boat_series(owner, device, ARRAY['navigation.sog','battery.soc'], 24);
  IF jsonb_array_length(series->'series'->'navigation.sog') <> 2 OR jsonb_array_length(series->'series'->'battery.soc') <> 0 THEN
    RAISE EXCEPTION 'Series wrong: %', series;
  END IF;
  IF public.cloud_boat_series(stranger, device, ARRAY['navigation.sog'], 24)->>'state' <> 'unknown' THEN
    RAISE EXCEPTION 'Other account sees series';
  END IF;
  BEGIN
    PERFORM public.cloud_boat_series(owner, device, ARRAY['navigation.sog'], 721);
    RAISE EXCEPTION 'Series window unbounded';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM NOT LIKE 'Invalid series%' THEN RAISE; END IF; END;
  result := public.cloud_job_maintenance();
  IF (result->>'pruned_history')::int <> 1 THEN RAISE EXCEPTION 'History not pruned: %', result; END IF;

  -- Overview lists the device; stopping sharing removes the live snapshot.
  result := public.cloud_overview(owner);
  IF jsonb_array_length(result->'devices') <> 1 OR NOT (result->'cloud'->>'active')::boolean
    OR result->'devices'->0->>'last_data_at' IS NULL THEN
    RAISE EXCEPTION 'Overview wrong: %', result;
  END IF;
  IF jsonb_array_length(public.cloud_overview(stranger)->'devices') <> 0 THEN RAISE EXCEPTION 'Overview leaks'; END IF;
  PERFORM public.cloud_telemetry_stop(device);
  IF EXISTS(SELECT 1 FROM skager_private.cloud_telemetry_latest WHERE device_id = device) THEN
    RAISE EXCEPTION 'Snapshot kept after stop';
  END IF;

  -- Revocation: another account cannot revoke; the owner can; the token stops resolving.
  IF public.cloud_device_revoke(stranger, device)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Stranger revoked'; END IF;
  IF public.cloud_device_revoke(owner, device)->>'state' <> 'revoked' THEN RAISE EXCEPTION 'Owner revoke failed'; END IF;
  IF public.cloud_device_auth(tok_a,NULL)->>'state' <> 'unknown'
    OR EXISTS(SELECT 1 FROM skager_private.cloud_telemetry_history WHERE device_id = device) THEN
    RAISE EXCEPTION 'Revocation incomplete';
  END IF;

  -- Closing the account (refund) revokes every device.
  PERFORM public.cloud_link_start(dev_c,'BCDF2348','Second','windows','0.4.0');
  PERFORM public.cloud_link_decide(owner,'BCDF2348',true);
  PERFORM public.cloud_link_claim(dev_c,tok_b);
  IF public.cloud_device_auth(tok_b,NULL)->>'state' <> 'active' THEN RAISE EXCEPTION 'Second device inactive'; END IF;
  PERFORM skager_private.store_revoke('skipper@example.com');
  IF public.cloud_device_auth(tok_b,NULL)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Revoked customer device resolves'; END IF;

  -- Browser and anonymous roles have no access to tables or functions.
  IF has_table_privilege('service_role','skager_private.cloud_devices','SELECT')
    OR has_function_privilege('anon','public.cloud_device_auth(text,text)','EXECUTE')
    OR has_function_privilege('authenticated','public.cloud_overview(uuid)','EXECUTE')
    OR NOT has_function_privilege('service_role','public.cloud_telemetry_put(uuid,jsonb)','EXECUTE')
    OR has_function_privilege('service_role','skager_private.cloud_status(text)','EXECUTE') THEN
    RAISE EXCEPTION 'Cloud grants wrong';
  END IF;
END $test$;
ROLLBACK;
