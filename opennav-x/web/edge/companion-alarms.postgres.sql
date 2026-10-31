-- Disposable CI database ONLY: rollback-only assertions for companion-alarms.sql.
BEGIN;
DO $test$
DECLARE owner uuid := 'aaaaaaaa-1111-4aaa-8aaa-aaaaaaaaaaaa';
  stranger uuid := 'aaaaaaaa-2222-4aaa-8aaa-aaaaaaaaaaaa';
  device uuid; result jsonb; rule bigint; n integer;
  soc jsonb;
BEGIN
  IF current_database() <> 'scrum94_schema' THEN RAISE EXCEPTION 'Disposable database required'; END IF;
  INSERT INTO auth.users(id,email,email_confirmed_at) VALUES (owner,'alarms@example.test',now()),
    (stranger,'alarms-stranger@example.test',now());
  INSERT INTO skager_private.store_customers(email,account_id,status,password_set)
    VALUES ('alarms@example.test',owner,'active',true), ('alarms-stranger@example.test',stranger,'active',true);
  INSERT INTO skager_private.cloud_devices(account_id,token_hash,name,platform,app_version)
    VALUES (owner,repeat('7',64),'Linnea','windows','0.4.0') RETURNING id INTO device;

  -- Only the boat's owner can see or set its alarms.
  IF public.companion_alarms(stranger,device)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Stranger listed alarms'; END IF;
  IF public.companion_alarm_put(stranger,device,null,'battery.soc','State of charge','%','below',30,2,true)->>'state'
      <> 'unknown' THEN RAISE EXCEPTION 'Stranger set an alarm'; END IF;
  result := public.companion_alarm_put(owner,device,null,'battery.soc','State of charge','%','below',30,2,true);
  rule := (result->'alarm'->>'id')::bigint;
  IF result->>'state' <> 'ok' OR result->'alarm'->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Put failed: %', result; END IF;
  IF jsonb_array_length(public.companion_alarms(owner,device)->'alarms') <> 1 THEN RAISE EXCEPTION 'Not listed'; END IF;

  -- No data yet: nothing happens.
  IF public.cloud_alarm_check(device) <> 0 THEN RAISE EXCEPTION 'Alarm without data'; END IF;
  soc := '{"battery.soc":{"v":55,"age":1,"q":"live"}}';
  INSERT INTO skager_private.cloud_telemetry_latest(device_id,received_at,items) VALUES (device,now(),soc);
  n := public.cloud_alarm_check(device);
  IF n <> 0
      OR (SELECT state FROM skager_private.cloud_alarm_rules WHERE id = rule) <> 'ok' THEN
    RAISE EXCEPTION 'Normal value raised an alarm';
  END IF;

  -- Crossing the limit raises one alarm, to be pushed; staying there raises no more.
  UPDATE skager_private.cloud_telemetry_latest SET items = '{"battery.soc":{"v":28.5,"age":1,"q":"live"}}' WHERE device_id = device;
  IF public.cloud_alarm_check(device) <> 1 THEN RAISE EXCEPTION 'Alarm not raised'; END IF;
  IF NOT EXISTS (SELECT 1 FROM skager_private.cloud_events WHERE device_id = device AND kind = 'threshold_alarm'
      AND severity = 'critical' AND push_state = 'pending' AND title = 'State of charge below 30 %'
      AND detail = 'Now 28.5 %. Your alarm is set at 30.') THEN
    RAISE EXCEPTION 'Alarm event wrong: %', (SELECT jsonb_agg(to_jsonb(e)) FROM skager_private.cloud_events e WHERE e.device_id = device);
  END IF;
  IF public.cloud_alarm_check(device) <> 0 THEN RAISE EXCEPTION 'Alarm repeated'; END IF;

  -- A stale reading neither raises nor clears.
  UPDATE skager_private.cloud_telemetry_latest SET items = '{"battery.soc":{"v":90,"age":600,"q":"stale"}}' WHERE device_id = device;
  n := public.cloud_alarm_check(device);
  IF n <> 0 OR (SELECT state FROM skager_private.cloud_alarm_rules WHERE id = rule) <> 'alarm' THEN
    RAISE EXCEPTION 'Stale reading changed the alarm';
  END IF;
  -- Back above the limit but inside the margin: still in alarm (no flapping).
  UPDATE skager_private.cloud_telemetry_latest SET items = '{"battery.soc":{"v":31,"age":1,"q":"live"}}' WHERE device_id = device;
  IF public.cloud_alarm_check(device) <> 0 THEN RAISE EXCEPTION 'Cleared inside the margin'; END IF;
  -- Past the margin: one all-clear.
  UPDATE skager_private.cloud_telemetry_latest SET items = '{"battery.soc":{"v":33,"age":1,"q":"live"}}' WHERE device_id = device;
  n := public.cloud_alarm_check(device);
  IF n <> 1 OR NOT EXISTS (SELECT 1 FROM skager_private.cloud_events
      WHERE device_id = device AND kind = 'threshold_cleared' AND severity = 'info' AND title = 'State of charge back to normal') THEN
    RAISE EXCEPTION 'All-clear missing';
  END IF;

  -- A disabled alarm is never checked; a changed one starts afresh.
  PERFORM public.companion_alarm_put(owner,device,rule,'battery.soc','State of charge','%','below',30,2,false);
  UPDATE skager_private.cloud_telemetry_latest SET items = '{"battery.soc":{"v":10,"age":1,"q":"live"}}' WHERE device_id = device;
  IF public.cloud_alarm_check(device) <> 0 THEN RAISE EXCEPTION 'Disabled alarm raised'; END IF;
  result := public.companion_alarm_put(owner,device,rule,'battery.soc','State of charge','%','below',30,2,true);
  IF result->'alarm'->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Change kept the old state'; END IF;
  IF public.cloud_alarm_check(device) <> 1 THEN RAISE EXCEPTION 'Re-enabled alarm not raised'; END IF;

  -- "Above" alarms, and at most 20 per boat.
  PERFORM public.companion_alarm_put(owner,device,null,'wind.true_speed','True wind speed','kn','above',25,1,true);
  UPDATE skager_private.cloud_telemetry_latest SET items = '{"wind.true_speed":{"v":27.04,"age":1,"q":"estimated"}}' WHERE device_id = device;
  n := public.cloud_alarm_check(device);
  IF n <> 1 OR NOT EXISTS (SELECT 1 FROM skager_private.cloud_events
      WHERE device_id = device AND title = 'True wind speed above 25 kn' AND detail = 'Now 27 kn. Your alarm is set at 25.') THEN
    RAISE EXCEPTION 'Above alarm wrong';
  END IF;
  FOR n IN 1..18 LOOP
    PERFORM public.companion_alarm_put(owner,device,null,'environment.depth','Depth below transducer','m','below',n,0.2,true);
  END LOOP;
  IF public.companion_alarm_put(owner,device,null,'environment.depth','Depth below transducer','m','below',2,0.2,true)->>'state'
      <> 'limit' THEN RAISE EXCEPTION 'More than 20 alarms'; END IF;

  -- Delete: only the owner's own.
  IF public.companion_alarm_delete(stranger,rule)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Stranger deleted'; END IF;
  IF public.companion_alarm_delete(owner,rule)->>'state' <> 'ok' THEN RAISE EXCEPTION 'Delete failed'; END IF;

  -- Grants: only the service role may call the public functions.
  IF has_function_privilege('anon','public.cloud_alarm_check(uuid)','execute')
      OR has_function_privilege('authenticated','public.companion_alarms(uuid,uuid)','execute')
      OR NOT has_function_privilege('service_role','public.companion_alarm_put(uuid,uuid,bigint,text,text,text,text,double precision,double precision,boolean)','execute')
      OR has_function_privilege('service_role','skager_private.companion_owns_boat(uuid,uuid)','execute') THEN
    RAISE EXCEPTION 'Wrong grants';
  END IF;
END $test$;
ROLLBACK;
