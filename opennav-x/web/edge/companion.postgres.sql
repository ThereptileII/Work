-- Disposable CI database ONLY: rollback-only assertions for companion.sql.
BEGIN;
DO $test$
DECLARE owner uuid := '88888888-8888-4888-8888-888888888888';
  stranger uuid := '99999999-9999-4999-8999-999999999999';
  dev_tok text := repeat('1', 64); other_tok text := repeat('3', 64);
  phone_tok text := repeat('5', 64); phone2_tok text := repeat('6', 64);
  challenge text := 'E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGjSstw-cM';
  result jsonb; device uuid; other uuid; phone uuid; claimed jsonb; ev bigint; n integer;
  item jsonb := '{"navigation.latitude":{"v":59.3,"age":1,"q":"live","src":"GPS"},'
    '"navigation.longitude":{"v":18.1,"age":1,"q":"live","src":"GPS"},"navigation.sog":{"v":0.4,"age":1,"q":"live","src":"GPS"}}';
BEGIN
  INSERT INTO auth.users(id,email,email_confirmed_at) VALUES (owner,'skipper@example.com',now()),
    (stranger,'stranger@example.com',now());
  INSERT INTO skager_private.store_customers(email,account_id,status,password_set)
    VALUES ('skipper@example.com',owner,'active',true), ('stranger@example.com',stranger,'active',true);
  INSERT INTO skager_private.cloud_devices(account_id,token_hash,name,platform,app_version)
    VALUES (owner,dev_tok,'Aurora helm','windows','0.4.0') RETURNING id INTO device;
  INSERT INTO skager_private.cloud_devices(account_id,token_hash,name,platform,app_version)
    VALUES (stranger,other_tok,'Other boat','windows','0.4.0') RETURNING id INTO other;

  -- Phone sign-in: a code is bound to one PKCE challenge, single use, two minutes.
  IF public.companion_grant_create(owner,repeat('a',64),challenge)->>'state' <> 'ok' THEN RAISE EXCEPTION 'Grant failed'; END IF;
  IF public.companion_grant_redeem(repeat('a',64),'X'||substr(challenge,2),phone_tok,'Pixel','android','1.0.0')->>'state'
      <> 'invalid' THEN RAISE EXCEPTION 'Wrong verifier accepted'; END IF;
  IF public.companion_grant_redeem(repeat('a',64),challenge,phone_tok,'Pixel','android','1.0.0')->>'state' <> 'invalid' THEN
    RAISE EXCEPTION 'Code usable after a failed attempt';
  END IF;
  PERFORM public.companion_grant_create(owner,repeat('b',64),challenge);
  UPDATE skager_private.companion_grants SET expires_at = now() - interval '1 second';
  IF public.companion_grant_redeem(repeat('b',64),challenge,phone_tok,'Pixel','android','1.0.0')->>'state' <> 'invalid' THEN
    RAISE EXCEPTION 'Expired code accepted';
  END IF;
  PERFORM public.companion_grant_create(owner,repeat('c',64),challenge);
  result := public.companion_grant_redeem(repeat('c',64),challenge,phone_tok,'Pixel','android','1.0.0');
  phone := (result->>'session_id')::uuid;
  IF result->>'state' <> 'ok' OR phone IS NULL THEN RAISE EXCEPTION 'Redeem failed: %', result; END IF;
  IF public.companion_grant_redeem(repeat('c',64),challenge,phone2_tok,'Pixel','android','1.0.0')->>'state' <> 'invalid' THEN
    RAISE EXCEPTION 'Code redeemed twice';
  END IF;
  FOR n IN 1..5 LOOP PERFORM public.companion_grant_create(owner,lpad(n::text,64,'d'),challenge); END LOOP;
  IF public.companion_grant_create(owner,repeat('e',64),challenge)->>'state' <> 'throttled' THEN
    RAISE EXCEPTION 'Unbounded pending codes';
  END IF;
  DELETE FROM skager_private.companion_grants;
  BEGIN
    PERFORM public.companion_grant_create(owner,repeat('f',64),'short');
    RAISE EXCEPTION 'Malformed challenge accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;

  result := public.companion_session_auth(phone_tok,'1.0.1');
  IF result->>'state' <> 'active' OR (result->>'account_id')::uuid <> owner OR (result->'cloud'->>'active')::boolean THEN
    RAISE EXCEPTION 'Phone auth wrong: %', result;
  END IF;
  IF (SELECT app_version FROM skager_private.companion_sessions WHERE id = phone) <> '1.0.1' THEN
    RAISE EXCEPTION 'Phone version not recorded';
  END IF;
  UPDATE skager_private.companion_sessions SET expires_at = now() - interval '1 second' WHERE id = phone;
  IF public.companion_session_auth(phone_tok,null)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Expired session usable'; END IF;
  UPDATE skager_private.companion_sessions SET expires_at = now() + interval '1 day' WHERE id = phone;

  -- Push tokens follow the phone: re-registering the token moves it to the new session.
  PERFORM public.companion_push_set(phone,'fcm','fcm-token-0123456789abcdef');
  INSERT INTO skager_private.companion_sessions(account_id,token_hash,name,platform,app_version)
    VALUES (owner,phone2_tok,'Pixel (new)','android','1.0.1');
  PERFORM public.companion_push_set((SELECT id FROM skager_private.companion_sessions WHERE token_hash = phone2_tok),
    'fcm','fcm-token-0123456789abcdef');
  IF (SELECT count(*) FROM skager_private.companion_sessions WHERE push_token IS NOT NULL) <> 1
    OR (SELECT push_token FROM skager_private.companion_sessions WHERE id = phone) IS NOT NULL THEN
    RAISE EXCEPTION 'Push token duplicated across sessions';
  END IF;
  DELETE FROM skager_private.companion_sessions WHERE token_hash = phone2_tok;
  PERFORM public.companion_push_set(phone,'apns',repeat('ab',32));

  -- Without Cloud the boat cannot write events.
  IF public.cloud_event_put(device,'[{"id":"evt-00000001","kind":"anchor_alarm","age":2}]')->>'state' <> 'inactive' THEN
    RAISE EXCEPTION 'Event stored without Cloud';
  END IF;
  INSERT INTO skager_private.store_subscriptions(subscription_id,email,status,current_period_end)
    VALUES ('sub_companion0001','skipper@example.com','active',now()+interval '30 days'),
           ('sub_companion0002','stranger@example.com','active',now()+interval '30 days');

  -- Snapshot with the anchor watch and a track point.
  IF public.cloud_telemetry_put(device,item)->>'state' <> 'stored' THEN RAISE EXCEPTION 'Telemetry not stored'; END IF;
  result := public.cloud_anchor_put(device,'{"armed":true,"alarm":false,"lat":59.3,"lon":18.1,"radius_m":50,"distance_m":12}');
  IF result->>'state' <> 'stored' OR (result->>'restored')::boolean THEN RAISE EXCEPTION 'Anchor put wrong: %', result; END IF;
  IF (SELECT count(*) FROM skager_private.cloud_track WHERE device_id = device) <> 1 THEN RAISE EXCEPTION 'No track point'; END IF;
  result := public.companion_boat(owner,device);
  IF result->'anchor'->>'radius_m' <> '50' OR result->'items'->'navigation.sog'->>'v' <> '0.4' THEN
    RAISE EXCEPTION 'Boat read wrong: %', result;
  END IF;
  IF public.companion_boat(stranger,device)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Other account read the boat'; END IF;
  result := public.companion_boats(owner);
  IF jsonb_array_length(result->'devices') <> 1 OR result->'devices'->0->'anchor'->>'armed' <> 'true'
    OR NOT (result->'cloud'->>'active')::boolean THEN
    RAISE EXCEPTION 'Boats wrong: %', result;
  END IF;
  -- Stale or estimated positions never enter the track.
  UPDATE skager_private.cloud_telemetry_latest SET received_at = now() - interval '10 seconds' WHERE device_id = device;
  PERFORM public.cloud_telemetry_put(device, jsonb_set(jsonb_set(item,'{navigation.latitude,q}','"stale"'),
    '{navigation.latitude,v}','59.4'));
  IF (SELECT count(*) FROM skager_private.cloud_track WHERE device_id = device) <> 1 THEN RAISE EXCEPTION 'Stale position tracked'; END IF;
  result := public.companion_track(owner,device,24);
  IF jsonb_array_length(result->'points') <> 1 OR (result->'points'->0->>1)::numeric <> 59.3 THEN
    RAISE EXCEPTION 'Track wrong: %', result;
  END IF;
  IF public.companion_track(stranger,device,24)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Other account read the track'; END IF;

  -- Events: stored once per client id, fixed wording, pushes queued only for alarms.
  result := public.cloud_event_put(device,'[{"id":"evt-00000001","kind":"anchor_alarm","age":2,"data":{"distance_m":61.5,"radius_m":50}},
    {"id":"evt-00000002","kind":"anchor_armed","age":600},{"id":"evt-00000003","kind":"alert_critical","age":1,"alert":"GPS position lost"}]');
  IF result->>'new' <> '3' THEN RAISE EXCEPTION 'Events not stored: %', result; END IF;
  IF public.cloud_event_put(device,'[{"id":"evt-00000001","kind":"anchor_alarm","age":2}]')->>'new' <> '0' THEN
    RAISE EXCEPTION 'Retried event stored twice';
  END IF;
  IF (SELECT title FROM skager_private.cloud_events WHERE client_id = 'evt-00000001') <> 'Anchor alarm'
    OR (SELECT title FROM skager_private.cloud_events WHERE client_id = 'evt-00000003') <> 'GPS position lost'
    OR (SELECT push_state FROM skager_private.cloud_events WHERE client_id = 'evt-00000002') <> 'none'
    OR (SELECT occurred_at FROM skager_private.cloud_events WHERE client_id = 'evt-00000002') > now() - interval '599 seconds' THEN
    RAISE EXCEPTION 'Event content wrong';
  END IF;
  BEGIN
    PERFORM public.cloud_event_put(device,'[{"id":"evt-00000009","kind":"self_destruct","age":1}]');
    RAISE EXCEPTION 'Unknown event kind accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;

  -- Push queue: claimed once with this account's phones only, retried, then done.
  claimed := public.cloud_push_claim(20);
  IF jsonb_array_length(claimed) <> 2 OR claimed->0->>'boat' <> 'Aurora helm'
    OR jsonb_array_length(claimed->0->'targets') <> 1 OR claimed->0->'targets'->0->>'service' <> 'apns' THEN
    RAISE EXCEPTION 'Push claim wrong: %', claimed;
  END IF;
  IF jsonb_array_length(public.cloud_push_claim(20)) <> 0 THEN RAISE EXCEPTION 'Event claimed twice'; END IF;
  ev := (claimed->0->>'id')::bigint;
  PERFORM public.cloud_push_done(ev,'sent',null);
  PERFORM public.cloud_push_done((claimed->1->>'id')::bigint,'failed',ARRAY[repeat('ab',32)]);
  IF (SELECT push_state FROM skager_private.cloud_events WHERE id = ev) <> 'sent'
    OR (SELECT push_state FROM skager_private.cloud_events WHERE id = (claimed->1->>'id')::bigint) <> 'pending'
    OR (SELECT push_token FROM skager_private.companion_sessions WHERE id = phone) IS NOT NULL THEN
    RAISE EXCEPTION 'Push completion wrong';
  END IF;
  UPDATE skager_private.cloud_events SET received_at = now() - interval '2 hours' WHERE id = (claimed->1->>'id')::bigint;
  claimed := public.cloud_push_claim(20);
  IF jsonb_array_length(claimed) <> 0
    OR (SELECT push_state FROM skager_private.cloud_events WHERE kind = 'alert_critical') <> 'expired' THEN
    RAISE EXCEPTION 'Old notification pushed';
  END IF;

  -- Lost contact: once per gap, only while the anchor watch is on; restored on the next upload.
  UPDATE skager_private.cloud_telemetry_latest SET received_at = now() - interval '4 minutes' WHERE device_id = device;
  IF public.cloud_contact_check() <> 1 OR public.cloud_contact_check() <> 0 THEN RAISE EXCEPTION 'Contact check wrong'; END IF;
  IF (SELECT push_state FROM skager_private.cloud_events WHERE kind = 'connection_lost') <> 'pending' THEN
    RAISE EXCEPTION 'Lost contact not queued';
  END IF;
  PERFORM public.cloud_telemetry_put(device,item);
  IF NOT (public.cloud_anchor_put(device,'{"armed":true,"alarm":false}')->>'restored')::boolean THEN
    RAISE EXCEPTION 'Restored contact not noticed';
  END IF;
  IF (public.cloud_anchor_put(device,'{"armed":true,"alarm":false}')->>'restored')::boolean THEN
    RAISE EXCEPTION 'Restored twice';
  END IF;
  UPDATE skager_private.cloud_telemetry_latest SET received_at = now() - interval '4 minutes', anchor = '{"armed":false}'
    WHERE device_id = device;
  IF public.cloud_contact_check() <> 0 THEN RAISE EXCEPTION 'Lost contact without anchor watch'; END IF;

  -- Alarm log: own boats only, newest first, paged.
  result := public.companion_events(owner,null,null,2);
  IF jsonb_array_length(result->'events') <> 2 OR (result->'events'->0->>'id')::bigint
      < (result->'events'->1->>'id')::bigint THEN
    RAISE EXCEPTION 'Events page wrong: %', result;
  END IF;
  IF jsonb_array_length(public.companion_events(owner,null,(result->'events'->1->>'id')::bigint,50)->'events') <> 3 THEN
    RAISE EXCEPTION 'Events paging wrong';
  END IF;
  IF jsonb_array_length(public.companion_events(stranger,null,null,50)->'events') <> 0 THEN
    RAISE EXCEPTION 'Other account saw events';
  END IF;

  -- Portal: list and sign out phones.
  IF jsonb_array_length(public.companion_phones(owner)) <> 1 THEN RAISE EXCEPTION 'Phones wrong'; END IF;
  IF public.companion_phone_revoke(stranger,phone)->>'state' <> 'unknown' THEN RAISE EXCEPTION 'Revoked another account''s phone'; END IF;

  -- Maintenance prunes old track and events.
  INSERT INTO skager_private.cloud_track(device_id,at,lat,lon) VALUES (device,now()-interval '8 days',1,1);
  result := public.companion_job_maintenance();
  IF (result->>'pruned_track')::int <> 1 THEN RAISE EXCEPTION 'Maintenance wrong: %', result; END IF;

  -- Unlinking the boat computer removes its track and events.
  PERFORM public.cloud_device_forget(device);
  IF EXISTS(SELECT 1 FROM skager_private.cloud_track WHERE device_id = device)
    OR EXISTS(SELECT 1 FROM skager_private.cloud_events WHERE device_id = device) THEN
    RAISE EXCEPTION 'Unlinked boat data kept';
  END IF;
  IF public.cloud_event_put(device,'[{"id":"evt-00000010","kind":"anchor_alarm","age":1}]')->>'state' <> 'revoked' THEN
    RAISE EXCEPTION 'Unlinked boat wrote events';
  END IF;

  -- Closing the account signs every phone out.
  PERFORM skager_private.store_revoke('skipper@example.com');
  IF EXISTS(SELECT 1 FROM skager_private.companion_sessions WHERE account_id = owner)
    OR public.companion_session_auth(phone_tok,null)->>'state' <> 'unknown' THEN
    RAISE EXCEPTION 'Phone session survived account closure';
  END IF;

  IF has_function_privilege('anon','public.companion_session_auth(text,text)','EXECUTE')
    OR has_function_privilege('authenticated','public.companion_boats(uuid)','EXECUTE')
    OR has_function_privilege('authenticated','public.cloud_push_claim(integer)','EXECUTE')
    OR NOT has_function_privilege('service_role','public.cloud_event_put(uuid,jsonb)','EXECUTE')
    OR has_function_privilege('service_role','skager_private.cloud_device_writable(uuid)','EXECUTE') THEN
    RAISE EXCEPTION 'Companion function privileges wrong';
  END IF;
END
$test$;
ROLLBACK;
