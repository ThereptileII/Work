-- Run in disposable CI after the versioned auth/commerce/portal SQL.
BEGIN;
DO $test$
DECLARE subject uuid := '44444444-4444-4444-8444-444444444444';
  other_subject uuid := '55555555-5555-4555-8555-555555555555';
  result jsonb;
BEGIN
  INSERT INTO auth.users(id,email,email_confirmed_at) VALUES
    (subject,'first@example.com',now()),(other_subject,'second@example.com',now());
  IF NOT public.skager_auth_throttle(ARRAY['test:email'],2,900)
    OR NOT public.skager_auth_throttle(ARRAY['test:email'],2,900)
    OR public.skager_auth_throttle(ARRAY['test:email'],2,900) THEN
    RAISE EXCEPTION 'Atomic throttle bound failed';
  END IF;
  PERFORM public.skager_auth_challenge_create(repeat('a',64),'first@example.com');
  IF (SELECT email FROM public.skager_auth_challenge_take(repeat('a',64))) <> 'first@example.com'
    OR EXISTS(SELECT * FROM public.skager_auth_challenge_take(repeat('a',64))) THEN
    RAISE EXCEPTION 'Challenge replay accepted';
  END IF;
  PERFORM public.skager_auth_session_create(repeat('b',64),subject);
  IF (SELECT id FROM public.skager_auth_session_get(repeat('b',64))) <> subject THEN
    RAISE EXCEPTION 'Session identity differs';
  END IF;
  UPDATE auth.users SET banned_until = now() + interval '1 hour' WHERE id=subject;
  IF EXISTS(SELECT * FROM public.skager_auth_session_get(repeat('b',64))) THEN RAISE EXCEPTION 'Banned session accepted'; END IF;
  UPDATE auth.users SET banned_until = NULL WHERE id=subject;
  PERFORM public.skager_auth_session_delete(repeat('b',64));
  IF EXISTS(SELECT * FROM public.skager_auth_session_get(repeat('b',64))) THEN RAISE EXCEPTION 'Logged-out session accepted'; END IF;
  PERFORM public.portal_create_boat(subject,'First vessel');
  IF jsonb_array_length(public.portal_list_boats(other_subject)) <> 0
    OR jsonb_array_length(public.portal_list_boats(subject)) <> 1 THEN RAISE EXCEPTION 'Boat owner isolation failed'; END IF;
  IF has_function_privilege('anon','public.skager_auth_session_get(text)','EXECUTE')
    OR has_function_privilege('authenticated','public.portal_list_boats(uuid)','EXECUTE')
    OR has_table_privilege('service_role','skager_private.commerce_entitlements','SELECT')
    OR has_schema_privilege('authenticated','skager_private','USAGE') THEN
    RAISE EXCEPTION 'Private authorization boundary widened';
  END IF;
END;
$test$;
ROLLBACK;
