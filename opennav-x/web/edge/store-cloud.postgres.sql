-- Disposable CI database ONLY: rollback-only revision, ownership and retry assertions.
BEGIN;
DO $test$
DECLARE account uuid := 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa';
  earlier bigint; later bigint; result jsonb; order_id uuid;
BEGIN
  IF current_database() <> 'scrum94_schema' THEN RAISE EXCEPTION 'Disposable database required'; END IF;
  INSERT INTO auth.users(id,email,email_confirmed_at) VALUES (account,'cloud-sync@example.test',now());
  INSERT INTO skager_private.store_customers(email,account_id,stripe_customer_id,password_set)
    VALUES ('cloud-sync@example.test',account,'cus_syncfixture0001',true);
  INSERT INTO skager_private.store_subscriptions(subscription_id,email,status,current_period_end)
    VALUES ('sub_syncfixture0001','cloud-sync@example.test','active',now() + interval '10 days');
  IF public.store_subscription_sync_begin('sub_unknown000001')->>'state' <> 'unknown' THEN
    RAISE EXCEPTION 'Unknown subscription reserved';
  END IF;
  result := public.store_subscription_sync_begin('sub_syncfixture0001');
  earlier := (result->>'revision')::bigint;
  IF result->>'customer_id' <> 'cus_syncfixture0001' THEN RAISE EXCEPTION 'Customer binding missing'; END IF;
  result := public.store_subscription_sync_begin('sub_syncfixture0001');
  later := (result->>'revision')::bigint;
  IF later <= earlier THEN RAISE EXCEPTION 'Revision did not advance'; END IF;
  -- An older provider fetch returns after a newer one has reserved its revision.
  IF public.store_apply_subscription('sub_syncfixture0001',earlier,'past_due',2000000000,false)->>'state' <> 'superseded' THEN
    RAISE EXCEPTION 'Stale provider result accepted';
  END IF;
  IF public.store_apply_subscription('sub_syncfixture0001',later,'canceled',2000000000,false)->>'state' <> 'applied' THEN
    RAISE EXCEPTION 'Current provider cancellation lost';
  END IF;
  result := public.store_subscription_sync_begin('sub_syncfixture0001');
  IF public.store_apply_subscription('sub_syncfixture0001',(result->>'revision')::bigint,'active',2000000000,false)->>'state' <> 'terminal' THEN
    RAISE EXCEPTION 'Canceled subscription restarted';
  END IF;
  IF (SELECT status FROM skager_private.store_subscriptions WHERE subscription_id = 'sub_syncfixture0001') <> 'canceled' THEN
    RAISE EXCEPTION 'Canceled state overwritten';
  END IF;
  INSERT INTO skager_private.store_subscriptions(subscription_id,email,status,current_period_end)
    VALUES ('sub_syncfixture0002','cloud-sync@example.test','incomplete',NULL);
  result := public.store_subscription_sync_begin('sub_syncfixture0002');
  BEGIN
    PERFORM public.store_apply_subscription('sub_syncfixture0002',(result->>'revision')::bigint,'active',NULL,false);
    RAISE EXCEPTION 'Missing period granted access';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM <> 'Missing subscription period' THEN RAISE; END IF; END;
  PERFORM public.store_apply_subscription('sub_syncfixture0002',(result->>'revision')::bigint,'active',2000000000,false);
  IF public.store_subscription_cancellations() <> '[]'::jsonb THEN RAISE EXCEPTION 'Active account queued for cancellation'; END IF;
  -- Zero-cost refund has no Stripe charge event; the durable queue still sees it.
  INSERT INTO skager_private.store_orders(kind,email,account_id,status,items,subtotal_minor,total_minor,paid_at,
      session_id,subscription_id)
    VALUES ('access','cloud-sync@example.test',account,'paid','[{"kind":"access"}]',2000,0,now(),
      'cs_test_syncfixture01','sub_syncfixture0002') RETURNING id INTO order_id;
  result := public.store_order_binding(order_id,'cs_test_syncfixture01');
  IF result->>'subscription_id' <> 'sub_syncfixture0002' THEN RAISE EXCEPTION 'Settled retry has no subscription ID'; END IF;
  result := public.store_refund_request(account,order_id);
  IF result->>'state' <> 'refunded' THEN RAISE EXCEPTION 'Zero-cost refund did not close'; END IF;
  IF public.store_subscription_cancellations() <> '["sub_syncfixture0002"]'::jsonb THEN
    RAISE EXCEPTION 'Refund cancellation not durable';
  END IF;
  -- A failed provider call has no effect; repeated ticks see the same due ID.
  result := public.store_subscription_sync_begin('sub_syncfixture0002');
  IF public.store_subscription_cancellations() <> '["sub_syncfixture0002"]'::jsonb THEN
    RAISE EXCEPTION 'Reservation consumed cancellation';
  END IF;
  PERFORM public.store_apply_subscription('sub_syncfixture0002',(result->>'revision')::bigint,'canceled',2000000000,false);
  IF public.store_subscription_cancellations() <> '[]'::jsonb THEN RAISE EXCEPTION 'Completed cancellation still queued'; END IF;
  IF to_regprocedure('public.store_apply_subscription(text,text,text,bigint,boolean)') IS NOT NULL
    OR has_function_privilege('anon','public.store_subscription_sync_begin(text)','EXECUTE')
    OR has_function_privilege('authenticated','public.store_subscription_cancellations()','EXECUTE')
    OR NOT has_function_privilege('service_role','public.store_apply_subscription(text,bigint,text,bigint,boolean)','EXECUTE') THEN
    RAISE EXCEPTION 'Subscription RPC grant boundary wrong';
  END IF;
END $test$;
ROLLBACK;
