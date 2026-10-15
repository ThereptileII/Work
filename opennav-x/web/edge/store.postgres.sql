-- Disposable CI database ONLY: rollback-only assertions for store.sql.
BEGIN;
DO $test$
DECLARE buyer uuid := '66666666-6666-4666-8666-666666666666';
  other uuid := '77777777-7777-4777-8777-777777777777';
  catalog jsonb; access_id uuid; gps_id uuid; pc_id uuid; cloud_id uuid;
  created jsonb; order_id uuid; result jsonb; claimed jsonb; shop jsonb; sub jsonb;
BEGIN
  catalog := public.store_catalog();
  SELECT (p->>'id')::uuid INTO access_id FROM jsonb_array_elements(catalog->'products') p WHERE p->>'slug' = 'skager-beta-access';
  SELECT (p->>'id')::uuid INTO gps_id FROM jsonb_array_elements(catalog->'products') p WHERE p->>'slug' = 'gps-module';
  SELECT (p->>'id')::uuid INTO pc_id FROM jsonb_array_elements(catalog->'products') p WHERE p->>'slug' = 'helm-pc';
  SELECT (p->>'id')::uuid INTO cloud_id FROM jsonb_array_elements(catalog->'products') p WHERE p->>'slug' = 'skager-cloud';
  IF access_id IS NULL OR gps_id IS NULL OR pc_id IS NULL OR cloud_id IS NULL OR (catalog->>'shipping_minor')::int <> 1500 THEN
    RAISE EXCEPTION 'Seed catalog incomplete';
  END IF;

  -- Guest access order with a shippable add-on: catalog prices and shipping, never caller prices.
  created := public.store_create_order('access','Buyer@Example.com',NULL,
    jsonb_build_array(jsonb_build_object('product_id',access_id,'quantity',1), jsonb_build_object('product_id',gps_id,'quantity',1)),NULL,NULL);
  order_id := (created->>'order_id')::uuid;
  IF created->>'state' <> 'created' OR (created->>'subtotal_minor')::int <> 6900 OR (created->>'shipping_minor')::int <> 1500 THEN
    RAISE EXCEPTION 'Access order totals wrong: %', created;
  END IF;
  BEGIN
    PERFORM public.store_create_order('access','x@example.com',NULL,jsonb_build_array(jsonb_build_object('product_id',pc_id,'quantity',1)),NULL,NULL);
    RAISE EXCEPTION 'Non-addon product accepted in access checkout';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM LIKE 'Non-addon%' THEN RAISE; END IF; END;
  BEGIN
    PERFORM public.store_create_order('shop','buyer@example.com',NULL,jsonb_build_array(jsonb_build_object('product_id',pc_id,'quantity',1)),NULL,NULL);
    RAISE EXCEPTION 'Shop order without account accepted';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM LIKE 'Shop order%' THEN RAISE; END IF; END;

  PERFORM public.store_bind_order(order_id,'cs_test_aaaaaaaaaaaa','https://checkout.stripe.com/c/pay/cs_test_aaaaaaaaaaaa');
  IF public.store_order_binding(order_id,'cs_test_other00000')->>'state' <> 'rejected'
    OR public.store_order_binding(order_id,'cs_test_aaaaaaaaaaaa')->>'state' <> 'bound' THEN
    RAISE EXCEPTION 'Session binding not exact';
  END IF;
  IF public.store_claim_state('cs_test_aaaaaaaaaaaa',NULL)->>'state' <> 'pending' THEN RAISE EXCEPTION 'Unpaid order claimable'; END IF;

  -- Webhook receipt is idempotent; a changed digest conflicts.
  IF public.store_receive_event('evt_paid00000000','aa' || repeat('0',62),'checkout.session.completed')->>'state' <> 'received' THEN RAISE EXCEPTION 'Receipt'; END IF;
  IF public.store_receive_event('evt_paid00000000','bb' || repeat('0',62),'checkout.session.completed')->>'state' <> 'conflict' THEN RAISE EXCEPTION 'Digest conflict missed'; END IF;
  result := public.store_apply_paid('evt_paid00000000',order_id,'cs_test_aaaaaaaaaaaa',jsonb_build_object(
    'total_minor',8400,'discount_minor',0,'shipping_minor',1500,'payment_intent_id','pi_aaaaaaaaaaaa','customer_id','cus_aaaaaaaaaaaa',
    'shipping',jsonb_build_object('name','A Sailor')));
  IF NOT (result->>'applied')::boolean THEN RAISE EXCEPTION 'Paid order not applied'; END IF;
  IF public.store_receive_event('evt_paid00000000','aa' || repeat('0',62),'checkout.session.completed')->>'state' <> 'applied' THEN RAISE EXCEPTION 'Duplicate not recognised'; END IF;
  IF (SELECT count(*) FROM skager_private.email_outbox WHERE template IN ('welcome','admin_order')) <> 2 THEN RAISE EXCEPTION 'Emails not queued'; END IF;
  IF public.store_claim_state('cs_test_aaaaaaaaaaaa',NULL)->>'state' <> 'ready' THEN RAISE EXCEPTION 'Paid order not claimable'; END IF;
  IF public.store_create_order('access','buyer@example.com',NULL,jsonb_build_array(jsonb_build_object('product_id',access_id,'quantity',1)),NULL,NULL)->>'state' <> 'setup' THEN
    RAISE EXCEPTION 'Second purchase for an unclaimed customer allowed';
  END IF;

  -- Claim: the verified Auth subject is linked; sessions resolve only for active customers.
  INSERT INTO auth.users(id,email,email_confirmed_at) VALUES (buyer,'buyer@example.com',now()), (other,'other@example.com',now());
  PERFORM public.store_link_account('buyer@example.com',buyer,'setup');
  BEGIN
    PERFORM public.store_link_account('buyer@example.com',other,'login');
    RAISE EXCEPTION 'Account re-linked to another subject';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM LIKE 'Account re-linked%' THEN RAISE; END IF; END;
  PERFORM public.skager_auth_session_create(repeat('c',64),buyer);
  IF (SELECT password_set FROM public.store_session_get(repeat('c',64))) IS DISTINCT FROM false THEN RAISE EXCEPTION 'Setup session should require a password'; END IF;
  PERFORM public.store_password_set(buyer);
  IF public.store_claim_state('cs_test_aaaaaaaaaaaa',NULL)->>'state' <> 'done' THEN RAISE EXCEPTION 'Claimed state'; END IF;
  IF public.store_create_order('access','buyer@example.com',NULL,jsonb_build_array(jsonb_build_object('product_id',access_id,'quantity',1)),NULL,NULL)->>'state' <> 'exists' THEN
    RAISE EXCEPTION 'Existing customer could buy access twice';
  END IF;
  IF public.store_portal_access(buyer)->'entitlement'->>'status' <> 'active' OR public.store_portal_access(other)->'entitlement' <> 'null'::jsonb THEN
    RAISE EXCEPTION 'Portal access isolation failed';
  END IF;

  -- Shop order for the signed-in customer, then admin shipping.
  shop := public.store_create_order('shop','buyer@example.com',buyer,jsonb_build_array(jsonb_build_object('product_id',pc_id,'quantity',2)),NULL,NULL);
  IF (shop->>'subtotal_minor')::int <> 109800 THEN RAISE EXCEPTION 'Shop subtotal wrong'; END IF;
  PERFORM public.store_bind_order((shop->>'order_id')::uuid,'cs_test_shop00000000','https://checkout.stripe.com/c/pay/cs_test_shop00000000');
  PERFORM public.store_receive_event('evt_shop00000000',repeat('1',64),'checkout.session.completed');
  PERFORM public.store_apply_paid('evt_shop00000000',(shop->>'order_id')::uuid,'cs_test_shop00000000',jsonb_build_object(
    'total_minor',111300,'discount_minor',0,'shipping_minor',1500,'payment_intent_id','pi_shop00000000'));
  PERFORM public.store_admin_ship('admin@example.com',(shop->>'order_id')::uuid,'TRACK123');
  IF (SELECT fulfillment FROM skager_private.store_orders WHERE id = (shop->>'order_id')::uuid) <> 'shipped' THEN RAISE EXCEPTION 'Ship failed'; END IF;

  -- Cloud subscription lifecycle.
  sub := public.store_create_order('subscription','buyer@example.com',buyer,jsonb_build_array(jsonb_build_object('product_id',cloud_id,'quantity',1)),NULL,NULL);
  PERFORM public.store_bind_order((sub->>'order_id')::uuid,'cs_test_sub000000000','https://checkout.stripe.com/c/pay/cs_test_sub000000000');
  PERFORM public.store_receive_event('evt_sub000000000',repeat('2',64),'checkout.session.completed');
  PERFORM public.store_apply_paid('evt_sub000000000',(sub->>'order_id')::uuid,'cs_test_sub000000000',jsonb_build_object(
    'total_minor',200,'discount_minor',0,'subscription_id','sub_aaaaaaaaaaaa','subscription_status','active','current_period_end','2000000000'));
  IF public.store_portal_access(buyer)->'subscription'->>'status' <> 'active' THEN RAISE EXCEPTION 'Subscription not active'; END IF;
  PERFORM public.store_receive_event('evt_subdel000000',repeat('3',64),'customer.subscription.deleted');
  PERFORM public.store_apply_subscription('evt_subdel000000','sub_aaaaaaaaaaaa','canceled',2000000000,false);
  IF public.store_portal_access(buyer)->'subscription'->>'status' <> 'canceled' THEN RAISE EXCEPTION 'Subscription not cancelled'; END IF;

  -- A refund of the access order closes the account immediately.
  PERFORM public.store_receive_event('evt_refund000000',repeat('4',64),'charge.refunded');
  result := public.store_apply_refund('evt_refund000000','pi_aaaaaaaaaaaa',8400);
  IF NOT (result->>'known')::boolean OR EXISTS(SELECT * FROM public.store_session_get(repeat('c',64)))
    OR public.store_customer_state('buyer@example.com')->>'state' <> 'revoked'
    OR public.store_portal_access(buyer)->'entitlement' <> 'null'::jsonb THEN
    RAISE EXCEPTION 'Refund did not revoke access';
  END IF;
  IF public.store_create_order('access','buyer@example.com',NULL,jsonb_build_array(jsonb_build_object('product_id',access_id,'quantity',1)),NULL,NULL)->>'state' <> 'revoked' THEN
    RAISE EXCEPTION 'Revoked customer could buy again';
  END IF;
  BEGIN
    PERFORM public.store_link_account('buyer@example.com',buyer,'login');
    RAISE EXCEPTION 'Revoked account could sign in';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM LIKE 'Revoked account%' THEN RAISE; END IF; END;

  -- Zero-total self-service refund closes at once; outside 14 days it is refused.
  created := public.store_create_order('access','free@example.com',NULL,jsonb_build_array(jsonb_build_object('product_id',access_id,'quantity',1)),'promo_aaaaaaaaaaaa','FREE100');
  PERFORM public.store_bind_order((created->>'order_id')::uuid,'cs_test_free00000000','https://checkout.stripe.com/c/pay/cs_test_free00000000');
  PERFORM public.store_receive_event('evt_free00000000',repeat('5',64),'checkout.session.completed');
  PERFORM public.store_apply_paid('evt_free00000000',(created->>'order_id')::uuid,'cs_test_free00000000',jsonb_build_object('total_minor',0,'discount_minor',2000));
  PERFORM public.store_link_account('free@example.com',other,'setup');
  UPDATE skager_private.store_orders SET paid_at = now() - interval '15 days' WHERE id = (created->>'order_id')::uuid;
  IF public.store_refund_request(other,(created->>'order_id')::uuid)->>'state' <> 'not_refundable' THEN RAISE EXCEPTION 'Late refund allowed'; END IF;
  UPDATE skager_private.store_orders SET paid_at = now() WHERE id = (created->>'order_id')::uuid;
  IF public.store_refund_request(buyer,(created->>'order_id')::uuid)->>'state' <> 'not_refundable' THEN RAISE EXCEPTION 'Foreign refund allowed'; END IF;
  -- Separate statements: a STABLE read in the same expression would see the pre-refund snapshot.
  result := public.store_refund_request(other,(created->>'order_id')::uuid);
  IF result->>'state' <> 'refunded' THEN RAISE EXCEPTION 'Zero-total refund state %', result; END IF;
  IF public.store_customer_state('free@example.com')->>'state' <> 'revoked' THEN RAISE EXCEPTION 'Zero-total refund did not close'; END IF;

  -- Outbox: claim, failure backoff, success.
  claimed := public.store_email_claim(50);
  IF jsonb_array_length(claimed) < 4 THEN RAISE EXCEPTION 'Outbox claim returned %', jsonb_array_length(claimed); END IF;
  IF jsonb_array_length(public.store_email_claim(50)) <> 0 THEN RAISE EXCEPTION 'Claimed twice'; END IF;
  PERFORM public.store_email_result((claimed->0->>'id')::bigint,false,NULL);
  PERFORM public.store_email_result((claimed->1->>'id')::bigint,true,'re_123');
  IF (SELECT status FROM skager_private.email_outbox WHERE id = (claimed->0->>'id')::bigint) <> 'pending'
    OR (SELECT status FROM skager_private.email_outbox WHERE id = (claimed->1->>'id')::bigint) <> 'sent' THEN RAISE EXCEPTION 'Outbox state'; END IF;
  PERFORM public.store_job_maintenance();

  IF has_function_privilege('anon','public.store_catalog()','EXECUTE')
    OR has_function_privilege('authenticated','public.store_portal_access(uuid)','EXECUTE')
    OR has_table_privilege('service_role','skager_private.store_orders','SELECT')
    OR NOT has_function_privilege('service_role','public.store_session_get(text)','EXECUTE') THEN
    RAISE EXCEPTION 'Store privilege boundary wrong';
  END IF;
END;
$test$;
ROLLBACK;
