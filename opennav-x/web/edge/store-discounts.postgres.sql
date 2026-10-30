-- Disposable CI database ONLY: rollback-only assertions for store-discounts.sql.
BEGIN;
DO $test$
DECLARE account uuid := 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb';
  cloud uuid; gps uuid; result jsonb;
BEGIN
  IF current_database() <> 'scrum94_schema' THEN RAISE EXCEPTION 'Disposable database required'; END IF;
  INSERT INTO auth.users(id,email,email_confirmed_at) VALUES (account,'cloud-code@example.test',now());
  INSERT INTO skager_private.store_customers(email,account_id,stripe_customer_id,password_set)
    VALUES ('cloud-code@example.test',account,'cus_codefixture0001',true);
  SELECT id INTO cloud FROM skager_private.store_products WHERE kind = 'subscription' AND active ORDER BY sort LIMIT 1;
  SELECT id INTO gps FROM skager_private.store_products WHERE kind = 'hardware' AND active ORDER BY sort LIMIT 1;

  -- A Cloud order keeps its validated promotion and coupon.
  result := public.store_create_order('subscription','cloud-code@example.test',account,
    jsonb_build_array(jsonb_build_object('product_id',cloud,'quantity',1)),'promo_codefixture0001','cloud_fixture');
  IF result->>'state' <> 'created' OR NOT EXISTS (SELECT 1 FROM skager_private.store_orders
      WHERE id = (result->>'order_id')::uuid AND promotion_id = 'promo_codefixture0001' AND coupon_id = 'cloud_fixture') THEN
    RAISE EXCEPTION 'Cloud discount not bound: %', result;
  END IF;

  -- Shop orders still never carry a discount, and promotion and coupon stay paired.
  BEGIN
    PERFORM public.store_create_order('shop','cloud-code@example.test',account,
      jsonb_build_array(jsonb_build_object('product_id',gps,'quantity',1)),'promo_codefixture0001','cloud_fixture');
    RAISE EXCEPTION 'Shop discount accepted';
  EXCEPTION WHEN raise_exception THEN
    IF SQLERRM <> 'Invalid discount' THEN RAISE; END IF;
  END;
  BEGIN
    PERFORM public.store_create_order('subscription','cloud-code@example.test',account,
      jsonb_build_array(jsonb_build_object('product_id',cloud,'quantity',1)),'promo_codefixture0001',NULL);
    RAISE EXCEPTION 'Unpaired discount accepted';
  EXCEPTION WHEN raise_exception THEN
    IF SQLERRM <> 'Invalid discount' THEN RAISE; END IF;
  END;
  -- Replacing the function keeps its grants: only the server may create orders.
  IF has_function_privilege('anon','public.store_create_order(text,text,uuid,jsonb,text,text)','EXECUTE')
    OR has_function_privilege('authenticated','public.store_create_order(text,text,uuid,jsonb,text,text)','EXECUTE')
    OR NOT has_function_privilege('service_role','public.store_create_order(text,text,uuid,jsonb,text,text)','EXECUTE') THEN
    RAISE EXCEPTION 'Order RPC grant boundary wrong';
  END IF;
END $test$;
ROLLBACK;
