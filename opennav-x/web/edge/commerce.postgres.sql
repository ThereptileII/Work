-- Run after commerce.sql in a disposable database with auth.users(id uuid primary
-- key) and the Supabase anon, authenticated, and service_role roles present.
-- The transaction rolls back all fixture data. Any failed assertion aborts psql.
begin;
do $test$
declare
  account_one uuid := '11111111-1111-4111-8111-111111111111';
  account_two uuid := '22222222-2222-4222-8222-222222222222';
  first_order uuid;
  second_order uuid;
  receipt jsonb;
  access jsonb;
  one_digest text := repeat('a',64);
  two_digest text := repeat('b',64);
begin
  insert into auth.users(id) values(account_one),(account_two);
  first_order := (public.commerce_reserve_checkout(account_one,'price_12345678',array['full100'])->>'order_id')::uuid;
  if first_order is null then raise exception 'reservation failed'; end if;
  perform public.commerce_bind_checkout(account_one,first_order,'cs_test_00000001',
    'https://checkout.stripe.com/c/pay/cs_test_00000001#test');
  begin
    perform public.commerce_bind_checkout(account_two,first_order,'cs_test_00000002',
      'https://checkout.stripe.com/c/pay/cs_test_00000002#test');
    raise exception 'cross-account binding was accepted';
  exception when sqlstate '22023' then null;
  end;
  receipt := public.commerce_receive_event('evt_00000001',one_digest,'checkout.session.completed',
    1700000000,first_order::text,'cs_test_00000001');
  if receipt->>'state' <> 'received' then raise exception 'receipt not new'; end if;
  receipt := public.commerce_apply_checkout('evt_00000001',one_digest,
    jsonb_build_object('order_id',first_order,'session_id','cs_test_00000001',
      'price_id','price_12345678','subtotal_minor',2000,'discount_minor',2000,
      'tax_minor',0,'total_minor',0,'payment_status','no_payment_required',
      'payment_intent_id',null,'coupon_ids',jsonb_build_array('full100')));
  if receipt->>'state' <> 'applied' then raise exception 'free checkout not applied'; end if;
  access := public.commerce_portal_access(account_one);
  if access #>> '{entitlement,status}' <> 'active'
    or access #>> '{orders,0,amount_minor}' <> '0' then
    raise exception 'zero-total coupon did not grant full active access';
  end if;
  if (public.commerce_portal_access(account_two))->'entitlement' <> 'null'::jsonb then
    raise exception 'cross-account access leak';
  end if;
  if (select details->>'real_funds_received_minor' from skager_private.commerce_effects
      where order_id = first_order) <> '0' then
    raise exception 'zero-money audit absent';
  end if;
  if (public.commerce_receive_event('evt_00000001',one_digest,'checkout.session.completed',
      1700000000,first_order::text,'cs_test_00000001'))->>'state' <> 'applied' then
    raise exception 'duplicate delivery was not idempotent';
  end if;
  if (public.commerce_receive_event('evt_00000001',two_digest,'checkout.session.completed',
      1700000000,first_order::text,'cs_test_00000001'))->>'state' <> 'conflict' then
    raise exception 'event-id digest collision accepted';
  end if;

  -- Refund arrives before its checkout completion. The first application must
  -- create a terminal refunded entitlement without any active interval.
  second_order := (public.commerce_reserve_checkout(account_two,'price_12345678',array['full100'])->>'order_id')::uuid;
  perform public.commerce_bind_checkout(account_two,second_order,'cs_test_00000002',
    'https://checkout.stripe.com/c/pay/cs_test_00000002#test');
  perform public.commerce_receive_event('evt_00000002',two_digest,'charge.refunded',
    1700000001,'ch_00000002','ch_00000002');
  receipt := public.commerce_apply_refund('evt_00000002',two_digest,'ch_00000002',
    'pi_00000002',2000,100);
  if receipt->>'state' <> 'applied' then raise exception 'refund-first marker rejected'; end if;
  perform public.commerce_receive_event('evt_00000003',one_digest,'checkout.session.completed',
    1700000002,second_order::text,'cs_test_00000002');
  receipt := public.commerce_apply_checkout('evt_00000003',one_digest,
    jsonb_build_object('order_id',second_order,'session_id','cs_test_00000002',
      'price_id','price_12345678','subtotal_minor',2000,'discount_minor',0,
      'tax_minor',0,'total_minor',2000,'payment_status','paid',
      'payment_intent_id','pi_00000002','coupon_ids',jsonb_build_array()));
  if receipt->>'state' <> 'applied' or
    (select status from skager_private.commerce_entitlements where order_id = second_order) <> 'refunded'
    or (select count(*) from skager_private.commerce_effects where order_id = second_order) <> 1 then
    raise exception 'refund-first completion was not terminal and exactly once';
  end if;
  perform public.commerce_receive_event('evt_00000004',two_digest,'checkout.session.expired',
    1700000003,second_order::text,'cs_test_00000002');
  perform public.commerce_close_checkout('evt_00000004',two_digest,second_order,'cs_test_00000002');
  if (select status from skager_private.commerce_orders where id = second_order) <> 'refunded' then
    raise exception 'late expiration changed terminal order';
  end if;
  if has_function_privilege('authenticated',
      'public.commerce_portal_access(uuid)','EXECUTE') then
    raise exception 'client can execute commerce RPC';
  end if;
end;
$test$;
rollback;
