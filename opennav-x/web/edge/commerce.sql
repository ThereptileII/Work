-- SCRUM-36/37/41: closed TEST-only commerce. Apply atomically as project owner.
-- Private tables are never exposed through PostgREST; only service_role may call
-- these fixed-search-path RPCs. The Edge router authenticates account subjects.
begin;
create schema if not exists skager_private;

create table skager_private.commerce_orders (
  id uuid primary key default gen_random_uuid(),
  account_id uuid not null references auth.users(id) on delete restrict,
  environment text not null default 'staging' check (environment = 'staging'),
  product_code text not null default 'skager-beta' check (product_code = 'skager-beta'),
  list_amount_minor bigint not null default 2000 check (list_amount_minor = 2000),
  currency text not null default 'USD' check (currency = 'USD'),
  price_id text not null check (price_id ~ '^price_[A-Za-z0-9]{8,128}$'),
  allowed_coupon_ids text[] not null default '{}',
  status text not null default 'pending' check (status in ('pending','fulfilled','refunded','suspended','expired','failed')),
  created_at timestamptz not null default now()
);
create index commerce_orders_owner on skager_private.commerce_orders(account_id, created_at desc);
create table skager_private.commerce_bindings (
  order_id uuid primary key references skager_private.commerce_orders(id) on delete restrict,
  session_id text unique not null check (session_id ~ '^cs_test_[A-Za-z0-9]{8,200}$'),
  checkout_url text not null,
  created_at timestamptz not null default now()
);
create table skager_private.commerce_events (
  event_id text primary key check (event_id ~ '^evt_[A-Za-z0-9]{8,200}$'),
  digest text not null check (digest ~ '^[0-9a-f]{64}$'),
  event_type text not null check (event_type in ('checkout.session.completed','checkout.session.expired',
    'checkout.session.async_payment_succeeded','checkout.session.async_payment_failed','charge.refunded')),
  provider_created bigint not null check (provider_created > 0),
  reference text not null,
  provider_id text not null,
  state text not null default 'received' check (state in ('received','applied','rejected')),
  received_at timestamptz not null default now(),
  processed_at timestamptz
);
create table skager_private.commerce_settlements (
  order_id uuid primary key references skager_private.commerce_orders(id) on delete restrict,
  event_id text not null references skager_private.commerce_events(event_id) on delete restrict,
  payment_intent_id text unique,
  payment_status text not null,
  subtotal_minor bigint not null check (subtotal_minor = 2000),
  discount_minor bigint not null check (discount_minor between 0 and 2000),
  tax_minor bigint not null check (tax_minor = 0),
  total_minor bigint not null check (total_minor = subtotal_minor - discount_minor),
  coupon_ids text[] not null default '{}',
  created_at timestamptz not null default now(),
  check ((total_minor > 0 and payment_status = 'paid' and payment_intent_id is not null and payment_intent_id ~ '^pi_[A-Za-z0-9]{8,200}$')
    or (total_minor = 0 and payment_status = 'no_payment_required' and payment_intent_id is null
      and discount_minor = 2000 and cardinality(coupon_ids) = 1))
);
create table skager_private.commerce_refunds (
  payment_intent_id text primary key check (payment_intent_id ~ '^pi_[A-Za-z0-9]{8,200}$'),
  event_id text not null references skager_private.commerce_events(event_id) on delete restrict,
  charged_minor bigint not null check (charged_minor between 1 and 2000),
  refunded_minor bigint not null check (refunded_minor > 0 and refunded_minor <= charged_minor),
  created_at timestamptz not null default now()
);
create table skager_private.commerce_entitlements (
  order_id uuid primary key references skager_private.commerce_orders(id) on delete restrict,
  account_id uuid not null references auth.users(id) on delete restrict,
  environment text not null default 'staging' check (environment = 'staging'),
  product_code text not null default 'skager-beta' check (product_code = 'skager-beta'),
  status text not null check (status in ('active','refunded','suspended')),
  state_version bigint not null default 1 check (state_version > 0),
  created_at timestamptz not null default now()
);
create index commerce_entitlements_owner on skager_private.commerce_entitlements(account_id, status);
create table skager_private.commerce_effects (
  id bigint generated always as identity primary key,
  event_id text not null references skager_private.commerce_events(event_id) on delete restrict,
  order_id uuid not null references skager_private.commerce_orders(id) on delete restrict,
  kind text not null,
  from_status text not null,
  to_status text not null,
  resulting_version bigint not null,
  -- Explicit money truth: a full discount records zero TEST funds received.
  details jsonb not null default '{}'::jsonb,
  created_at timestamptz not null default now(),
  unique(event_id, kind)
);

create function skager_private.commerce_immutable() returns trigger language plpgsql set search_path = pg_catalog as $$
begin
  raise exception 'Commerce evidence is immutable' using errcode = '23514';
end;
$$;
create function skager_private.commerce_order_guard() returns trigger language plpgsql set search_path = pg_catalog as $$
begin
  if tg_op = 'DELETE' or (to_jsonb(new) - 'status') is distinct from (to_jsonb(old) - 'status')
    or (old.status in ('refunded','suspended') and new.status <> old.status) then
    raise exception 'Commerce order ownership and terms are immutable' using errcode = '23514';
  end if;
  return new;
end;
$$;
create function skager_private.commerce_event_guard() returns trigger language plpgsql set search_path = pg_catalog as $$
begin
  if tg_op = 'DELETE' or (to_jsonb(new) - array['state','processed_at']) is distinct from
    (to_jsonb(old) - array['state','processed_at'])
    or (old.state <> 'received' and new.state <> old.state) then
    raise exception 'Commerce event receipt is immutable' using errcode = '23514';
  end if;
  return new;
end;
$$;
create function skager_private.commerce_entitlement_guard() returns trigger language plpgsql set search_path = pg_catalog as $$
declare owner_id uuid;
begin
  if tg_op = 'DELETE' then
    raise exception 'Commerce entitlement is immutable' using errcode = '23514';
  end if;
  select account_id into owner_id from skager_private.commerce_orders where id = new.order_id;
  if owner_id is distinct from new.account_id then
    raise exception 'Commerce entitlement owner mismatch' using errcode = '23514';
  end if;
  if tg_op = 'UPDATE' then
    if (to_jsonb(new) - array['status','state_version']) is distinct from
      (to_jsonb(old) - array['status','state_version'])
      or old.status in ('refunded','suspended') or new.status not in ('refunded','suspended')
      or new.state_version <> old.state_version + 1 then
      raise exception 'Commerce entitlement cannot be rebound or reactivated' using errcode = '23514';
    end if;
  end if;
  return new;
end;
$$;
create trigger commerce_order_guard before update or delete on skager_private.commerce_orders
  for each row execute function skager_private.commerce_order_guard();
create trigger commerce_binding_immutable before update or delete on skager_private.commerce_bindings
  for each row execute function skager_private.commerce_immutable();
create trigger commerce_event_guard before update or delete on skager_private.commerce_events
  for each row execute function skager_private.commerce_event_guard();
create trigger commerce_settlement_immutable before update or delete on skager_private.commerce_settlements
  for each row execute function skager_private.commerce_immutable();
create trigger commerce_refund_immutable before update or delete on skager_private.commerce_refunds
  for each row execute function skager_private.commerce_immutable();
create trigger commerce_entitlement_guard before insert or update or delete on skager_private.commerce_entitlements
  for each row execute function skager_private.commerce_entitlement_guard();
create trigger commerce_effect_immutable before update or delete on skager_private.commerce_effects
  for each row execute function skager_private.commerce_immutable();

create function public.commerce_reserve_checkout(p_account_id uuid, p_price_id text, p_allowed_coupon_ids text[])
returns jsonb language plpgsql security definer set search_path = pg_catalog as $$
declare item skager_private.commerce_orders; binding skager_private.commerce_bindings;
begin
  if p_account_id is null or p_price_id !~ '^price_[A-Za-z0-9]{8,128}$'
    or cardinality(p_allowed_coupon_ids) > 100 or p_allowed_coupon_ids is null
    or exists(select 1 from unnest(p_allowed_coupon_ids) x where x is null or x !~ '^[A-Za-z0-9_-]{1,128}$') then
    raise exception 'Invalid checkout configuration' using errcode = '22023';
  end if;
  perform pg_advisory_xact_lock(hashtextextended(p_account_id::text, 36));
  -- Provider idempotency has finite retention; never reuse an old unresolved order.
  update skager_private.commerce_orders set status = 'expired'
    where account_id = p_account_id and status = 'pending' and created_at < now() - interval '23 hours';
  select * into item from skager_private.commerce_orders where account_id = p_account_id
    and status = 'pending' and price_id = p_price_id and allowed_coupon_ids = p_allowed_coupon_ids
    order by created_at limit 1 for update;
  if not found then
    insert into skager_private.commerce_orders(account_id, price_id, allowed_coupon_ids)
      values(p_account_id, p_price_id, p_allowed_coupon_ids) returning * into item;
  end if;
  select * into binding from skager_private.commerce_bindings where order_id = item.id;
  return jsonb_build_object('order_id',item.id,'session_id',binding.session_id,'checkout_url',binding.checkout_url);
end;
$$;

create function public.commerce_bind_checkout(p_account_id uuid, p_order_id uuid, p_session_id text, p_checkout_url text)
returns jsonb language plpgsql security definer set search_path = pg_catalog as $$
declare item skager_private.commerce_orders; binding skager_private.commerce_bindings;
begin
  select * into item from skager_private.commerce_orders where id = p_order_id for update;
  if not found or item.account_id is distinct from p_account_id
    or p_session_id !~ '^cs_test_[A-Za-z0-9]{8,200}$'
    or length(p_checkout_url) > 4096 or p_checkout_url !~
      ('^https://checkout[.]stripe[.]com/c/pay/' || p_session_id || '(#[^[:cntrl:][:space:]]*)?$') then
    raise exception 'Checkout binding rejected' using errcode = '22023';
  end if;
  insert into skager_private.commerce_bindings(order_id, session_id, checkout_url)
    values(p_order_id,p_session_id,p_checkout_url) on conflict(order_id) do nothing;
  select * into binding from skager_private.commerce_bindings where order_id = p_order_id;
  if binding.session_id <> p_session_id or binding.checkout_url <> p_checkout_url then
    raise exception 'Checkout binding conflict' using errcode = '23514';
  end if;
  return jsonb_build_object('state','bound');
end;
$$;

create function public.commerce_receive_event(p_event_id text, p_digest text, p_event_type text,
  p_created bigint, p_reference text, p_provider_id text)
returns jsonb language plpgsql security definer set search_path = pg_catalog as $$
declare item skager_private.commerce_events;
begin
  insert into skager_private.commerce_events(event_id,digest,event_type,provider_created,reference,provider_id)
    values(p_event_id,p_digest,p_event_type,p_created,p_reference,p_provider_id) on conflict(event_id) do nothing;
  select * into item from skager_private.commerce_events where event_id = p_event_id for update;
  if item.digest <> p_digest or item.event_type <> p_event_type or item.provider_created <> p_created
    or item.reference <> p_reference or item.provider_id <> p_provider_id then
    return jsonb_build_object('state','conflict');
  end if;
  return jsonb_build_object('state',item.state);
end;
$$;

create function public.commerce_checkout_binding(p_order_id uuid, p_session_id text)
returns jsonb language plpgsql security definer set search_path = pg_catalog as $$
declare item skager_private.commerce_orders; binding skager_private.commerce_bindings;
begin
  select * into item from skager_private.commerce_orders where id = p_order_id;
  if not found then return jsonb_build_object('state','rejected'); end if;
  select * into binding from skager_private.commerce_bindings where order_id = p_order_id;
  if not found then return jsonb_build_object('state','pending'); end if;
  if binding.session_id <> p_session_id then return jsonb_build_object('state','rejected'); end if;
  return jsonb_build_object('state','bound','price_id',item.price_id,'allowed_coupon_ids',item.allowed_coupon_ids);
end;
$$;

create function public.commerce_reject_event(p_event_id text, p_digest text)
returns jsonb language plpgsql security definer set search_path = pg_catalog as $$
begin
  update skager_private.commerce_events set state = 'rejected', processed_at = now()
    where event_id = p_event_id and digest = p_digest and state = 'received';
  return jsonb_build_object('state','rejected');
end;
$$;

create function public.commerce_close_checkout(p_event_id text,p_digest text,p_order_id uuid,p_session_id text)
returns jsonb language plpgsql security definer set search_path = pg_catalog as $$
declare receipt skager_private.commerce_events;
begin
  perform pg_advisory_xact_lock(1936417121,36);
  select * into receipt from skager_private.commerce_events where event_id = p_event_id for update;
  if not found or receipt.digest <> p_digest then raise exception 'Event rejected'; end if;
  if receipt.state <> 'received' then return jsonb_build_object('state',receipt.state); end if;
  if receipt.reference <> p_order_id::text or receipt.provider_id <> p_session_id
    or receipt.event_type not in ('checkout.session.expired','checkout.session.async_payment_failed')
    or not exists(select 1 from skager_private.commerce_bindings where order_id = p_order_id and session_id = p_session_id) then
    return public.commerce_reject_event(p_event_id,p_digest);
  end if;
  update skager_private.commerce_orders set status = case when receipt.event_type = 'checkout.session.expired' then 'expired' else 'failed' end
    where id = p_order_id and status = 'pending';
  update skager_private.commerce_events set state = 'applied', processed_at = now() where event_id = p_event_id;
  return jsonb_build_object('state','applied');
end;
$$;

create function public.commerce_apply_checkout(p_event_id text,p_digest text,p_checkout jsonb)
returns jsonb language plpgsql security definer set search_path = pg_catalog as $$
declare receipt skager_private.commerce_events; item skager_private.commerce_orders; refund skager_private.commerce_refunds;
  order_uuid uuid; coupons text[]; total bigint; discount bigint; intent text; target text; effect text;
begin
  -- One short atomic beta fulfillment stream closes the absent-row race between
  -- a refund-first event and a concurrent completion from another worker.
  perform pg_advisory_xact_lock(1936417121,36);
  select * into receipt from skager_private.commerce_events where event_id = p_event_id for update;
  if not found or receipt.digest <> p_digest then raise exception 'Event rejected'; end if;
  if receipt.state <> 'received' then return jsonb_build_object('state',receipt.state); end if;
  order_uuid := (p_checkout->>'order_id')::uuid;
  select * into item from skager_private.commerce_orders where id = order_uuid for update;
  if not found or receipt.reference <> order_uuid::text or receipt.provider_id <> p_checkout->>'session_id'
    or receipt.event_type not in ('checkout.session.completed','checkout.session.async_payment_succeeded')
    or not exists(select 1 from skager_private.commerce_bindings where order_id = item.id and session_id = p_checkout->>'session_id') then
    return public.commerce_reject_event(p_event_id,p_digest);
  end if;
  if item.status in ('refunded','suspended') or exists(select 1 from skager_private.commerce_settlements where order_id = item.id) then
    update skager_private.commerce_events set state = 'applied',processed_at = now() where event_id = p_event_id;
    return jsonb_build_object('state','applied');
  end if;
  select coalesce(array_agg(value),'{}'::text[]) into coupons from jsonb_array_elements_text(p_checkout->'coupon_ids');
  total := (p_checkout->>'total_minor')::bigint; discount := (p_checkout->>'discount_minor')::bigint;
  intent := p_checkout->>'payment_intent_id';
  if p_checkout->>'price_id' is distinct from item.price_id
    or (p_checkout->>'subtotal_minor')::bigint is distinct from 2000::bigint
    or (p_checkout->>'tax_minor')::bigint is distinct from 0::bigint
    or total is null or discount is null or total < 0 or total > 2000 or discount < 0 or discount > 2000
    or total + discount <> 2000 or not (coupons <@ item.allowed_coupon_ids)
    or cardinality(coupons) > 1 or (discount > 0 and cardinality(coupons) <> 1)
    or (discount = 0 and cardinality(coupons) <> 0)
    or not ((total > 0 and p_checkout->>'payment_status' = 'paid' and intent is not null and intent ~ '^pi_[A-Za-z0-9]{8,200}$')
      or (total = 0 and p_checkout->>'payment_status' = 'no_payment_required' and intent is null and discount = 2000)) then
    return public.commerce_reject_event(p_event_id,p_digest);
  end if;
  select * into refund from skager_private.commerce_refunds where payment_intent_id = intent;
  if found and refund.charged_minor <> total then raise exception 'Refund binding rejected'; end if;
  target := case when refund.payment_intent_id is null then 'active' else 'refunded' end;
  effect := case when target = 'active' then 'checkout_fulfilled' else 'refund_revoked' end;
  insert into skager_private.commerce_settlements(order_id,event_id,payment_intent_id,payment_status,
    subtotal_minor,discount_minor,tax_minor,total_minor,coupon_ids)
    values(item.id,p_event_id,intent,p_checkout->>'payment_status',2000,discount,0,total,coupons);
  insert into skager_private.commerce_entitlements(order_id,account_id,status) values(item.id,item.account_id,target);
  update skager_private.commerce_orders set status = case when target = 'active' then 'fulfilled' else 'refunded' end where id = item.id;
  insert into skager_private.commerce_effects(event_id,order_id,kind,from_status,to_status,resulting_version,details)
    values(p_event_id,item.id,effect,'',target,1,jsonb_build_object('test_only',true,
      'payment_status',p_checkout->>'payment_status','provider_test_paid_minor',total,'real_funds_received_minor',0,'discount_minor',discount));
  update skager_private.commerce_events set state = 'applied',processed_at = now() where event_id = p_event_id;
  return jsonb_build_object('state','applied');
end;
$$;

create function public.commerce_apply_refund(p_event_id text,p_digest text,p_charge_id text,p_intent_id text,
  p_charged_minor bigint,p_refunded_minor bigint)
returns jsonb language plpgsql security definer set search_path = pg_catalog as $$
declare receipt skager_private.commerce_events; settlement skager_private.commerce_settlements; entitlement skager_private.commerce_entitlements;
  marker skager_private.commerce_refunds;
begin
  perform pg_advisory_xact_lock(1936417121,36);
  select * into receipt from skager_private.commerce_events where event_id = p_event_id for update;
  if not found or receipt.digest <> p_digest then raise exception 'Event rejected'; end if;
  if receipt.state <> 'received' then return jsonb_build_object('state',receipt.state); end if;
  if receipt.event_type <> 'charge.refunded' or receipt.reference <> p_charge_id
    or receipt.provider_id <> p_charge_id or p_charge_id !~ '^ch_[A-Za-z0-9]{8,200}$'
    or p_intent_id !~ '^pi_[A-Za-z0-9]{8,200}$'
    or p_charged_minor not between 1 and 2000 or p_refunded_minor not between 1 and p_charged_minor then
    return public.commerce_reject_event(p_event_id,p_digest);
  end if;
  select * into settlement from skager_private.commerce_settlements where payment_intent_id = p_intent_id;
  if found and settlement.total_minor <> p_charged_minor then
    return public.commerce_reject_event(p_event_id,p_digest);
  end if;
  insert into skager_private.commerce_refunds(payment_intent_id,event_id,charged_minor,refunded_minor)
    values(p_intent_id,p_event_id,p_charged_minor,p_refunded_minor) on conflict(payment_intent_id) do nothing;
  select * into marker from skager_private.commerce_refunds where payment_intent_id = p_intent_id;
  if marker.charged_minor <> p_charged_minor then return public.commerce_reject_event(p_event_id,p_digest); end if;
  if settlement.order_id is not null then
    select * into entitlement from skager_private.commerce_entitlements where order_id = settlement.order_id for update;
    if entitlement.status = 'active' then
      update skager_private.commerce_entitlements set status = 'refunded',state_version = state_version + 1 where order_id = settlement.order_id;
      update skager_private.commerce_orders set status = 'refunded' where id = settlement.order_id and status <> 'suspended';
      insert into skager_private.commerce_effects(event_id,order_id,kind,from_status,to_status,resulting_version,details)
        values(p_event_id,settlement.order_id,'refund_revoked','active','refunded',entitlement.state_version + 1,
          jsonb_build_object('refund_policy','any_refund_revokes','refunded_minor',p_refunded_minor));
    end if;
  end if;
  update skager_private.commerce_events set state = 'applied',processed_at = now() where event_id = p_event_id;
  return jsonb_build_object('state','applied');
end;
$$;

create function public.commerce_portal_access(p_account_id uuid)
returns jsonb language sql stable security definer set search_path = pg_catalog as $$
  select jsonb_build_object('entitlement',(
    select jsonb_build_object('status',e.status,'product_code',e.product_code,'environment',e.environment)
    from skager_private.commerce_entitlements e where e.account_id = p_account_id
      and e.environment = 'staging' and e.product_code = 'skager-beta'
    order by (e.status = 'active') desc,e.created_at desc limit 1),
    'orders',coalesce((select jsonb_agg(item order by created_at desc) from (
      select o.id,o.created_at,o.status,coalesce(s.total_minor,o.list_amount_minor) as amount_minor,o.currency
      from skager_private.commerce_orders o left join skager_private.commerce_settlements s on s.order_id = o.id
      where o.account_id = p_account_id and o.environment = 'staging' and o.product_code = 'skager-beta'
      order by o.created_at desc limit 100) item),'[]'::jsonb));
$$;

-- Every table uses RLS with no client policy. Definer RPCs alone own mutations.
do $$
declare name text;
begin
  foreach name in array array['commerce_orders','commerce_bindings','commerce_events','commerce_settlements',
    'commerce_refunds','commerce_entitlements','commerce_effects'] loop
    execute format('alter table skager_private.%I enable row level security',name);
    execute format('revoke all on table skager_private.%I from public,anon,authenticated,service_role',name);
  end loop;
end;
$$;
revoke all on function skager_private.commerce_immutable(),skager_private.commerce_order_guard(),skager_private.commerce_event_guard(),
  skager_private.commerce_entitlement_guard() from public,anon,authenticated,service_role;
revoke all on function public.commerce_reserve_checkout(uuid,text,text[]),
  public.commerce_bind_checkout(uuid,uuid,text,text), public.commerce_receive_event(text,text,text,bigint,text,text),
  public.commerce_checkout_binding(uuid,text),public.commerce_reject_event(text,text),
  public.commerce_close_checkout(text,text,uuid,text),public.commerce_apply_checkout(text,text,jsonb),
  public.commerce_apply_refund(text,text,text,text,bigint,bigint),public.commerce_portal_access(uuid)
  from public,anon,authenticated;
grant execute on function public.commerce_reserve_checkout(uuid,text,text[]),
  public.commerce_bind_checkout(uuid,uuid,text,text),public.commerce_receive_event(text,text,text,bigint,text,text),
  public.commerce_checkout_binding(uuid,text),public.commerce_reject_event(text,text),
  public.commerce_close_checkout(text,text,uuid,text),public.commerce_apply_checkout(text,text,jsonb),
  public.commerce_apply_refund(text,text,text,text,bigint,bigint),public.commerce_portal_access(uuid) to service_role;
commit;
