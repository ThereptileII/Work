-- SKAGER store: guest checkout that creates the account, shop, cloud subscription,
-- refunds that close the account, email outbox and admin. Apply after auth.sql,
-- commerce.sql and portal.sql, as the database owner, in one transaction.
-- Private tables are never exposed; only service_role may execute the RPCs.
begin;
create schema if not exists skager_private;
revoke all on schema skager_private from public, anon, authenticated;

-- ---------------------------------------------------------------- catalog
create table skager_private.store_settings (
  id integer primary key default 1 check (id = 1),
  shipping_minor integer not null default 1500 check (shipping_minor between 0 and 100000),
  shipping_countries text[] not null default array['SE','NO','DK','FI','DE','NL','BE','FR','ES','PT','IT','AT',
    'IE','PL','EE','LV','LT','CZ','SK','SI','HR','GR','LU','MT','CY','HU','RO','BG','IS','GB','CH'],
  updated_at timestamptz not null default now()
);
insert into skager_private.store_settings(id) values (1);

create table skager_private.store_products (
  id uuid primary key default gen_random_uuid(),
  slug text not null unique check (slug ~ '^[a-z0-9][a-z0-9-]{1,63}$'),
  kind text not null check (kind in ('access','hardware','subscription')),
  name text not null check (char_length(name) between 1 and 80 and name !~ '[[:cntrl:]]'),
  summary text not null default '' check (char_length(summary) <= 200 and summary !~ '[[:cntrl:]]'),
  description text not null default '' check (char_length(description) <= 4000),
  price_minor integer not null check (price_minor between 0 and 10000000),
  currency text not null default 'usd' check (currency = 'usd'),
  billing_interval text check (billing_interval is null or billing_interval = 'month'),
  image_key text check (image_key is null or image_key ~ '^products/[0-9a-f-]{36}/[0-9a-f]{64}\.(png|jpg|webp)$'),
  shippable boolean not null default false,
  checkout_addon boolean not null default false,
  active boolean not null default true,
  sort integer not null default 100 check (sort between 0 and 10000),
  created_at timestamptz not null default now(),
  updated_at timestamptz not null default now(),
  check ((kind = 'subscription') = (billing_interval is not null)),
  check (kind = 'hardware' or (not shippable and not checkout_addon))
);
-- The access price is fixed by the configured Stripe price; the row describes it.
insert into skager_private.store_products(slug,kind,name,summary,description,price_minor,billing_interval,shippable,checkout_addon,sort) values
 ('skager-beta-access','access','SKAGER Public Beta Access','SKAGER for Windows with every beta update, your account and support.','',2000,null,false,false,1),
 ('skager-cloud','subscription','SKAGER Cloud','Boat data, online dashboard and web route planning.','Cloud services for your boat. New services are added as they launch.',200,'month',false,false,2),
 ('gps-module','hardware','SKAGER GPS module','Plug-and-play USB multi-GNSS receiver for your helm PC.','A compact USB GPS/GNSS receiver that SKAGER and OpenCPN recognise straight away.',4900,null,true,true,10),
 ('helm-pc','hardware','SKAGER Helm PC','Fanless NUC with SKAGER and OpenCPN preinstalled.','A quiet, fanless mini PC prepared for the helm, with OpenCPN and SKAGER installed and ready to connect.',54900,null,true,false,20);

-- ---------------------------------------------------------------- customers and orders
create table skager_private.store_customers (
  email text primary key check (length(email) between 3 and 254 and email = lower(email)),
  account_id uuid unique references auth.users(id) on delete set null,
  status text not null default 'active' check (status in ('active','revoked')),
  password_set boolean not null default false,
  stripe_customer_id text check (stripe_customer_id is null or stripe_customer_id ~ '^cus_[A-Za-z0-9]{8,200}$'),
  created_at timestamptz not null default now(),
  revoked_at timestamptz
);
create table skager_private.store_orders (
  id uuid primary key default gen_random_uuid(),
  kind text not null check (kind in ('access','shop','subscription')),
  email text not null check (length(email) between 3 and 254 and email = lower(email)),
  account_id uuid references auth.users(id) on delete set null,
  status text not null default 'pending' check (status in ('pending','paid','refund_requested','refunded','expired')),
  fulfillment text not null default 'none' check (fulfillment in ('none','unfulfilled','shipped')),
  items jsonb not null check (jsonb_typeof(items) = 'array' and jsonb_array_length(items) between 1 and 10),
  promotion_id text check (promotion_id is null or promotion_id ~ '^promo_[A-Za-z0-9]{8,200}$'),
  coupon_id text check (coupon_id is null or coupon_id ~ '^[A-Za-z0-9_-]{1,128}$'),
  subtotal_minor integer not null check (subtotal_minor >= 0),
  shipping_minor integer not null default 0 check (shipping_minor >= 0),
  discount_minor integer not null default 0 check (discount_minor >= 0),
  total_minor integer check (total_minor is null or total_minor >= 0),
  session_id text unique check (session_id is null or session_id ~ '^cs_test_[A-Za-z0-9]{8,200}$'),
  checkout_url text,
  payment_intent_id text unique check (payment_intent_id is null or payment_intent_id ~ '^pi_[A-Za-z0-9]{8,200}$'),
  subscription_id text check (subscription_id is null or subscription_id ~ '^sub_[A-Za-z0-9]{8,200}$'),
  shipping jsonb,
  tracking text check (tracking is null or char_length(tracking) <= 200),
  created_at timestamptz not null default now(),
  paid_at timestamptz, refunded_at timestamptz, shipped_at timestamptz
);
create index store_orders_email on skager_private.store_orders(email, created_at desc);
create index store_orders_account on skager_private.store_orders(account_id, created_at desc);
create table skager_private.store_subscriptions (
  subscription_id text primary key check (subscription_id ~ '^sub_[A-Za-z0-9]{8,200}$'),
  email text not null references skager_private.store_customers(email),
  status text not null check (status in ('incomplete','incomplete_expired','trialing','active','past_due','canceled','unpaid','paused')),
  current_period_end timestamptz,
  cancel_at_period_end boolean not null default false,
  updated_at timestamptz not null default now()
);
create table skager_private.store_events (
  event_id text primary key check (event_id ~ '^evt_[A-Za-z0-9]{8,200}$'),
  digest text not null check (digest ~ '^[0-9a-f]{64}$'),
  event_type text not null check (length(event_type) <= 80),
  state text not null default 'received' check (state in ('received','applied','ignored')),
  received_at timestamptz not null default now()
);
create table skager_private.email_outbox (
  id bigint generated always as identity primary key,
  to_email text not null check (length(to_email) between 3 and 254),
  template text not null check (template ~ '^[a-z_]{3,40}$'),
  data jsonb not null default '{}'::jsonb,
  status text not null default 'pending' check (status in ('pending','sending','sent','failed')),
  attempts integer not null default 0,
  next_attempt_at timestamptz not null default now(),
  provider_id text,
  created_at timestamptz not null default now(),
  sent_at timestamptz
);
create index email_outbox_due on skager_private.email_outbox(status, next_attempt_at);
create table skager_private.store_admin_audit (
  id bigint generated always as identity primary key,
  admin_email text not null,
  action text not null,
  target text,
  at timestamptz not null default now()
);

-- Login purposes for the existing challenge table (login, account setup, password reset).
alter table skager_private.auth_challenges add column if not exists purpose text not null default 'login';
alter table skager_private.auth_challenges drop constraint if exists auth_challenges_purpose_check;
alter table skager_private.auth_challenges add constraint auth_challenges_purpose_check check (purpose in ('login','setup','reset'));

revoke all on all tables in schema skager_private from public, anon, authenticated;
revoke all on skager_private.store_settings, skager_private.store_products, skager_private.store_customers,
  skager_private.store_orders, skager_private.store_subscriptions, skager_private.store_events,
  skager_private.email_outbox, skager_private.store_admin_audit from service_role;
alter table skager_private.store_settings enable row level security;
alter table skager_private.store_products enable row level security;
alter table skager_private.store_customers enable row level security;
alter table skager_private.store_orders enable row level security;
alter table skager_private.store_subscriptions enable row level security;
alter table skager_private.store_events enable row level security;
alter table skager_private.email_outbox enable row level security;
alter table skager_private.store_admin_audit enable row level security;

-- ---------------------------------------------------------------- helpers (private)
create function skager_private.store_product_json(p skager_private.store_products) returns jsonb
language sql immutable set search_path = '' as $$
  select jsonb_build_object('id',p.id,'slug',p.slug,'kind',p.kind,'name',p.name,'summary',p.summary,
    'description',p.description,'price_minor',p.price_minor,'currency',p.currency,'interval',p.billing_interval,
    'image_key',p.image_key,'shippable',p.shippable,'checkout_addon',p.checkout_addon,'active',p.active,'sort',p.sort);
$$;
create function skager_private.store_order_json(o skager_private.store_orders) returns jsonb
language sql stable set search_path = '' as $$
  select jsonb_build_object('id',o.id,'kind',o.kind,'email',o.email,'status',o.status,'fulfillment',o.fulfillment,
    'items',o.items,'subtotal_minor',o.subtotal_minor,'shipping_minor',o.shipping_minor,
    'discount_minor',o.discount_minor,'total_minor',o.total_minor,'shipping',o.shipping,'tracking',o.tracking,
    'created_at',o.created_at,'paid_at',o.paid_at,'refunded_at',o.refunded_at,'shipped_at',o.shipped_at,
    'refundable', o.kind = 'access' and o.status = 'paid' and o.paid_at > now() - interval '14 days');
$$;
create function skager_private.store_enqueue(p_to text, p_template text, p_data jsonb) returns bigint
language sql set search_path = '' as $$
  insert into skager_private.email_outbox(to_email,template,data) values (lower(p_to),p_template,coalesce(p_data,'{}'::jsonb)) returning id;
$$;
create function skager_private.store_revoke(p_email text) returns void
language plpgsql set search_path = '' as $$
declare v_account uuid;
begin
  update skager_private.store_customers set status = 'revoked', revoked_at = coalesce(revoked_at, now())
    where email = p_email returning account_id into v_account;
  if v_account is not null then
    delete from skager_private.auth_sessions where subject = v_account;
  end if;
end $$;

-- ---------------------------------------------------------------- public catalog
create function public.store_catalog() returns jsonb
language sql stable security definer set search_path = '' as $$
  select jsonb_build_object(
    'products', coalesce((select jsonb_agg(skager_private.store_product_json(p) order by p.sort, p.name)
      from skager_private.store_products p where p.active), '[]'::jsonb),
    'shipping_minor', s.shipping_minor, 'shipping_countries', to_jsonb(s.shipping_countries))
  from skager_private.store_settings s where s.id = 1;
$$;

-- ---------------------------------------------------------------- checkout
-- p_items: [{"product_id": uuid, "quantity": int}] — prices always come from the catalog.
create function public.store_create_order(p_kind text, p_email text, p_account_id uuid, p_items jsonb,
  p_promotion_id text, p_coupon_id text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_email text := lower(btrim(p_email)); v_item jsonb; v_product skager_private.store_products;
  v_qty integer; v_snapshot jsonb := '[]'::jsonb; v_subtotal integer := 0; v_shippable boolean := false;
  v_shipping integer := 0; v_order skager_private.store_orders; v_customer skager_private.store_customers;
  v_access integer := 0; v_seen uuid[] := '{}';
begin
  if p_kind not in ('access','shop','subscription') or v_email !~ '^[^\s@]+@[^\s@]+\.[^\s@]+$'
     or jsonb_typeof(p_items) <> 'array' or jsonb_array_length(p_items) not between 1 and 10 then
    raise exception 'Invalid order';
  end if;
  select * into v_customer from skager_private.store_customers where email = v_email;
  if p_kind = 'access' then
    if v_customer.email is not null then
      return jsonb_build_object('state', case when v_customer.status = 'revoked' then 'revoked'
        when v_customer.account_id is null or not v_customer.password_set then 'setup' else 'exists' end);
    end if;
  else
    if p_account_id is null or v_customer.email is null or v_customer.status <> 'active'
       or v_customer.account_id is distinct from p_account_id then
      raise exception 'Account required';
    end if;
  end if;
  for v_item in select * from jsonb_array_elements(p_items) loop
    if jsonb_typeof(v_item) <> 'object' or (select count(*) from jsonb_object_keys(v_item)) <> 2
       or not (v_item ? 'product_id' and v_item ? 'quantity') or jsonb_typeof(v_item->'quantity') <> 'number' then
      raise exception 'Invalid item';
    end if;
    v_qty := (v_item->>'quantity')::integer;
    select * into v_product from skager_private.store_products where id = (v_item->>'product_id')::uuid and active;
    if v_product.id is null or v_qty not between 1 and 5 or v_product.id = any(v_seen) then raise exception 'Invalid item'; end if;
    v_seen := v_seen || v_product.id;
    if (p_kind = 'access' and not (v_product.kind = 'access' or v_product.checkout_addon))
       or (p_kind = 'shop' and v_product.kind <> 'hardware')
       or (p_kind = 'subscription' and v_product.kind <> 'subscription')
       or (v_product.kind <> 'hardware' and v_qty <> 1) then
      raise exception 'Invalid item';
    end if;
    if v_product.kind = 'access' then v_access := v_access + 1; end if;
    v_shippable := v_shippable or v_product.shippable;
    v_subtotal := v_subtotal + v_product.price_minor * v_qty;
    v_snapshot := v_snapshot || jsonb_build_array(jsonb_build_object('product_id',v_product.id,'slug',v_product.slug,
      'kind',v_product.kind,'name',v_product.name,'quantity',v_qty,'unit_minor',v_product.price_minor,
      'interval',v_product.billing_interval,'shippable',v_product.shippable,'image_key',v_product.image_key));
  end loop;
  if (p_kind = 'access' and v_access <> 1) or (p_kind = 'subscription' and jsonb_array_length(v_snapshot) <> 1) then
    raise exception 'Invalid order';
  end if;
  if (p_promotion_id is null) <> (p_coupon_id is null) or (p_promotion_id is not null and p_kind <> 'access') then
    raise exception 'Invalid discount';
  end if;
  if v_shippable then select s.shipping_minor into v_shipping from skager_private.store_settings s where s.id = 1; end if;
  insert into skager_private.store_orders(kind,email,account_id,fulfillment,items,promotion_id,coupon_id,subtotal_minor,shipping_minor)
    values (p_kind, v_email, p_account_id, case when v_shippable then 'unfulfilled' else 'none' end, v_snapshot,
      p_promotion_id, p_coupon_id, v_subtotal, v_shipping)
    returning * into v_order;
  return jsonb_build_object('state','created','order_id',v_order.id,'items',v_order.items,
    'subtotal_minor',v_order.subtotal_minor,'shipping_minor',v_order.shipping_minor,
    'stripe_customer_id',v_customer.stripe_customer_id,
    'shipping_countries',(select to_jsonb(s.shipping_countries) from skager_private.store_settings s where s.id = 1));
end $$;

create function public.store_bind_order(p_order_id uuid, p_session_id text, p_checkout_url text) returns boolean
language plpgsql security definer set search_path = '' as $$
begin
  update skager_private.store_orders set session_id = p_session_id, checkout_url = p_checkout_url
    where id = p_order_id and session_id is null and status = 'pending';
  if not found then raise exception 'Order not bindable'; end if;
  return true;
end $$;

-- Ownership check for webhooks: an order and its session must match exactly.
create function public.store_order_binding(p_order_id uuid, p_session_id text) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select skager_private.store_order_json(o) || jsonb_build_object('state','bound',
      'promotion_id',o.promotion_id,'coupon_id',o.coupon_id,'account_id',o.account_id)
    from skager_private.store_orders o where o.id = p_order_id and o.session_id = p_session_id),
    jsonb_build_object('state','rejected'));
$$;

create function public.store_receive_event(p_event_id text, p_digest text, p_event_type text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_events;
begin
  insert into skager_private.store_events(event_id,digest,event_type) values (p_event_id,p_digest,p_event_type)
    on conflict (event_id) do nothing;
  select * into v from skager_private.store_events where event_id = p_event_id for update;
  if v.digest <> p_digest then return jsonb_build_object('state','conflict'); end if;
  return jsonb_build_object('state', v.state);
end $$;

create function public.store_mark_event(p_event_id text, p_state text) returns boolean
language sql security definer set search_path = '' as $$
  update skager_private.store_events set state = p_state where event_id = p_event_id and p_state in ('applied','ignored') returning true;
$$;

-- A verified completed Checkout Session settles its bound order exactly once.
create function public.store_apply_paid(p_event_id text, p_order_id uuid, p_session_id text, p_paid jsonb) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_order skager_private.store_orders; v_customer text := nullif(p_paid->>'customer_id','');
  v_sub text := nullif(p_paid->>'subscription_id','');
begin
  select * into v_order from skager_private.store_orders where id = p_order_id and session_id = p_session_id for update;
  if v_order.id is null then raise exception 'Order not bound'; end if;
  if (select state from skager_private.store_events where event_id = p_event_id) <> 'received' then
    return jsonb_build_object('applied', false);
  end if;
  if v_order.status = 'pending' then
    update skager_private.store_orders set status = 'paid', paid_at = now(),
      total_minor = (p_paid->>'total_minor')::integer, discount_minor = (p_paid->>'discount_minor')::integer,
      shipping_minor = coalesce((p_paid->>'shipping_minor')::integer, shipping_minor),
      payment_intent_id = nullif(p_paid->>'payment_intent_id',''), subscription_id = v_sub,
      shipping = p_paid->'shipping'
      where id = v_order.id returning * into v_order;
    if v_order.kind = 'access' then
      insert into skager_private.store_customers(email, stripe_customer_id) values (v_order.email, v_customer)
        on conflict (email) do update set stripe_customer_id = coalesce(skager_private.store_customers.stripe_customer_id, excluded.stripe_customer_id);
      perform skager_private.store_enqueue(v_order.email,'welcome',jsonb_build_object('order',skager_private.store_order_json(v_order)));
    elsif v_order.kind = 'shop' then
      update skager_private.store_customers set stripe_customer_id = coalesce(stripe_customer_id, v_customer) where email = v_order.email;
      perform skager_private.store_enqueue(v_order.email,'shop_receipt',jsonb_build_object('order',skager_private.store_order_json(v_order)));
    else
      update skager_private.store_customers set stripe_customer_id = coalesce(stripe_customer_id, v_customer) where email = v_order.email;
      if v_sub is not null then
        insert into skager_private.store_subscriptions(subscription_id,email,status,current_period_end)
          values (v_sub, v_order.email, coalesce(p_paid->>'subscription_status','active'),
            to_timestamp(nullif(p_paid->>'current_period_end','')::bigint))
          on conflict (subscription_id) do update set status = excluded.status, updated_at = now();
      end if;
      perform skager_private.store_enqueue(v_order.email,'cloud_started',jsonb_build_object('order',skager_private.store_order_json(v_order)));
    end if;
    if v_order.fulfillment = 'unfulfilled' then
      perform skager_private.store_enqueue('admin','admin_order',jsonb_build_object('order',skager_private.store_order_json(v_order)));
    end if;
  end if;
  update skager_private.store_events set state = 'applied' where event_id = p_event_id;
  return jsonb_build_object('applied', true, 'kind', v_order.kind, 'status', v_order.status);
end $$;

create function public.store_apply_expired(p_event_id text, p_order_id uuid, p_session_id text) returns boolean
language plpgsql security definer set search_path = '' as $$
begin
  update skager_private.store_orders set status = 'expired'
    where id = p_order_id and session_id = p_session_id and status = 'pending';
  update skager_private.store_events set state = 'applied' where event_id = p_event_id;
  return true;
end $$;

-- Any refund of an access order closes the account: sessions are deleted and sign-in is refused.
create function public.store_apply_refund(p_event_id text, p_intent_id text, p_refunded_minor integer) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_order skager_private.store_orders; v_subs jsonb := '[]'::jsonb;
begin
  select * into v_order from skager_private.store_orders where payment_intent_id = p_intent_id for update;
  if v_order.id is null then
    update skager_private.store_events set state = 'ignored' where event_id = p_event_id;
    return jsonb_build_object('known', false);
  end if;
  if v_order.status <> 'refunded' and p_refunded_minor > 0 then
    update skager_private.store_orders set status = 'refunded', refunded_at = now() where id = v_order.id returning * into v_order;
    perform skager_private.store_enqueue(v_order.email,'refunded',jsonb_build_object('order',skager_private.store_order_json(v_order)));
    if v_order.kind = 'access' then
      perform skager_private.store_revoke(v_order.email);
      select coalesce(jsonb_agg(s.subscription_id),'[]'::jsonb) into v_subs from skager_private.store_subscriptions s
        where s.email = v_order.email and s.status not in ('canceled','incomplete_expired');
    end if;
  end if;
  update skager_private.store_events set state = 'applied' where event_id = p_event_id;
  return jsonb_build_object('known', true, 'kind', v_order.kind, 'cancel_subscriptions', v_subs);
end $$;

create function public.store_apply_subscription(p_event_id text, p_subscription_id text, p_status text,
  p_period_end bigint, p_cancel_at_period_end boolean) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_subscriptions;
begin
  update skager_private.store_subscriptions set status = p_status,
    current_period_end = coalesce(to_timestamp(p_period_end), current_period_end),
    cancel_at_period_end = coalesce(p_cancel_at_period_end, false), updated_at = now()
    where subscription_id = p_subscription_id returning * into v;
  update skager_private.store_events set state = case when v.subscription_id is null then 'ignored' else 'applied' end
    where event_id = p_event_id;
  if v.subscription_id is not null and p_status = 'canceled' then
    perform skager_private.store_enqueue(v.email,'cloud_ended','{}'::jsonb);
  end if;
  return jsonb_build_object('known', v.subscription_id is not null);
end $$;

-- ---------------------------------------------------------------- account setup and sign-in
create function public.store_claim_state(p_session_id text, p_email text) returns jsonb
language sql stable security definer set search_path = '' as $$
  with o as (
    select * from skager_private.store_orders
    where kind = 'access' and ((p_session_id is not null and session_id = p_session_id)
      or (p_session_id is null and email = lower(btrim(p_email))))
    order by created_at desc limit 1)
  select coalesce((select jsonb_build_object('email', o.email,
      'state', case when c.status = 'revoked' then 'revoked'
        when o.status = 'pending' then 'pending'
        when o.status in ('expired') then 'expired'
        when c.email is null then 'pending'
        when c.account_id is not null and c.password_set then 'done'
        else 'ready' end)
    from o left join skager_private.store_customers c on c.email = o.email),
    jsonb_build_object('state','unknown'));
$$;

create function public.store_challenge_create(p_hash text, p_email text, p_purpose text) returns boolean
language plpgsql security definer set search_path = '' as $$
begin
  delete from skager_private.auth_challenges where expires_at <= now();
  insert into skager_private.auth_challenges(token_hash,email,purpose) values (p_hash, lower(p_email), p_purpose);
  return true;
end $$;
create function public.store_challenge_take(p_hash text) returns table(email text, purpose text)
language sql security definer set search_path = '' as $$
  delete from skager_private.auth_challenges c where c.token_hash = p_hash and c.expires_at > now()
    returning c.email, c.purpose;
$$;

-- Who may receive a code for which purpose. Returns the customer's state.
create function public.store_customer_state(p_email text) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select jsonb_build_object('state', c.status, 'claimed', c.account_id is not null,
      'password_set', c.password_set, 'account_id', c.account_id)
    from skager_private.store_customers c where c.email = lower(btrim(p_email))),
    jsonb_build_object('state','none'));
$$;

-- After a verified code: bind the canonical Auth subject to the customer and their orders.
create function public.store_link_account(p_email text, p_account_id uuid, p_purpose text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_customers;
begin
  select * into v from skager_private.store_customers where email = lower(p_email) for update;
  if v.email is null or v.status <> 'active' or (v.account_id is not null and v.account_id <> p_account_id) then
    raise exception 'Account unavailable';
  end if;
  update skager_private.store_customers set account_id = p_account_id,
    password_set = case when p_purpose in ('setup','reset') then false else password_set end
    where email = v.email returning * into v;
  update skager_private.store_orders set account_id = p_account_id where email = v.email and account_id is null;
  return jsonb_build_object('password_set', v.password_set);
end $$;

create function public.store_password_set(p_account_id uuid) returns boolean
language plpgsql security definer set search_path = '' as $$
begin
  update skager_private.store_customers set password_set = true where account_id = p_account_id and status = 'active';
  return found;
end $$;

-- Sessions only resolve for active customers. Revocation is immediate.
create function public.store_session_get(p_hash text) returns table(id uuid, email text, password_set boolean)
language sql stable security definer set search_path = '' as $$
  select u.id, u.email::text, c.password_set from skager_private.auth_sessions s
    join auth.users u on u.id = s.subject
    join skager_private.store_customers c on c.account_id = u.id and c.status = 'active'
    where s.token_hash = p_hash and s.expires_at > now() and u.email_confirmed_at is not null
      and (u.banned_until is null or u.banned_until <= now()) and u.deleted_at is null;
$$;

-- ---------------------------------------------------------------- portal
create function public.store_portal_access(p_account_id uuid) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select jsonb_build_object(
    'entitlement', case when c.status = 'active' and exists(select 1 from skager_private.store_orders o
        where o.email = c.email and o.kind = 'access' and o.status in ('paid','refund_requested'))
      then jsonb_build_object('status','active','product_code','skager-beta','environment','staging') else null end,
    'customer', jsonb_build_object('email', c.email, 'password_set', c.password_set, 'since', c.created_at,
      'has_billing', c.stripe_customer_id is not null),
    'orders', coalesce((select jsonb_agg(skager_private.store_order_json(o) order by o.created_at desc)
      from skager_private.store_orders o where o.email = c.email and o.status <> 'expired'), '[]'::jsonb),
    'subscription', (select jsonb_build_object('status', s.status, 'current_period_end', s.current_period_end,
        'cancel_at_period_end', s.cancel_at_period_end)
      from skager_private.store_subscriptions s where s.email = c.email order by s.updated_at desc limit 1),
    'stripe_customer_id', c.stripe_customer_id)
  from skager_private.store_customers c where c.account_id = p_account_id and c.status = 'active'),
  jsonb_build_object('entitlement', null));
$$;

-- Self-service refund of Beta Access within 14 days. Zero-total orders close immediately.
create function public.store_refund_request(p_account_id uuid, p_order_id uuid) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_orders;
begin
  select o.* into v from skager_private.store_orders o join skager_private.store_customers c on c.email = o.email
    where o.id = p_order_id and c.account_id = p_account_id and c.status = 'active' for update of o;
  if v.id is null or v.kind <> 'access' or v.status <> 'paid' or v.paid_at <= now() - interval '14 days' then
    return jsonb_build_object('state','not_refundable');
  end if;
  if coalesce(v.total_minor,0) = 0 or v.payment_intent_id is null then
    update skager_private.store_orders set status = 'refunded', refunded_at = now() where id = v.id returning * into v;
    perform skager_private.store_enqueue(v.email,'refunded',jsonb_build_object('order',skager_private.store_order_json(v)));
    perform skager_private.store_revoke(v.email);
    return jsonb_build_object('state','refunded');
  end if;
  update skager_private.store_orders set status = 'refund_requested' where id = v.id;
  return jsonb_build_object('state','requested','payment_intent_id',v.payment_intent_id,'order_id',v.id);
end $$;

-- ---------------------------------------------------------------- admin
create function public.store_admin_overview() returns jsonb
language sql stable security definer set search_path = '' as $$
  select jsonb_build_object(
    'products', coalesce((select jsonb_agg(skager_private.store_product_json(p) order by p.sort, p.name) from skager_private.store_products p), '[]'::jsonb),
    'orders', coalesce((select jsonb_agg(skager_private.store_order_json(o) order by o.created_at desc)
      from (select * from skager_private.store_orders order by created_at desc limit 200) o), '[]'::jsonb),
    'customers', coalesce((select jsonb_agg(jsonb_build_object('email',c.email,'status',c.status,'claimed',c.account_id is not null,
        'password_set',c.password_set,'created_at',c.created_at,'revoked_at',c.revoked_at) order by c.created_at desc)
      from (select * from skager_private.store_customers order by created_at desc limit 200) c), '[]'::jsonb),
    'settings', (select jsonb_build_object('shipping_minor',s.shipping_minor,'shipping_countries',to_jsonb(s.shipping_countries))
      from skager_private.store_settings s where s.id = 1));
$$;

create function public.store_admin_save_product(p_admin text, p_product jsonb) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_products; v_id uuid := nullif(p_product->>'id','')::uuid;
begin
  if v_id is null then
    insert into skager_private.store_products(slug,kind,name,summary,description,price_minor,billing_interval,shippable,checkout_addon,active,sort)
    values (p_product->>'slug', coalesce(p_product->>'kind','hardware'), p_product->>'name', coalesce(p_product->>'summary',''),
      coalesce(p_product->>'description',''), (p_product->>'price_minor')::integer, nullif(p_product->>'interval',''),
      coalesce((p_product->>'shippable')::boolean,false), coalesce((p_product->>'checkout_addon')::boolean,false),
      coalesce((p_product->>'active')::boolean,true), coalesce((p_product->>'sort')::integer,100))
    returning * into v;
  else
    update skager_private.store_products set name = p_product->>'name', summary = coalesce(p_product->>'summary',''),
      description = coalesce(p_product->>'description',''),
      -- The access price is bound to the configured Stripe price and is not edited here.
      price_minor = case when kind = 'access' then price_minor else (p_product->>'price_minor')::integer end,
      shippable = case when kind = 'hardware' then coalesce((p_product->>'shippable')::boolean,false) else false end,
      checkout_addon = case when kind = 'hardware' then coalesce((p_product->>'checkout_addon')::boolean,false) else false end,
      active = case when kind = 'access' then true else coalesce((p_product->>'active')::boolean,true) end,
      sort = coalesce((p_product->>'sort')::integer, sort), updated_at = now()
    where id = v_id returning * into v;
    if v.id is null then raise exception 'Unknown product'; end if;
  end if;
  insert into skager_private.store_admin_audit(admin_email,action,target) values (p_admin,'save_product',v.id::text);
  return skager_private.store_product_json(v);
end $$;

create function public.store_admin_set_image(p_admin text, p_product_id uuid, p_key text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_products;
begin
  update skager_private.store_products set image_key = p_key, updated_at = now() where id = p_product_id returning * into v;
  if v.id is null then raise exception 'Unknown product'; end if;
  insert into skager_private.store_admin_audit(admin_email,action,target) values (p_admin,'set_image',v.id::text);
  return skager_private.store_product_json(v);
end $$;

create function public.store_admin_ship(p_admin text, p_order_id uuid, p_tracking text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_orders;
begin
  update skager_private.store_orders set fulfillment = 'shipped', shipped_at = now(), tracking = nullif(btrim(p_tracking),'')
    where id = p_order_id and fulfillment = 'unfulfilled' and status = 'paid' returning * into v;
  if v.id is null then raise exception 'Order not shippable'; end if;
  perform skager_private.store_enqueue(v.email,'shipped',jsonb_build_object('order',skager_private.store_order_json(v)));
  insert into skager_private.store_admin_audit(admin_email,action,target) values (p_admin,'ship',v.id::text);
  return skager_private.store_order_json(v);
end $$;

create function public.store_admin_refund_target(p_admin text, p_order_id uuid) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_orders;
begin
  select * into v from skager_private.store_orders where id = p_order_id for update;
  if v.id is null or v.status not in ('paid','refund_requested') then return jsonb_build_object('state','not_refundable'); end if;
  insert into skager_private.store_admin_audit(admin_email,action,target) values (p_admin,'refund',v.id::text);
  if coalesce(v.total_minor,0) = 0 or v.payment_intent_id is null then
    update skager_private.store_orders set status = 'refunded', refunded_at = now() where id = v.id returning * into v;
    perform skager_private.store_enqueue(v.email,'refunded',jsonb_build_object('order',skager_private.store_order_json(v)));
    if v.kind = 'access' then perform skager_private.store_revoke(v.email); end if;
    return jsonb_build_object('state','refunded');
  end if;
  update skager_private.store_orders set status = 'refund_requested' where id = v.id;
  return jsonb_build_object('state','requested','payment_intent_id',v.payment_intent_id,'order_id',v.id);
end $$;

create function public.store_admin_settings(p_admin text, p_shipping_minor integer, p_countries text[]) returns jsonb
language plpgsql security definer set search_path = '' as $$
begin
  if p_countries is null or cardinality(p_countries) not between 1 and 60
     or exists(select 1 from unnest(p_countries) x where x !~ '^[A-Z]{2}$') then raise exception 'Invalid countries'; end if;
  update skager_private.store_settings set shipping_minor = p_shipping_minor, shipping_countries = p_countries, updated_at = now() where id = 1;
  insert into skager_private.store_admin_audit(admin_email,action,target) values (p_admin,'settings',null);
  return (select jsonb_build_object('shipping_minor',s.shipping_minor,'shipping_countries',to_jsonb(s.shipping_countries))
    from skager_private.store_settings s where s.id = 1);
end $$;

-- ---------------------------------------------------------------- email outbox and jobs
create function public.store_email_claim(p_limit integer) returns jsonb
language sql security definer set search_path = '' as $$
  with due as (
    select id from skager_private.email_outbox
    where status = 'pending' and next_attempt_at <= now()
    order by id limit least(greatest(coalesce(p_limit,1),1),50) for update skip locked),
  claimed as (
    update skager_private.email_outbox e set status = 'sending', attempts = e.attempts + 1,
      next_attempt_at = now() + interval '10 minutes'
    from due where e.id = due.id
    returning e.id, e.to_email, e.template, e.data, e.attempts)
  select coalesce(jsonb_agg(jsonb_build_object('id', id, 'to', to_email, 'template', template, 'data', data,
    'attempts', attempts) order by id), '[]'::jsonb) from claimed;
$$;

create function public.store_email_result(p_id bigint, p_ok boolean, p_provider_id text) returns boolean
language plpgsql security definer set search_path = '' as $$
begin
  update skager_private.email_outbox set
    status = case when p_ok then 'sent' when attempts >= 6 then 'failed' else 'pending' end,
    provider_id = case when p_ok then left(p_provider_id, 100) else provider_id end,
    sent_at = case when p_ok then now() else sent_at end,
    next_attempt_at = case when p_ok then next_attempt_at else now() + make_interval(mins => power(2, attempts)::integer) end
  where id = p_id and status = 'sending';
  return found;
end $$;

create function public.store_job_maintenance() returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v_expired integer; v_stuck integer;
begin
  update skager_private.store_orders set status = 'expired' where status = 'pending' and created_at < now() - interval '24 hours';
  get diagnostics v_expired = row_count;
  update skager_private.email_outbox set status = 'pending' where status = 'sending' and next_attempt_at <= now();
  get diagnostics v_stuck = row_count;
  delete from skager_private.auth_challenges where expires_at <= now();
  delete from skager_private.auth_sessions where expires_at <= now();
  delete from skager_private.auth_throttles where expires_at <= now();
  return jsonb_build_object('expired_orders', v_expired, 'requeued_emails', v_stuck);
end $$;

-- ---------------------------------------------------------------- grants
do $$
declare f record;
begin
  for f in select p.oid::regprocedure as sig from pg_proc p join pg_namespace n on n.oid = p.pronamespace
           where n.nspname = 'public' and p.proname like 'store\_%' loop
    execute format('revoke all on function %s from public, anon, authenticated', f.sig);
    execute format('grant execute on function %s to service_role', f.sig);
  end loop;
  for f in select p.oid::regprocedure as sig from pg_proc p join pg_namespace n on n.oid = p.pronamespace
           where n.nspname = 'skager_private' and p.proname like 'store\_%' loop
    execute format('revoke all on function %s from public, anon, authenticated, service_role', f.sig);
  end loop;
end $$;
commit;
