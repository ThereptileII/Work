-- SKAGER store steps: admin-defined stages of the checkout (hardware choices such as a
-- helm PC or a GPS, each with an "I already have one" answer) and of the download page
-- (such as installing OpenCPN before SKAGER). Apply after store.sql, as the database
-- owner, in one transaction. Additive: existing products and orders keep their data.
begin;

create table skager_private.store_steps (
  id uuid primary key default gen_random_uuid(),
  flow text not null check (flow in ('checkout','download')),
  title text not null check (char_length(title) between 1 and 60 and title !~ '[[:cntrl:]]'),
  summary text not null default '' check (char_length(summary) <= 600 and summary !~ '[[:cntrl:]]'),
  -- The answer that moves on without a product: "I already have a PC", "I already have OpenCPN installed".
  skip_label text not null check (char_length(skip_label) between 1 and 80 and skip_label !~ '[[:cntrl:]]'),
  multiple boolean not null default false,
  link_url text check (link_url is null or (char_length(link_url) <= 500 and link_url ~ '^https://[A-Za-z0-9.-]+(/[^[:space:][:cntrl:]<>"]*)?$')),
  link_label text not null default '' check (char_length(link_label) <= 80 and link_label !~ '[[:cntrl:]]'),
  active boolean not null default true,
  sort integer not null default 100 check (sort between 0 and 10000),
  created_at timestamptz not null default now(),
  updated_at timestamptz not null default now(),
  check ((link_url is null) = (link_label = ''))
);
alter table skager_private.store_steps enable row level security;
revoke all on skager_private.store_steps from public, anon, authenticated, service_role;

alter table skager_private.store_products add column step_id uuid references skager_private.store_steps(id) on delete set null;
alter table skager_private.store_products add constraint store_products_step_hardware check (step_id is null or kind = 'hardware');

-- Seed: the two hardware questions and the OpenCPN prerequisite. The admin edits them freely.
with s as (insert into skager_private.store_steps(flow,title,summary,skip_label,sort) values ('checkout','Your helm PC',
  'SKAGER runs on a Windows 11 PC at the helm. Get one that arrives ready to go, or use a PC you already have.',
  'I already have a PC',10) returning id)
update skager_private.store_products set step_id = (select id from s), checkout_addon = true where slug = 'helm-pc';
with s as (insert into skager_private.store_steps(flow,title,summary,skip_label,sort) values ('checkout','GPS',
  'SKAGER needs a position to follow your boat on the chart. Add our plug-and-play receiver, or use the GPS you already have.',
  'I already have a GPS',20) returning id)
update skager_private.store_products set step_id = (select id from s), checkout_addon = true where slug = 'gps-module';
insert into skager_private.store_steps(flow,title,summary,skip_label,link_url,link_label,sort) values ('download','Install OpenCPN',
  'SKAGER runs on top of OpenCPN, the free open-source chart plotter. Install OpenCPN 5.12.4 for Windows, open it once, then come back for SKAGER.',
  'I already have OpenCPN installed',
  'https://github.com/OpenCPN/OpenCPN/releases/download/Release_5.12.4/opencpn_5.12.4-0%2B3720.37fd0cd_setup.exe',
  'Download OpenCPN 5.12.4 for Windows',10);

-- ---------------------------------------------------------------- json and catalog
create or replace function skager_private.store_product_json(p skager_private.store_products) returns jsonb
language sql immutable set search_path = '' as $$
  select jsonb_build_object('id',p.id,'slug',p.slug,'kind',p.kind,'name',p.name,'summary',p.summary,
    'description',p.description,'price_minor',p.price_minor,'currency',p.currency,'interval',p.billing_interval,
    'image_key',p.image_key,'shippable',p.shippable,'checkout_addon',p.checkout_addon,'active',p.active,'sort',p.sort,
    'step_id',p.step_id);
$$;
create function skager_private.store_step_json(s skager_private.store_steps) returns jsonb
language sql immutable set search_path = '' as $$
  select jsonb_build_object('id',s.id,'flow',s.flow,'title',s.title,'summary',s.summary,'skip_label',s.skip_label,
    'multiple',s.multiple,'link_url',s.link_url,'link_label',s.link_label,'active',s.active,'sort',s.sort);
$$;

create or replace function public.store_catalog() returns jsonb
language sql stable security definer set search_path = '' as $$
  select jsonb_build_object(
    'products', coalesce((select jsonb_agg(skager_private.store_product_json(p) order by p.sort, p.name)
      from skager_private.store_products p where p.active), '[]'::jsonb),
    'steps', coalesce((select jsonb_agg(skager_private.store_step_json(st) order by st.flow, st.sort, st.title)
      from skager_private.store_steps st where st.active), '[]'::jsonb),
    'shipping_minor', s.shipping_minor, 'shipping_countries', to_jsonb(s.shipping_countries))
  from skager_private.store_settings s where s.id = 1;
$$;

-- ---------------------------------------------------------------- admin
create or replace function public.store_admin_overview() returns jsonb
language sql stable security definer set search_path = '' as $$
  select jsonb_build_object(
    'products', coalesce((select jsonb_agg(skager_private.store_product_json(p) order by p.sort, p.name) from skager_private.store_products p), '[]'::jsonb),
    'steps', coalesce((select jsonb_agg(skager_private.store_step_json(st) order by st.flow, st.sort, st.title) from skager_private.store_steps st), '[]'::jsonb),
    'orders', coalesce((select jsonb_agg(skager_private.store_order_json(o) order by o.created_at desc)
      from (select * from skager_private.store_orders order by created_at desc limit 200) o), '[]'::jsonb),
    'customers', coalesce((select jsonb_agg(jsonb_build_object('email',c.email,'status',c.status,'claimed',c.account_id is not null,
        'password_set',c.password_set,'created_at',c.created_at,'revoked_at',c.revoked_at) order by c.created_at desc)
      from (select * from skager_private.store_customers order by created_at desc limit 200) c), '[]'::jsonb),
    'settings', (select jsonb_build_object('shipping_minor',s.shipping_minor,'shipping_countries',to_jsonb(s.shipping_countries))
      from skager_private.store_settings s where s.id = 1));
$$;

-- A hardware product joins the checkout by belonging to an active checkout step;
-- checkout_addon is kept in step with that, so store_create_order needs no change.
create or replace function public.store_admin_save_product(p_admin text, p_product jsonb) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_products; v_id uuid := nullif(p_product->>'id','')::uuid;
  v_step uuid := nullif(p_product->>'step_id','')::uuid; v_step_active boolean := false;
begin
  if v_step is not null then
    select st.active into v_step_active from skager_private.store_steps st where st.id = v_step and st.flow = 'checkout';
    if not found then raise exception 'Unknown step'; end if;
  end if;
  if v_id is null then
    insert into skager_private.store_products(slug,kind,name,summary,description,price_minor,billing_interval,shippable,checkout_addon,active,sort,step_id)
    values (p_product->>'slug', coalesce(p_product->>'kind','hardware'), p_product->>'name', coalesce(p_product->>'summary',''),
      coalesce(p_product->>'description',''), (p_product->>'price_minor')::integer, nullif(p_product->>'interval',''),
      coalesce((p_product->>'shippable')::boolean,false), coalesce(v_step_active,false),
      coalesce((p_product->>'active')::boolean,true), coalesce((p_product->>'sort')::integer,100), v_step)
    returning * into v;
  else
    update skager_private.store_products set name = p_product->>'name', summary = coalesce(p_product->>'summary',''),
      description = coalesce(p_product->>'description',''),
      -- The access price is bound to the configured Stripe price and is not edited here.
      price_minor = case when kind = 'access' then price_minor else (p_product->>'price_minor')::integer end,
      shippable = case when kind = 'hardware' then coalesce((p_product->>'shippable')::boolean,false) else false end,
      step_id = case when kind = 'hardware' then v_step else null end,
      checkout_addon = case when kind = 'hardware' then coalesce(v_step_active,false) else false end,
      active = case when kind = 'access' then true else coalesce((p_product->>'active')::boolean,true) end,
      sort = coalesce((p_product->>'sort')::integer, sort), updated_at = now()
    where id = v_id returning * into v;
    if v.id is null then raise exception 'Unknown product'; end if;
  end if;
  insert into skager_private.store_admin_audit(admin_email,action,target) values (p_admin,'save_product',v.id::text);
  return skager_private.store_product_json(v);
end $$;

create function public.store_admin_save_step(p_admin text, p_step jsonb) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_steps; v_id uuid := nullif(p_step->>'id','')::uuid;
begin
  if v_id is null then
    insert into skager_private.store_steps(flow,title,summary,skip_label,multiple,link_url,link_label,active,sort)
    values (p_step->>'flow', p_step->>'title', coalesce(p_step->>'summary',''), p_step->>'skip_label',
      coalesce((p_step->>'multiple')::boolean,false), nullif(p_step->>'link_url',''), coalesce(p_step->>'link_label',''),
      coalesce((p_step->>'active')::boolean,true), coalesce((p_step->>'sort')::integer,100))
    returning * into v;
  else
    -- The flow of an existing step is fixed; products in it would otherwise change meaning.
    update skager_private.store_steps set title = p_step->>'title', summary = coalesce(p_step->>'summary',''),
      skip_label = p_step->>'skip_label', multiple = coalesce((p_step->>'multiple')::boolean,false),
      link_url = nullif(p_step->>'link_url',''), link_label = coalesce(p_step->>'link_label',''),
      active = coalesce((p_step->>'active')::boolean,true), sort = coalesce((p_step->>'sort')::integer, sort), updated_at = now()
    where id = v_id returning * into v;
    if v.id is null then raise exception 'Unknown step'; end if;
  end if;
  if v.flow = 'checkout' then
    update skager_private.store_products set checkout_addon = v.active, updated_at = now() where step_id = v.id and kind = 'hardware';
  end if;
  insert into skager_private.store_admin_audit(admin_email,action,target) values (p_admin,'save_step',v.id::text);
  return skager_private.store_step_json(v);
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
