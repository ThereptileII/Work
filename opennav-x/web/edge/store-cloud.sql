-- Apply once after store.sql and before cloud.sql, as owner, in one transaction.
-- A fresh provider read is reserved before it starts. Its result may commit only
-- if no later read has begun. Stripe event timestamps alone cannot order snapshots
-- (events can arrive out of order and multiple events can share a second).
begin;
alter table skager_private.store_subscriptions add column sync_revision bigint not null default 0;

-- A retried settled checkout must be able to finish its subscription sync without
-- repeating order effects. This binding is returned only to the server webhook.
create or replace function public.store_order_binding(p_order_id uuid, p_session_id text) returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce((select skager_private.store_order_json(o) || jsonb_build_object('state','bound',
      'promotion_id',o.promotion_id,'coupon_id',o.coupon_id,'account_id',o.account_id,'subscription_id',o.subscription_id)
    from skager_private.store_orders o where o.id = p_order_id and o.session_id = p_session_id),
    jsonb_build_object('state','rejected'));
$$;

create function public.store_subscription_sync_begin(p_subscription_id text) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_subscriptions; v_customer text;
begin
  update skager_private.store_subscriptions set sync_revision = sync_revision + 1
    where subscription_id = p_subscription_id returning * into v;
  if v.subscription_id is null then return jsonb_build_object('state','unknown'); end if;
  select stripe_customer_id into v_customer from skager_private.store_customers where email = v.email;
  return jsonb_build_object('state','known','revision',v.sync_revision,'customer_id',v_customer);
end $$;

-- The old payload-only RPC must not remain callable after this migration.
drop function public.store_apply_subscription(text,text,text,bigint,boolean);
create function public.store_apply_subscription(p_subscription_id text, p_revision bigint,
  p_status text, p_period_end bigint, p_cancel_at_period_end boolean) returns jsonb
language plpgsql security definer set search_path = '' as $$
declare v skager_private.store_subscriptions; v_previous text;
begin
  select * into v from skager_private.store_subscriptions where subscription_id = p_subscription_id for update;
  if v.subscription_id is null then return jsonb_build_object('state','unknown'); end if;
  if p_revision is distinct from v.sync_revision then return jsonb_build_object('state','superseded'); end if;
  -- A canceled Stripe subscription cannot be restarted; a new purchase has a new ID.
  if v.status = 'canceled' and p_status <> 'canceled' then return jsonb_build_object('state','terminal'); end if;
  if p_status in ('active','trialing','past_due') and (p_period_end is null or p_period_end <= 0) then
    raise exception 'Missing subscription period';
  end if;
  v_previous := v.status;
  update skager_private.store_subscriptions set status = p_status,
    current_period_end = to_timestamp(p_period_end), cancel_at_period_end = coalesce(p_cancel_at_period_end, false),
    updated_at = now() where subscription_id = p_subscription_id;
  if v_previous <> 'canceled' and p_status = 'canceled' then
    perform skager_private.store_enqueue(v.email,'cloud_ended','{}'::jsonb);
  end if;
  return jsonb_build_object('state','applied');
end $$;

-- This is a durable, reconstructable cancellation queue, including zero-cost
-- account refunds. Failed provider calls leave the row eligible for the next tick.
create function public.store_subscription_cancellations() returns jsonb
language sql stable security definer set search_path = '' as $$
  select coalesce(jsonb_agg(s.subscription_id), '[]'::jsonb) from (
    select s.subscription_id from skager_private.store_subscriptions s
      join skager_private.store_customers c on c.email = s.email
      where c.status = 'revoked' and s.status not in ('canceled','incomplete_expired')
      order by s.updated_at, s.subscription_id limit 10) s;
$$;

revoke all on function public.store_subscription_sync_begin(text),
  public.store_apply_subscription(text,bigint,text,bigint,boolean), public.store_subscription_cancellations()
  from public, anon, authenticated;
grant execute on function public.store_subscription_sync_begin(text),
  public.store_apply_subscription(text,bigint,text,bigint,boolean), public.store_subscription_cancellations()
  to service_role;
commit;
