-- Apply once after store-cloud.sql, as owner, in one transaction.
-- SKAGER Cloud orders may carry a discount code. Only the discount rule changes:
-- a promotion is allowed on access and subscription orders, never on shop orders.
-- The server validates every code with Stripe before the order exists.
begin;
create or replace function public.store_create_order(p_kind text, p_email text, p_account_id uuid, p_items jsonb,
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
  if (p_promotion_id is null) <> (p_coupon_id is null) or (p_promotion_id is not null and p_kind not in ('access','subscription')) then
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

commit;
