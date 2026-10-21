/** SKAGER store (Stripe TEST): guest checkout that creates the account, shop, cloud
 * subscription, billing portal, refunds, product images, scheduled job and webhooks.
 * Prices always come from the database catalog or the configured access Price; a
 * browser can choose products and quantities, never amounts. Fulfilment happens only
 * from a signature-verified webhook after re-fetching the session from Stripe. */
import {
  applicableCoupon, approvedPromotion, boundedBytes, checkoutUrl, configuration, integer, object,
  requireValue, strictJson, stripeRequest, verifiedSignature, type CommerceContext, type Configuration,
} from './commerce.ts';
import { sendDue } from './email.ts';
import { getImage } from './media.ts';
import { PUBLIC_ORIGIN } from './platform.ts';

type Json = Record<string, any>;
export type StoreAccount = { id: string; email?: string; passwordSet?: boolean };
export type StoreContext = Omit<CommerceContext, 'getAccount'> & { getAccount: (req: Request) => Promise<StoreAccount | null> };

const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-8][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/;
const SESSION = /^cs_test_[A-Za-z0-9]{8,200}$/;
const INTENT = /^pi_[A-Za-z0-9]{8,200}$/;
const CHARGE = /^ch_[A-Za-z0-9]{8,200}$/;
const CUSTOMER = /^cus_[A-Za-z0-9]{8,200}$/;
const SUBSCRIPTION = /^sub_[A-Za-z0-9]{8,200}$/;
const PROMOTION = /^promo_[A-Za-z0-9]{8,200}$/;
const EMAIL = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const CODE = /^[A-Za-z0-9]{1,64}$/;
const SUB_STATUS = ['incomplete', 'incomplete_expired', 'trialing', 'active', 'past_due', 'canceled', 'unpaid', 'paused'];
const LIVE_SUB = ['trialing', 'active', 'past_due'];
const MAX_WEBHOOK_BYTES = 256 * 1024;
const encoder = new TextEncoder();

const reply = (status: number, value: Json) => new Response(JSON.stringify(value), { status, headers: {
  'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'private, no-store, max-age=0',
  'X-Content-Type-Options': 'nosniff', 'Referrer-Policy': 'no-referrer' } });

async function sha256(value: string): Promise<string> {
  return [...new Uint8Array(await crypto.subtle.digest('SHA-256', encoder.encode(value)))]
    .map(byte => byte.toString(16).padStart(2, '0')).join('');
}

async function body(req: Request, maximum = 2048): Promise<Json> {
  requireValue(req.headers.get('Content-Type')?.split(';')[0].trim().toLowerCase() === 'application/json');
  return strictJson(await boundedBytes(req.body, maximum));
}

function only(value: Json, keys: string[]): void {
  requireValue(Object.keys(value).every(key => keys.includes(key)));
}

function publicProduct(product: Json): Json {
  return { id: product.id, slug: product.slug, kind: product.kind, name: product.name, summary: product.summary,
    description: product.description, priceMinor: product.price_minor, currency: product.currency,
    interval: product.interval ?? null, shippable: product.shippable, checkoutAddon: product.checkout_addon,
    imageUrl: product.image_key ? `/api/media/${product.image_key}` : null, stepId: product.step_id ?? null };
}

function publicStep(step: Json): Json {
  return { id: step.id, flow: step.flow, title: step.title, summary: step.summary, skipLabel: step.skip_label,
    multiple: step.multiple === true, linkUrl: step.link_url ?? null, linkLabel: step.link_label || null, sort: step.sort };
}

/** Build and bind a hosted Checkout Session for a freshly created order. */
async function openSession(ctx: StoreContext, config: Configuration, order: Json, options: {
  mode: 'payment' | 'subscription'; email: string; customerId?: string | null; successPath: string; cancelPath: string;
  promotion?: Json | null; countries?: string[];
}): Promise<string> {
  const form = new URLSearchParams({ mode: options.mode, client_reference_id: order.order_id,
    'metadata[order_id]': order.order_id, success_url: PUBLIC_ORIGIN + options.successPath,
    cancel_url: PUBLIC_ORIGIN + options.cancelPath });
  if (options.customerId && CUSTOMER.test(options.customerId)) form.set('customer', options.customerId);
  else {
    form.set('customer_email', options.email);
    if (options.mode === 'payment') form.set('customer_creation', 'always');
  }
  order.items.forEach((item: Json, index: number) => {
    const prefix = `line_items[${index}]`;
    if (item.kind === 'access') form.set(`${prefix}[price]`, config.price);
    else {
      form.set(`${prefix}[price_data][currency]`, 'usd');
      form.set(`${prefix}[price_data][unit_amount]`, String(item.unit_minor));
      form.set(`${prefix}[price_data][product_data][name]`, item.name);
      form.set(`${prefix}[price_data][product_data][metadata][skager_product]`, item.slug);
      if (item.interval === 'month') form.set(`${prefix}[price_data][recurring][interval]`, 'month');
    }
    form.set(`${prefix}[quantity]`, String(item.quantity));
  });
  if (order.shipping_minor > 0) {
    (options.countries ?? []).forEach((country, index) =>
      form.set(`shipping_address_collection[allowed_countries][${index}]`, country));
    form.set('shipping_options[0][shipping_rate_data][type]', 'fixed_amount');
    form.set('shipping_options[0][shipping_rate_data][fixed_amount][amount]', String(order.shipping_minor));
    form.set('shipping_options[0][shipping_rate_data][fixed_amount][currency]', 'usd');
    form.set('shipping_options[0][shipping_rate_data][display_name]', 'Standard shipping');
  }
  if (options.promotion) form.set('discounts[0][promotion_code]', options.promotion.promotion_id);
  // Hosted code entry stays off: discounts are validated here before the session exists.
  else form.set('allow_promotion_codes', 'false');
  const session = await stripeRequest(ctx as CommerceContext, config, 'checkout/sessions', 'POST', form,
    `skager-store-v1-${order.order_id}`);
  requireValue(session.object === 'checkout.session' && session.livemode === false && SESSION.test(session.id)
    && session.client_reference_id === order.order_id && session.mode === options.mode && session.status === 'open');
  const url = checkoutUrl(session.url, session.id);
  await ctx.rpc('store_bind_order', { p_order_id: order.order_id, p_session_id: session.id, p_checkout_url: url });
  return url;
}

async function catalog(ctx: StoreContext): Promise<Json> {
  const data = await ctx.rpc('store_catalog', {});
  requireValue(object(data) && Array.isArray(data.products));
  return data;
}

async function guestCheckout(req: Request, ctx: StoreContext, config: Configuration): Promise<Response> {
  let input: Json;
  try {
    input = await body(req);
    only(input, ['email', 'addons', 'discountCode']);
    requireValue(typeof input.email === 'string' && input.email.length <= 254 && EMAIL.test(input.email.trim()));
    requireValue(input.addons === undefined || (Array.isArray(input.addons) && input.addons.length <= 9
      && input.addons.every((id: unknown) => typeof id === 'string' && UUID.test(id))));
    requireValue(input.discountCode === undefined || input.discountCode === '' || (typeof input.discountCode === 'string' && CODE.test(input.discountCode)));
  } catch { return reply(400, { error: 'Check your email address and try again.' }); }
  const email = input.email.trim().toLowerCase();
  const allowed = await ctx.rpc('skager_auth_throttle', { p_keys: ['checkout:global'], p_limit: 100, p_seconds: 3600 }) === true
    && await ctx.rpc('skager_auth_throttle', { p_keys: ['checkout:email:' + await sha256(email)], p_limit: 10, p_seconds: 3600 }) === true;
  if (!allowed) return reply(429, { error: 'Too many attempts. Please try again later.' });
  const products = (await catalog(ctx)).products;
  const access = products.find((product: Json) => product.kind === 'access');
  requireValue(access && UUID.test(access.id));
  const items = [{ product_id: access.id, quantity: 1 },
    ...[...new Set<string>(input.addons ?? [])].map(id => ({ product_id: id, quantity: 1 }))];
  const promotion = input.discountCode ? await approvedPromotion(ctx as CommerceContext, config, input.discountCode) : null;
  if (input.discountCode && !promotion) return reply(400, { error: 'That discount code is not valid.' });
  let order: Json;
  try {
    order = await ctx.rpc('store_create_order', { p_kind: 'access', p_email: email, p_account_id: null, p_items: items,
      p_promotion_id: promotion?.promotion_id ?? null, p_coupon_id: promotion?.coupon_id ?? null });
  } catch { return reply(400, { error: 'That combination is not available.' }); }
  if (order.state === 'exists') return reply(409, { state: 'exists', error: 'You already own SKAGER. Log in to your account.' });
  if (order.state === 'setup') return reply(409, { state: 'setup', error: 'You already bought SKAGER. Finish setting up your account.' });
  if (order.state === 'revoked') return reply(409, { state: 'revoked', error: 'This account has been closed. Contact us if you think this is a mistake.' });
  requireValue(order.state === 'created' && UUID.test(order.order_id));
  const url = await openSession(ctx, config, order, { mode: 'payment', email, successPath: '/welcome/?session_id={CHECKOUT_SESSION_ID}',
    cancelPath: '/checkout/', promotion, countries: order.shipping_countries });
  return reply(200, { url });
}

async function member(req: Request, ctx: StoreContext): Promise<StoreAccount | Response> {
  const account = await ctx.getAccount(req);
  if (!account || !UUID.test(account.id) || !account.email) return reply(401, { error: 'Sign in required.' });
  if (account.passwordSet === false) return reply(403, { error: 'Choose a password first.', setup: true });
  return account;
}

async function shopCheckout(req: Request, ctx: StoreContext, config: Configuration, account: StoreAccount): Promise<Response> {
  let input: Json;
  try {
    input = await body(req);
    only(input, ['items']);
    requireValue(Array.isArray(input.items) && input.items.length >= 1 && input.items.length <= 10
      && input.items.every((item: Json) => object(item) && Object.keys(item).length === 2 && typeof item.productId === 'string'
        && UUID.test(item.productId) && integer(item.quantity) && item.quantity >= 1 && item.quantity <= 5));
  } catch { return reply(400, { error: 'Choose at least one product.' }); }
  let order: Json;
  try {
    order = await ctx.rpc('store_create_order', { p_kind: 'shop', p_email: account.email, p_account_id: account.id,
      p_items: input.items.map((item: Json) => ({ product_id: item.productId, quantity: item.quantity })),
      p_promotion_id: null, p_coupon_id: null });
    requireValue(order.state === 'created');
  } catch { return reply(400, { error: 'One of these products is not available.' }); }
  const url = await openSession(ctx, config, order, { mode: 'payment', email: account.email!, customerId: order.stripe_customer_id,
    successPath: '/portal/orders/?paid=1', cancelPath: '/portal/shop/', countries: order.shipping_countries });
  return reply(200, { url });
}

async function portalAccess(ctx: StoreContext, account: StoreAccount): Promise<Json> {
  const access = await ctx.rpc('store_portal_access', { p_account_id: account.id });
  requireValue(object(access));
  return access;
}

async function cloudCheckout(ctx: StoreContext, config: Configuration, account: StoreAccount): Promise<Response> {
  const access = await portalAccess(ctx, account);
  if (!access.entitlement) return reply(403, { error: 'SKAGER Beta Access is required.' });
  if (access.subscription && LIVE_SUB.includes(access.subscription.status)) {
    return reply(409, { error: 'Your SKAGER Cloud subscription is already active.' });
  }
  const cloud = (await catalog(ctx)).products.find((product: Json) => product.kind === 'subscription');
  if (!cloud) return reply(404, { error: 'SKAGER Cloud is not available right now.' });
  const order = await ctx.rpc('store_create_order', { p_kind: 'subscription', p_email: account.email, p_account_id: account.id,
    p_items: [{ product_id: cloud.id, quantity: 1 }], p_promotion_id: null, p_coupon_id: null });
  requireValue(order.state === 'created');
  const url = await openSession(ctx, config, order, { mode: 'subscription', email: account.email!,
    customerId: order.stripe_customer_id, successPath: '/portal/cloud/?activated=1', cancelPath: '/portal/cloud/' });
  return reply(200, { url });
}

async function billingPortal(ctx: StoreContext, config: Configuration, account: StoreAccount): Promise<Response> {
  const access = await portalAccess(ctx, account);
  if (typeof access.stripe_customer_id !== 'string' || !CUSTOMER.test(access.stripe_customer_id)) {
    return reply(404, { error: 'There is no billing account yet.' });
  }
  const form = new URLSearchParams({ customer: access.stripe_customer_id, return_url: PUBLIC_ORIGIN + '/portal/cloud/' });
  const session = await stripeRequest(ctx as CommerceContext, config, 'billing_portal/sessions', 'POST', form,
    `skager-billing-${account.id}-${Math.floor(Date.now() / 60000)}`);
  requireValue(session.object === 'billing_portal.session' && session.livemode === false && typeof session.url === 'string');
  const url = new URL(session.url);
  requireValue(url.protocol === 'https:' && url.hostname === 'billing.stripe.com' && !url.username && !url.password);
  return reply(200, { url: url.href });
}

export async function stripeRefund(ctx: StoreContext, config: Configuration, target: Json): Promise<void> {
  requireValue(INTENT.test(target.payment_intent_id) && UUID.test(target.order_id));
  const form = new URLSearchParams({ payment_intent: target.payment_intent_id, 'metadata[order_id]': target.order_id });
  const refund = await stripeRequest(ctx as CommerceContext, config, 'refunds', 'POST', form, `skager-refund-${target.order_id}`);
  requireValue(refund.object === 'refund' && refund.livemode !== true && refund.payment_intent === target.payment_intent_id);
}

async function selfRefund(req: Request, ctx: StoreContext, config: Configuration, account: StoreAccount): Promise<Response> {
  let input: Json;
  try { input = await body(req); only(input, ['orderId']); requireValue(typeof input.orderId === 'string' && UUID.test(input.orderId)); }
  catch { return reply(400, { error: 'Choose an order to refund.' }); }
  const result = await ctx.rpc('store_refund_request', { p_account_id: account.id, p_order_id: input.orderId });
  requireValue(object(result));
  if (result.state === 'not_refundable') return reply(400, { error: 'This order can no longer be refunded online. Contact us and we will help.' });
  if (result.state === 'refunded') {
    await sendDue(ctx, 5);
    const response = reply(200, { state: 'refunded' });
    response.headers.append('Set-Cookie', '__Host-skager_session=; Path=/; Max-Age=0; HttpOnly; Secure; SameSite=Lax');
    return response;
  }
  requireValue(result.state === 'requested');
  await stripeRefund(ctx, config, result);
  return reply(200, { state: 'requested' });
}

// ---------------------------------------------------------------- webhook
function sanitizeShipping(value: unknown): Json | null {
  if (!object(value)) return null;
  const text = (input: unknown) => typeof input === 'string' ? input.slice(0, 200) : null;
  const address = object(value.address) ? value.address : {};
  return { name: text(value.name), address: { line1: text(address.line1), line2: text(address.line2),
    postal_code: text(address.postal_code), city: text(address.city), state: text(address.state),
    country: typeof address.country === 'string' && /^[A-Z]{2}$/.test(address.country) ? address.country : null } };
}

/** Re-fetch a completed session and prove it matches the bound order exactly. */
async function confirmedPaid(ctx: StoreContext, config: Configuration, binding: Json, sessionId: string): Promise<Json> {
  const session = await stripeRequest(ctx as CommerceContext, config,
    `checkout/sessions/${sessionId}?expand%5B%5D=line_items&expand%5B%5D=discounts.coupon`, 'GET');
  const subscription = binding.kind === 'subscription';
  requireValue(session.object === 'checkout.session' && session.livemode === false && session.id === sessionId
    && session.client_reference_id === binding.id && session.status === 'complete' && session.currency === 'usd'
    && session.mode === (subscription ? 'subscription' : 'payment'));
  const lines = session.line_items;
  requireValue(object(lines) && lines.has_more === false && Array.isArray(lines.data) && lines.data.length === binding.items.length);
  let subtotal = 0; let accessProduct: string | null = null;
  binding.items.forEach((item: Json, index: number) => {
    const line = lines.data[index]; const price = line?.price;
    requireValue(object(line) && object(price) && price.livemode === false && price.currency === 'usd'
      && line.quantity === item.quantity && price.unit_amount === item.unit_minor
      && line.amount_subtotal === item.unit_minor * item.quantity);
    if (item.kind === 'access') {
      requireValue(price.id === config.price && price.type === 'one_time' && item.unit_minor === 2000);
      accessProduct = price.product;
    } else if (item.interval === 'month') requireValue(object(price.recurring) && price.recurring.interval === 'month');
    else requireValue(price.type === 'one_time');
    subtotal += line.amount_subtotal;
  });
  requireValue(subtotal === binding.subtotal_minor && session.amount_subtotal === subtotal);
  const totals = session.total_details;
  requireValue(object(totals) && totals.amount_tax === 0 && integer(totals.amount_discount)
    && (totals.amount_shipping ?? 0) === binding.shipping_minor);
  let discount = 0;
  if (binding.coupon_id) {
    // The bound promotion and its product-scoped coupon are re-read; embedded fields never decide.
    requireValue(typeof binding.promotion_id === 'string' && PROMOTION.test(binding.promotion_id)
      && config.coupons.includes(binding.coupon_id) && config.promotions.includes(binding.promotion_id));
    const bound = { ...config, coupons: [binding.coupon_id], selectedPromotion: binding.promotion_id };
    const promotion = await stripeRequest(ctx as CommerceContext, bound, `promotion_codes/${binding.promotion_id}`, 'GET');
    requireValue(promotion.object === 'promotion_code' && promotion.id === binding.promotion_id && promotion.livemode === false
      && object(promotion.promotion) && promotion.promotion.coupon === binding.coupon_id);
    const coupon = await applicableCoupon(ctx as CommerceContext, bound, binding.coupon_id, false);
    requireValue(coupon && coupon.applies_to.products[0] === accessProduct);
    discount = typeof coupon!.percent_off === 'number' ? Math.round(2000 * coupon!.percent_off / 100) : Math.min(2000, coupon!.amount_off);
    requireValue(Array.isArray(session.discounts) && session.discounts.length === 1
      && session.discounts[0]?.promotion_code === binding.promotion_id);
  } else requireValue(!Array.isArray(session.discounts) || session.discounts.length === 0);
  requireValue(totals.amount_discount === discount
    && session.amount_total === binding.subtotal_minor - discount + binding.shipping_minor);
  const paid: Json = { total_minor: session.amount_total, discount_minor: discount, shipping_minor: binding.shipping_minor,
    customer_id: typeof session.customer === 'string' && CUSTOMER.test(session.customer) ? session.customer : null,
    shipping: binding.shipping_minor > 0 ? sanitizeShipping(session.collected_information?.shipping_details ?? session.shipping_details) : null };
  if (session.amount_total === 0) {
    requireValue(session.payment_status === 'no_payment_required' && !subscription && session.payment_intent === null);
  } else if (subscription) {
    requireValue(session.payment_status === 'paid' && typeof session.subscription === 'string' && SUBSCRIPTION.test(session.subscription));
    const sub = await stripeRequest(ctx as CommerceContext, config, `subscriptions/${session.subscription}`, 'GET');
    requireValue(sub.object === 'subscription' && sub.id === session.subscription && sub.livemode === false && SUB_STATUS.includes(sub.status));
    const periodEnd = sub.current_period_end ?? sub.items?.data?.[0]?.current_period_end;
    Object.assign(paid, { subscription_id: sub.id, subscription_status: sub.status,
      current_period_end: integer(periodEnd) ? String(periodEnd) : '' });
  } else {
    requireValue(session.payment_status === 'paid' && typeof session.payment_intent === 'string' && INTENT.test(session.payment_intent));
    paid.payment_intent_id = session.payment_intent;
  }
  return paid;
}

async function webhook(req: Request, ctx: StoreContext, config: Configuration): Promise<Response> {
  let verified: Json;
  try {
    const length = req.headers.get('Content-Length');
    if (length !== null) requireValue(/^[0-9]{1,8}$/.test(length) && Number(length) <= MAX_WEBHOOK_BYTES);
    const raw = await boundedBytes(req.body, MAX_WEBHOOK_BYTES);
    verified = await verifiedSignature(raw, req.headers.get('Stripe-Signature') ?? '', config.secret,
      Math.floor((ctx.now?.() ?? Date.now()) / 1000));
  } catch { return reply(400, { error: 'Webhook rejected' }); }
  const { event, digest } = verified;
  const item = event.data.object;
  const receive = async () => {
    const receipt = await ctx.rpc('store_receive_event', { p_event_id: event.id, p_digest: digest, p_event_type: event.type });
    requireValue(object(receipt) && ['received', 'applied', 'ignored', 'conflict'].includes(receipt.state));
    return receipt.state as string;
  };
  try {
    if (['checkout.session.completed', 'checkout.session.async_payment_succeeded',
      'checkout.session.expired', 'checkout.session.async_payment_failed'].includes(event.type)) {
      requireValue(item.object === 'checkout.session' && item.livemode === false && SESSION.test(item.id));
      // Ownership first: our creator always binds a canonical order UUID to the exact session.
      if (typeof item.client_reference_id !== 'string' || !UUID.test(item.client_reference_id)) return reply(200, { received: true });
      const binding = await ctx.rpc('store_order_binding', { p_order_id: item.client_reference_id, p_session_id: item.id });
      requireValue(object(binding));
      if (binding.state !== 'bound') return reply(200, { received: true });
      const state = await receive();
      if (state === 'conflict') return reply(400, { error: 'Webhook rejected' });
      if (state !== 'received') return reply(200, { received: true });
      if (event.type.endsWith('.expired') || event.type.endsWith('.async_payment_failed')) {
        await ctx.rpc('store_apply_expired', { p_event_id: event.id, p_order_id: binding.id, p_session_id: item.id });
        return reply(200, { received: true });
      }
      if (item.status !== 'complete' || !['paid', 'no_payment_required'].includes(item.payment_status)) {
        // Delayed payment methods settle on async_payment_succeeded.
        await ctx.rpc('store_mark_event', { p_event_id: event.id, p_state: 'ignored' });
        return reply(200, { received: true });
      }
      const paid = await confirmedPaid(ctx, config, binding, item.id);
      await ctx.rpc('store_apply_paid', { p_event_id: event.id, p_order_id: binding.id, p_session_id: item.id, p_paid: paid });
      await sendDue(ctx, 5);
      return reply(200, { received: true });
    }
    if (event.type === 'charge.refunded') {
      requireValue(item.object === 'charge' && CHARGE.test(item.id));
      const charge = await stripeRequest(ctx as CommerceContext, config, `charges/${item.id}`, 'GET');
      requireValue(charge.object === 'charge' && charge.id === item.id && charge.livemode === false
        && typeof charge.payment_intent === 'string' && INTENT.test(charge.payment_intent)
        && integer(charge.amount_refunded) && charge.amount_refunded > 0 && charge.amount_refunded <= charge.amount);
      const state = await receive();
      if (state === 'conflict') return reply(400, { error: 'Webhook rejected' });
      if (state !== 'received') return reply(200, { received: true });
      const result = await ctx.rpc('store_apply_refund', { p_event_id: event.id, p_intent_id: charge.payment_intent,
        p_refunded_minor: charge.amount_refunded });
      // Closing an account also ends its cloud subscription.
      for (const id of Array.isArray(result?.cancel_subscriptions) ? result.cancel_subscriptions : []) {
        if (typeof id === 'string' && SUBSCRIPTION.test(id)) {
          try { await stripeRequest(ctx as CommerceContext, config, `subscriptions/${id}`, 'DELETE', undefined, `skager-cancel-${id}`); }
          catch { /* Stripe's own deletion event, or the admin, completes this. */ }
        }
      }
      await sendDue(ctx, 5);
      return reply(200, { received: true });
    }
    if (event.type === 'customer.subscription.updated' || event.type === 'customer.subscription.deleted') {
      requireValue(item.object === 'subscription' && SUBSCRIPTION.test(item.id) && SUB_STATUS.includes(item.status));
      const state = await receive();
      if (state === 'conflict') return reply(400, { error: 'Webhook rejected' });
      if (state !== 'received') return reply(200, { received: true });
      const periodEnd = item.current_period_end ?? item.items?.data?.[0]?.current_period_end;
      await ctx.rpc('store_apply_subscription', { p_event_id: event.id, p_subscription_id: item.id,
        p_status: event.type.endsWith('.deleted') ? 'canceled' : item.status,
        p_period_end: integer(periodEnd) ? periodEnd : null, p_cancel_at_period_end: item.cancel_at_period_end === true });
      await sendDue(ctx, 5);
      return reply(200, { received: true });
    }
    return reply(200, { received: true });
  } catch { return reply(503, { error: 'Confirmation pending' }); }
}

// ---------------------------------------------------------------- job
async function constantEqual(a: string, b: string): Promise<boolean> {
  const [x, y] = await Promise.all([a, b].map(value => crypto.subtle.digest('SHA-256', encoder.encode(value))));
  const left = new Uint8Array(x); const right = new Uint8Array(y);
  let difference = 0;
  for (let i = 0; i < left.length; i++) difference |= left[i] ^ right[i];
  return difference === 0;
}

async function tick(req: Request, ctx: StoreContext): Promise<Response> {
  const secret = ctx.env.SKAGER_JOB_SECRET ?? '';
  if (secret.length < 32 || !await constantEqual(req.headers.get('x-skager-job') ?? '', secret)) {
    return reply(403, { error: 'Forbidden' });
  }
  const maintenance = await ctx.rpc('store_job_maintenance', {});
  // Cloud maintenance (expired pairings, 30-day history) must not hold back email.
  const cloud = await ctx.rpc('cloud_job_maintenance', {}).catch(() => null);
  const email = await sendDue(ctx, 20);
  return reply(200, { ok: true, maintenance, cloud, email });
}

// ---------------------------------------------------------------- router
export async function handleStore(req: Request, ctx: StoreContext): Promise<Response | null> {
  const path = new URL(req.url).pathname;
  if (path.startsWith('/api/media/') && req.method === 'GET') return getImage(ctx.env, path.slice('/api/media/'.length), ctx.fetch);
  if (path === '/api/jobs/tick') {
    if (req.method !== 'POST') return reply(405, { error: 'POST required' });
    try { return await tick(req, ctx); } catch { return reply(503, { error: 'Job unavailable' }); }
  }
  const routes = ['/api/store/catalog', '/api/store/checkout', '/api/store/shop', '/api/store/cloud',
    '/api/store/billing', '/api/store/refund', '/api/stripe/webhook'];
  if (!routes.includes(path)) return null;
  if (path === '/api/store/catalog') {
    if (req.method !== 'GET') return reply(405, { error: 'GET required' });
    try {
      const data = await catalog(ctx);
      return reply(200, { products: data.products.map(publicProduct),
        steps: (Array.isArray(data.steps) ? data.steps : []).map(publicStep), shippingMinor: data.shipping_minor,
        shippingCountries: data.shipping_countries, checkoutOpen: ctx.env.STRIPE_TEST_CHECKOUT_ENABLED === 'true' });
    } catch { return reply(503, { error: 'The shop is temporarily unavailable.' }); }
  }
  if (req.method !== 'POST') return reply(405, { error: 'POST required' });
  let config: Configuration;
  try { config = configuration(ctx.env); } catch { return reply(404, { error: 'Checkout is not available right now.' }); }
  if (path === '/api/stripe/webhook') return webhook(req, ctx, config);
  // The router checks CSRF; the exact Origin is enforced here as well for direct calls.
  if (req.headers.get('Origin') !== config.origin) return reply(403, { error: 'Request rejected' });
  try {
    if (path === '/api/store/checkout') return await guestCheckout(req, ctx, config);
    const account = await member(req, ctx);
    if (account instanceof Response) return account;
    if (path === '/api/store/shop') return await shopCheckout(req, ctx, config, account);
    if (path === '/api/store/cloud') return await cloudCheckout(ctx, config, account);
    if (path === '/api/store/billing') return await billingPortal(ctx, config, account);
    return await selfRefund(req, ctx, config, account);
  } catch { return reply(503, { error: 'Checkout is temporarily unavailable. Please try again.' }); }
}
