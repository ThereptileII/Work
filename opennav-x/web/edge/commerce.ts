/** Closed Stripe TEST commerce. Secrets exist only in the server environment. */
export const STRIPE_API_VERSION = '2026-09-30.endive';
const STRIPE_API = 'https://api.stripe.com/v1/';
const MAX_PROVIDER_BYTES = 64 * 1024;
const MAX_WEBHOOK_BYTES = 256 * 1024;
const encoder = new TextEncoder();
const decoder = new TextDecoder('utf-8', { fatal: true });
const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-8][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/;
const SESSION = /^cs_test_[A-Za-z0-9]{8,200}$/;
const INTENT = /^pi_[A-Za-z0-9]{8,200}$/;
const CHARGE = /^ch_[A-Za-z0-9]{8,200}$/;
const EVENT = /^evt_[A-Za-z0-9]{8,200}$/;
const PROMOTION = /^promo_[A-Za-z0-9]{8,200}$/;
const CODE = /^[A-Za-z0-9]{1,64}$/;
const TYPES = new Set(['checkout.session.completed', 'checkout.session.expired',
  'checkout.session.async_payment_succeeded', 'checkout.session.async_payment_failed', 'charge.refunded']);
type Json = Record<string, any>;
export type CommerceContext = {
  env: Record<string, string | undefined>;
  getAccount: (req: Request) => Promise<{ id: string } | null>;
  rpc: (name: string, args: Json) => Promise<any>;
  fetch?: typeof fetch;
  now?: () => number;
};
type Configuration = { key: string; secret: string; price: string; origin: string; coupons: string[];
  promotions: string[]; selectedPromotion?: string | null };

class BoundaryError extends Error {
  constructor() { super('Stripe test operation unavailable'); }
}
const requireValue = (condition: unknown): void => { if (!condition) throw new BoundaryError(); };
const object = (value: unknown): value is Json => value !== null && typeof value === 'object' && !Array.isArray(value);
const integer = (value: unknown): value is number => Number.isSafeInteger(value);
const reply = (status: number, value: Json) => new Response(JSON.stringify(value), { status,
  headers: { 'Content-Type': 'application/json', 'Cache-Control': 'no-store',
    'X-Content-Type-Options': 'nosniff' } });

function configuration(ctx: CommerceContext): Configuration {
  const env = ctx.env;
  requireValue(env.STRIPE_TEST_CHECKOUT_ENABLED === 'true');
  requireValue(/^(?:sk|rk)_test_[A-Za-z0-9]{16,256}$/.test(env.STRIPE_TEST_SECRET_KEY ?? ''));
  requireValue(/^whsec_[A-Za-z0-9]{16,256}$/.test(env.STRIPE_TEST_WEBHOOK_SECRET ?? ''));
  requireValue(/^price_[A-Za-z0-9]{8,128}$/.test(env.STRIPE_TEST_PRICE_ID ?? ''));
  const origin = env.STRIPE_TEST_RETURN_ORIGIN ?? '';
  requireValue(/^https:\/\/(?=.{1,253}$)(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?\.)+[a-z](?:[a-z0-9-]{0,61}[a-z0-9])?$/.test(origin));
  const coupons = (env.STRIPE_TEST_ALLOWED_COUPON_IDS ?? '').split(',').map(value => value.trim()).filter(Boolean);
  requireValue(coupons.length <= 100 && coupons.every(value => /^[A-Za-z0-9_-]{1,128}$/.test(value)));
  const promotions = (env.STRIPE_TEST_ALLOWED_PROMOTION_IDS ?? '').split(',').map(value => value.trim()).filter(Boolean);
  requireValue(promotions.length <= 100 && promotions.every(value => PROMOTION.test(value)));
  return { key: env.STRIPE_TEST_SECRET_KEY!, secret: env.STRIPE_TEST_WEBHOOK_SECRET!,
    price: env.STRIPE_TEST_PRICE_ID!, origin, coupons: [...new Set(coupons)],
    promotions: [...new Set(promotions)] };
}

async function discountCode(req: Request): Promise<string | null> {
  if (!req.body) return null;
  requireValue(req.headers.get('Content-Type')?.split(';')[0].toLowerCase() === 'application/json');
  const length = req.headers.get('Content-Length');
  if (length !== null) requireValue(/^[0-9]{1,4}$/.test(length) && Number(length) <= 1024);
  const body = strictJson(await boundedBytes(req.body, 1024));
  requireValue(Object.keys(body).every(key => key === 'discountCode'));
  if (body.discountCode === undefined || body.discountCode === null || body.discountCode === '') return null;
  requireValue(typeof body.discountCode === 'string' && CODE.test(body.discountCode));
  return body.discountCode;
}

async function approvedPromotion(ctx: CommerceContext, config: Configuration, code: string): Promise<Json | null> {
  const query = new URLSearchParams({ code, active: 'true', limit: '10' });
  const list = await stripeRequest(ctx, config, `promotion_codes?${query}`, 'GET');
  requireValue(list.object === 'list' && list.has_more === false && Array.isArray(list.data)
    && list.data.length <= 10);
  // A code with customer-specific or ambiguous matches is never applied to an
  // unrelated account; this beta has no Stripe customer binding.
  if (list.data.length !== 1) return null;
  const promotion = list.data[0];
  const now = Math.floor((ctx.now?.() ?? Date.now()) / 1000);
  if (!object(promotion) || promotion.object !== 'promotion_code'
    || !PROMOTION.test(promotion.id) || !config.promotions.includes(promotion.id)
    || promotion.livemode !== false || promotion.active !== true || promotion.customer !== null
    || (promotion.customer_account !== null && promotion.customer_account !== undefined)
    || !object(promotion.promotion) || promotion.promotion.type !== 'coupon'
    || typeof promotion.promotion.coupon !== 'string' || !config.coupons.includes(promotion.promotion.coupon)
    || typeof promotion.code !== 'string' || promotion.code.toLowerCase() !== code.toLowerCase()
    || !object(promotion.restrictions) || promotion.restrictions.first_time_transaction !== false
    || promotion.restrictions.minimum_amount !== null
    || (promotion.expires_at !== null && (!integer(promotion.expires_at) || promotion.expires_at <= now))
    || (promotion.max_redemptions !== null && (!integer(promotion.max_redemptions)
      || !integer(promotion.times_redeemed) || promotion.times_redeemed >= promotion.max_redemptions))) return null;
  const coupon = await applicableCoupon(ctx, config, promotion.promotion.coupon, true);
  if (!coupon) return null;
  return { promotion_id: promotion.id, coupon_id: coupon.id };
}

async function applicableCoupon(ctx: CommerceContext, config: Configuration,
                                couponId: string, available: boolean): Promise<Json | null> {
  requireValue(config.coupons.includes(couponId) && /^[A-Za-z0-9_-]{1,128}$/.test(couponId));
  requireValue(/^price_[A-Za-z0-9]{8,128}$/.test(config.price));
  const price = await stripeRequest(ctx, config, `prices/${config.price}`, 'GET');
  if (price.object !== 'price' || price.id !== config.price || price.livemode !== false
    || price.currency !== 'usd' || price.unit_amount !== 2000 || price.type !== 'one_time'
    || price.recurring !== null || typeof price.product !== 'string' || !/^prod_[A-Za-z0-9]{8,128}$/.test(price.product)
    || (available && price.active !== true)) return null;
  // applies_to is includable. Absence never establishes product applicability.
  const coupon = await stripeRequest(ctx, config, `coupons/${couponId}?expand%5B%5D=applies_to`, 'GET');
  if (coupon.object !== 'coupon' || coupon.id !== couponId || coupon.livemode !== false
    || !object(coupon.applies_to) || !Array.isArray(coupon.applies_to.products)
    || coupon.applies_to.products.length !== 1 || coupon.applies_to.products[0] !== price.product) return null;
  const now = Math.floor((ctx.now?.() ?? Date.now()) / 1000);
  // Redemption availability belongs to creation, not historical settlement.
  if (available && (coupon.valid !== true
    || (coupon.redeem_by !== null && (!integer(coupon.redeem_by) || coupon.redeem_by <= now))
    || (coupon.max_redemptions !== null && (!integer(coupon.max_redemptions)
      || !integer(coupon.times_redeemed) || coupon.times_redeemed >= coupon.max_redemptions)))) return null;
  const percent = typeof coupon.percent_off === 'number' && Number.isFinite(coupon.percent_off)
    && coupon.percent_off > 0 && coupon.percent_off <= 100 && coupon.amount_off === null;
  const amount = coupon.percent_off === null && integer(coupon.amount_off)
    && coupon.amount_off > 0 && coupon.currency === 'usd';
  if (!percent && !amount) return null;
  return coupon;
}

async function boundedBytes(stream: ReadableStream<Uint8Array> | null, maximum: number): Promise<Uint8Array> {
  requireValue(stream !== null);
  const reader = stream!.getReader();
  const chunks: Uint8Array[] = [];
  let length = 0;
  try {
    for (;;) {
      const chunk = await reader.read();
      if (chunk.done) break;
      length += chunk.value.byteLength;
      requireValue(length <= maximum);
      chunks.push(chunk.value);
    }
  } finally {
    try { await reader.cancel(); } catch { /* No upstream exception detail escapes. */ }
    reader.releaseLock();
  }
  requireValue(length > 0);
  const value = new Uint8Array(length);
  let offset = 0;
  for (const chunk of chunks) { value.set(chunk, offset); offset += chunk.byteLength; }
  return value;
}

/** Reject duplicate keys as well as non-JSON/unsafe integer input before normalization. */
export function strictJson(raw: Uint8Array): Json {
  const text = decoder.decode(raw);
  let cursor = 0;
  const white = () => { while (/\s/.test(text[cursor] ?? '') && cursor < text.length) cursor++; };
  const stringToken = (): string => {
    const start = cursor++;
    while (cursor < text.length) {
      if (text[cursor] === '\\') { cursor += 2; continue; }
      if (text[cursor++] === '"') return JSON.parse(text.slice(start, cursor));
    }
    throw new BoundaryError();
  };
  const value = (depth = 0): void => {
    requireValue(depth <= 32);
    white();
    if (text[cursor] === '{') {
      cursor++; white(); const keys = new Set<string>();
      if (text[cursor] === '}') { cursor++; return; }
      for (;;) {
        requireValue(text[cursor] === '"'); const key = stringToken();
        requireValue(!keys.has(key)); keys.add(key); white();
        requireValue(text[cursor++] === ':'); value(depth + 1); white();
        const delimiter = text[cursor++]; if (delimiter === '}') return;
        requireValue(delimiter === ','); white();
      }
    }
    if (text[cursor] === '[') {
      cursor++; white(); if (text[cursor] === ']') { cursor++; return; }
      for (;;) { value(depth + 1); white(); const delimiter = text[cursor++];
        if (delimiter === ']') return; requireValue(delimiter === ','); }
    }
    if (text[cursor] === '"') { stringToken(); return; }
    const token = /^(?:true|false|null|-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)/.exec(text.slice(cursor));
    requireValue(token); cursor += token![0].length;
    if (/^-?[0-9]/.test(token![0])) {
      const number = Number(token![0]); requireValue(Number.isFinite(number));
      if (Number.isInteger(number)) requireValue(Number.isSafeInteger(number));
    }
  };
  value(); white(); requireValue(cursor === text.length);
  const result = JSON.parse(text); requireValue(object(result)); return result;
}

async function stripeRequest(ctx: CommerceContext, config: Configuration, path: string,
                             method: string, body?: URLSearchParams): Promise<Json> {
  // Callers supply fixed paths plus regex-validated provider identifiers only.
  const url = STRIPE_API + path;
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 10_000);
  let response: Response | undefined;
  try {
    const headers: Record<string, string> = { Authorization: `Bearer ${config.key}`,
      'Stripe-Version': STRIPE_API_VERSION, Accept: 'application/json', 'Accept-Encoding': 'identity' };
    if (body) {
      headers['Content-Type'] = 'application/x-www-form-urlencoded';
      headers['Idempotency-Key'] = `skager-edge-test-v1-${body.get('client_reference_id')}`;
    }
    response = await (ctx.fetch ?? fetch)(url, { method, headers, body,
      redirect: 'manual', signal: controller.signal });
    requireValue(response.status === 200 && !response.redirected
      && (!response.url || response.url === url));
    requireValue(response.headers.get('Content-Type')?.split(';')[0].toLowerCase() === 'application/json');
    requireValue(['', 'identity'].includes(response.headers.get('Content-Encoding')?.toLowerCase() ?? ''));
    const length = response.headers.get('Content-Length');
    if (length !== null) requireValue(/^[0-9]{1,8}$/.test(length) && Number(length) <= MAX_PROVIDER_BYTES);
    const raw = await boundedBytes(response.body, MAX_PROVIDER_BYTES);
    if (length !== null) requireValue(raw.byteLength === Number(length));
    return strictJson(raw);
  } catch {
    try { await response?.body?.cancel(); } catch { /* No provider exception escapes. */ }
  }
  finally { clearTimeout(timer); }
  throw new BoundaryError();
}

function sessionIdentity(session: Json, orderId?: string): void {
  requireValue(session.object === 'checkout.session' && session.livemode === false
    && session.mode === 'payment' && SESSION.test(session.id) && UUID.test(session.client_reference_id)
    && object(session.metadata) && session.metadata.order_id === session.client_reference_id);
  if (orderId) requireValue(session.client_reference_id === orderId);
  requireValue(session.currency === 'usd' && integer(session.amount_subtotal) && session.amount_subtotal === 2000
    && integer(session.amount_total) && session.amount_total >= 0 && session.amount_total <= 2000
    && ['paid', 'unpaid', 'no_payment_required'].includes(session.payment_status));
}

function checkoutUrl(value: unknown, sessionId: string): string {
  requireValue(typeof value === 'string' && value.length <= 4096 && !/[\x00-\x20\x7f]/.test(value));
  const url = new URL(value as string);
  requireValue(url.protocol === 'https:' && url.hostname === 'checkout.stripe.com'
    && url.host === 'checkout.stripe.com' && !url.username && !url.password
    && url.pathname === `/c/pay/${sessionId}` && !url.search
    && (value as string).startsWith(`https://checkout.stripe.com/c/pay/${sessionId}`));
  return value as string;
}

/** Normalize only finalized provider evidence. No account IDs come from Stripe metadata. */
export function confirmedCheckout(session: Json, config: Configuration, orderId: string, sessionId: string,
                                  couponEvidence?: Json): Json {
  sessionIdentity(session, orderId);
  requireValue(session.id === sessionId && session.status === 'complete'
    && ['hosted', 'hosted_page'].includes(session.ui_mode));
  const totals = session.total_details;
  requireValue(object(totals) && integer(totals.amount_discount) && totals.amount_discount >= 0
    && totals.amount_discount <= 2000 && totals.amount_tax === 0 && totals.amount_shipping === 0
    && session.amount_total === 2000 - totals.amount_discount);
  const lines = session.line_items;
  requireValue(object(lines) && lines.object === 'list' && lines.has_more === false
    && Array.isArray(lines.data) && lines.data.length === 1);
  const line = lines.data[0]; const price = line?.price;
  requireValue(object(line) && object(price) && price.object === 'price' && price.id === config.price
    && price.livemode === false && price.currency === 'usd' && price.unit_amount === 2000
    && price.type === 'one_time' && price.recurring === null
    && line.quantity === 1 && line.currency === 'usd' && line.amount_subtotal === 2000
    && line.amount_total === session.amount_total && line.amount_discount === totals.amount_discount
    && line.amount_tax === 0);
  requireValue(Array.isArray(session.discounts) && session.discounts.length <= 1);
  const coupons: string[] = [];
  if (totals.amount_discount > 0) {
    const discount = session.discounts[0]; const coupon = couponEvidence;
    requireValue(session.discounts.length === 1 && object(coupon) && coupon.object === 'coupon'
      && coupon.livemode === false && config.coupons.includes(coupon.id)
      && object(coupon.applies_to) && Array.isArray(coupon.applies_to.products)
      && coupon.applies_to.products.length === 1 && coupon.applies_to.products[0] === price.product
      && typeof price.product === 'string' && /^prod_[A-Za-z0-9]{8,128}$/.test(price.product)
      && typeof config.selectedPromotion === 'string' && PROMOTION.test(config.selectedPromotion)
      && discount?.promotion_code === config.selectedPromotion
      && (discount.coupon === null || discount.coupon === coupon.id
        || (object(discount.coupon) && discount.coupon.object === 'coupon'
          && discount.coupon.livemode === false && discount.coupon.id === coupon.id)));
    let expected = -1;
    if (typeof coupon.percent_off === 'number' && Number.isFinite(coupon.percent_off)
      && coupon.percent_off > 0 && coupon.percent_off <= 100 && coupon.amount_off === null) {
      expected = Math.round(2000 * coupon.percent_off / 100);
    } else if (coupon.percent_off === null && integer(coupon.amount_off) && coupon.amount_off > 0
      && coupon.currency === 'usd') expected = Math.min(2000, coupon.amount_off);
    requireValue(totals.amount_discount === expected);
    coupons.push(coupon.id);
  } else requireValue(session.discounts.length === 0);
  if (session.amount_total === 0) {
    requireValue(session.payment_status === 'no_payment_required' && totals.amount_discount === 2000
      && coupons.length === 1 && session.payment_intent === null);
  } else requireValue(session.payment_status === 'paid' && INTENT.test(session.payment_intent));
  return { order_id: orderId, session_id: sessionId, price_id: price.id, subtotal_minor: 2000,
    discount_minor: totals.amount_discount, tax_minor: 0, total_minor: session.amount_total,
    payment_status: session.payment_status, payment_intent_id: session.payment_intent, coupon_ids: coupons };
}

function constantEqual(left: Uint8Array, right: Uint8Array): boolean {
  let different = left.length ^ right.length;
  for (let i = 0; i < Math.max(left.length, right.length); i++) different |= (left[i] ?? 0) ^ (right[i] ?? 0);
  return different === 0;
}
const hex = (bytes: ArrayBuffer): string => [...new Uint8Array(bytes)].map(value => value.toString(16).padStart(2, '0')).join('');

function structuralSession(item: Json): void {
  requireValue(item.object === 'checkout.session' && item.livemode === false && SESSION.test(item.id)
    && ['payment', 'setup', 'subscription'].includes(item.mode)
    && (item.client_reference_id === null || typeof item.client_reference_id === 'string')
    && (item.metadata === null || object(item.metadata))
    && (item.currency === null || (typeof item.currency === 'string' && /^[a-z]{3}$/.test(item.currency)))
    && (item.amount_subtotal === null || (integer(item.amount_subtotal) && item.amount_subtotal >= 0))
    && (item.amount_total === null || (integer(item.amount_total) && item.amount_total >= 0))
    && ['open', 'complete', 'expired'].includes(item.status)
    && ['paid', 'unpaid', 'no_payment_required'].includes(item.payment_status)
    && (item.payment_intent === null || (typeof item.payment_intent === 'string' && INTENT.test(item.payment_intent))));
}

async function existingBinding(ctx: CommerceContext, session: Json): Promise<Json | null> {
  // Our creator always supplies a canonical order UUID. A session without one
  // cannot be ours. Metadata and current price never establish ownership.
  if (typeof session.client_reference_id !== 'string' || !UUID.test(session.client_reference_id)) return null;
  const binding = await ctx.rpc('commerce_checkout_binding', {
    p_order_id: session.client_reference_id, p_session_id: session.id,
  });
  requireValue(object(binding));
  // SQL deliberately does not distinguish absent orders from mismatched sessions.
  // Both are ineligible; neither result authorizes an event receipt or effect.
  if (binding.state === 'rejected') return null;
  requireValue(binding.state === 'bound' && typeof binding.price_id === 'string'
    && /^price_[A-Za-z0-9]{8,128}$/.test(binding.price_id));
  return binding;
}

export async function verifiedEvent(raw: Uint8Array, header: string, secret: string, now: number): Promise<Json> {
  requireValue(raw.length > 0 && raw.length <= MAX_WEBHOOK_BYTES && /^[\x20-\x7e]{1,2048}$/.test(header));
  const parts = header.split(','); requireValue(parts.length <= 32);
  const timestamps: string[] = []; const signatures: string[] = [];
  for (const part of parts) {
    const match = /^([^=]+)=(.*)$/.exec(part.trim()); requireValue(match);
    if (match![1] === 't') { requireValue(/^[1-9][0-9]{0,11}$/.test(match![2])); timestamps.push(match![2]); }
    if (match![1] === 'v1') { requireValue(/^[0-9a-f]{64}$/.test(match![2])); signatures.push(match![2]); }
  }
  requireValue(timestamps.length === 1 && signatures.length >= 1 && signatures.length <= 8
    && integer(now) && Math.abs(now - Number(timestamps[0])) <= 300);
  requireValue(/^whsec_[A-Za-z0-9]{16,256}$/.test(secret));
  const key = await crypto.subtle.importKey('raw', encoder.encode(secret), { name: 'HMAC', hash: 'SHA-256' }, false, ['sign']);
  const prefix = encoder.encode(timestamps[0] + '.');
  const input = new Uint8Array(prefix.length + raw.length); input.set(prefix); input.set(raw, prefix.length);
  const expected = encoder.encode(hex(await crypto.subtle.sign('HMAC', key, input)));
  let matched = false;
  for (const signature of signatures) matched = constantEqual(expected, encoder.encode(signature)) || matched;
  requireValue(matched);
  const event = strictJson(raw);
  requireValue(event.object === 'event' && event.livemode === false && EVENT.test(event.id)
    && event.api_version === STRIPE_API_VERSION && !('account' in event) && !('context' in event)
    && integer(event.created) && event.created > 0 && event.created < 1e12 && TYPES.has(event.type)
    && object(event.data) && object(event.data.object));
  const item = event.data.object;
  if (event.type === 'charge.refunded') {
    requireValue(item.object === 'charge' && item.livemode === false && CHARGE.test(item.id)
      && INTENT.test(item.payment_intent) && typeof item.currency === 'string'
      && /^[a-z]{3}$/.test(item.currency) && item.paid === true
      && integer(item.amount) && item.amount > 0
      && integer(item.amount_refunded) && item.amount_refunded > 0 && item.amount_refunded <= item.amount);
  } else structuralSession(item);
  return { event_id: event.id, event_type: event.type, created: event.created,
    digest: hex(await crypto.subtle.digest('SHA-256', raw)), object: item };
}

async function webhook(req: Request, ctx: CommerceContext, config: Configuration): Promise<Response> {
  let event: Json;
  try {
    const length = req.headers.get('Content-Length');
    if (length !== null) requireValue(/^[0-9]{1,8}$/.test(length) && Number(length) <= MAX_WEBHOOK_BYTES);
    const raw = await boundedBytes(req.body, MAX_WEBHOOK_BYTES);
    event = await verifiedEvent(raw, req.headers.get('Stripe-Signature') ?? '', config.secret,
      Math.floor((ctx.now?.() ?? Date.now()) / 1000));
  } catch { return reply(400, { error: 'Webhook rejected' }); }
  const item = event.object;
  try {
    let ownedSession = item;
    let charge: Json | undefined;
    if (event.event_type === 'charge.refunded') {
      charge = await stripeRequest(ctx, config, `charges/${item.id}`, 'GET');
      requireValue(charge.object === 'charge' && charge.id === item.id && charge.livemode === false
        && charge.payment_intent === item.payment_intent && charge.currency === item.currency && charge.paid === true
        && charge.amount === item.amount && integer(charge.amount_refunded)
        && charge.amount_refunded >= item.amount_refunded && charge.amount_refunded <= charge.amount);
      // Stripe's documented PaymentIntent filter links a charge to its session.
      // Empty/ambiguous results can be a provider visibility race: retry, no writes.
      const query = new URLSearchParams({ payment_intent: charge.payment_intent, limit: '2' });
      const sessions = await stripeRequest(ctx, config, `checkout/sessions?${query}`, 'GET');
      requireValue(sessions.object === 'list' && sessions.has_more === false
        && Array.isArray(sessions.data) && sessions.data.length === 1 && object(sessions.data[0]));
      ownedSession = sessions.data[0];
      structuralSession(ownedSession);
      requireValue(ownedSession.mode === 'payment' && ownedSession.status === 'complete'
        && ownedSession.payment_status === 'paid' && ownedSession.payment_intent === charge.payment_intent
        && ownedSession.amount_total === charge.amount && ownedSession.currency === charge.currency);
    }
    const binding = await existingBinding(ctx, ownedSession);
    if (!binding) return reply(200, { received: true });
    // Known owned sessions retain the strict SKAGER identity/money contract.
    // Corrupted own metadata is retryable, never silently classified as foreign.
    sessionIdentity(ownedSession);
    let confirmed: Json | undefined;
    if (event.event_type !== 'charge.refunded' && !event.event_type.endsWith('.expired')
      && !event.event_type.endsWith('.async_payment_failed') && item.status === 'complete'
      && ['paid', 'no_payment_required'].includes(item.payment_status)) {
      const boundConfig = { ...config, price: binding.price_id,
        coupons: binding.selected_coupon_id ? [binding.selected_coupon_id] : [],
        selectedPromotion: binding.promotion_id };
      requireValue(!binding.selected_coupon_id || config.coupons.includes(binding.selected_coupon_id));
      requireValue(!binding.promotion_id || config.promotions.includes(binding.promotion_id));
      const session = await stripeRequest(ctx, boundConfig,
        `checkout/sessions/${item.id}?expand%5B%5D=line_items&expand%5B%5D=discounts.coupon`, 'GET');
      let couponEvidence: Json | undefined;
      if (binding.selected_coupon_id) {
        requireValue(typeof binding.promotion_id === 'string' && PROMOTION.test(binding.promotion_id));
        const promotion = await stripeRequest(ctx, boundConfig, `promotion_codes/${binding.promotion_id}`, 'GET');
        requireValue(promotion.object === 'promotion_code' && promotion.id === binding.promotion_id
          && promotion.livemode === false && object(promotion.promotion) && promotion.promotion.type === 'coupon'
          && promotion.promotion.coupon === binding.selected_coupon_id);
        const coupon = await applicableCoupon(ctx, boundConfig, binding.selected_coupon_id, false);
        requireValue(coupon);
        couponEvidence = coupon!;
      }
      confirmed = confirmedCheckout(session, boundConfig, ownedSession.client_reference_id, ownedSession.id, couponEvidence);
      requireValue(confirmed.total_minor === item.amount_total && confirmed.payment_status === item.payment_status);
    }
    // No receipt is attempted until ownership and all required provider reads
    // have succeeded. Original SQL remains the atomic/idempotent effect boundary.
    const receipt = await ctx.rpc('commerce_receive_event', { p_event_id: event.event_id,
      p_digest: event.digest, p_event_type: event.event_type, p_created: event.created,
      p_reference: event.event_type === 'charge.refunded' ? item.id : item.client_reference_id,
      p_provider_id: item.id });
    if (receipt.state === 'conflict') return reply(400, { error: 'Webhook rejected' });
    if (receipt.state === 'applied' || receipt.state === 'rejected') return reply(200, { received: true });
    requireValue(receipt.state === 'received');
    if (charge) {
      await ctx.rpc('commerce_apply_refund', { p_event_id: event.event_id, p_digest: event.digest,
        p_charge_id: charge.id, p_intent_id: charge.payment_intent, p_charged_minor: charge.amount,
        p_refunded_minor: charge.amount_refunded });
    } else if (event.event_type.endsWith('.expired') || event.event_type.endsWith('.async_payment_failed')) {
      await ctx.rpc('commerce_close_checkout', { p_event_id: event.event_id, p_digest: event.digest,
        p_order_id: item.client_reference_id, p_session_id: item.id });
    } else if (confirmed) {
      await ctx.rpc('commerce_apply_checkout', { p_event_id: event.event_id, p_digest: event.digest,
        p_checkout: confirmed });
    } else {
      await ctx.rpc('commerce_reject_event', { p_event_id: event.event_id, p_digest: event.digest });
    }
    return reply(200, { received: true });
  } catch { return reply(503, { error: 'Confirmation pending' }); }
}

export async function handleCommerce(req: Request, ctx: CommerceContext): Promise<Response | null> {
  const path = new URL(req.url).pathname;
  if (!['/api/checkout/start', '/api/stripe/webhook'].includes(path)) return null;
  if (req.method !== 'POST') return reply(405, { error: 'POST required' });
  let config: Configuration;
  try { config = configuration(ctx); } catch { return reply(404, { error: 'Unavailable' }); }
  if (path === '/api/stripe/webhook') return webhook(req, ctx, config);
  // Root router also verifies CSRF. A fixed exact Origin is mandatory here so
  // direct invocation cannot bypass browser request origin checks.
  if (req.headers.get('Origin') !== config.origin) return reply(403, { error: 'Request rejected' });
  try {
    const account = await ctx.getAccount(req);
    if (!account || !UUID.test(account.id)) return reply(401, { error: 'Sign in required' });
    let code: string | null;
    try { code = await discountCode(req); }
    catch { return reply(400, { error: 'Invalid discount code' }); }
    const promotion = code ? await approvedPromotion(ctx, config, code) : null;
    if (code && !promotion) return reply(400, { error: 'Invalid discount code' });
    const order = await ctx.rpc('commerce_reserve_checkout', { p_account_id: account.id,
      p_price_id: config.price, p_allowed_coupon_ids: config.coupons,
      p_promotion_id: promotion?.promotion_id ?? null, p_selected_coupon_id: promotion?.coupon_id ?? null });
    requireValue(UUID.test(order.order_id));
    if (order.session_id && order.checkout_url) {
      requireValue(SESSION.test(order.session_id));
      return reply(200, { url: checkoutUrl(order.checkout_url, order.session_id) });
    }
    const form = new URLSearchParams({ mode: 'payment', currency: 'usd',
      'line_items[0][price]': config.price, 'line_items[0][quantity]': '1',
      client_reference_id: order.order_id, 'metadata[order_id]': order.order_id,
      success_url: config.origin + '/checkout/test/success/',
      cancel_url: config.origin + '/checkout/test/cancel/' });
    if (promotion) form.set('discounts[0][promotion_code]', promotion.promotion_id);
    // These parameters are mutually exclusive even with an explicit false.
    // Without allow_promotion_codes, hosted promotion entry remains disabled.
    else form.set('allow_promotion_codes', 'false');
    const session = await stripeRequest(ctx, config, 'checkout/sessions', 'POST', form);
    sessionIdentity(session, order.order_id);
    requireValue(session.status === 'open' && ['hosted', 'hosted_page'].includes(session.ui_mode));
    const url = checkoutUrl(session.url, session.id);
    await ctx.rpc('commerce_bind_checkout', { p_account_id: account.id, p_order_id: order.order_id,
      p_session_id: session.id, p_checkout_url: url });
    return reply(200, { url });
  } catch { return reply(503, { error: 'Checkout temporarily unavailable' }); }
}
