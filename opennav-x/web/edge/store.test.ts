import assert from 'node:assert/strict';
import test from 'node:test';
import { STRIPE_API_VERSION } from './commerce.ts';
import { handleStore, type StoreContext } from './store.ts';
import { handleAdmin, productInput, stepInput } from './admin.ts';
import { compose, sendDue } from './email.ts';
import { imageExtension, MEDIA_KEY } from './media.ts';

const ORDER = '6f1d2b3c-4d5e-4f60-8a71-b2c3d4e5f607';
const ACCESS = 'a1111111-1111-4111-8111-111111111111';
const GPS = 'b2222222-2222-4222-8222-222222222222';
const CLOUD = 'c3333333-3333-4333-8333-333333333333';
const ACCOUNT = 'd4444444-4444-4444-8444-444444444444';
const SECRET = 'whsec_' + 'a'.repeat(32);
const csrf = 'c'.repeat(64);
const env = {
  STRIPE_TEST_CHECKOUT_ENABLED: 'true', STRIPE_TEST_SECRET_KEY: 'rk_test_abc123', STRIPE_TEST_WEBHOOK_SECRET: SECRET,
  STRIPE_TEST_PRICE_ID: 'price_access0001', STRIPE_TEST_RETURN_ORIGIN: 'https://beta.skager.app',
  STRIPE_TEST_ALLOWED_COUPON_IDS: 'FREE100', STRIPE_TEST_ALLOWED_PROMOTION_IDS: 'promo_free10000',
  SKAGER_JOB_SECRET: 'j'.repeat(40), SKAGER_ADMIN_EMAILS: 'admin@example.com', RESEND_API_KEY: 're_testkey_123456',
};
const products = [
  { id: ACCESS, slug: 'skager-beta-access', kind: 'access', name: 'SKAGER Public Beta Access', price_minor: 2000, checkout_addon: false, shippable: false },
  { id: GPS, slug: 'gps-module', kind: 'hardware', name: 'SKAGER GPS module', price_minor: 4900, checkout_addon: true, shippable: true, image_key: null },
  { id: CLOUD, slug: 'skager-cloud', kind: 'subscription', name: 'SKAGER Cloud', price_minor: 200, interval: 'month' },
];
const STEP = 'e5555555-5555-4555-8555-555555555555';
const steps = [{ id: STEP, flow: 'checkout', title: 'GPS', summary: 'Position for the chart.', skip_label: 'I already have a GPS', multiple: false,
  link_url: null, link_label: '', active: true, sort: 20 }];
const accessItems = [{ product_id: ACCESS, slug: 'skager-beta-access', kind: 'access', name: 'SKAGER Public Beta Access', quantity: 1, unit_minor: 2000, interval: null },
  { product_id: GPS, slug: 'gps-module', kind: 'hardware', name: 'SKAGER GPS module', quantity: 1, unit_minor: 4900, interval: null, shippable: true }];

function json(value: unknown, status = 200) {
  return new Response(JSON.stringify(value), { status, headers: { 'Content-Type': 'application/json' } });
}
function fixture(overrides: Record<string, any> = {}) {
  const calls: { name: string; args: any }[] = [];
  const stripe: { url: string; method: string; body: URLSearchParams | null; headers: Headers }[] = [];
  const state: Record<string, any> = { order: { state: 'created', order_id: ORDER, items: accessItems, subtotal_minor: 6900,
    shipping_minor: 1500, stripe_customer_id: null, shipping_countries: ['SE', 'NO'] },
    binding: { state: 'bound', id: ORDER, kind: 'access', items: accessItems, subtotal_minor: 6900, shipping_minor: 1500,
      promotion_id: null, coupon_id: null }, receipt: 'received', throttle: true, session: null, ...overrides };
  const ctx: StoreContext = {
    env: { ...env, ...(overrides.env ?? {}) },
    now: () => (overrides.now ?? Date.now()),
    getAccount: async () => state.account ?? null,
    rpc: async (name, args) => {
      calls.push({ name, args });
      if (name === 'skager_auth_throttle') {
        // Same bounds as public.skager_auth_throttle in auth.sql, which raises outside them.
        const { p_keys, p_limit, p_seconds } = args as { p_keys: string[]; p_limit: number; p_seconds: number };
        if (p_keys.length < 1 || p_keys.length > 3 || p_keys.some(key => key.length < 1 || key.length > 100)
          || p_limit < 1 || p_limit > 100 || p_seconds < 1 || p_seconds > 3600) throw new Error('Invalid throttle configuration');
        return state.throttle;
      }
      if (name === 'store_catalog') return { products, steps, shipping_minor: 1500, shipping_countries: ['SE'] };
      if (name === 'store_create_order') return typeof state.order === 'function' ? state.order(args) : state.order;
      if (name === 'store_order_binding') return state.binding;
      if (name === 'store_receive_event') return { state: state.receipt };
      if (name === 'store_portal_access') return state.access ?? { entitlement: { status: 'active' }, stripe_customer_id: 'cus_customer0001', subscription: null };
      if (name === 'store_refund_request') return state.refund;
      if (name === 'store_apply_refund') return { known: true, kind: 'access', cancel_subscriptions: ['sub_cloud00000001'] };
      if (name === 'store_email_claim') return state.outbox ?? [];
      if (name === 'store_job_maintenance') return { expired_orders: 0 };
      if (name === 'store_admin_overview') return { products: [], orders: [], customers: [] };
      if (name === 'store_admin_save_product') return { id: GPS, ...args.p_product };
      if (name === 'store_admin_save_step') return { id: STEP, ...args.p_step };
      return true;
    },
    fetch: async (input: any, init?: any) => {
      const url = String(input instanceof Request ? input.url : input);
      const body = init?.body instanceof URLSearchParams ? init.body : null;
      stripe.push({ url, method: init?.method ?? 'GET', body, headers: new Headers(init?.headers) });
      if (url === 'https://api.stripe.com/v1/checkout/sessions' && init?.method === 'POST') {
        return json({ object: 'checkout.session', livemode: false, id: 'cs_test_session0001', client_reference_id: body!.get('client_reference_id'),
          mode: body!.get('mode'), status: 'open', url: 'https://checkout.stripe.com/c/pay/cs_test_session0001#frag' });
      }
      if (url.startsWith('https://api.stripe.com/v1/checkout/sessions/cs_test_session0001')) return json(state.session);
      if (url === 'https://api.stripe.com/v1/charges/ch_refund000001') {
        return json({ object: 'charge', id: 'ch_refund000001', livemode: false, payment_intent: 'pi_paid00000001', amount: 8400, amount_refunded: 8400 });
      }
      if (url === 'https://api.stripe.com/v1/refunds') return json({ object: 'refund', livemode: false, payment_intent: body!.get('payment_intent') });
      if (url.startsWith('https://api.stripe.com/v1/subscriptions/')) return json({ object: 'subscription', id: 'sub_cloud00000001', livemode: false, status: 'canceled' });
      if (url === 'https://api.stripe.com/v1/billing_portal/sessions') return json({ object: 'billing_portal.session', livemode: false, url: 'https://billing.stripe.com/p/session/x' });
      if (url === 'https://api.resend.com/emails') return json({ id: 'email-1' });
      throw new Error('unexpected ' + url);
    },
  };
  return { ctx, calls, stripe, state };
}
function post(path: string, data: unknown, origin = 'https://beta.skager.app') {
  return new Request('https://beta.skager.app' + path, { method: 'POST', body: JSON.stringify(data),
    headers: { Origin: origin, 'Content-Type': 'application/json', Cookie: `__Host-skager_csrf=${csrf}`, 'X-CSRF-Token': csrf } });
}
async function signed(event: unknown, secret = SECRET, at = Math.floor(Date.now() / 1000)) {
  const raw = JSON.stringify(event);
  const key = await crypto.subtle.importKey('raw', new TextEncoder().encode(secret), { name: 'HMAC', hash: 'SHA-256' }, false, ['sign']);
  const signature = [...new Uint8Array(await crypto.subtle.sign('HMAC', key, new TextEncoder().encode(`${at}.${raw}`)))]
    .map(b => b.toString(16).padStart(2, '0')).join('');
  return new Request('https://beta.skager.app/api/stripe/webhook', { method: 'POST', body: raw,
    headers: { 'Stripe-Signature': `t=${at},v1=${signature}`, 'Content-Type': 'application/json' } });
}
function event(type: string, object: unknown) {
  return { object: 'event', id: 'evt_test0000000001', livemode: false, api_version: STRIPE_API_VERSION, created: 1_800_000_000, type, data: { object } };
}
function completedSession(overrides: Record<string, any> = {}) {
  return { object: 'checkout.session', livemode: false, id: 'cs_test_session0001', client_reference_id: ORDER, status: 'complete',
    mode: 'payment', currency: 'usd', payment_status: 'paid', payment_intent: 'pi_paid00000001', customer: 'cus_customer0001',
    amount_subtotal: 6900, amount_total: 8400, total_details: { amount_discount: 0, amount_tax: 0, amount_shipping: 1500 },
    discounts: [], collected_information: { shipping_details: { name: 'A Sailor', address: { line1: 'Kaj 1', city: 'Sandhamn', postal_code: '13039', country: 'SE', extra: 'x' } } },
    line_items: { object: 'list', has_more: false, data: [
      { quantity: 1, amount_subtotal: 2000, price: { id: 'price_access0001', livemode: false, currency: 'usd', unit_amount: 2000, type: 'one_time', product: 'prod_access0001' } },
      { quantity: 1, amount_subtotal: 4900, price: { id: 'price_inline0001', livemode: false, currency: 'usd', unit_amount: 4900, type: 'one_time', product: 'prod_gps00000001' } },
    ] }, ...overrides };
}

test('guest checkout builds Stripe parameters from the catalog and binds the session', async () => {
  const f = fixture();
  const response = await handleStore(post('/api/store/checkout', { email: ' Buyer@Example.com ', addons: [GPS] }), f.ctx);
  assert.equal(response!.status, 200);
  assert.deepEqual(await response!.json(), { url: 'https://checkout.stripe.com/c/pay/cs_test_session0001#frag' });
  const order = f.calls.find(c => c.name === 'store_create_order')!.args;
  assert.equal(order.p_email, 'buyer@example.com');
  assert.deepEqual(order.p_items, [{ product_id: ACCESS, quantity: 1 }, { product_id: GPS, quantity: 1 }]);
  const form = f.stripe[0].body!;
  assert.equal(form.get('customer_email'), 'buyer@example.com');
  assert.equal(form.get('customer_creation'), 'always');
  assert.equal(form.get('line_items[0][price]'), 'price_access0001');
  assert.equal(form.get('line_items[1][price_data][unit_amount]'), '4900');
  assert.equal(form.get('shipping_options[0][shipping_rate_data][fixed_amount][amount]'), '1500');
  assert.equal(form.get('shipping_address_collection[allowed_countries][1]'), 'NO');
  assert.equal(form.get('allow_promotion_codes'), 'false');
  assert.equal(form.get('success_url'), 'https://beta.skager.app/welcome/?session_id={CHECKOUT_SESSION_ID}');
  assert.equal(f.stripe[0].headers.get('Idempotency-Key'), `skager-store-v1-${ORDER}`);
  assert.deepEqual(f.calls.find(c => c.name === 'store_bind_order')!.args.p_session_id, 'cs_test_session0001');
});

test('existing, unfinished and closed customers are not charged again', async () => {
  for (const [state, status, pattern] of [['exists', 409, /Log in/], ['setup', 409, /Finish setting up/], ['revoked', 409, /closed/]] as const) {
    const f = fixture({ order: { state } });
    const response = await handleStore(post('/api/store/checkout', { email: 'b@example.com' }), f.ctx);
    assert.equal(response!.status, status);
    assert.match((await response!.json()).error, pattern);
    assert.equal(f.stripe.length, 0);
  }
});

test('invalid input, unknown discount codes, throttling and foreign origins stop before Stripe checkout', async () => {
  const cases: [Request, Record<string, any>, number][] = [
    [post('/api/store/checkout', { email: 'not-an-email' }), {}, 400],
    [post('/api/store/checkout', { email: 'b@example.com', price: 1 }), {}, 400],
    [post('/api/store/checkout', { email: 'b@example.com', addons: ['x'] }), {}, 400],
    [post('/api/store/checkout', { email: 'b@example.com', addons: Array(10).fill(GPS) }), {}, 400],
    [post('/api/store/checkout', { email: 'b@example.com' }), { throttle: false }, 429],
    [post('/api/store/checkout', { email: 'b@example.com' }, 'https://evil.example'), {}, 403],
  ];
  for (const [req, overrides, status] of cases) {
    const f = fixture(overrides);
    assert.equal((await handleStore(req, f.ctx))!.status, status);
    assert.equal(f.stripe.length, 0);
  }
  const f = fixture();
  f.ctx.fetch = async (input: any) => String(input).includes('promotion_codes') ? json({ object: 'list', has_more: false, data: [] }) : assert.fail('no checkout');
  assert.equal((await handleStore(post('/api/store/checkout', { email: 'b@example.com', discountCode: 'NOPE' }), f.ctx))!.status, 400);
});

test('shop, cloud, billing and refunds need a signed-in account with a password', async () => {
  for (const account of [null, { id: ACCOUNT, email: 'b@example.com', passwordSet: false }]) {
    const f = fixture({ account });
    for (const path of ['/api/store/shop', '/api/store/cloud', '/api/store/billing', '/api/store/refund']) {
      const response = await handleStore(post(path, {}), f.ctx);
      assert.equal(response!.status, account ? 403 : 401);
    }
    assert.equal(f.stripe.length, 0);
  }
});

test('cloud subscription opens a monthly Checkout and refuses a second live subscription', async () => {
  const f = fixture({ account: { id: ACCOUNT, email: 'b@example.com', passwordSet: true },
    order: { state: 'created', order_id: ORDER, items: [{ kind: 'subscription', name: 'SKAGER Cloud', slug: 'skager-cloud', quantity: 1, unit_minor: 200, interval: 'month' }],
      subtotal_minor: 200, shipping_minor: 0, stripe_customer_id: 'cus_customer0001' } });
  assert.equal((await handleStore(post('/api/store/cloud', {}), f.ctx))!.status, 200);
  const form = f.stripe[0].body!;
  assert.equal(form.get('mode'), 'subscription');
  assert.equal(form.get('customer'), 'cus_customer0001');
  assert.equal(form.get('line_items[0][price_data][recurring][interval]'), 'month');
  const g = fixture({ account: { id: ACCOUNT, email: 'b@example.com', passwordSet: true },
    access: { entitlement: { status: 'active' }, subscription: { status: 'active' }, stripe_customer_id: 'cus_customer0001' } });
  assert.equal((await handleStore(post('/api/store/cloud', {}), g.ctx))!.status, 409);
  const billing = await handleStore(post('/api/store/billing', {}), g.ctx);
  assert.equal((await billing!.json()).url, 'https://billing.stripe.com/p/session/x');
});

test('a signed completed session is re-fetched, verified line by line and settled once', async () => {
  const f = fixture({ session: completedSession() });
  const response = await handleStore(await signed(event('checkout.session.completed', completedSession())), f.ctx);
  assert.equal(response!.status, 200);
  const apply = f.calls.find(c => c.name === 'store_apply_paid')!.args;
  assert.equal(apply.p_paid.total_minor, 8400);
  assert.equal(apply.p_paid.payment_intent_id, 'pi_paid00000001');
  assert.deepEqual(apply.p_paid.shipping.address, { line1: 'Kaj 1', line2: null, postal_code: '13039', city: 'Sandhamn', state: null, country: 'SE' });
  // Duplicate delivery: the receipt is already applied, so nothing is re-fetched or re-applied.
  const g = fixture({ session: completedSession(), receipt: 'applied' });
  await handleStore(await signed(event('checkout.session.completed', completedSession())), g.ctx);
  assert.equal(g.calls.some(c => c.name === 'store_apply_paid'), false);
});

test('tampered amounts, foreign sessions and bad signatures never settle', async () => {
  for (const session of [completedSession({ amount_total: 100 }), completedSession({ total_details: { amount_discount: 0, amount_tax: 0, amount_shipping: 0 } }),
    completedSession({ line_items: { object: 'list', has_more: false, data: [completedSession().line_items.data[0]] } })]) {
    const f = fixture({ session });
    assert.equal((await handleStore(await signed(event('checkout.session.completed', completedSession())), f.ctx))!.status, 503);
    assert.equal(f.calls.some(c => c.name === 'store_apply_paid'), false);
  }
  const foreign = fixture({ binding: { state: 'rejected' } });
  assert.equal((await handleStore(await signed(event('checkout.session.completed', completedSession())), foreign.ctx))!.status, 200);
  assert.equal(foreign.calls.some(c => c.name === 'store_receive_event'), false);
  const forged = fixture();
  assert.equal((await handleStore(await signed(event('checkout.session.completed', completedSession()), 'whsec_' + 'b'.repeat(32)), forged.ctx))!.status, 400);
  assert.equal(forged.calls.length, 0);
});

test('a refund is applied from Stripe evidence and ends the cloud subscription', async () => {
  const f = fixture();
  const charge = { object: 'charge', id: 'ch_refund000001', livemode: false };
  assert.equal((await handleStore(await signed(event('charge.refunded', charge)), f.ctx))!.status, 200);
  assert.deepEqual(f.calls.find(c => c.name === 'store_apply_refund')!.args.p_intent_id, 'pi_paid00000001');
  const cancel = f.stripe.find(s => s.url.endsWith('/subscriptions/sub_cloud00000001'))!;
  assert.equal(cancel.method, 'DELETE');
});

test('subscription updates are recorded; deletion is recorded as cancelled', async () => {
  const f = fixture();
  const sub = { object: 'subscription', id: 'sub_cloud00000001', status: 'active', cancel_at_period_end: true, items: { data: [{ current_period_end: 1_900_000_000 }] } };
  await handleStore(await signed(event('customer.subscription.deleted', sub)), f.ctx);
  const args = f.calls.find(c => c.name === 'store_apply_subscription')!.args;
  assert.equal(args.p_status, 'canceled');
  assert.equal(args.p_period_end, 1_900_000_000);
});

test('self-service refund: zero total closes now and logs out; paid orders ask Stripe', async () => {
  const account = { id: ACCOUNT, email: 'b@example.com', passwordSet: true };
  const free = fixture({ account, refund: { state: 'refunded' } });
  const closed = await handleStore(post('/api/store/refund', { orderId: ORDER }), free.ctx);
  assert.equal((await closed!.json()).state, 'refunded');
  assert.match(closed!.headers.get('Set-Cookie')!, /__Host-skager_session=; .*Max-Age=0/);
  const paid = fixture({ account, refund: { state: 'requested', payment_intent_id: 'pi_paid00000001', order_id: ORDER } });
  assert.equal((await (await handleStore(post('/api/store/refund', { orderId: ORDER }), paid.ctx))!.json()).state, 'requested');
  const refund = paid.stripe.find(s => s.url.endsWith('/refunds'))!;
  assert.equal(refund.body!.get('payment_intent'), 'pi_paid00000001');
  assert.equal(refund.headers.get('Idempotency-Key'), `skager-refund-${ORDER}`);
});

test('the scheduled job needs its secret; media keys are strict', async () => {
  const f = fixture();
  const job = (secret: string) => new Request('https://beta.skager.app/api/jobs/tick', { method: 'POST', headers: { 'x-skager-job': secret } });
  assert.equal((await handleStore(job('wrong'), f.ctx))!.status, 403);
  assert.equal((await handleStore(job('j'.repeat(40)), f.ctx))!.status, 200);
  assert.equal(MEDIA_KEY.test(`products/${GPS}/${'a'.repeat(64)}.png`), true);
  for (const key of ['../secret', `products/${GPS}/x.png`, `releases/${GPS}/${'a'.repeat(64)}.png`]) {
    // URL parsing removes '..' before routing; whatever reaches the store is refused.
    assert.equal((await handleStore(new Request(`https://beta.skager.app/api/media/${key}`), f.ctx))?.status ?? 404, 404);
  }
  assert.equal(imageExtension(new Uint8Array([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0])), 'png');
  assert.equal(imageExtension(new TextEncoder().encode('<svg onload=alert(1)>')), null);
});

test('emails escape customer data and admin mail goes only to the allowlist', async () => {
  const message = compose('shipped', { order: { tracking: '<b>X</b>', items: [{ name: '<script>', quantity: 1, unit_minor: 100 }], total_minor: 100 } })!;
  assert.equal(message.html.includes('<script>'), false);
  assert.match(message.html, /&lt;b&gt;X&lt;\/b&gt;/);
  const sent: any[] = [];
  const f = fixture({ outbox: [{ id: 7, to: 'admin', template: 'admin_order', data: { order: { id: ORDER, items: [] } } }] });
  f.ctx.fetch = async (_url: any, init: any) => { sent.push(JSON.parse(init.body)); return json({ id: 'email-7' }); };
  assert.deepEqual(await sendDue(f.ctx, 5), { sent: 1, failed: 0 });
  assert.deepEqual(sent[0].to, ['admin@example.com']);
  assert.deepEqual(f.calls.find(c => c.name === 'store_email_result')!.args, { p_id: 7, p_ok: true, p_provider_id: 'email-7' });
});

test('admin routes need an allowlisted, password-protected account and validate products', async () => {
  for (const account of [null, { id: ACCOUNT, email: 'b@example.com', passwordSet: true }, { id: ACCOUNT, email: 'admin@example.com', passwordSet: false }]) {
    const f = fixture({ account });
    const response = await handleAdmin(new Request('https://beta.skager.app/api/admin/overview'), f.ctx);
    assert.ok([401, 403].includes(response!.status));
    assert.equal(f.calls.length, 0);
  }
  const f = fixture({ account: { id: ACCOUNT, email: 'admin@example.com', passwordSet: true } });
  assert.equal((await handleAdmin(new Request('https://beta.skager.app/api/admin/overview'), f.ctx))!.status, 200);
  assert.equal(productInput({ name: 'Helm PC', priceMinor: 54900, kind: 'access' }), null);
  assert.equal(productInput({ name: '', priceMinor: 100 }), null);
  assert.equal(productInput({ name: 'X', priceMinor: -1 }), null);
  assert.equal(productInput({ name: 'Helm PC', priceMinor: 54900 })!.kind, 'hardware');
  const saved = await handleAdmin(post('/api/admin/product', { name: 'Helm PC', priceMinor: 54900, shippable: true, stepId: STEP }), f.ctx);
  assert.equal(saved!.status, 200);
  assert.equal(f.calls.find(c => c.name === 'store_admin_save_product')!.args.p_product.step_id, STEP);
  assert.equal(productInput({ name: 'Helm PC', priceMinor: 54900, stepId: 'gps' }), null);
  assert.equal(productInput({ name: 'Helm PC', priceMinor: 54900, checkoutAddon: true }), null);
});

test('checkout and download steps: public in the catalog, edited only by admins with safe links', async () => {
  const f = fixture();
  const catalog = await (await handleStore(new Request('https://beta.skager.app/api/store/catalog'), f.ctx))!.json();
  assert.deepEqual(catalog.steps, [{ id: STEP, flow: 'checkout', title: 'GPS', summary: 'Position for the chart.', skipLabel: 'I already have a GPS',
    multiple: false, linkUrl: null, linkLabel: null, sort: 20 }]);
  const step = { flow: 'download', title: 'Install OpenCPN', skipLabel: 'I already have OpenCPN installed',
    linkUrl: 'https://github.com/OpenCPN/OpenCPN/releases/tag/Release_5.12.4', linkLabel: 'Download OpenCPN' };
  assert.equal(stepInput(step)!.link_url, step.linkUrl);
  for (const bad of [{ ...step, flow: 'shop' }, { ...step, title: '' }, { ...step, skipLabel: ' ' }, { ...step, linkUrl: 'http://opencpn.org/' },
    { ...step, linkUrl: 'javascript:alert(1)' }, { ...step, linkUrl: 'https://user:pw@example.com/' }, { ...step, linkLabel: '' },
    { ...step, linkUrl: '', linkLabel: 'Go' }, { ...step, extra: 1 }, { ...step, sort: -1 }]) {
    assert.equal(stepInput(bad), null, JSON.stringify(bad));
  }
  assert.equal(stepInput({ id: STEP, title: 'GPS', skipLabel: 'Skip' })!.flow, undefined);
  // Guide steps keep their line breaks (one numbered step per line); titles and labels stay single-line.
  const guide = { flow: 'guide', title: 'Online AIS', skipLabel: 'Skip for now', summary: 'Optional.\r\n  Get a key. \n\nPaste it in SKAGER.  ' };
  assert.equal(stepInput(guide)!.summary, 'Optional.\nGet a key.\nPaste it in SKAGER.');
  for (const bad of [{ ...guide, summary: 'a\tb' }, { ...guide, summary: 'a\u0007b' }, { ...guide, summary: 'a'.repeat(1201) },
    { ...guide, title: 'Online\nAIS' }, { ...guide, skipLabel: 'Skip\r' }, { ...guide, summary: 7 }]) {
    assert.equal(stepInput(bad), null, JSON.stringify(bad));
  }
  const outsider = fixture({ account: { id: ACCOUNT, email: 'b@example.com', passwordSet: true } });
  assert.equal((await handleAdmin(post('/api/admin/step', step), outsider.ctx))!.status, 403);
  assert.equal(outsider.calls.length, 0);
  const admin = fixture({ account: { id: ACCOUNT, email: 'admin@example.com', passwordSet: true } });
  assert.equal((await handleAdmin(post('/api/admin/step', { ...step, linkUrl: 'http://x.example/' }), admin.ctx))!.status, 400);
  const saved = await handleAdmin(post('/api/admin/step', step), admin.ctx);
  assert.equal(saved!.status, 200);
  assert.equal(admin.calls.find(c => c.name === 'store_admin_save_step')!.args.p_step.skip_label, 'I already have OpenCPN installed');
});
