import test from 'node:test';
import assert from 'node:assert/strict';
import { createHmac } from 'node:crypto';
import { confirmedCheckout, handleCommerce, strictJson, verifiedEvent, STRIPE_API_VERSION } from './commerce.ts';

const order = '11111111-1111-4111-8111-111111111111';
const sessionId = 'cs_test_12345678';
const priceId = 'price_12345678';
const intentId = 'pi_12345678';
const secret = 'whsec_1234567890abcdef';
const config = { price: priceId, coupons: ['full100'], key: 'sk_test_1234567890abcdef',
  secret, origin: 'https://beta.skager.app' };
const raw = (value: unknown) => new TextEncoder().encode(JSON.stringify(value));

function session(discount = 0): Record<string, any> {
  const total = 2000 - discount;
  return { object: 'checkout.session', id: sessionId, client_reference_id: order,
    metadata: { order_id: order }, livemode: false, mode: 'payment', status: 'complete',
    ui_mode: 'hosted', currency: 'usd', amount_subtotal: 2000, amount_total: total,
    payment_status: total ? 'paid' : 'no_payment_required', payment_intent: total ? intentId : null,
    total_details: { amount_discount: discount, amount_tax: 0, amount_shipping: 0 },
    discounts: discount ? [{ coupon: { object: 'coupon', id: 'full100', livemode: false,
      percent_off: discount * 100 / 2000, amount_off: null } }] : [],
    line_items: { object: 'list', has_more: false, data: [{ quantity: 1, currency: 'usd',
      amount_subtotal: 2000, amount_total: total, amount_discount: discount, amount_tax: 0,
      price: { object: 'price', id: priceId, livemode: false, currency: 'usd', unit_amount: 2000,
        type: 'one_time', recurring: null } }] } };
}

function event(object: object, type = 'checkout.session.completed') {
  return { object: 'event', id: 'evt_12345678', livemode: false, api_version: STRIPE_API_VERSION,
    created: 1_700_000_000, type, data: { object } };
}
function signed(value: object, timestamp = 1_700_000_000) {
  const bytes = raw(value);
  const signature = createHmac('sha256', secret).update(`${timestamp}.`).update(bytes).digest('hex');
  return { bytes, header: `t=${timestamp},v1=${signature}` };
}

test('strict JSON refuses duplicate keys and unsafe integers', () => {
  assert.throws(() => strictJson(new TextEncoder().encode('{"id":1,"id":2}')));
  assert.throws(() => strictJson(new TextEncoder().encode('{"id":9007199254740992}')));
  assert.deepEqual(strictJson(raw({ id: 1 })), { id: 1 });
});

test('confirmed full coupon gives active-equivalent zero payment evidence', () => {
  const result = confirmedCheckout(session(2000), config, order, sessionId);
  assert.equal(result.payment_status, 'no_payment_required');
  assert.equal(result.total_minor, 0);
  assert.equal(result.payment_intent_id, null);
  assert.deepEqual(result.coupon_ids, ['full100']);
  assert.throws(() => confirmedCheckout({ ...session(2000), payment_status: 'unpaid' }, config, order, sessionId));
  assert.throws(() => confirmedCheckout({ ...session(2000), discounts: [] }, config, order, sessionId));
  assert.throws(() => confirmedCheckout({ ...session(2000), payment_intent: intentId }, config, order, sessionId));
  assert.throws(() => confirmedCheckout(session(2000), { ...config, coupons: [] }, order, sessionId));
});

test('paid checkout requires the bound price, exact total, and provider payment', () => {
  assert.equal(confirmedCheckout(session(), config, order, sessionId).total_minor, 2000);
  assert.throws(() => confirmedCheckout({ ...session(), amount_total: 1999 }, config, order, sessionId));
  const wrongPrice = session(); wrongPrice.line_items.data[0].price.id = 'price_elsewhere';
  assert.throws(() => confirmedCheckout(wrongPrice, config, order, sessionId));
  assert.throws(() => confirmedCheckout({ ...session(), payment_status: 'no_payment_required' }, config, order, sessionId));
});

test('webhook signature binds exact raw body and rejects duplicate-key JSON', async () => {
  const { bytes, header } = signed(event(session()));
  assert.equal((await verifiedEvent(bytes, header, secret, 1_700_000_000)).event_id, 'evt_12345678');
  await assert.rejects(verifiedEvent(bytes, header, secret, 1_700_000_301));
  await assert.rejects(verifiedEvent(raw({ changed: true }), header, secret, 1_700_000_000));
  const duplicate = new TextEncoder().encode('{"object":"event","object":"event"}');
  const duplicateSignature = createHmac('sha256', secret).update('1700000000.').update(duplicate).digest('hex');
  await assert.rejects(verifiedEvent(duplicate, `t=1700000000,v1=${duplicateSignature}`, secret, 1_700_000_000));
});

test('checkout start binds authenticated account and returns only hosted URL', async () => {
  const calls: Array<[string, any]> = [];
  const ctx = {
    env: { STRIPE_TEST_CHECKOUT_ENABLED: 'true', STRIPE_TEST_SECRET_KEY: config.key,
      STRIPE_TEST_WEBHOOK_SECRET: secret, STRIPE_TEST_PRICE_ID: priceId,
      STRIPE_TEST_RETURN_ORIGIN: config.origin, STRIPE_TEST_ALLOWED_COUPON_IDS: 'full100' },
    getAccount: async () => ({ id: order }),
    rpc: async (name: string, args: any) => {
      calls.push([name, args]);
      if (name === 'commerce_reserve_checkout') return { order_id: order };
      if (name === 'commerce_bind_checkout') return { state: 'bound' };
      throw Error(name);
    },
    fetch: async (url: string, init: RequestInit) => {
      assert.equal(url, 'https://api.stripe.com/v1/checkout/sessions');
      assert.equal((init.body as URLSearchParams).get('client_reference_id'), order);
      return new Response(JSON.stringify({ ...session(), status: 'open',
        url: `https://checkout.stripe.com/c/pay/${sessionId}#test` }),
        { status: 200, headers: { 'Content-Type': 'application/json' } });
    }
  };
  const request = new Request('https://beta.skager.app/api/checkout/start',
    { method: 'POST', headers: { Origin: config.origin } });
  const response = await handleCommerce(request, ctx);
  assert.equal(response?.status, 200);
  assert.deepEqual(await response?.json(), { url: `https://checkout.stripe.com/c/pay/${sessionId}#test` });
  assert.deepEqual(calls.map(([name]) => name), ['commerce_reserve_checkout', 'commerce_bind_checkout']);
  const denied = await handleCommerce(new Request(request.url, { method: 'POST', headers: { Origin: 'https://evil.test' } }), ctx);
  assert.equal(denied?.status, 403);
});

test('signed zero-total completion reaches one verified apply call', async () => {
  const calls: Array<[string, any]> = [];
  const { bytes, header } = signed(event(session(2000)));
  const ctx = {
    env: { STRIPE_TEST_CHECKOUT_ENABLED: 'true', STRIPE_TEST_SECRET_KEY: config.key,
      STRIPE_TEST_WEBHOOK_SECRET: secret, STRIPE_TEST_PRICE_ID: priceId,
      STRIPE_TEST_RETURN_ORIGIN: config.origin, STRIPE_TEST_ALLOWED_COUPON_IDS: 'full100' },
    getAccount: async () => null,
    now: () => 1_700_000_000_000,
    rpc: async (name: string, args: any) => {
      calls.push([name, args]);
      if (name === 'commerce_receive_event') return { state: 'received' };
      if (name === 'commerce_checkout_binding') return { state: 'bound', price_id: priceId, allowed_coupon_ids: ['full100'] };
      if (name === 'commerce_apply_checkout') return { state: 'applied' };
      throw Error(name);
    },
    fetch: async () => new Response(JSON.stringify(session(2000)),
      { status: 200, headers: { 'Content-Type': 'application/json' } })
  };
  const response = await handleCommerce(new Request('https://beta.skager.app/api/stripe/webhook',
    { method: 'POST', headers: { 'Stripe-Signature': header }, body: bytes }), ctx);
  assert.equal(response?.status, 200);
  assert.deepEqual(calls.map(([name]) => name),
    ['commerce_receive_event', 'commerce_checkout_binding', 'commerce_apply_checkout']);
  assert.equal(calls[2][1].p_checkout.total_minor, 0);
});
