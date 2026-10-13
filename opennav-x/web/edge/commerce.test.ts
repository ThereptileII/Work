import test from 'node:test';
import assert from 'node:assert/strict';
import { createHmac } from 'node:crypto';
import { confirmedCheckout, handleCommerce, strictJson, verifiedEvent, STRIPE_API_VERSION } from './commerce.ts';

const order = '11111111-1111-4111-8111-111111111111';
const sessionId = 'cs_test_12345678';
const priceId = 'price_12345678';
const intentId = 'pi_12345678';
const productId = 'prod_12345678';
const promotionId = 'promo_12345678';
const secret = 'whsec_1234567890abcdef';
const config = { price: priceId, coupons: ['full100'], promotions: ['promo_12345678'], key: 'sk_test_1234567890abcdef',
  secret, origin: 'https://beta.skager.app', selectedPromotion: promotionId };
function coupon(discount = 2000): Record<string, any> {
  return { object: 'coupon', id: 'full100', livemode: false, valid: true,
    percent_off: discount * 100 / 2000, amount_off: null, redeem_by: null,
    max_redemptions: 25, times_redeemed: 24, applies_to: { products: [productId] } };
}
function price(): Record<string, any> {
  return { object: 'price', id: priceId, active: true, product: productId, livemode: false,
    currency: 'usd', unit_amount: 2000, type: 'one_time', recurring: null };
}
function promotion(): Record<string, any> {
  return { object: 'promotion_code', id: promotionId, code: 'FREE100', active: true,
    livemode: false, customer: null, customer_account: null, expires_at: null, max_redemptions: 25,
    times_redeemed: 24, restrictions: { first_time_transaction: false, minimum_amount: null },
    promotion: { type: 'coupon', coupon: 'full100' } };
}
const providerReply = (value: unknown) => new Response(JSON.stringify(value), {
  status: 200, headers: { 'Content-Type': 'application/json' },
});
const raw = (value: unknown) => new TextEncoder().encode(JSON.stringify(value));

function session(discount = 0): Record<string, any> {
  const total = 2000 - discount;
  return { object: 'checkout.session', id: sessionId, client_reference_id: order,
    metadata: { order_id: order }, livemode: false, mode: 'payment', status: 'complete',
    ui_mode: 'hosted', currency: 'usd', amount_subtotal: 2000, amount_total: total,
    payment_status: total ? 'paid' : 'no_payment_required', payment_intent: total ? intentId : null,
    total_details: { amount_discount: discount, amount_tax: 0, amount_shipping: 0 },
    discounts: discount ? [{ promotion_code: promotionId, coupon: null }] : [],
    line_items: { object: 'list', has_more: false, data: [{ quantity: 1, currency: 'usd',
      amount_subtotal: 2000, amount_total: total, amount_discount: discount, amount_tax: 0,
      price: price() }] } };
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
  const result = confirmedCheckout(session(2000), config, order, sessionId, coupon());
  assert.equal(result.payment_status, 'no_payment_required');
  assert.equal(result.total_minor, 0);
  assert.equal(result.payment_intent_id, null);
  assert.deepEqual(result.coupon_ids, ['full100']);
  assert.throws(() => confirmedCheckout({ ...session(2000), payment_status: 'unpaid' }, config, order, sessionId, coupon()));
  assert.throws(() => confirmedCheckout({ ...session(2000), discounts: [] }, config, order, sessionId, coupon()));
  assert.throws(() => confirmedCheckout({ ...session(2000), payment_intent: intentId }, config, order, sessionId, coupon()));
  assert.throws(() => confirmedCheckout(session(2000), { ...config, coupons: [] }, order, sessionId, coupon()));
  assert.throws(() => confirmedCheckout(session(2000), config, order, sessionId));
  for (const attached of ['full100', coupon()]) {
    const direct = session(2000); direct.discounts[0].coupon = attached;
    assert.equal(confirmedCheckout(direct, config, order, sessionId, coupon()).total_minor, 0);
  }
  const wrongCoupon = session(2000); wrongCoupon.discounts[0].coupon = 'foreigncoupon';
  assert.throws(() => confirmedCheckout(wrongCoupon, config, order, sessionId, coupon()));
});

const runtimeEnv = {
  STRIPE_TEST_CHECKOUT_ENABLED: 'true', STRIPE_TEST_SECRET_KEY: config.key,
  STRIPE_TEST_WEBHOOK_SECRET: secret, STRIPE_TEST_PRICE_ID: priceId,
  STRIPE_TEST_RETURN_ORIGIN: config.origin, STRIPE_TEST_ALLOWED_COUPON_IDS: 'full100',
  STRIPE_TEST_ALLOWED_PROMOTION_IDS: promotionId,
};

function charge(): Record<string, any> {
  return { object: 'charge', id: 'ch_12345678', livemode: false, payment_intent: intentId,
    currency: 'usd', paid: true, amount: 2000, amount_refunded: 500 };
}
const ownedBinding = { state: 'bound', price_id: priceId, selected_coupon_id: null, promotion_id: null };
async function deliverWebhook(item: Record<string, any>, type = 'checkout.session.completed', options: any = {}) {
  const calls: Array<[string, any]> = [];
  const urls: string[] = [];
  const value = event(item, type);
  if (options.eventId) value.id = options.eventId;
  const { bytes, header } = signed(value);
  const response = await handleCommerce(new Request(config.origin + '/api/stripe/webhook', {
    method: 'POST', headers: { 'Stripe-Signature': options.signature ?? header }, body: bytes,
  }), {
    env: { ...runtimeEnv, ...options.env }, now: () => 1_700_000_000_000,
    getAccount: async () => { assert.fail('webhook ownership never comes from a browser account'); },
    rpc: async (name, args) => {
      calls.push([name, args]);
      if (options.rpc) return options.rpc(name, args);
      if (name === 'commerce_checkout_binding') {
        assert.equal(args.p_order_id, options.orderId ?? order);
        assert.equal(args.p_session_id, options.sessionId ?? sessionId);
        if (options.bindingError) throw Error('synthetic database uncertainty');
        return options.binding ?? ownedBinding;
      }
      if (name === 'commerce_receive_event') return { state: options.receiptState ?? 'received' };
      return { state: 'applied' };
    },
    fetch: async (input, init) => {
      const url = String(input); urls.push(url);
      assert.equal(init?.method, 'GET');
      assert.equal(init?.redirect, 'manual');
      assert.equal((init?.headers as Record<string, string>)['Stripe-Version'], STRIPE_API_VERSION);
      assert.equal((init?.headers as Record<string, string>).Authorization, `Bearer ${runtimeEnv.STRIPE_TEST_SECRET_KEY}`);
      if (options.provider) return options.provider(url);
      if (url === 'https://api.stripe.com/v1/charges/ch_12345678') return providerReply(options.charge ?? charge());
      if (url === `https://api.stripe.com/v1/checkout/sessions?payment_intent=${intentId}&limit=2`)
        return providerReply(options.list ?? { object: 'list', has_more: false, data: [session()] });
      if (url === `https://api.stripe.com/v1/checkout/sessions/${sessionId}?expand%5B%5D=line_items&expand%5B%5D=discounts.coupon`)
        return providerReply(options.session ?? session());
      assert.fail('unexpected provider URL');
    },
  });
  return { response, calls, urls, writes: calls.filter(([name]) => name !== 'commerce_checkout_binding') };
}

test('every foreign checkout event is ignored before ingestion, including ordinary unrelated product metadata', async () => {
  for (const type of ['checkout.session.completed', 'checkout.session.expired',
    'checkout.session.async_payment_succeeded', 'checkout.session.async_payment_failed']) {
    const foreign = { ...session(), client_reference_id: 'ember-reference', metadata: { product: 'ember' },
      currency: 'eur', amount_subtotal: 9900, amount_total: 9900 };
    const excluded = await deliverWebhook(foreign, type);
    assert.equal(excluded.response?.status, 200);
    assert.deepEqual(excluded.calls, []);
    assert.deepEqual(excluded.urls, []);
    const canonical = await deliverWebhook({ ...foreign, client_reference_id: order }, type,
      { binding: { state: 'rejected' } });
    assert.equal(canonical.response?.status, 200);
    assert.deepEqual(canonical.calls.map(([name]) => name), ['commerce_checkout_binding']);
    assert.deepEqual(canonical.writes, []);
    assert.deepEqual(canonical.urls, []);
  }
  const noReference = await deliverWebhook({ ...session(), client_reference_id: null, metadata: null });
  assert.equal(noReference.response?.status, 200);
  assert.deepEqual(noReference.calls, []);
  assert.deepEqual(noReference.urls, []);
});

test('spoofed SKAGER metadata or a different session never establishes ownership', async () => {
  const foreignId = 'cs_test_foreign123';
  const result = await deliverWebhook({ ...session(), id: foreignId }, 'checkout.session.completed', {
    sessionId: foreignId, binding: { state: 'rejected' },
  });
  assert.equal(result.response?.status, 200);
  assert.deepEqual(result.writes, []);
  assert.deepEqual(result.urls, []);
});

test('pending, unknown, malformed or failing binding reads retry all checkout types without a write', async () => {
  for (const type of ['checkout.session.completed', 'checkout.session.expired',
    'checkout.session.async_payment_succeeded', 'checkout.session.async_payment_failed']) {
    for (const options of [{ binding: { state: 'pending' } }, { binding: { state: 'unexpected' } },
      { binding: 'bound' }, { binding: { state: 'bound', price_id: '../untrusted' } }, { bindingError: true }]) {
      const result = await deliverWebhook(session(), type, options);
      assert.equal(result.response?.status, 503);
      assert.deepEqual(result.writes, []);
      assert.deepEqual(result.urls, []);
    }
  }
});

test('known owned sessions with altered SKAGER metadata or money retry instead of being discarded as foreign', async () => {
  for (const changed of [{ ...session(), metadata: {} }, { ...session(), metadata: null },
    { ...session(), metadata: { order_id: 'foreign' } },
    { ...session(), amount_subtotal: 3000 }, { ...session(), currency: 'eur' }, { ...session(), mode: 'subscription' }]) {
    const result = await deliverWebhook(changed);
    assert.equal(result.response?.status, 503);
    assert.deepEqual(result.calls.map(([name]) => name), ['commerce_checkout_binding']);
    assert.deepEqual(result.writes, []);
  }
});

test('owned completion provider uncertainty retries before any receipt; original bound price survives configuration rotation', async () => {
  for (const provider of [async () => { throw Error('synthetic provider uncertainty'); },
    async () => new Response('{}', { status: 503 }),
    async () => providerReply({ ...session(), id: 'cs_test_unexpected' })]) {
    const failed = await deliverWebhook(session(), 'checkout.session.completed', { provider });
    assert.equal(failed.response?.status, 503);
    assert.deepEqual(failed.writes, []);
  }
  const valid = await deliverWebhook(session(), 'checkout.session.completed', {
    env: { STRIPE_TEST_PRICE_ID: 'price_rotated123' },
  });
  assert.equal(valid.response?.status, 200);
  assert.deepEqual(valid.calls.map(([name]) => name),
    ['commerce_checkout_binding', 'commerce_receive_event', 'commerce_apply_checkout']);
  assert.equal(valid.writes[1][1].p_checkout.price_id, priceId);
});

test('owned expiry and asynchronous failure use the original close effect only after binding', async () => {
  for (const type of ['checkout.session.expired', 'checkout.session.async_payment_failed']) {
    const result = await deliverWebhook({ ...session(), status: 'expired', payment_status: 'unpaid' }, type);
    assert.equal(result.response?.status, 200);
    assert.deepEqual(result.calls.map(([name]) => name),
      ['commerce_checkout_binding', 'commerce_receive_event', 'commerce_close_checkout']);
    assert.deepEqual(result.urls, []);
  }
});

test('foreign refunds require authenticated charge and unambiguous session lookup but never create receipts or markers', async () => {
  for (const reference of [order, 'ember-reference']) {
    const foreign = { ...session(), client_reference_id: reference, metadata: { product: 'ember' },
      currency: 'eur', amount_subtotal: 9900, amount_total: 9900 };
    const foreignCharge = { ...charge(), currency: 'eur', amount: 9900 };
    const result = await deliverWebhook(foreignCharge, 'charge.refunded', {
      binding: { state: 'rejected' }, charge: foreignCharge,
      list: { object: 'list', has_more: false, data: [foreign] },
    });
    assert.equal(result.response?.status, 200);
    assert.equal(result.urls.length, 2);
    assert.deepEqual(result.writes, []);
  }
});

test('refund list races, ambiguity, provider mismatches and binding uncertainty leave no receipt or marker', async () => {
  for (const options of [
    { list: { object: 'list', has_more: false, data: [] } },
    { list: { object: 'list', has_more: false, data: [session(), session()] } },
    { list: { object: 'list', has_more: true, data: [session()] } },
    { list: { object: 'not_list', has_more: false, data: [session()] } },
    { list: { object: 'list', has_more: false, data: [{ ...session(), payment_intent: 'pi_foreign123' }] } },
    { list: { object: 'list', has_more: false, data: [{ ...session(), currency: 'eur' }] } },
    { list: { object: 'list', has_more: false, data: [{ ...session(), amount_total: 1999 }] } },
    { list: { object: 'list', has_more: false, data: [{ ...session(), livemode: true }] } },
    { list: { object: 'list', has_more: false, data: [{ ...session(), metadata: {} }] } },
    { charge: { ...charge(), payment_intent: 'pi_foreign123' } },
    { charge: { ...charge(), id: 'ch_foreign123' } },
    { charge: { ...charge(), livemode: true } },
    { binding: { state: 'pending' } }, { binding: { state: 'unknown' } }, { bindingError: true },
    { provider: async () => { throw Error('synthetic provider uncertainty'); } },
  ]) {
    const result = await deliverWebhook(charge(), 'charge.refunded', options);
    assert.equal(result.response?.status, 503);
    assert.deepEqual(result.writes, []);
  }
  const mismatch = await deliverWebhook(charge(), 'charge.refunded', {
    sessionId: 'cs_test_foreign123', binding: { state: 'rejected' },
    list: { object: 'list', has_more: false, data: [{ ...session(), id: 'cs_test_foreign123' }] },
  });
  assert.equal(mismatch.response?.status, 200);
  assert.deepEqual(mismatch.writes, []);
});

test('owned refund before completion invokes unchanged atomic effects and duplicate receipts do not reapply', async () => {
  const seen = new Set<string>();
  const effects: string[] = [];
  const rpc = async (name: string, args: any) => {
    if (name === 'commerce_checkout_binding') return ownedBinding;
    if (name === 'commerce_receive_event') {
      const duplicate = seen.has(args.p_event_id); seen.add(args.p_event_id);
      return { state: duplicate ? 'applied' : 'received' };
    }
    effects.push(name); return { state: 'applied' };
  };
  const refund = await deliverWebhook(charge(), 'charge.refunded', { rpc });
  assert.equal(refund.response?.status, 200);
  assert.deepEqual(refund.calls.map(([name]) => name),
    ['commerce_checkout_binding', 'commerce_receive_event', 'commerce_apply_refund']);
  const completed = await deliverWebhook(session(), 'checkout.session.completed', { rpc, eventId: 'evt_completed123' });
  assert.equal(completed.response?.status, 200);
  assert.deepEqual(effects, ['commerce_apply_refund', 'commerce_apply_checkout']);
  const duplicate = await deliverWebhook(charge(), 'charge.refunded', { rpc });
  assert.equal(duplicate.response?.status, 200);
  assert.deepEqual(effects, ['commerce_apply_refund', 'commerce_apply_checkout']);
  const race = await deliverWebhook(charge(), 'charge.refunded', { list: { object: 'list', has_more: false, data: [] } });
  assert.equal(race.response?.status, 503);
  assert.deepEqual(race.writes, []);
  const retried = await deliverWebhook(charge(), 'charge.refunded');
  assert.equal(retried.response?.status, 200);
  assert.deepEqual(retried.writes.map(([name]) => name), ['commerce_receive_event', 'commerce_apply_refund']);
});

test('malformed or unauthenticated shared-sandbox events still refuse before provider or database access', async () => {
  for (const invalid of [{ ...session(), livemode: true }, { ...session(), amount_total: -1 },
    { ...session(), metadata: 'not an object' }, { ...session(), status: 'unknown' }]) {
    const result = await deliverWebhook(invalid);
    assert.equal(result.response?.status, 400);
    assert.deepEqual(result.calls, []);
    assert.deepEqual(result.urls, []);
  }
  const unsigned = await deliverWebhook(session(), 'checkout.session.completed', { signature: 'invalid' });
  assert.equal(unsigned.response?.status, 400);
  assert.deepEqual(unsigned.calls, []);
  assert.deepEqual(unsigned.urls, []);
});

test('restricted TEST keys work while live, publishable, missing and malformed keys refuse before effects', async () => {
  for (const key of ['rk_test_1234567890abcdef', config.key, 'rk_live_1234567890abcdef',
    'sk_live_1234567890abcdef', 'pk_test_1234567890abcdef', '', 'rk_test_short', 'rk_test_1234567890abcdef\n']) {
    let effects = 0;
    const ctx = {
      env: { ...runtimeEnv, STRIPE_TEST_SECRET_KEY: key },
      getAccount: async () => { effects++; return { id: order }; },
      rpc: async () => { effects++; return { order_id: order }; },
      fetch: async (_url: string, init: RequestInit) => {
        effects++;
        assert.equal((init.headers as Record<string, string>).Authorization, `Bearer ${key}`);
        assert.equal((init.headers as Record<string, string>)['Stripe-Version'], STRIPE_API_VERSION);
        return providerReply({ ...session(), status: 'open', url: `https://checkout.stripe.com/c/pay/${sessionId}` });
      },
    };
    const response = await handleCommerce(new Request(config.origin + '/api/checkout/start', {
      method: 'POST', headers: { Origin: config.origin },
    }), ctx);
    const valid = key === config.key || key === 'rk_test_1234567890abcdef';
    assert.equal(response?.status, valid ? 200 : 404);
    assert.equal(effects, valid ? 4 : 0);
  }
});

test('last permitted coupon redemption settles after expiry, exhaustion and archive for paid and zero totals', async () => {
  for (const discount of [500, 2000]) {
    let exhausted = false;
    const calls: Array<[string, any]> = [];
    const ctx = {
      env: runtimeEnv, now: () => 1_700_000_000_000, getAccount: async () => ({ id: order }),
      rpc: async (name: string, args: any) => {
        calls.push([name, args]);
        if (name === 'commerce_reserve_checkout') return { order_id: order };
        if (name === 'commerce_checkout_binding') return { state: 'bound', price_id: priceId,
          selected_coupon_id: 'full100', promotion_id: promotionId };
        return { state: name === 'commerce_receive_event' ? 'received' : 'applied' };
      },
      fetch: async (url: string, init: RequestInit) => {
        assert.equal((init.headers as Record<string, string>)['Stripe-Version'], STRIPE_API_VERSION);
        if (url.includes('/promotion_codes?')) return providerReply({ object: 'list', has_more: false, data: [promotion()] });
        if (url.endsWith(`/promotion_codes/${promotionId}`)) return providerReply({ ...promotion(),
          active: false, expires_at: 1, times_redeemed: 25 });
        if (url.endsWith(`/prices/${priceId}`)) return providerReply({ ...price(), active: !exhausted });
        if (url.endsWith('/coupons/full100?expand%5B%5D=applies_to')) return providerReply({ ...coupon(discount),
          valid: !exhausted, redeem_by: exhausted ? 1 : null, times_redeemed: exhausted ? 25 : 24 });
        if (url.endsWith('/checkout/sessions')) {
          assert.equal((init.body as URLSearchParams).has('allow_promotion_codes'), false);
          assert.equal((init.body as URLSearchParams).get('discounts[0][promotion_code]'), promotionId);
          return providerReply({ ...session(discount), status: 'open', url: `https://checkout.stripe.com/c/pay/${sessionId}` });
        }
        if (url.includes(`/checkout/sessions/${sessionId}?`)) return providerReply(session(discount));
        throw Error('unexpected provider request');
      },
    };
    const start = await handleCommerce(new Request(config.origin + '/api/checkout/start', {
      method: 'POST', headers: { Origin: config.origin, 'Content-Type': 'application/json' },
      body: JSON.stringify({ discountCode: 'FREE100' }),
    }), ctx);
    assert.equal(start?.status, 200);
    exhausted = true;
    const { bytes, header } = signed(event(session(discount)));
    const settled = await handleCommerce(new Request(config.origin + '/api/stripe/webhook', {
      method: 'POST', headers: { 'Stripe-Signature': header }, body: bytes,
    }), ctx);
    assert.equal(settled?.status, 200);
    const applied = calls.filter(([name]) => name === 'commerce_apply_checkout');
    assert.equal(applied.length, 1);
    assert.equal(applied[0][1].p_checkout.total_minor, 2000 - discount);
  }
});

test('settlement refuses unauthenticated coupon-product facts or changed provider promotion binding', async () => {
  for (const override of [
    { coupon: { ...coupon(), applies_to: undefined } },
    { coupon: { ...coupon(), applies_to: { products: ['prod_unrelated'] } } },
    { coupon: { ...coupon(), applies_to: { products: [productId, 'prod_unrelated'] } } },
    { coupon: { ...coupon(), livemode: true } },
    { coupon: { ...coupon(), id: 'foreigncoupon' } },
    { coupon: coupon(500) },
    { promotion: { ...promotion(), promotion: { type: 'coupon', coupon: 'foreigncoupon' } } },
    { promotion: { ...promotion(), livemode: true } },
    { promotion: { ...promotion(), promotion: undefined, coupon: coupon() } },
    { price: { ...price(), product: 'prod_unrelated' } },
  ]) {
    const calls: string[] = [];
    const { bytes, header } = signed(event(session(2000)));
    const response = await handleCommerce(new Request(config.origin + '/api/stripe/webhook', {
      method: 'POST', headers: { 'Stripe-Signature': header }, body: bytes,
    }), {
      env: runtimeEnv, now: () => 1_700_000_000_000, getAccount: async () => null,
      rpc: async name => {
        calls.push(name);
        if (name === 'commerce_receive_event') return { state: 'received' };
        if (name === 'commerce_checkout_binding') return { state: 'bound', price_id: priceId,
          selected_coupon_id: 'full100', promotion_id: promotionId };
        assert.fail('unverified provider evidence must not apply effects');
      },
      fetch: async url => {
        const target = String(url);
        if (target.includes('/checkout/sessions/')) return providerReply(session(2000));
        if (target.endsWith(`/promotion_codes/${promotionId}`)) return providerReply(override.promotion ?? promotion());
        if (target.endsWith(`/prices/${priceId}`)) return providerReply(override.price ?? price());
        if (target.endsWith('/coupons/full100?expand%5B%5D=applies_to')) return providerReply(override.coupon ?? coupon());
        assert.fail('unexpected provider request');
      },
    });
    assert.equal(response?.status, 503);
    assert.deepEqual(calls, ['commerce_checkout_binding']);
  }
});

test('paid checkout requires the bound price, exact total, and provider payment', () => {
  assert.equal(confirmedCheckout(session(), config, order, sessionId, coupon()).total_minor, 2000);
  assert.equal(confirmedCheckout(session(500), { ...config, selectedPromotion: 'promo_12345678' },
    order, sessionId, coupon(500)).total_minor, 1500);
  const wrongPromotion = session(500); wrongPromotion.discounts[0].promotion_code = 'promo_othercode';
  assert.throws(() => confirmedCheckout(wrongPromotion,
    { ...config, selectedPromotion: promotionId }, order, sessionId, coupon(500)));
  assert.throws(() => confirmedCheckout({ ...session(), amount_total: 1999 }, config, order, sessionId, coupon()));
  const wrongPrice = session(); wrongPrice.line_items.data[0].price.id = 'price_elsewhere';
  assert.throws(() => confirmedCheckout(wrongPrice, config, order, sessionId, coupon()));
  assert.throws(() => confirmedCheckout({ ...session(), payment_status: 'no_payment_required' }, config, order, sessionId, coupon()));
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
      STRIPE_TEST_RETURN_ORIGIN: config.origin, STRIPE_TEST_ALLOWED_COUPON_IDS: 'full100',
      STRIPE_TEST_ALLOWED_PROMOTION_IDS: 'promo_12345678' },
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
      assert.equal((init.body as URLSearchParams).get('allow_promotion_codes'), 'false');
      assert.equal((init.body as URLSearchParams).has('discounts[0][promotion_code]'), false);
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

test('only an approved active exact promotion reaches Checkout creation', async () => {
  const approved = promotion();
  const base = { env: { STRIPE_TEST_CHECKOUT_ENABLED: 'true', STRIPE_TEST_SECRET_KEY: config.key,
    STRIPE_TEST_WEBHOOK_SECRET: secret, STRIPE_TEST_PRICE_ID: priceId,
    STRIPE_TEST_RETURN_ORIGIN: config.origin, STRIPE_TEST_ALLOWED_COUPON_IDS: 'full100',
    STRIPE_TEST_ALLOWED_PROMOTION_IDS: 'promo_12345678' }, getAccount: async () => ({ id: order }) };
  async function start(code: string, candidates: any[], selectedCoupon = coupon(), selectedPrice = price()) {
    const visited: string[] = []; const calls: Array<[string, any]> = [];
    const ctx = { ...base, rpc: async (name: string, args: any) => {
      calls.push([name, args]);
      if (name === 'commerce_reserve_checkout') return { order_id: order };
      if (name === 'commerce_bind_checkout') return { state: 'bound' };
      throw Error(name);
    }, fetch: async (url: string, init: RequestInit) => {
      visited.push(url);
      if (url.includes('/promotion_codes?')) return new Response(JSON.stringify({ object: 'list',
        has_more: false, data: candidates }), { status: 200, headers: { 'Content-Type': 'application/json' } });
      if (url === `https://api.stripe.com/v1/prices/${priceId}`) return providerReply(selectedPrice);
      if (url === 'https://api.stripe.com/v1/coupons/full100?expand%5B%5D=applies_to') return providerReply(selectedCoupon);
      assert.equal((init.body as URLSearchParams).has('allow_promotion_codes'), false);
      assert.equal((init.body as URLSearchParams).get('discounts[0][promotion_code]'), 'promo_12345678');
      return new Response(JSON.stringify({ ...session(2000), status: 'open',
        url: `https://checkout.stripe.com/c/pay/${sessionId}#test` }),
        { status: 200, headers: { 'Content-Type': 'application/json' } });
    } };
    const response = await handleCommerce(new Request('https://beta.skager.app/api/checkout/start', {
      method: 'POST', headers: { Origin: config.origin, 'Content-Type': 'application/json' },
      body: JSON.stringify({ discountCode: code }) }), ctx);
    return { response, visited, calls };
  }
  for (const [code, candidates] of [
    ['NOPE', []], ['FREE100', [{ ...approved, active: false }]],
    ['FREE100', [{ ...approved, id: 'promo_unapproved' }]],
    ['FREE100', [{ ...approved, expires_at: 1 }]],
    ['FREE100', [{ ...approved, promotion: { type: 'coupon', coupon: 'unapproved' } }]],
    ['FREE100', [{ ...approved, promotion: undefined, coupon: coupon() }]],
    ['FREE100', [{ ...approved, promotion: { type: 'other', coupon: 'full100' } }]],
    ['FREE100', [{ ...approved, customer_account: 'acct_private' }]],
    ['FREE100', [{ ...approved, times_redeemed: 25 }]],
    ['FREE100', [approved, approved]]
  ] as [string, any[]][]) {
    const result = await start(code, candidates);
    assert.equal(result.response?.status, 400);
    assert.equal(result.visited.length, 1);
    assert.equal(result.calls.length, 0);
  }
  const valid = await start('FREE100', [approved]);
  assert.equal(valid.response?.status, 200);
  assert.equal(valid.visited.length, 4);
  assert.equal(valid.calls[0][1].p_promotion_id, 'promo_12345678');
  assert.equal(valid.calls[0][1].p_selected_coupon_id, 'full100');
  for (const invalid of [
    { ...coupon(), valid: false }, { ...coupon(), livemode: true },
    { ...coupon(), id: 'anothercoupon' }, { ...coupon(), times_redeemed: 25 },
    { ...coupon(), redeem_by: 1 }, { ...coupon(), applies_to: undefined },
    { ...coupon(), applies_to: null }, { ...coupon(), applies_to: { products: [] } },
    { ...coupon(), applies_to: { products: ['prod_unrelated'] } },
    { ...coupon(), applies_to: { products: [productId, 'prod_unrelated'] } },
  ]) {
    const denied = await start('FREE100', [approved], invalid);
    assert.equal(denied.response?.status, 400);
    assert.equal(denied.calls.length, 0);
    assert.equal(denied.visited.length, 3);
  }
  for (const invalid of [
    { ...price(), id: 'price_unrelated' }, { ...price(), livemode: true },
    { ...price(), active: false }, { ...price(), product: null },
    { ...price(), unit_amount: 1999 }, { ...price(), currency: 'eur' },
  ]) {
    const denied = await start('FREE100', [approved], coupon(), invalid);
    assert.equal(denied.response?.status, 400);
    assert.equal(denied.calls.length, 0);
    assert.equal(denied.visited.length, 2);
  }
  const malformed = await start('FREE 100', [approved]);
  assert.equal(malformed.response?.status, 400);
  assert.equal(malformed.visited.length, 0);
});

test('signed zero-total completion reaches one verified apply call', async () => {
  const calls: Array<[string, any]> = [];
  const { bytes, header } = signed(event(session(2000)));
  const ctx = {
    env: { STRIPE_TEST_CHECKOUT_ENABLED: 'true', STRIPE_TEST_SECRET_KEY: config.key,
      STRIPE_TEST_WEBHOOK_SECRET: secret, STRIPE_TEST_PRICE_ID: priceId,
      STRIPE_TEST_RETURN_ORIGIN: config.origin, STRIPE_TEST_ALLOWED_COUPON_IDS: 'full100',
      STRIPE_TEST_ALLOWED_PROMOTION_IDS: 'promo_12345678' },
    getAccount: async () => null,
    now: () => 1_700_000_000_000,
    rpc: async (name: string, args: any) => {
      calls.push([name, args]);
      if (name === 'commerce_receive_event') return { state: 'received' };
      if (name === 'commerce_checkout_binding') return { state: 'bound', price_id: priceId,
        selected_coupon_id: 'full100', promotion_id: 'promo_12345678' };
      if (name === 'commerce_apply_checkout') return { state: 'applied' };
      throw Error(name);
    },
    fetch: async (url: string) => {
      if (url.includes('/checkout/sessions/')) return providerReply(session(2000));
      if (url.endsWith(`/promotion_codes/${promotionId}`)) return providerReply(promotion());
      if (url.endsWith(`/prices/${priceId}`)) return providerReply(price());
      if (url.endsWith('/coupons/full100?expand%5B%5D=applies_to')) return providerReply(coupon());
      throw Error('unexpected provider request');
    }
  };
  const response = await handleCommerce(new Request('https://beta.skager.app/api/stripe/webhook',
    { method: 'POST', headers: { 'Stripe-Signature': header }, body: bytes }), ctx);
  assert.equal(response?.status, 200);
  assert.deepEqual(calls.map(([name]) => name),
    ['commerce_checkout_binding', 'commerce_receive_event', 'commerce_apply_checkout']);
  assert.equal(calls[2][1].p_checkout.total_minor, 0);
});
