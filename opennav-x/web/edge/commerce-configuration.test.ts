import test from 'node:test';
import assert from 'node:assert/strict';
import { handleCommerce, reportCommerceConfiguration } from './commerce.ts';

const env = {
  STRIPE_TEST_CHECKOUT_ENABLED: 'true', STRIPE_TEST_SECRET_KEY: 'rk_test_SYNTHETIC123456789',
  STRIPE_TEST_WEBHOOK_SECRET: 'whsec_SYNTHETIC123456789', STRIPE_TEST_PRICE_ID: 'price_fixture123',
  STRIPE_TEST_RETURN_ORIGIN: 'https://beta.skager.app', STRIPE_TEST_ALLOWED_COUPON_IDS: 'fixture-coupon',
  STRIPE_TEST_ALLOWED_PROMOTION_IDS: 'promo_fixture123',
};
const sentinel = 'PRIVATE-SENTINEL-NEVER-LOG';

test('startup configuration diagnostics name only the first failed static binding and preserve public refusal', async () => {
  const invalid = {
    STRIPE_TEST_SECRET_KEY: sentinel, STRIPE_TEST_WEBHOOK_SECRET: sentinel,
    STRIPE_TEST_PRICE_ID: sentinel, STRIPE_TEST_RETURN_ORIGIN: sentinel,
    STRIPE_TEST_ALLOWED_COUPON_IDS: sentinel + '!', STRIPE_TEST_ALLOWED_PROMOTION_IDS: sentinel,
  };
  const original = console.warn;
  const warnings: unknown[][] = [];
  console.warn = (...args) => { warnings.push(args); };
  try {
    for (const [binding, value] of Object.entries(invalid)) {
      warnings.length = 0;
      const configured = { ...env, [binding]: value };
      reportCommerceConfiguration(configured);
      assert.deepEqual(warnings, [[`skager-commerce:configuration-v1:${binding}`]]);
      let effects = 0;
      const ctx = { env: configured,
        getAccount: async () => { effects++; throw Error(sentinel); },
        rpc: async () => { effects++; throw Error(sentinel); },
        fetch: async () => { effects++; throw Error(sentinel); },
      };
      for (let attempt = 0; attempt < 3; attempt++) {
        const response = await handleCommerce(new Request('https://beta.skager.app/api/checkout/start', {
          method: 'POST', headers: { Origin: env.STRIPE_TEST_RETURN_ORIGIN },
        }), ctx);
        assert.equal(response?.status, 404);
        assert.deepEqual(await response?.json(), { error: 'Unavailable' });
      }
      assert.equal(effects, 0);
      assert.deepEqual(warnings, [[`skager-commerce:configuration-v1:${binding}`]]);
      assert.equal(JSON.stringify(warnings).includes(sentinel), false);
    }
    warnings.length = 0;
    reportCommerceConfiguration({ ...env, ...invalid });
    assert.deepEqual(warnings, [['skager-commerce:configuration-v1:STRIPE_TEST_SECRET_KEY']]);
  } finally { console.warn = original; }
});

test('valid startup emits one static marker, intentional disabled states are silent, and diagnostic failures are contained', () => {
  const original = console.warn;
  const warnings: unknown[][] = [];
  console.warn = (...args) => { warnings.push(args); };
  try {
    reportCommerceConfiguration(env);
    assert.deepEqual(warnings, [['skager-commerce:configuration-v1:valid']]);
    warnings.length = 0;
    for (const value of [undefined, '', 'false', 'TRUE', sentinel]) {
      reportCommerceConfiguration({ ...env, STRIPE_TEST_CHECKOUT_ENABLED: value, STRIPE_TEST_SECRET_KEY: sentinel });
    }
    assert.deepEqual(warnings, []);
    const broken = { ...env };
    Object.defineProperty(broken, 'STRIPE_TEST_SECRET_KEY', { get() { throw Error(sentinel); } });
    reportCommerceConfiguration(broken);
    assert.deepEqual(warnings, [['skager-commerce:configuration-v1:unknown']]);
    assert.equal(JSON.stringify(warnings).includes(sentinel), false);
    console.warn = () => { throw Error(sentinel); };
    assert.doesNotThrow(() => reportCommerceConfiguration(env));
  } finally { console.warn = original; }
});
