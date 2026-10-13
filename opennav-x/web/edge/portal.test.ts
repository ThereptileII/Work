import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { handlePortal, type PortalContext } from "./portal.ts";
import { escapeHTML, orderTotal, renderDashboard } from "./static/portal.js";

const MINE = "11111111-1111-4111-8111-111111111111";
const OTHER = "22222222-2222-4222-8222-222222222222";
const BOAT = "33333333-3333-4333-8333-333333333333";
const ORDER = "44444444-4444-4444-8444-444444444444";
const created = "2026-10-13T10:00:00Z";
function request(path = "/api/portal/dashboard", body?: unknown) {
  return new Request(`https://beta.skager.app${path}`, body === undefined ? {} : {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  });
}
function context(overrides: Partial<PortalContext> = {}): PortalContext {
  return {
    env: { PORTAL_ENABLED: "true", STRIPE_TEST_ENVIRONMENT: "staging" },
    getAccount: async () => ({ id: MINE, email: "sailor@example.test" }),
    rpc: async name => name === "commerce_portal_access" ? { entitlement: null, orders: [] } : [],
    ...overrides,
  };
}
async function result(ctx = context(), req = request()) {
  const response = await handlePortal(req, ctx);
  assert.ok(response);
  return { response, data: await response.json() };
}
test("anonymous/invalid session never queries private data and returns 401", async () => {
  for (const account of [null, { id: "untrusted-account" }]) {
    const { response } = await result(context({ getAccount: async () => account, rpc: async () => assert.fail("RPC must not run") }));
    assert.equal(response.status, 401);
    assert.match(response.headers.get("Cache-Control")!, /private, no-store/);
  }
});
test("portal is default-off, staging-only and rejects unsupported methods", async () => {
  for (const env of [{}, { PORTAL_ENABLED: "true", STRIPE_TEST_ENVIRONMENT: "production" }, { PORTAL_ENABLED: "true", STRIPE_TEST_ENVIRONMENT: "development" }]) {
    assert.equal((await result(context({ env }))).response.status, 404);
  }
  assert.equal((await result(context(), request("/api/portal/dashboard", {}))).response.status, 405);
  assert.equal(await handlePortal(request("/unrelated"), context()), null);
});
test("owner comes from verified session, never account/owner query parameters", async () => {
  const calls: string[] = [];
  const { data } = await result(context({ rpc: async (name, args) => {
    assert.deepEqual(args, { p_account_id: MINE }); calls.push(name);
    return name === "commerce_portal_access" ? { entitlement: null, orders: [] }
      : [{ id: BOAT, name: "My boat", created_at: created, account_id: OTHER, secret: "not returned" }];
  } }), request(`/api/portal/dashboard?account_id=${OTHER}&owner=${OTHER}`));
  assert.deepEqual(calls.sort(), ["commerce_portal_access", "portal_list_boats"]);
  assert.deepEqual(data.boats, [{ id: BOAT, name: "My boat", createdAt: created }]);
  assert.equal(JSON.stringify(data).includes(OTHER), false);
  assert.equal(JSON.stringify(data).includes("not returned"), false);
});
test("paid and zero-total fulfillment grant identical active access; output sanitizes provider IDs", async () => {
  for (const amount of [2000, 0]) {
    const { data } = await result(context({ rpc: async name => name === "commerce_portal_access" ? {
      entitlement: { status: "active", environment: "staging", product_code: "skager-beta", provider_id: "secret-entitlement" },
      orders: [{ id: ORDER, created_at: created, status: "fulfilled", amount_minor: amount, currency: "USD", checkout_id: "cs_secret" }],
    } : [] }));
    assert.equal(data.betaAccess, "active");
    assert.equal(data.orders[0].fullyDiscounted, amount === 0);
    assert.equal(Object.hasOwn(data, "downloads"), false, "release availability comes from its independently authorized endpoint");
    assert.equal(JSON.stringify(data).includes("secret"), false);
  }
});
test("refunded/suspended, other product/environment and orders alone grant no access", async () => {
  for (const entitlement of [null,
    { status: "refunded", environment: "staging", product_code: "skager-beta" },
    { status: "suspended", environment: "staging", product_code: "skager-beta" },
    { status: "active", environment: "production", product_code: "skager-beta" },
    { status: "active", environment: "staging", product_code: "other" },
  ]) {
    const { data } = await result(context({ rpc: async name => name === "commerce_portal_access" ? {
      entitlement, orders: [{ id: ORDER, created_at: created, status: "fulfilled", amount_minor: 2000, currency: "USD" }],
    } : [] }));
    assert.equal(data.betaAccess, "inactive");
  }
});
test("suspended orders remain visible without granting access", async () => {
  const { data } = await result(context({ rpc: async name => name === "commerce_portal_access" ? {
    entitlement: { status: "suspended", environment: "staging", product_code: "skager-beta" },
    orders: [{ id: ORDER, created_at: created, status: "suspended", amount_minor: 2000, currency: "USD" }],
  } : [] }));
  assert.equal(data.betaAccess, "inactive");
  assert.equal(data.orders[0].status, "suspended");
  assert.match(renderDashboard(data, "orders"), /Suspended/);
});
test("boat creation trims name and sends only session owner to the service RPC", async () => {
  const { response, data } = await result(context({ rpc: async (name, args) => {
    assert.equal(name, "portal_create_boat");
    assert.deepEqual(args, { p_account_id: MINE, p_name: "S/Y Aurora" });
    return { id: BOAT, name: args.p_name, created_at: created };
  } }), request("/api/portal/boats", { name: "  S/Y Aurora  " }));
  assert.equal(response.status, 201);
  assert.equal(data.boat.name, "S/Y Aurora");
});
test("boat API rejects forged ownership, blank/overlong/control names and invalid bodies", async () => {
  for (const body of [{ name: "Mine", account_id: OTHER }, { name: "Mine", owner: OTHER },
    { name: "" }, { name: "  " }, { name: "a".repeat(81) }, { name: "a\nb" }, { name: 42 }, [], null]) {
    const { response } = await result(context({ rpc: async () => assert.fail("No write for invalid input") }), request("/api/portal/boats", body));
    assert.equal(response.status, 400);
  }
  const huge = request("/api/portal/boats", { name: "a".repeat(5000) });
  assert.equal((await result(context(), huge)).response.status, 413);
  const invalid = new Request("https://beta.skager.app/api/portal/boats", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{" });
  assert.equal((await result(context(), invalid)).response.status, 400);
});
test("GET never creates a boat; private failure details are not exposed", async () => {
  const { data } = await result(context({ rpc: async (name, args) => {
    assert.equal(name, "portal_list_boats"); assert.deepEqual(args, { p_account_id: MINE }); return [];
  } }), request("/api/portal/boats?name=Do-not-create"));
  assert.deepEqual(data, { boats: [] });
  const failure = await result(context({ rpc: async () => { throw new Error("service_role_secret private database SQL"); } }));
  assert.equal(failure.response.status, 503);
  assert.equal(JSON.stringify(failure.data).includes("secret"), false);
});
test("empty customer pages render honestly; remote account/boat/order text is escaped", () => {
  const data = { betaAccess: "inactive", account: { email: '<img src=x onerror="alert(1)">' }, boats: [], orders: [], checkoutAvailable: false };
  for (const page of ["dashboard", "account", "boats", "orders", "downloads"]) {
    const html = renderDashboard(data, page);
    assert.ok(html.length > 100);
    assert.equal(html.includes('<img src=x'), false);
  }
  const html = renderDashboard({ ...data, boats: [{ id: BOAT, name: "<script>alert(1)</script>", createdAt: created }] }, "boats");
  assert.match(html, /&lt;script&gt;/);
  assert.equal(html.includes("<script>"), false);
  assert.equal(renderDashboard(data).includes('id="checkout"'), false);
  assert.match(renderDashboard({ ...data, checkoutAvailable: true }), /Continue to test checkout/);
  assert.match(renderDashboard(data, "downloads"), /Release delivery is closed/);
  assert.equal(escapeHTML("&<>'\""), "&amp;&lt;&gt;&#39;&quot;");
});
test("full discount states no money collected; paid totals have correct test label", () => {
  assert.match(orderTotal({ fullyDiscounted: true, amountMinor: 0, currency: "USD" }), /no payment collected/);
  assert.match(orderTotal({ fullyDiscounted: false, amountMinor: 2000, currency: "USD" }), /20.00 USD/);
  assert.match(orderTotal({ fullyDiscounted: false, amountMinor: 2000, currency: "JPY" }), /Total recorded in JPY/);
});
test("static shells contain no customer data or inline execution; SQL grants no browser access", async () => {
  const sql = await readFile(new URL("./portal.sql", import.meta.url), "utf8");
  assert.match(sql, /CREATE SCHEMA IF NOT EXISTS skager_private/);
  assert.match(sql, /CREATE TABLE IF NOT EXISTS skager_private\.portal_boats/);
  assert.match(sql, /REVOKE ALL ON SCHEMA skager_private FROM PUBLIC, anon, authenticated/);
  assert.match(sql, /REVOKE ALL ON skager_private\.portal_boats FROM PUBLIC, anon, authenticated, service_role/);
  assert.match(sql, /WHERE b.account_id = p_account_id/);
  assert.match(sql, /SET search_path = pg_catalog/g);
  assert.match(sql, /ENABLE ROW LEVEL SECURITY/);
  assert.match(sql, /TO service_role/);
  for (const file of ["portal.html", "auth.html"]) {
    const html = await readFile(new URL(`./static/${file}`, import.meta.url), "utf8");
    assert.equal(html.includes(MINE), false);
    assert.equal(html.includes("sailor@example.test"), false);
    assert.equal(/<script(?![^>]*\bsrc=)[^>]*>/i.test(html), false);
  }
});
