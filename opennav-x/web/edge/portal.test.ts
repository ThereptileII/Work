import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { Script } from "node:vm";
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
    rpc: async name => name === "store_portal_access" ? { entitlement: null, orders: [] } : [],
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
    return name === "store_portal_access" ? { entitlement: null, orders: [] }
      : [{ id: BOAT, name: "My boat", created_at: created, account_id: OTHER, secret: "not returned" }];
  } }), request(`/api/portal/dashboard?account_id=${OTHER}&owner=${OTHER}`));
  assert.deepEqual(calls.sort(), ["portal_list_boats", "store_portal_access"]);
  assert.deepEqual(data.boats, [{ id: BOAT, name: "My boat", createdAt: created }]);
  assert.equal(JSON.stringify(data).includes(OTHER), false);
  assert.equal(JSON.stringify(data).includes("not returned"), false);
});
test("paid and zero-total fulfillment grant identical active access; output sanitizes provider IDs", async () => {
  for (const amount of [2000, 0]) {
    const { data } = await result(context({ rpc: async name => name === "store_portal_access" ? {
      entitlement: { status: "active", environment: "staging", product_code: "skager-beta", provider_id: "secret-entitlement" },
      orders: [{ id: ORDER, created_at: created, status: "paid", kind: "access", total_minor: amount, checkout_id: "cs_secret",
        items: [{ name: "SKAGER Public Beta Access", kind: "access", quantity: 1, unit_minor: 2000, secret: "x" }] }],
    } : [] }));
    assert.equal(data.betaAccess, "active");
    assert.equal(data.orders[0].totalMinor, amount);
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
    const { data } = await result(context({ rpc: async name => name === "store_portal_access" ? {
      entitlement, orders: [{ id: ORDER, created_at: created, status: "paid", items: [] }],
    } : [] }));
    assert.equal(data.betaAccess, "inactive");
  }
});
test("refunded orders remain visible without granting access, and setup accounts see nothing yet", async () => {
  const { data } = await result(context({ rpc: async name => name === "store_portal_access" ? {
    entitlement: null,
    orders: [{ id: ORDER, created_at: created, status: "refunded", kind: "access", total_minor: 2000, items: [] }],
  } : [] }));
  assert.equal(data.betaAccess, "inactive");
  assert.equal(data.orders[0].status, "refunded");
  assert.match(renderDashboard(data, "orders"), /Refunded/);
  const setup = await result(context({ getAccount: async () => ({ id: MINE, email: "s@example.test", passwordSet: false }),
    rpc: async () => assert.fail("no data before a password exists") }));
  assert.equal(setup.response.status, 403);
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
  const data = { betaAccess: "inactive", account: { email: '<img src=x onerror="alert(1)">' }, boats: [], orders: [], subscription: null };
  for (const page of ["dashboard", "account", "orders", "downloads", "shop", "cloud"]) {
    const html = renderDashboard(data, page);
    assert.ok(html.length > 30, page);
    assert.equal(html.includes('<img src=x'), false);
  }
  const html = renderDashboard({ ...data, boats: [{ id: BOAT, name: "<script>alert(1)</script>", createdAt: created }] }, "account");
  assert.match(html, /&lt;script&gt;/);
  assert.equal(html.includes("<script>"), false);
  assert.match(renderDashboard(data, "downloads"), /Downloads are being prepared/);
  assert.match(renderDashboard(data, "orders"), /no orders yet/);
  assert.match(renderDashboard(data, "shop"), /being stocked/);
  // Inactive access never offers Cloud; the future platforms are visible but disabled.
  assert.match(renderDashboard(data, "cloud", { products: [{ kind: "subscription", priceMinor: 200 }] }), /needs an active SKAGER Beta Access/);
  const downloads = renderDashboard(data, "downloads");
  for (const platform of ["Linux x64", "Linux ARM64", "macOS"]) assert.match(downloads, new RegExp(platform));
  assert.equal((downloads.match(/disabled>Coming</g) || []).length, 4);
  assert.equal(escapeHTML("&<>'\""), "&amp;&lt;&gt;&#39;&quot;");
});
test("shop, cloud and orders render server data escaped, with refund only when the server allows it", () => {
  const shop = renderDashboard({}, "shop", { shippingMinor: 1500, products: [
    { id: BOAT, kind: "hardware", name: "<b>GPS</b>", summary: "s", priceMinor: 4900, imageUrl: "javascript:alert(1)" },
    { id: ORDER, kind: "subscription", name: "Cloud", priceMinor: 200 }] });
  assert.match(shop, /&lt;b&gt;GPS/);
  assert.match(shop, /USD 49\.00/);
  assert.equal(shop.includes("javascript:"), false);
  assert.equal(shop.includes("Cloud"), false);
  const cloud = renderDashboard({ betaAccess: "active", subscription: null }, "cloud", { products: [{ kind: "subscription", priceMinor: 200 }] });
  assert.match(cloud, /id="cloud-subscribe"/);
  assert.match(cloud, /USD 2\.00\/month/);
  for (const feature of ["Boat data", "Online dashboard", "Web route planning"]) assert.match(cloud, new RegExp(feature));
  const live = renderDashboard({ betaAccess: "active", subscription: { status: "active", currentPeriodEnd: created } }, "cloud", { products: [] });
  assert.match(live, /id="cloud-manage"/);
  assert.doesNotMatch(live, /cloud-subscribe/);
  const order = { id: ORDER, kind: "access", createdAt: created, status: "paid", fulfillment: "unfulfilled", items: [
    { name: "SKAGER Public Beta Access", quantity: 1, unitMinor: 2000 }, { name: "<i>GPS</i>", quantity: 1, unitMinor: 4900 }],
    subtotalMinor: 6900, shippingMinor: 1500, discountMinor: 0, totalMinor: 8400, refundable: true };
  const orders = renderDashboard({ orders: [order] }, "orders");
  assert.match(orders, /data-refund=/);
  assert.match(orders, /USD 84\.00/);
  assert.match(orders, /Preparing to ship/);
  assert.match(orders, /&lt;i&gt;GPS/);
  assert.doesNotMatch(renderDashboard({ orders: [{ ...order, refundable: false }] }, "orders"), /data-refund=/);
});
test("full discount states nothing was charged; paid totals show the amount", () => {
  assert.match(orderTotal({ totalMinor: 0, discountMinor: 2000 }), /nothing charged/);
  assert.match(orderTotal({ totalMinor: 2000, discountMinor: 0 }), /USD 20\.00/);
  assert.doesNotMatch(orderTotal({ totalMinor: 2000, discountMinor: 0 }), /nothing charged/);
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

test("auth shell: login sends password then code; forgot sends email; no signup exists", async t => {
  const source = await readFile(new URL("./static/auth.js", import.meta.url), "utf8");
  assert.doesNotMatch(source, /signup|sign up|create an account|recover/i);
  for (const mode of ["login", "forgot", "verify"]) {
    for (const slash of ["", "/"]) {
      await t.test(`/auth/${mode}${slash}`, async () => {
        const elements = new Map<string, any>();
        let submit: (event: any) => Promise<void>;
        const calls: { path: string; init: RequestInit }[] = [];
        const navigations: string[] = [];
        const values: Record<string, string> = { "auth-email": " friend@example.test ", "auth-password": "correct horse", "auth-code": " 123456 " };
        const document = {
          title: "",
          getElementById: (id: string) => {
            if (!elements.has(id)) elements.set(id, { id, hidden: id === "auth-form", value: values[id] ?? "",
              addEventListener: (name: string, handler: typeof submit) => { assert.equal(name, "submit"); submit = handler; } });
            return elements.get(id);
          },
        };
        await new Script(source).runInNewContext({
          document, location: { pathname: `/auth/${mode}${slash}`, assign: (path: string) => navigations.push(path) },
          fetch: async (path: string, init: RequestInit) => {
            calls.push({ path, init });
            if (path === "/api/auth/session") return new Response(JSON.stringify({ account: null, csrfToken: "c".repeat(64) }));
            return new Response(JSON.stringify(path === "/api/auth/verify" ? { ok: true, next: "https://evil.example/portal/" } : { ok: true, email: "f***@example.test" }));
          },
        });
        await new Promise(resolve => setImmediate(resolve));
        assert.equal(elements.get("auth-form").hidden, false);
        assert.equal(elements.get("password-field").hidden, mode !== "login");
        assert.equal(elements.get("code-field").hidden, mode !== "verify");
        await submit!({ preventDefault() {} });
        const call = calls[1];
        assert.equal(call.path, `/api/auth/${mode}`);
        assert.equal(call.init.method, "POST");
        assert.equal(call.init.credentials, "same-origin");
        assert.equal((call.init.headers as Record<string, string>)["X-CSRF-Token"], "c".repeat(64));
        const sent = JSON.parse(call.init.body as string);
        if (mode === "verify") {
          assert.deepEqual(sent, { code: "123456" });
          assert.deepEqual(navigations, ["/portal/"], "verification only ever navigates inside the portal");
          return;
        }
        assert.deepEqual(sent, mode === "login" ? { email: "friend@example.test", password: "correct horse" } : { email: "friend@example.test" });
        assert.equal(elements.get("auth-password").value, "", "the password is not kept in the form");
        assert.equal(elements.get("code-field").hidden, false);
        await submit!({ preventDefault() {} });
        assert.equal(calls[2].path, "/api/auth/verify");
      });
    }
  }
});

// Execute the shipped browser module and its registered handlers, with explicit
// HTTP responses and navigation observations. Provider authorization stays in
// the separately tested server handlers; these checks exercise browser control.
async function portalBrowser(path: string, respond: (path: string, init: RequestInit) => Promise<Response>) {
  const source = (await readFile(new URL('./static/portal.js', import.meta.url), 'utf8')).replace(/^export /gm, '');
  function element() {
    const handlers = new Map<string, (event: any) => Promise<void>>();
    return {
      hidden: false, disabled: false, innerHTML: '', textContent: '', value: '', dataset: {} as Record<string, string>,
      attributes: {} as Record<string, string>,
      setAttribute(name: string, value: string) { this.attributes[name] = value; },
      addEventListener(name: string, handler: (event: any) => Promise<void>) { handlers.set(name, handler); },
      async dispatch(name: string) {
        const handler = handlers.get(name); assert.ok(handler, `Missing ${name} handler`);
        await handler({ target: this, currentTarget: this, preventDefault() {} });
      },
    };
  }
  const content = element(), message = element(), logout = element(), heading = element();
  logout.hidden = true;
  const boatForm = { ...element(), querySelector: () => boatButton };
  const boatButton = element(), boatName = { ...element(), value: 'S/Y Aurora' };
  const checkout = element(), discount = { ...element(), value: ' FREE100 ' };
  const download = { ...element(), dataset: { releaseId: 'staging-test', artifactId: 'installer' } };
  const buy = { ...element(), dataset: { buy: BOAT } };
  Object.assign(content, { querySelector: () => null, querySelectorAll: (selector: string) =>
    selector === '.portal-download' && content.innerHTML.includes('portal-download') ? [download]
      : selector === '[data-buy]' && content.innerHTML.includes('data-buy') ? [buy] : [] });
  const calls: { path: string; init: RequestInit }[] = [];
  const navigations: string[] = [], anchors: any[] = [];
  const document = {
    getElementById: (id: string) => {
      if (id === 'portal-content') return content;
      if (id === 'portal-message') return message;
      if (id === 'logout') return logout;
      const controls: Record<string, any> = { 'boat-form': boatForm, 'boat-name': boatName, checkout, 'discount-code': discount };
      return content.innerHTML.includes(`id="${id}"`) ? controls[id] : null;
    },
    querySelector: (selector: string) => selector === 'h1' ? heading : element(),
    body: { appendChild: () => {} },
    createElement: () => {
      const anchor = { clicked: false, removed: false, click() { this.clicked = true; }, remove() { this.removed = true; } };
      anchors.push(anchor); return anchor;
    },
  };
  const url = new URL(path, 'https://beta.skager.app');
  await new Script(source).runInNewContext({ document, URL, URLSearchParams,
    location: { pathname: url.pathname, search: url.search, assign: (value: string) => navigations.push(value) },
    fetch: async (path: string, init: RequestInit) => { calls.push({ path, init }); return respond(path, init); },
  });
  return { content, message, logout, boatForm, checkout, download, buy, calls, navigations, anchors };
}
const browserAccount = { betaAccess: 'active', account: { email: 'friend@example.test' }, boats: [], orders: [], subscription: null };
const browserSession = { account: { id: MINE }, csrfToken: 'c'.repeat(64) };
const browserJSON = (body: unknown, status = 200) => new Response(JSON.stringify(body), { status });
function browserDownload() {
  const artifact = { id: 'installer', name: 'SKAGER-Setup.exe', bytes: 1200, sha256: 'b'.repeat(64) };
  const release = { id: 'staging-test', version: '0.4.0-beta2.5', sourceCommit: 'a'.repeat(40), channel: 'staging', artifacts: [artifact] };
  const issued = Math.floor(Date.now() / 1000) * 1000;
  const date = new Date(issued).toISOString().replace(/[-:]/g, '').replace(/\.\d+Z$/, 'Z');
  const url = new URL(`https://7869adcf376c630b175c94729321e016.r2.cloudflarestorage.com/skager-private-releases/staging/${release.sourceCommit}/${release.id}/${artifact.id}/${artifact.sha256}/${artifact.name}`);
  for (const [key, value] of Object.entries({ 'response-content-disposition': `attachment; filename="${artifact.name}"`,
    'X-Amz-Expires': '60', 'X-Amz-Date': date, 'X-Amz-Algorithm': 'AWS4-HMAC-SHA256',
    'X-Amz-Credential': 'f'.repeat(32) + `/${date.slice(0, 8)}/auto/s3/aws4_request`,
    'X-Amz-SignedHeaders': 'host', 'X-Amz-Signature': 'e'.repeat(64) })) url.searchParams.set(key, value);
  return { artifact, release, signed: { releaseId: release.id, artifactId: artifact.id, name: artifact.name,
    bytes: artifact.bytes, sha256: artifact.sha256, expiresAt: new Date(issued + 60000).toISOString(), url: url.href } };
}

test('expiry during downloads load leaves no authenticated controls and retains sign-in guidance', async () => {
  const browser = await portalBrowser('/portal/downloads/', async path => browserJSON(
    path === '/api/auth/session' ? browserSession : path === '/api/portal/dashboard' ? browserAccount : { error: 'Sign in required' },
    path === '/api/portal/downloads' ? 401 : 200));
  assert.deepEqual(browser.navigations, ['/auth/login/']);
  assert.equal(browser.content.innerHTML, '');
  assert.equal(browser.logout.hidden, true);
  assert.match(browser.message.textContent, /session has ended.*sign in/i);
  assert.equal(browser.content.attributes['aria-busy'], 'false');
});

test('logout clears loaded private data only after successful server revocation', async () => {
  for (const status of [200, 503]) {
    const browser = await portalBrowser('/portal/account/', async path => browserJSON(
      path === '/api/auth/session' ? browserSession : path === '/api/portal/dashboard' ? browserAccount
        : status === 200 ? { ok: true } : { error: 'Please try again.' }, path === '/api/auth/logout' ? status : 200));
    assert.match(browser.content.innerHTML, /friend@example.test/);
    await browser.logout.dispatch('click');
    const call = browser.calls.at(-1)!;
    assert.equal(call.path, '/api/auth/logout');
    assert.equal(call.init.method, 'POST');
    assert.equal(call.init.credentials, 'same-origin');
    assert.equal((call.init.headers as Record<string, string>)['X-CSRF-Token'], 'c'.repeat(64));
    if (status === 200) {
      assert.equal(browser.content.innerHTML, '');
      assert.equal(browser.logout.hidden, true);
      assert.deepEqual(browser.navigations, ['/auth/login/']);
    } else {
      assert.match(browser.content.innerHTML, /friend@example.test/);
      assert.equal(browser.logout.disabled, false);
      assert.deepEqual(browser.navigations, []);
    }
  }
});

test('expired session during boat refresh cannot be reported as successful account refresh', async () => {
  let sessions = 0;
  const browser = await portalBrowser('/portal/boats/', async path => {
    if (path === '/api/auth/session') return browserJSON(++sessions === 1 ? browserSession : { account: null, csrfToken: 'd'.repeat(64) });
    return browserJSON(path === '/api/portal/dashboard' ? browserAccount : { boat: { name: 'S/Y Aurora' } }, path === '/api/portal/boats' ? 201 : 200);
  });
  await browser.boatForm.dispatch('submit');
  assert.equal(browser.content.innerHTML, '');
  assert.match(browser.message.textContent, /session has ended.*sign in/i);
  assert.doesNotMatch(browser.message.textContent, /has been added/);
  assert.deepEqual(browser.navigations, ['/auth/login/']);
});

test('checkout return reads actual server order state and never trusts success query parameters', async () => {
  const account = { ...browserAccount, betaAccess: 'inactive', orders: [{ id: ORDER, kind: 'access', status: 'pending', createdAt: created,
    items: [{ name: 'SKAGER Public Beta Access', quantity: 1, unitMinor: 2000 }], subtotalMinor: 2000, shippingMinor: 0, discountMinor: 0, totalMinor: 2000 }] };
  const browser = await portalBrowser('/portal/orders/?success=true&betaAccess=active&paid=true', async path => browserJSON(
    path === '/api/auth/session' ? browserSession : account));
  assert.deepEqual(browser.calls.map(call => call.path), ['/api/auth/session', '/api/portal/dashboard']);
  assert.match(browser.content.innerHTML, /Awaiting payment/);
  assert.doesNotMatch(browser.content.innerHTML, /Paid|portal-download/);
  assert.deepEqual(browser.navigations, []);
});

test('anonymous checkout return goes to fixed login without querying private orders', async () => {
  const browser = await portalBrowser('/portal/orders/?next=https://untrusted.example.test', async path => {
    assert.equal(path, '/api/auth/session'); return browserJSON({ account: null, csrfToken: 'c'.repeat(64) });
  });
  assert.deepEqual(browser.navigations, ['/auth/login/']);
  assert.equal(browser.content.innerHTML, '');
  assert.equal(browser.calls.length, 1);
});

test('download selection rechecks server authorization and cannot navigate after logout', async () => {
  const { artifact, release, signed } = browserDownload();
  let finishDownload!: (value: Response) => void;
  const pending = new Promise<Response>(resolve => { finishDownload = resolve; });
  const browser = await portalBrowser('/portal/downloads/', async (path, init) => {
    if (path === '/api/auth/session') return browserJSON(browserSession);
    if (path === '/api/portal/dashboard') return browserJSON(browserAccount);
    if (path === '/api/auth/logout') return browserJSON({ ok: true });
    assert.equal(path, '/api/portal/downloads');
    return init.method === 'POST' ? pending : browserJSON({ available: true, releases: [release] });
  });
  assert.equal(browser.anchors.length, 0, 'catalog must not pre-mint or navigate to bearer links');
  const downloading = browser.download.dispatch('click');
  const call = browser.calls.at(-1)!;
  assert.equal(call.init.method, 'POST');
  assert.deepEqual(JSON.parse(call.init.body as string), { releaseId: release.id, artifactId: artifact.id });
  assert.equal((call.init.headers as Record<string, string>)['X-CSRF-Token'], 'c'.repeat(64));
  await browser.logout.dispatch('click');
  finishDownload(browserJSON(signed));
  await downloading;
  assert.equal(browser.anchors.length, 0);
  assert.equal(browser.content.innerHTML, '');
  assert.deepEqual(browser.navigations, ['/auth/login/']);
  assert.match(browser.message.textContent, /session has ended/i);
});

test('authorized download obtains a fresh selection-bound link only on click and suppresses referrers', async () => {
  const { release, artifact, signed } = browserDownload();
  const browser = await portalBrowser('/portal/downloads/', async (path, init) => {
    if (path === '/api/auth/session') return browserJSON(browserSession);
    if (path === '/api/portal/dashboard') return browserJSON(browserAccount);
    assert.equal(path, '/api/portal/downloads');
    return browserJSON(init.method === 'POST' ? signed : { available: true, releases: [release] });
  });
  assert.equal(browser.anchors.length, 0);
  assert.doesNotMatch(browser.content.innerHTML, /X-Amz-Signature/);
  await browser.download.dispatch('click');
  assert.equal(browser.anchors.length, 1);
  assert.equal(browser.anchors[0].href, signed.url);
  assert.equal(browser.anchors[0].rel, 'noreferrer');
  assert.equal(browser.anchors[0].referrerPolicy, 'no-referrer');
  assert.equal(browser.anchors[0].download, artifact.name);
  assert.equal(browser.anchors[0].clicked, true);
  assert.equal(browser.anchors[0].removed, true);
  assert.equal(browser.download.disabled, false);
  assert.deepEqual(browser.navigations, []);
});

function deferredJSON() {
  let complete!: (value: any) => void;
  let entered!: () => void;
  const waiting = new Promise<void>(resolve => { entered = resolve; });
  const body = new Promise<any>(resolve => { complete = resolve; });
  const response = browserJSON({});
  response.json = () => { entered(); return body; };
  return { response, waiting, complete };
}

test('same-turn logout and download JSON completion cannot start a bearer-link download', async () => {
  const { release, signed } = browserDownload();
  const downloadBody = deferredJSON(), logoutBody = deferredJSON();
  const browser = await portalBrowser('/portal/downloads/', async (path, init) => {
    if (path === '/api/auth/session') return browserJSON(browserSession);
    if (path === '/api/portal/dashboard') return browserJSON(browserAccount);
    if (path === '/api/auth/logout') return logoutBody.response;
    assert.equal(path, '/api/portal/downloads');
    return init.method === 'POST' ? downloadBody.response : browserJSON({ available: true, releases: [release] });
  });
  const downloading = browser.download.dispatch('click');
  const loggingOut = browser.logout.dispatch('click');
  await Promise.all([downloadBody.waiting, logoutBody.waiting]);
  logoutBody.complete({ ok: true });
  downloadBody.complete(signed);
  await Promise.all([downloading, loggingOut]);
  assert.equal(browser.anchors.length, 0);
  assert.equal(browser.content.innerHTML, '');
  assert.equal(browser.logout.hidden, true);
  assert.deepEqual(browser.navigations, ['/auth/login/']);
  assert.match(browser.message.textContent, /session has ended.*sign in/i);
});

test('same-turn logout and refreshed dashboard JSON completion cannot restore private DOM', async () => {
  const dashboardBody = deferredJSON(), logoutBody = deferredJSON();
  let dashboards = 0;
  const browser = await portalBrowser('/portal/boats/', async path => {
    if (path === '/api/auth/session') return browserJSON(browserSession);
    if (path === '/api/portal/dashboard') return ++dashboards === 1 ? browserJSON(browserAccount) : dashboardBody.response;
    if (path === '/api/auth/logout') return logoutBody.response;
    assert.equal(path, '/api/portal/boats'); return browserJSON({ boat: { name: 'Private vessel' } }, 201);
  });
  const saving = browser.boatForm.dispatch('submit');
  await dashboardBody.waiting;
  const loggingOut = browser.logout.dispatch('click');
  await logoutBody.waiting;
  logoutBody.complete({ ok: true });
  dashboardBody.complete({ ...browserAccount, boats: [{ name: 'Private vessel', createdAt: created }] });
  await Promise.all([saving, loggingOut]);
  assert.equal(browser.content.innerHTML, '');
  assert.equal(browser.logout.hidden, true);
  assert.deepEqual(browser.navigations, ['/auth/login/']);
  assert.match(browser.message.textContent, /session has ended.*sign in/i);
  assert.doesNotMatch(browser.message.textContent, /has been added/);
});

test('same-turn logout and shop checkout JSON completion cannot navigate to Stripe', async () => {
  const checkoutBody = deferredJSON(), logoutBody = deferredJSON();
  const browser = await portalBrowser('/portal/shop/', async path => {
    if (path === '/api/auth/session') return browserJSON(browserSession);
    if (path === '/api/portal/dashboard') return browserJSON(browserAccount);
    if (path === '/api/store/catalog') return browserJSON({ shippingMinor: 1500, products: [{ id: BOAT, kind: 'hardware', name: 'GPS', priceMinor: 4900 }] });
    if (path === '/api/auth/logout') return logoutBody.response;
    assert.equal(path, '/api/store/shop'); return checkoutBody.response;
  });
  const checkingOut = browser.buy.dispatch('click');
  const loggingOut = browser.logout.dispatch('click');
  await Promise.all([checkoutBody.waiting, logoutBody.waiting]);
  logoutBody.complete({ ok: true });
  checkoutBody.complete({ url: 'https://checkout.stripe.com/c/pay/cs_test_fixture' });
  await Promise.all([checkingOut, loggingOut]);
  assert.equal(browser.content.innerHTML, '');
  assert.deepEqual(browser.navigations, ['/auth/login/']);
  assert.match(browser.message.textContent, /session has ended.*sign in/i);
});

test('fetch rejection after successful logout cannot overwrite sign-in guidance', async () => {
  const { release } = browserDownload();
  let rejectDownload!: (reason: Error) => void;
  const pending = new Promise<Response>((resolve, reject) => { rejectDownload = reject; });
  const browser = await portalBrowser('/portal/downloads/', async (path, init) => {
    if (path === '/api/auth/session') return browserJSON(browserSession);
    if (path === '/api/portal/dashboard') return browserJSON(browserAccount);
    if (path === '/api/auth/logout') return browserJSON({ ok: true });
    assert.equal(path, '/api/portal/downloads');
    return init.method === 'POST' ? pending : browserJSON({ available: true, releases: [release] });
  });
  const downloading = browser.download.dispatch('click');
  await browser.logout.dispatch('click');
  rejectDownload(new Error('Network disconnected.'));
  await downloading;
  assert.equal(browser.content.innerHTML, '');
  assert.deepEqual(browser.navigations, ['/auth/login/']);
  assert.match(browser.message.textContent, /session has ended.*sign in/i);
});
