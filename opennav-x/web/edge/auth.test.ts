import assert from "node:assert/strict";
import test from "node:test";
import { type AuthContext, getAccount, handleAuth, validPassword } from "./auth.ts";

const csrf = "c".repeat(64);
const buyer = "buyer@example.com";
const subject = "11111111-2222-4333-8444-555555555555";
const PASSWORD = "correct horse battery";

type Customer = { state: "active" | "revoked"; claimed: boolean; password_set: boolean; account_id: string | null };

function fixture() {
  const customers = new Map<string, Customer>([[buyer, { state: "active", claimed: true, password_set: true, account_id: subject }]]);
  const claims = new Map<string, { state: string; email?: string }>();
  const challenges = new Map<string, { email: string; purpose: string }>();
  const sessions = new Map<string, string>();
  const passwords = new Map<string, string>([[buyer, PASSWORD]]);
  const calls: { name: string; args: Record<string, any> }[] = [];
  const remote: { path: string; search: string; method: string; body?: any; headers: Headers }[] = [];
  const state = { throttled: false, dbFailure: false, wrongUser: false, providerFailure: false, inviteConflict: false };
  const ctx: AuthContext = {
    env: {
      SKAGER_AUTH_ENABLED: "true",
      SKAGER_ADMIN_EMAILS: "Admin@Example.com",
      SUPABASE_URL: "https://kelwgubkuvditalsbqpr.supabase.co",
      SUPABASE_ANON_KEY: "test-public-key",
      SUPABASE_SERVICE_ROLE_KEY: "SECRET service role",
    },
    clientIp: "192.0.2.10",
    rpc: async (name, args: Record<string, any>) => {
      calls.push({ name, args });
      if (state.dbFailure) throw new Error("SECRET upstream diagnostic");
      switch (name) {
        case "skager_auth_throttle": return !state.throttled;
        case "store_customer_state": {
          const found = customers.get(args.p_email);
          return found ? { ...found, state: found.state } : { state: "none" };
        }
        case "store_claim_state":
          return args.p_session_id ? claims.get(args.p_session_id) ?? { state: "unknown" }
            : [...claims.values()].find(c => c.email === args.p_email) ?? { state: "unknown" };
        case "store_challenge_create": challenges.set(args.p_hash, { email: args.p_email, purpose: args.p_purpose }); return true;
        case "store_challenge_take": {
          const found = challenges.get(args.p_hash); challenges.delete(args.p_hash); return found ? [found] : [];
        }
        case "store_link_account": {
          const found = customers.get(args.p_email);
          if (!found || found.state !== "active" || (found.account_id && found.account_id !== args.p_account_id)) throw new Error("Account unavailable");
          found.account_id = args.p_account_id; found.claimed = true;
          if (args.p_purpose !== "login") found.password_set = false;
          return { password_set: found.password_set };
        }
        case "store_password_set": {
          for (const c of customers.values()) if (c.account_id === args.p_account_id) c.password_set = true;
          return true;
        }
        case "skager_auth_session_create": sessions.set(args.p_hash, args.p_subject); return true;
        case "store_session_get": {
          const id = sessions.get(args.p_hash);
          const entry = [...customers.entries()].find(([, c]) => c.account_id === id && c.state === "active");
          return id && entry ? [{ id, email: entry[0], password_set: entry[1].password_set }] : [];
        }
        case "skager_auth_session_delete": sessions.delete(args.p_hash); return true;
        default: throw new Error("unknown RPC " + name);
      }
    },
    fetch: async (input, init) => {
      const url = new URL(String(input));
      const body = init?.body ? JSON.parse(String(init.body)) : undefined;
      remote.push({ path: url.pathname, search: url.search, method: String(init?.method), body, headers: new Headers(init?.headers) });
      const ok = (data: unknown) => new Response(JSON.stringify(data), { status: 200, headers: { "content-type": "application/json" } });
      if (state.providerFailure) return new Response('{"error":"SECRET provider diagnostic"}', { status: 400 });
      if (url.pathname === "/auth/v1/token") {
        return passwords.get(body.email) === body.password ? ok({ access_token: "SECRET pw token" }) : new Response("{}", { status: 400 });
      }
      if (url.pathname === "/auth/v1/invite" && state.inviteConflict) return new Response("{}", { status: 422 });
      if (url.pathname === "/auth/v1/verify") return ok({ access_token: "SECRET access token", refresh_token: "SECRET refresh" });
      if (url.pathname === "/auth/v1/user") {
        return ok({ id: subject, email: state.wrongUser ? "attacker@example.com" : body?.email ?? [...challengesEmails()][0] ?? buyer,
          email_confirmed_at: new Date(Date.now() - 60_000).toISOString(), aud: "authenticated", role: "authenticated", is_anonymous: false });
      }
      if (url.pathname.startsWith("/auth/v1/admin/users/")) {
        passwords.set(buyer, body.password); return ok({ id: url.pathname.split("/").pop() });
      }
      return ok({});
    },
  };
  // The fake /user answers with the email of the most recently verified challenge.
  let lastVerified = buyer;
  const challengesEmails = () => [lastVerified];
  const originalFetch = ctx.fetch!;
  ctx.fetch = async (input, init) => {
    const url = new URL(String(input));
    if (url.pathname === "/auth/v1/verify") lastVerified = JSON.parse(String(init!.body)).email;
    return originalFetch(input, init);
  };
  return { ctx, customers, claims, challenges, sessions, passwords, calls, remote, state };
}

function request(path: string, data?: unknown, cookies = "", origin = "https://beta.skager.app"): Request {
  return new Request("https://beta.skager.app/api/auth/" + path, {
    method: data === undefined ? "GET" : "POST",
    headers: { origin, cookie: `__Host-skager_csrf=${csrf}; ${cookies}`, "x-csrf-token": csrf, "content-type": "application/json" },
    ...(data === undefined ? {} : { body: JSON.stringify(data) }),
  });
}
function cookieOf(res: Response, name: string): string {
  const match = (res.headers.get("set-cookie") || "").match(new RegExp(`(?:^|, )(${name}=[a-f0-9]{64})`));
  assert.ok(match, `missing ${name}`);
  return match[1];
}
async function send(f: ReturnType<typeof fixture>, path: string, data?: unknown, cookies = "") {
  const res = await handleAuth(request(path, data, cookies), f.ctx);
  assert.ok(res);
  return { res, data: await res.clone().json() };
}

test("password, then emailed code: login creates an opaque session for an active customer", async () => {
  const f = fixture();
  const started = await send(f, "login", { email: buyer, password: PASSWORD });
  assert.equal(started.res.status, 200);
  const token = f.remote.find(r => r.path === "/auth/v1/token")!;
  assert.equal(token.search, "?grant_type=password");
  assert.equal(token.headers.get("apikey"), "test-public-key");
  assert.equal(token.headers.get("authorization"), null);
  const otp = f.remote.find(r => r.path === "/auth/v1/otp")!;
  assert.deepEqual(otp.body, { email: buyer, create_user: false });
  const challenge = cookieOf(started.res, "__Host-skager_challenge");
  assert.equal([...f.challenges.values()][0].purpose, "login");
  const verified = await send(f, "verify", { code: "123456" }, challenge);
  assert.equal(verified.res.status, 200);
  assert.equal(verified.data.next, "/portal/");
  const session = cookieOf(verified.res, "__Host-skager_session");
  const account = await getAccount(request("session", undefined, session), f.ctx);
  assert.deepEqual(account, { id: subject, email: buyer, passwordSet: true });
  assert.ok(!JSON.stringify(verified.data).includes("SECRET"));
});

test("wrong password, unknown and unclaimed addresses get the same answer and no code", async () => {
  const f = fixture();
  f.customers.set("unclaimed@example.com", { state: "active", claimed: false, password_set: false, account_id: null });
  const bodies = [];
  for (const [email, password] of [[buyer, "wrong password!!"], ["nobody@example.com", PASSWORD], ["unclaimed@example.com", PASSWORD]]) {
    const { res, data } = await send(f, "login", { email, password });
    assert.equal(res.status, 400);
    bodies.push(JSON.stringify(data));
  }
  assert.equal(new Set(bodies).size, 1);
  assert.ok(!f.remote.some(r => r.path === "/auth/v1/otp"));
  assert.equal(f.challenges.size, 0);
});

test("a refunded (closed) account is refused at login and at verification", async () => {
  const f = fixture();
  f.customers.get(buyer)!.state = "revoked";
  const login = await send(f, "login", { email: buyer, password: PASSWORD });
  assert.equal(login.res.status, 403);
  assert.match(login.data.error, /closed/);
  assert.ok(!f.remote.some(r => r.path === "/auth/v1/otp"));
  // A code issued before the refund cannot open a session afterwards.
  const g = fixture();
  const started = await send(g, "login", { email: buyer, password: PASSWORD });
  g.customers.get(buyer)!.state = "revoked";
  const verified = await send(g, "verify", { code: "123456" }, cookieOf(started.res, "__Host-skager_challenge"));
  assert.equal(verified.res.status, 403);
  assert.equal(g.sessions.size, 0);
});

test("claim after checkout: pending payment waits, then an invited code leads to choosing a password", async () => {
  const f = fixture();
  f.customers.delete(buyer); // the fake identity provider gives every email the same subject
  const newBuyer = "new@example.com";
  f.claims.set("cs_test_pending1234", { state: "pending" });
  assert.equal((await send(f, "claim", { sessionId: "cs_test_pending1234" })).res.status, 202);
  f.claims.set("cs_test_ready12345", { state: "ready", email: newBuyer });
  f.customers.set(newBuyer, { state: "active", claimed: false, password_set: false, account_id: null });
  const claimed = await send(f, "claim", { sessionId: "cs_test_ready12345" });
  assert.equal(claimed.data.state, "code_sent");
  assert.match(claimed.data.email, /^ne•+@example\.com$/);
  const invite = f.remote.find(r => r.path === "/auth/v1/invite")!;
  assert.equal(invite.headers.get("authorization"), "Bearer SECRET service role");
  assert.deepEqual(invite.body, { email: newBuyer });
  const verified = await send(f, "verify", { code: "654321" }, cookieOf(claimed.res, "__Host-skager_challenge"));
  assert.equal(verified.data.next, "/portal/account/?setup=1");
  const session = cookieOf(verified.res, "__Host-skager_session");
  assert.equal((await getAccount(request("session", undefined, session), f.ctx))!.passwordSet, false);
  // Setting the first password needs no current password, only the verified session.
  assert.equal((await send(f, "password", { password: "short" }, session)).res.status, 400);
  const saved = await send(f, "password", { password: "a strong new passphrase" }, session);
  assert.equal(saved.res.status, 200);
  const update = f.remote.find(r => r.path.startsWith("/auth/v1/admin/users/"))!;
  assert.equal(update.method, "PUT");
  assert.equal(update.path, `/auth/v1/admin/users/${subject}`);
  assert.equal(update.headers.get("authorization"), "Bearer SECRET service role");
  assert.equal(f.customers.get(newBuyer)!.password_set, true);
});

test("an existing identity falls back to an ordinary code when the invitation conflicts", async () => {
  const f = fixture();
  f.state.inviteConflict = true;
  f.claims.set("cs_test_ready12345", { state: "ready", email: buyer });
  const claimed = await send(f, "claim", { sessionId: "cs_test_ready12345" });
  assert.equal(claimed.data.state, "code_sent");
  assert.deepEqual(f.remote.find(r => r.path === "/auth/v1/otp")!.body, { email: buyer, create_user: false });
});

test("claim by email and forgot password answer identically for everyone", async () => {
  const f = fixture();
  f.claims.set("cs_test_ready12345", { state: "ready", email: "ready@example.com" });
  f.customers.set("ready@example.com", { state: "active", claimed: false, password_set: false, account_id: null });
  const answers = [];
  for (const email of ["ready@example.com", "nobody@example.com"]) answers.push(JSON.stringify((await send(f, "claim", { email })).data));
  assert.equal(new Set(answers).size, 1);
  assert.equal(f.remote.filter(r => r.path === "/auth/v1/invite").length, 1);
  const forgot = [];
  for (const email of [buyer, "nobody@example.com", "ready@example.com"]) forgot.push(JSON.stringify((await send(f, "forgot", { email })).data));
  assert.equal(new Set(forgot).size, 1);
  assert.equal(f.remote.filter(r => r.path === "/auth/v1/otp").length, 1);
  assert.equal([...f.challenges.values()].find(c => c.email === buyer)!.purpose, "reset");
});

test("changing an existing password requires the current one", async () => {
  const f = fixture();
  const started = await send(f, "login", { email: buyer, password: PASSWORD });
  const session = cookieOf((await send(f, "verify", { code: "123456" }, cookieOf(started.res, "__Host-skager_challenge"))).res, "__Host-skager_session");
  assert.equal((await send(f, "password", { password: "another long password" }, session)).res.status, 400);
  assert.equal((await send(f, "password", { password: "another long password", current: "wrong one" }, session)).res.status, 400);
  assert.equal((await send(f, "password", { password: "another long password", current: PASSWORD }, session)).res.status, 200);
  assert.equal(f.passwords.get(buyer), "another long password");
  assert.ok(validPassword("ten chars!") && !validPassword("         x") && !validPassword("a".repeat(129)));
});

test("every mutation rejects cross-origin and missing CSRF; signup and recovery routes no longer exist", async () => {
  const f = fixture();
  for (const path of ["login", "claim", "forgot", "verify", "password", "logout"]) {
    const res = await handleAuth(request(path, { email: buyer }, "", "https://evil.example"), f.ctx);
    assert.equal(res!.status, 403);
    const missing = new Request("https://beta.skager.app/api/auth/" + path, { method: "POST",
      headers: { origin: "https://beta.skager.app", "content-type": "application/json" }, body: "{}" });
    assert.equal((await handleAuth(missing, f.ctx))!.status, 403);
  }
  for (const path of ["signup", "recover"]) assert.equal(await handleAuth(request(path, { email: buyer }), f.ctx), null);
  assert.equal(f.calls.length, 0);
});

test("a challenge is bound to its browser and consumed exactly once, even under replay", async () => {
  const f = fixture();
  const started = await send(f, "login", { email: buyer, password: PASSWORD });
  const challenge = cookieOf(started.res, "__Host-skager_challenge");
  assert.equal((await send(f, "verify", { code: "123456" })).res.status, 400);
  const results = await Promise.all([1, 2, 3].map(() => send(f, "verify", { code: "123456" }, challenge)));
  assert.equal(results.filter(r => r.res.status === 200).length, 1);
  assert.equal(f.sessions.size, 1);
});

test("canonical user mismatch or provider failure creates no session", async () => {
  for (const flag of ["wrongUser", "providerFailure"] as const) {
    const f = fixture();
    const started = await send(f, "login", { email: buyer, password: PASSWORD });
    f.state[flag] = true;
    const verified = await send(f, "verify", { code: "123456" }, cookieOf(started.res, "__Host-skager_challenge"));
    assert.equal(verified.res.status, 400);
    assert.equal(f.sessions.size, 0);
    assert.ok(!JSON.stringify(verified.data).includes("SECRET"));
  }
});

test("other Supabase projects are refused before any key is sent", async () => {
  const f = fixture();
  f.ctx.env.SUPABASE_URL = "https://otherproject.supabase.co";
  const { res } = await send(f, "login", { email: buyer, password: PASSWORD });
  assert.equal(res.status, 400);
  assert.equal(f.remote.length, 0);
});

test("duplicate JSON keys and throttling stop requests before the provider", async () => {
  const f = fixture();
  const duplicate = new Request("https://beta.skager.app/api/auth/login", { method: "POST",
    headers: { origin: "https://beta.skager.app", cookie: `__Host-skager_csrf=${csrf}`, "x-csrf-token": csrf, "content-type": "application/json" },
    body: `{"email":"${buyer}","email":"other@example.com","password":"${PASSWORD}"}` });
  assert.notEqual((await handleAuth(duplicate, f.ctx))!.status, 200);
  f.state.throttled = true;
  assert.equal((await send(f, "login", { email: buyer, password: PASSWORD })).res.status, 429);
  assert.equal(f.remote.length, 0);
});

test("sessions vanish for closed customers, disabled auth or malformed cookies; logout revokes", async () => {
  const f = fixture();
  const started = await send(f, "login", { email: buyer, password: PASSWORD });
  const session = cookieOf((await send(f, "verify", { code: "123456" }, cookieOf(started.res, "__Host-skager_challenge"))).res, "__Host-skager_session");
  f.customers.get(buyer)!.state = "revoked";
  assert.equal(await getAccount(request("session", undefined, session), f.ctx), null);
  f.customers.get(buyer)!.state = "active";
  const calls = f.calls.length;
  assert.equal(await getAccount(request("session", undefined, "__Host-skager_session=not-a-token"), f.ctx), null);
  assert.equal(f.calls.length, calls);
  f.ctx.env.SKAGER_AUTH_ENABLED = "false";
  assert.equal(await getAccount(request("session", undefined, session), f.ctx), null);
  f.ctx.env.SKAGER_AUTH_ENABLED = "true";
  const out = await send(f, "logout", {}, session);
  assert.match(out.res.headers.get("set-cookie")!, /__Host-skager_session=; Path=\/; Max-Age=0/);
  assert.equal(await getAccount(request("session", undefined, session), f.ctx), null);
});

test("session reports administrators only from the exact server allowlist", async () => {
  const f = fixture();
  f.customers.set("admin@example.com", { state: "active", claimed: true, password_set: true, account_id: "99999999-2222-4333-8444-555555555555" });
  const started = await send(f, "login", { email: buyer, password: PASSWORD });
  const session = cookieOf((await send(f, "verify", { code: "123456" }, cookieOf(started.res, "__Host-skager_challenge"))).res, "__Host-skager_session");
  assert.equal((await send(f, "session", undefined, session)).data.account.admin, false);
});

test("database failure during verification exposes nothing and creates no session", async () => {
  const f = fixture();
  const started = await send(f, "login", { email: buyer, password: PASSWORD });
  f.state.dbFailure = true;
  const verified = await send(f, "verify", { code: "123456" }, cookieOf(started.res, "__Host-skager_challenge"));
  assert.equal(verified.res.status, 400);
  assert.ok(!JSON.stringify(verified.data).includes("SECRET"));
});
