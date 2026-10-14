import test from "node:test";
import { type AuthContext, getAccount, handleAuth } from "./auth.ts";
// Runs with Node --test --experimental-strip-types; no network or third-party modules.
function assert(
  condition: unknown,
  message = "assertion failed",
): asserts condition {
  if (!condition) throw new Error(message);
}
function same(actual: unknown, expected: unknown): void {
  assert(
    JSON.stringify(actual) === JSON.stringify(expected),
    `${JSON.stringify(actual)} != ${JSON.stringify(expected)}`,
  );
}
const csrf = "c".repeat(64),
  address = "friend@example.com",
  subject = "11111111-2222-3333-4444-555555555555";
function fixture() {
  const challenges = new Map<string, string>(),
    sessions = new Map<string, { id: string; email: string }>();
  const calls: { name: string; args: Record<string, unknown> }[] = [],
    remote: {
      path: string;
      body?: Record<string, unknown>;
      headers: Headers;
    }[] = [];
  const state = {
    throttled: false,
    failure: false,
    wrongUser: false,
    badSession: false,
    providerFailure: false,
  };
  const ctx: AuthContext = {
    env: {
      SKAGER_AUTH_ENABLED: "true",
      SKAGER_INVITE_EMAILS: address,
      SUPABASE_URL: "https://kelwgubkuvditalsbqpr.supabase.co",
      SUPABASE_ANON_KEY: "test-public-key",
      SUPABASE_SERVICE_ROLE_KEY: "SECRET service role",
    },
    clientIp: "192.0.2.10",
    rpc: async (name, args) => {
      calls.push({ name, args });
      if (state.failure) throw new Error("SECRET upstream diagnostic");
      if (name === "skager_auth_throttle") return !state.throttled;
      if (name === "skager_auth_challenge_create") {
        challenges.set(String(args.p_hash), String(args.p_email));
        return true;
      }
      if (name === "skager_auth_challenge_take") {
        const email = challenges.get(String(args.p_hash));
        challenges.delete(String(args.p_hash));
        return email ? [{ email }] : [];
      }
      if (name === "skager_auth_session_create") {
        if (state.badSession) return false;
        sessions.set(String(args.p_hash), {
          id: String(args.p_subject),
          email: address,
        });
        return true;
      }
      if (name === "skager_auth_session_get") {
        return sessions.has(String(args.p_hash))
          ? [sessions.get(String(args.p_hash))]
          : [];
      }
      if (name === "skager_auth_session_delete") {
        sessions.delete(String(args.p_hash));
        return true;
      }
      throw new Error("unknown RPC");
    },
    fetch: async (input, init) => {
      const path = new URL(String(input)).pathname;
      remote.push({
        path,
        body: init?.body ? JSON.parse(String(init.body)) : undefined,
        headers: new Headers(init?.headers),
      });
      if (state.providerFailure) {
        return new Response(
          JSON.stringify({ error: "SECRET provider diagnostic" }),
          { status: 400 },
        );
      }
      const data = path.endsWith("/verify")
        ? {
          access_token: "SECRET access token",
          refresh_token: "SECRET refresh token",
        }
        : path.endsWith("/user")
        ? {
          id: subject,
          email: state.wrongUser ? "attacker@example.com" : address,
          email_confirmed_at: new Date(Date.now() - 60_000).toISOString(),
          aud: "authenticated",
          role: "authenticated",
          is_anonymous: false,
        }
        : {};
      return new Response(JSON.stringify(data), {
        status: 200,
        headers: { "content-type": "application/json" },
      });
    },
  };
  return { ctx, calls, remote, state, challenges, sessions };
}
function request(
  path: string,
  data?: unknown,
  cookies = "",
  origin = "https://beta.skager.app",
): Request {
  const method = data === undefined ? "GET" : "POST";
  return new Request("https://beta.skager.app/api/auth/" + path, {
    method,
    headers: {
      origin,
      cookie: `__Host-skager_csrf=${csrf}; ${cookies}`,
      "x-csrf-token": csrf,
      "content-type": "application/json",
    },
    ...(data === undefined ? {} : { body: JSON.stringify(data) }),
  });
}
function setCookie(res: Response, name: string): string {
  const match = (res.headers.get("set-cookie") || "").match(
    new RegExp(`(?:^|, )(${name}=[a-f0-9]{64})`),
  );
  assert(match);
  return match[1];
}
async function begin(
  f: ReturnType<typeof fixture>,
  flow = "login",
): Promise<string> {
  const res = await handleAuth(request(flow, { email: address }), f.ctx);
  assert(res?.status === 200);
  return setCookie(res, "__Host-skager_challenge");
}
test("reserved legacy configuration cannot enable authentication", async () => {
  for (const value of [undefined, "", "false", "TRUE"]) {
    const f = fixture();
    f.ctx.env.SKAGER_AUTH_ENABLED = value;
    f.ctx.env.SUPABASE_AUTH_ENABLED = "true";
    f.ctx.env.SUPABASE_INVITE_EMAILS = address;
    assert((await handleAuth(request("login", { email: address }), f.ctx))?.status === 503);
    same(await getAccount(request("session", undefined, `__Host-skager_session=${"a".repeat(64)}`), f.ctx), null);
    same(f.calls.length, 0);
    same(f.remote.length, 0);
  }
});
test("SKAGER invitation allowlist is exact and has no reserved-name fallback", async () => {
  for (const value of [undefined, "", `${address}.attacker.test`]) {
    const f = fixture();
    f.ctx.env.SKAGER_INVITE_EMAILS = value;
    f.ctx.env.SUPABASE_INVITE_EMAILS = address;
    const res = await handleAuth(request("login", { email: address }), f.ctx);
    assert(res?.status === 200);
    same(f.remote.length, 0);
    same(f.challenges.size, 0);
    same(f.sessions.size, 0);
  }
  const f = fixture();
  f.ctx.env.SKAGER_INVITE_EMAILS = ` other@example.com, ${address.toUpperCase()} `;
  await begin(f);
  same(f.remote.map(c => c.path), ["/auth/v1/otp"]);
  same(f.remote[0].body, { email: address, create_user: false });
  same(f.challenges.size, 1);
});
test("allowlisted OTP login verifies canonical user then issues opaque bounded session", async () => {
  const f = fixture(), challenge = await begin(f);
  const res = await handleAuth(
    request("verify", { code: "123456" }, challenge),
    f.ctx,
  );
  assert(res?.status === 200);
  const session = setCookie(res, "__Host-skager_session");
  assert(
    res.headers.get("set-cookie")?.includes(
      "Max-Age=1200; HttpOnly; Secure; SameSite=Lax",
    ),
  );
  same(await getAccount(request("session", undefined, session), f.ctx), {
    id: subject,
    email: address,
  });
  same(f.remote.map((c) => c.path), [
    "/auth/v1/otp",
    "/auth/v1/verify",
    "/auth/v1/user",
  ]);
  same(f.remote[0].body, { email: address, create_user: false });
  same(f.remote[1].body, { email: address, token: "123456", type: "email" });
  assert(
    f.remote[2].headers.get("authorization") === "Bearer SECRET access token",
  );
  for (const call of f.remote) same(call.headers.get("apikey"), "test-public-key");
  same(f.remote[0].headers.get("authorization"), null);
  same(f.remote[1].headers.get("authorization"), null);
  assert(!JSON.stringify(f.calls).includes("SECRET"));
  assert(!JSON.stringify(f.calls).includes(session.split("=")[1]));
  assert(!(await res.text()).includes("SECRET"));
});
test("signup sends privileged unconfirmed invitation; recovery remains existing-user OTP only", async () => {
  const f = fixture();
  const challenge = await begin(f, "signup");
  await begin(f, "recover");
  same(f.remote.map((c) => c.path), ["/auth/v1/invite", "/auth/v1/otp"]);
  same(f.remote[0].body, { email: address });
  same(f.remote[1].body, { email: address, create_user: false });
  same(f.remote[0].headers.get("authorization"), "Bearer SECRET service role");
  same(f.remote[0].headers.get("apikey"), "SECRET service role");
  same(f.remote[1].headers.get("apikey"), "test-public-key");
  same(f.remote[1].headers.get("authorization"), null);
  same(f.sessions.size, 0);
  const denied = await handleAuth(
    request("signup", { email: "stranger@example.com" }),
    f.ctx,
  );
  assert(denied?.status === 200);
  same(f.remote.length, 2);
  assert(challenge.startsWith("__Host-skager_challenge="));
});
test("admin invitation needs its server key independently of public OTP configuration", async () => {
  const f = fixture();
  delete f.ctx.env.SUPABASE_ANON_KEY;
  await begin(f, "signup");
  same(f.remote.length, 1);
  same(f.remote[0].path, "/auth/v1/invite");
  same(f.remote[0].headers.get("apikey"), "SECRET service role");
  same(f.remote[0].headers.get("authorization"), "Bearer SECRET service role");
  same(f.sessions.size, 0);

  const missing = fixture();
  delete missing.ctx.env.SUPABASE_SERVICE_ROLE_KEY;
  missing.state.providerFailure = true;
  const result = await handleAuth(request("signup", { email: address }), missing.ctx);
  assert(result?.status === 200);
  same(missing.remote.length, 1);
  same(missing.remote[0].path, "/auth/v1/otp");
  same(missing.remote[0].body, { email: address, create_user: false });
  same(missing.remote[0].headers.get("apikey"), "test-public-key");
  same(missing.remote[0].headers.get("authorization"), null);
  same(missing.challenges.size, 0);
  same(missing.sessions.size, 0);
  assert(!(await result.text()).includes("SECRET"));
});
test("unknown emails, provider errors, throttle errors and denial have generic identical body", async () => {
  const f = fixture();
  const good = await handleAuth(request("login", { email: address }), f.ctx);
  assert(good);
  const expected = await good.text();
  for (const kind of ["uninvited", "providerFailure", "throttled", "failure"]) {
    const g = fixture();
    if (kind !== "uninvited") {
      g.state[kind as "providerFailure" | "throttled" | "failure"] = true;
    }
    const res = await handleAuth(
      request("login", {
        email: kind === "uninvited" ? "unknown@example.com" : address,
      }),
      g.ctx,
    );
    assert(res?.status === 200);
    same(await res.text(), expected);
    if (kind === "uninvited" || kind === "throttled" || kind === "failure") {
      same(g.remote.length, 0);
    }
  }
});
test("all mutations reject cross-origin, missing CSRF and duplicate CSRF cookies", async () => {
  for (const path of ["login", "signup", "recover", "verify", "logout"]) {
    const f = fixture();
    const foreign = await handleAuth(
      request(path, {}, "", "https://attacker.example"),
      f.ctx,
    );
    assert(foreign?.status === 403);
    const missing = request(path, {});
    missing.headers.delete("x-csrf-token");
    assert((await handleAuth(missing, f.ctx))?.status === 403);
    const duplicate = request(path, {}, `__Host-skager_csrf=${csrf}`);
    assert((await handleAuth(duplicate, f.ctx))?.status === 403);
    same(f.calls.length, 0);
  }
});
test("challenge binds verification to browser and is consumed exactly once under concurrent replay", async () => {
  const f = fixture(), challenge = await begin(f);
  assert(
    (await handleAuth(request("verify", { code: "123456" }), f.ctx))?.status ===
      400,
  );
  const replies = await Promise.all([
    handleAuth(request("verify", { code: "123456" }, challenge), f.ctx),
    handleAuth(request("verify", { code: "123456" }, challenge), f.ctx),
  ]);
  same(replies.map((r) => r?.status).sort(), [200, 400]);
  same(f.sessions.size, 1);
});
test("canonical user mismatch and provider verification failure cannot create sessions", async () => {
  for (const kind of ["wrongUser", "providerFailure"] as const) {
    const f = fixture(), challenge = await begin(f);
    f.state[kind] = true;
    const res = await handleAuth(
      request("verify", { code: "123456" }, challenge),
      f.ctx,
    );
    assert(res?.status === 400);
    same(f.sessions.size, 0);
    assert(!(await res.text()).includes("SECRET"));
  }
});
test("rate limiting separates global20/email5 and omits untrusted IP headers", async () => {
  const f = fixture();
  f.ctx.clientIp = undefined;
  for (const ip of ["203.0.113.1", "203.0.113.2"]) {
    const req = request("login", { email: address });
    req.headers.set("x-forwarded-for", ip);
    req.headers.set("cf-connecting-ip", ip);
    await handleAuth(req, f.ctx);
  }
  const limits = f.calls.filter((c) => c.name === "skager_auth_throttle");
  same(limits.map((c) => c.args.p_limit), [20, 5, 20, 5]);
  same(limits[0].args.p_keys, ["send:global"]);
  same(limits[0].args, limits[2].args);
  same(limits[1].args, limits[3].args);
  assert(
    limits.every((c) =>
      (c.args.p_keys as string[]).every((k) => !k.includes(":ip:"))
    ),
  );
  f.ctx.clientIp = "192.0.2.20";
  await begin(f);
  const trusted = f.calls.filter((c) => c.name === "skager_auth_throttle").at(
    -1,
  )!;
  assert(
    (trusted.args.p_keys as string[]).some((k) => k.startsWith("send:ip:")),
  );
  f.state.throttled = true;
  const challenge = await begin(f);
  assert(
    (await handleAuth(request("verify", { code: "123456" }, challenge), f.ctx))
      ?.status === 400,
  );
});
test("logout revokes server session, clears cookie, and prior session is rotated on relogin", async () => {
  const f = fixture(), challenge = await begin(f);
  const logged = await handleAuth(
    request("verify", { code: "123456" }, challenge),
    f.ctx,
  );
  assert(logged);
  const old = setCookie(logged, "__Host-skager_session");
  const nextChallenge = await begin(f);
  const fresh = await handleAuth(
    request("verify", { code: "123456" }, `${nextChallenge}; ${old}`),
    f.ctx,
  );
  assert(fresh?.status === 200);
  const next = setCookie(fresh, "__Host-skager_session");
  assert(next !== old);
  same(await getAccount(request("session", undefined, old), f.ctx), null);
  const out = await handleAuth(request("logout", {}, next), f.ctx);
  assert(out?.status === 200);
  assert(out.headers.get("set-cookie")?.includes("Max-Age=0"));
  same(await getAccount(request("session", undefined, next), f.ctx), null);
});
test("disabled and revoked invitation deny existing sessions, malformed cookies do not reach DB", async () => {
  const f = fixture();
  same(
    await getAccount(
      request("session", undefined, "__Host-skager_session=bad"),
      f.ctx,
    ),
    null,
  );
  same(f.calls.length, 0);
  f.ctx.env.SKAGER_AUTH_ENABLED = "false";
  assert(
    (await handleAuth(request("login", { email: address }), f.ctx))?.status ===
      503,
  );
  same(await getAccount(request("session"), f.ctx), null);
  f.ctx.env.SKAGER_AUTH_ENABLED = "true";
  const challenge = await begin(f);
  const res = await handleAuth(
    request("verify", { code: "123456" }, challenge),
    f.ctx,
  );
  assert(res);
  const session = setCookie(res, "__Host-skager_session");
  f.ctx.env.SKAGER_INVITE_EMAILS = "";
  same(await getAccount(request("session", undefined, session), f.ctx), null);
});
test("oversize body and DB session write failure expose no credentials and establish no session", async () => {
  const f = fixture();
  const big = await handleAuth(
    request("verify", { code: "123456", extra: "x".repeat(3000) }),
    f.ctx,
  );
  assert(big?.status === 400);
  same(f.remote.length, 0);
  const challenge = await begin(f);
  f.state.badSession = true;
  const res = await handleAuth(
    request("verify", { code: "123456" }, challenge),
    f.ctx,
  );
  assert(res?.status === 400);
  assert(!res.headers.has("set-cookie"));
  assert(!(await res.text()).includes("SECRET"));
});

test("existing confirmed signup invitation conflict falls back to OTP, never creates public signup", async () => {
  const f = fixture(), original = f.ctx.fetch!;
  f.ctx.fetch = async (input, init) =>
    new URL(String(input)).pathname.endsWith("/invite")
      ? new Response('{"error":"SECRET duplicate"}', { status: 422 })
      : await original(input, init);
  const challenge = await begin(f, "signup");
  same(f.sessions.size, 0);
  same(f.remote[0].body, { email: address, create_user: false });
  const verified = await handleAuth(
    request("verify", { code: "123456" }, challenge),
    f.ctx,
  );
  assert(verified?.status === 200);
});

test("provider response parsing rejects malformed, duplicate, oversized and encoded JSON", async () => {
  const cases: (() => Response)[] = [
    () =>
      new Response("{" + '"padding":"' + "x".repeat(65536) + '"}', {
        headers: { "content-type": "application/json" },
      }),
    () =>
      new Response('{"access_token":"good","access_token":"SECRET"}', {
        headers: { "content-type": "application/json" },
      }),
    () =>
      new Response('{"access_token":"good","\\u0061ccess_token":"SECRET"}', {
        headers: { "content-type": "application/json" },
      }),
    () =>
      new Response('{"access_token":"good","n":1e999}', {
        headers: { "content-type": "application/json" },
      }),
    () =>
      new Response(
        new Uint8Array([123, 34, 120, 34, 58, 34, 0xc0, 0xaf, 34, 125]),
        { headers: { "content-type": "application/json" } },
      ),
    () =>
      new Response('{"access_token":"good"}', {
        headers: { "content-type": "text/plain" },
      }),
    () =>
      new Response('{"access_token":"good"}', {
        headers: {
          "content-type": "application/json",
          "content-encoding": "gzip",
        },
      }),
    () =>
      new Response("{}", {
        headers: {
          "content-type": "application/json",
          "content-length": "65537",
        },
      }),
    () =>
      new Response('{"access_token":"good"} trailing', {
        headers: { "content-type": "application/json" },
      }),
    () =>
      new Response("[".repeat(34) + "0" + "]".repeat(34), {
        headers: { "content-type": "application/json" },
      }),
    () => {
      const r = new Response('{"access_token":"good"}', {
        headers: { "content-type": "application/json" },
      });
      Object.defineProperty(r, "redirected", { value: true });
      return r;
    },
    () => {
      const r = new Response('{"access_token":"good"}', {
        headers: { "content-type": "application/json" },
      });
      Object.defineProperty(r, "url", {
        value: "https://other.supabase.co/auth/v1/verify",
      });
      return r;
    },
  ];
  for (const make of cases) {
    const f = fixture(), challenge = await begin(f), original = f.ctx.fetch!;
    f.ctx.fetch = async (input, init) =>
      new URL(String(input)).pathname.endsWith("/verify")
        ? make()
        : await original(input, init);
    const res = await handleAuth(
      request("verify", { code: "123456" }, challenge),
      f.ctx,
    );
    assert(res?.status === 400);
    same(f.sessions.size, 0);
    assert(!(await res.text()).includes("SECRET"));
  }
});
test("canonical identity requires authenticated confirmed non-anonymous lower-case nonzero UUID", async () => {
  const overrides: Record<string, unknown>[] = [
    { id: "00000000-0000-0000-0000-000000000000" },
    { id: "ABCDEFAB-2222-3333-4444-555555555555" },
    { id: "invalid" },
    { aud: "anon" },
    { aud: ["authenticated"] },
    { role: "service_role" },
    { role: null },
    { is_anonymous: true },
    { is_anonymous: undefined },
    { email: "FRIEND@example.com" },
    { email_confirmed_at: "invalid" },
    { email_confirmed_at: new Date(Date.now() + 86400000).toISOString() },
    { email_confirmed_at: "2026-02-30T10:00:00Z" },
    { email_confirmed_at: "2026-01-01T24:00:00Z" },
    { email_confirmed_at: null },
  ];
  for (const override of overrides) {
    const f = fixture(), challenge = await begin(f), original = f.ctx.fetch!;
    f.ctx.fetch = async (input, init) => {
      const result = await original(input, init);
      if (!new URL(String(input)).pathname.endsWith("/user")) return result;
      return new Response(
        JSON.stringify({ ...await result.json(), ...override }),
        { headers: { "content-type": "application/json" } },
      );
    };
    const res = await handleAuth(
      request("verify", { code: "123456" }, challenge),
      f.ctx,
    );
    assert(res?.status === 400);
    same(f.sessions.size, 0);
  }
});
test("provider project pin refuses other Supabase projects before transmitting API keys", async () => {
  for (
    const url of [
      "https://other.supabase.co",
      "http://kelwgubkuvditalsbqpr.supabase.co",
      "https://kelwgubkuvditalsbqpr.supabase.co/path",
      "https://kelwgubkuvditalsbqpr.supabase.co?x=1",
    ]
  ) {
    const f = fixture();
    f.ctx.env.SUPABASE_URL = url;
    await begin(f);
    same(f.remote.length, 0);
    same(f.challenges.size, 0);
  }
});
test("request JSON duplicate fields and invalid UTF-8 are rejected before provider calls", async () => {
  for (
    const raw of [
      '{"email":"friend@example.com","email":"other@example.com"}',
      new Uint8Array([123, 34, 120, 34, 58, 34, 0xc0, 0xaf, 34, 125]),
    ]
  ) {
    const f = fixture(), req = request("login", {});
    const changed = new Request(req.url, {
      method: "POST",
      headers: req.headers,
      body: raw,
    });
    const res = await handleAuth(changed, f.ctx);
    assert(res?.status === 200);
    same(f.remote.length, 0);
    same(f.calls.length, 0);
  }
});
