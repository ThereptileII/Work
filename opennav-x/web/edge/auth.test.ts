import { type AuthContext, getAccount, handleAuth } from "./auth.ts";
// Runs with `deno test auth.test.ts`; no network permission or third-party modules.
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
      SUPABASE_AUTH_ENABLED: "true",
      SUPABASE_INVITE_EMAILS: address,
      SUPABASE_URL: "https://example.supabase.co",
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
          email_confirmed_at: "2026-10-13T10:00:00Z",
        }
        : {};
      return new Response(JSON.stringify(data), { status: 200 });
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
Deno.test("allowlisted OTP login verifies canonical user then issues opaque bounded session", async () => {
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
  assert(!JSON.stringify(f.calls).includes("SECRET"));
  assert(!JSON.stringify(f.calls).includes(session.split("=")[1]));
  assert(!(await res.text()).includes("SECRET"));
});
Deno.test("signup sends privileged unconfirmed invitation; recovery remains existing-user OTP only", async () => {
  const f = fixture();
  const challenge = await begin(f, "signup");
  await begin(f, "recover");
  same(f.remote.map((c) => c.path), ["/auth/v1/invite", "/auth/v1/otp"]);
  same(f.remote[0].body, { email: address });
  same(f.remote[1].body, { email: address, create_user: false });
  same(f.remote[0].headers.get("authorization"), "Bearer SECRET service role");
  same(f.sessions.size, 0);
  const denied = await handleAuth(
    request("signup", { email: "stranger@example.com" }),
    f.ctx,
  );
  assert(denied?.status === 200);
  same(f.remote.length, 2);
  assert(challenge.startsWith("__Host-skager_challenge="));
});
Deno.test("unknown emails, provider errors, throttle errors and denial have generic identical body", async () => {
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
Deno.test("all mutations reject cross-origin, missing CSRF and duplicate CSRF cookies", async () => {
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
Deno.test("challenge binds verification to browser and is consumed exactly once under concurrent replay", async () => {
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
Deno.test("canonical user mismatch and provider verification failure cannot create sessions", async () => {
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
Deno.test("rate limiting uses trusted ingress only and falls back to a global shared bucket", async () => {
  const f = fixture();
  f.ctx.clientIp = undefined;
  const a = request("login", { email: address });
  a.headers.set("x-forwarded-for", "203.0.113.1");
  await handleAuth(a, f.ctx);
  const b = request("login", { email: address });
  b.headers.set("x-forwarded-for", "203.0.113.2");
  await handleAuth(b, f.ctx);
  same(
    f.calls.filter((c) => c.name === "skager_auth_throttle").map((c) =>
      (c.args.p_keys as string[])[0]
    )[0],
    f.calls.filter((c) => c.name === "skager_auth_throttle").map((c) =>
      (c.args.p_keys as string[])[0]
    )[1],
  );
  f.state.throttled = true;
  const challenge = await begin(f);
  assert(
    (await handleAuth(request("verify", { code: "123456" }, challenge), f.ctx))
      ?.status === 400,
  );
});
Deno.test("logout revokes server session, clears cookie, and prior session is rotated on relogin", async () => {
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
Deno.test("disabled and revoked invitation deny existing sessions, malformed cookies do not reach DB", async () => {
  const f = fixture();
  same(
    await getAccount(
      request("session", undefined, "__Host-skager_session=bad"),
      f.ctx,
    ),
    null,
  );
  same(f.calls.length, 0);
  f.ctx.env.SUPABASE_AUTH_ENABLED = "false";
  assert(
    (await handleAuth(request("login", { email: address }), f.ctx))?.status ===
      503,
  );
  same(await getAccount(request("session"), f.ctx), null);
  f.ctx.env.SUPABASE_AUTH_ENABLED = "true";
  const challenge = await begin(f);
  const res = await handleAuth(
    request("verify", { code: "123456" }, challenge),
    f.ctx,
  );
  assert(res);
  const session = setCookie(res, "__Host-skager_session");
  f.ctx.env.SUPABASE_INVITE_EMAILS = "";
  same(await getAccount(request("session", undefined, session), f.ctx), null);
});
Deno.test("oversize body and DB session write failure expose no credentials and establish no session", async () => {
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

Deno.test("existing confirmed signup invitation conflict falls back to OTP, never creates public signup", async () => {
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
