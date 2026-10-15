/** Server-only Supabase identity with opaque, bounded sessions.
 * Accounts are created only by a purchase: the buyer confirms an emailed code and
 * chooses a password. Later sign-ins need the password and then an emailed code. */
export type Account = { id: string; email: string; passwordSet: boolean };
export type AuthContext = {
  env: Record<string, string | undefined>;
  rpc: (name: string, args: Record<string, unknown>) => Promise<unknown>;
  fetch?: typeof fetch;
  /** Trusted ingress metadata only; never read client-supplied forwarding headers. */
  clientIp?: string;
};
const SESSION = "__Host-skager_session";
const CHALLENGE = "__Host-skager_challenge";
const GENERIC = "If this address has a SKAGER account, we have sent it a code.";
const WRONG = "Email or password is incorrect.";
const CLOSED = "This account has been closed. Contact us if you think this is a mistake.";
const INVALID =
  "The code could not be verified. Request a new code and try again.";
const tokenPattern = /^[a-f0-9]{64}$/;
const uuidPattern =
  /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/;
export function readCookie(request: Request, name: string): string {
  const matches = (request.headers.get("cookie") || "").split(";").map((s) =>
    s.trim()
  ).filter((s) => s.startsWith(name + "="));
  return matches.length === 1 ? matches[0].slice(name.length + 1) : "";
}
function response(
  body: unknown,
  status = 200,
  cookies: string[] = [],
): Response {
  const headers = new Headers({
    "content-type": "application/json; charset=utf-8",
    "cache-control": "no-store",
    "x-content-type-options": "nosniff",
  });
  for (const cookie of cookies) headers.append("set-cookie", cookie);
  return new Response(JSON.stringify(body), { status, headers });
}
function cookie(name: string, value: string, age: number): string {
  return `${name}=${value}; Path=/; Max-Age=${age}; HttpOnly; Secure; SameSite=Lax`;
}
function randomToken(): string {
  return Array.from(
    crypto.getRandomValues(new Uint8Array(32)),
    (b) => b.toString(16).padStart(2, "0"),
  ).join("");
}
async function hash(value: string): Promise<string> {
  return Array.from(
    new Uint8Array(
      await crypto.subtle.digest("SHA-256", new TextEncoder().encode(value)),
    ),
    (b) => b.toString(16).padStart(2, "0"),
  ).join("");
}
function email(value: unknown): string {
  return typeof value === "string" && value.length <= 254
    ? value.trim().toLowerCase()
    : "";
}
function enabled(ctx: AuthContext): boolean {
  return ctx.env.SKAGER_AUTH_ENABLED === "true";
}
function equal(a: string, b: string): boolean {
  if (a.length !== b.length) return false;
  let diff = 0;
  for (let i = 0; i < a.length; i++) diff |= a.charCodeAt(i) ^ b.charCodeAt(i);
  return diff === 0;
}
function mutationAllowed(request: Request): boolean {
  const csrf = readCookie(request, "__Host-skager_csrf");
  return request.headers.get("origin") === "https://beta.skager.app" &&
    /^[a-f0-9]{64}$/.test(csrf) &&
    equal(csrf, request.headers.get("x-csrf-token") || "");
}
/** Reject duplicate keys and non-finite numbers instead of JSON.parse's last-key wins. */
function strictJson(text: string): unknown {
  let i = 0;
  const fail = (): never => {
    throw new Error("invalid JSON");
  };
  const whitespace = () => {
    while (i < text.length && /[ \t\r\n]/.test(text[i])) i++;
  };
  function string(): string {
    const start = i++;
    while (i < text.length) {
      const c = text[i++];
      if (c === '"') {
        try {
          return JSON.parse(text.slice(start, i));
        } catch {
          return fail();
        }
      }
      if (c === "\\") {
        if (i >= text.length) fail();
        i++;
      } else if (c.charCodeAt(0) < 32) fail();
    }
    return fail();
  }
  function value(depth: number): unknown {
    if (depth > 32) fail();
    whitespace();
    if (text[i] === '"') return string();
    if (text[i] === "{") {
      i++;
      whitespace();
      const out: Record<string, unknown> = Object.create(null);
      if (text[i] === "}") {
        i++;
        return out;
      }
      while (true) {
        if (text[i] !== '"') fail();
        const key = string();
        whitespace();
        if (text[i++] !== ":" || Object.hasOwn(out, key)) fail();
        out[key] = value(depth + 1);
        whitespace();
        const next = text[i++];
        if (next === "}") return out;
        if (next !== ",") fail();
        whitespace();
      }
    }
    if (text[i] === "[") {
      i++;
      whitespace();
      const out: unknown[] = [];
      if (text[i] === "]") {
        i++;
        return out;
      }
      while (true) {
        out.push(value(depth + 1));
        whitespace();
        const next = text[i++];
        if (next === "]") return out;
        if (next !== ",") fail();
      }
    }
    for (
      const [literal, parsed] of [["true", true], ["false", false], [
        "null",
        null,
      ]] as const
    ) {
      if (text.startsWith(literal, i)) {
        i += literal.length;
        return parsed;
      }
    }
    const number = /^-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?/.exec(
      text.slice(i),
    );
    if (!number) return fail();
    i += number[0].length;
    const parsed = Number(number[0]);
    if (!Number.isFinite(parsed)) fail();
    return parsed;
  }
  const parsed = value(0);
  whitespace();
  if (i !== text.length) fail();
  return parsed;
}
async function readJson(
  message: Request | Response,
  max: number,
): Promise<Record<string, unknown>> {
  if (
    message.headers.get("content-type")?.split(";")[0].trim().toLowerCase() !==
      "application/json"
  ) throw new Error("invalid JSON response");
  const encoding = message.headers.get("content-encoding");
  if (encoding && encoding.toLowerCase() !== "identity") {
    throw new Error("invalid JSON encoding");
  }
  const declared = message.headers.get("content-length");
  if (declared && (!/^\d+$/.test(declared) || Number(declared) > max)) {
    throw new Error("invalid JSON size");
  }
  const reader = message.body?.getReader();
  if (!reader) throw new Error("invalid JSON body");
  let total = 0;
  const chunks: Uint8Array[] = [];
  while (true) {
    const item = await reader.read();
    if (item.done) break;
    total += item.value.length;
    if (total > max) {
      await reader.cancel();
      throw new Error("invalid JSON size");
    }
    chunks.push(item.value);
  }
  const data = new Uint8Array(total);
  let offset = 0;
  for (const chunk of chunks) {
    data.set(chunk, offset);
    offset += chunk.length;
  }
  const parsed = strictJson(
    new TextDecoder("utf-8", { fatal: true, ignoreBOM: true }).decode(data),
  );
  if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) {
    throw new Error("invalid JSON shape");
  }
  return parsed as Record<string, unknown>;
}
function canonicalId(value: unknown): value is string {
  return typeof value === "string" && uuidPattern.test(value) &&
    value !== "00000000-0000-0000-0000-000000000000";
}
function confirmedTimestamp(value: unknown): boolean {
  if (typeof value !== "string") return false;
  const parts =
    /^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})(?:\.\d{1,6})?(?:Z|[+-](?:[01]\d|2[0-3]):[0-5]\d)$/
      .exec(value);
  if (!parts) return false;
  const [, year, month, day, hour, minute, second] = parts.map(Number);
  const stamp = Date.parse(value);
  return month >= 1 && month <= 12 && day >= 1 &&
    day <= new Date(Date.UTC(year, month, 0)).getUTCDate() && hour <= 23 &&
    minute <= 59 && second <= 59 && Number.isFinite(stamp) && stamp > 0 &&
    stamp <= Date.now();
}
async function body(request: Request): Promise<Record<string, unknown>> {
  return await readJson(request, 2048);
}
type ProviderCall = {
  method: "GET" | "POST" | "PUT";
  path: string;
  payload?: unknown;
  service?: boolean;
  accessToken?: string;
};
async function provider(ctx: AuthContext, call: ProviderCall): Promise<Record<string, unknown> | null> {
  const base = new URL(ctx.env.SUPABASE_URL || "");
  if (base.href !== "https://kelwgubkuvditalsbqpr.supabase.co/") return null;
  // Admin calls use the server identity; OTP, password and user checks use the public API key.
  const apiKey = call.service ? ctx.env.SUPABASE_SERVICE_ROLE_KEY : ctx.env.SUPABASE_ANON_KEY;
  if (!apiKey) return null;
  const headers: Record<string, string> = {
    "apikey": apiKey,
    "content-type": "application/json",
    "accept-encoding": "identity",
  };
  if (call.service) headers.authorization = `Bearer ${apiKey}`;
  else if (call.accessToken) headers.authorization = `Bearer ${call.accessToken}`;
  const target = new URL(`/auth/v1/${call.path}`, base);
  const result = await (ctx.fetch || fetch)(target, {
    method: call.method,
    headers,
    ...(call.payload ? { body: JSON.stringify(call.payload) } : {}),
    redirect: "error",
    signal: AbortSignal.timeout(8000),
  });
  if (!result.ok || result.redirected || (result.url && result.url !== target.href)) {
    await result.body?.cancel().catch(() => {});
    return null;
  }
  return await readJson(result, 65536);
}
function row(value: unknown): Record<string, unknown> | null {
  const found = Array.isArray(value) ? value[0] : value;
  return found && typeof found === "object"
    ? found as Record<string, unknown>
    : null;
}
function validEmail(value: string): boolean {
  return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(value);
}
export function validPassword(value: unknown): value is string {
  return typeof value === "string" && value.length >= 10 && value.length <= 128 &&
    value.trim().length >= 10 && !/[\u0000-\u001f\u007f]/.test(value);
}
async function passwordValid(ctx: AuthContext, address: string, password: string): Promise<boolean> {
  if (typeof password !== "string" || !password || password.length > 128) return false;
  const result = await provider(ctx, {
    method: "POST", path: "token?grant_type=password", payload: { email: address, password },
  });
  // The tokens are discarded: SKAGER keeps its own opaque session.
  return !!result && typeof result.access_token === "string";
}
async function sendThrottled(ctx: AuthContext, address: string): Promise<boolean> {
  const ipHash = ctx.clientIp ? await hash(ctx.clientIp) : null;
  const global = await ctx.rpc("skager_auth_throttle", { p_keys: ["send:global"], p_limit: 60, p_seconds: 900 });
  return global === true && await ctx.rpc("skager_auth_throttle", {
    p_keys: ["send:email:" + await hash(address), ...(ipHash ? ["send:ip:" + ipHash] : [])],
    p_limit: 5, p_seconds: 900,
  }) === true;
}
/** Send a code and bind a fresh challenge to this browser. */
async function issueCode(request: Request, ctx: AuthContext, address: string,
  purpose: "login" | "setup" | "reset"): Promise<string | null> {
  const pending = readCookie(request, CHALLENGE);
  if (tokenPattern.test(pending)) await ctx.rpc("store_challenge_take", { p_hash: await hash(pending) });
  // A buyer without an Auth identity is invited (this creates it, unconfirmed);
  // an existing identity receives an ordinary code. Public signup stays disabled.
  const invited = purpose === "setup"
    ? await provider(ctx, { method: "POST", path: "invite", payload: { email: address }, service: true })
    : null;
  const sent = invited ||
    await provider(ctx, { method: "POST", path: "otp", payload: { email: address, create_user: false } });
  if (!sent) return null;
  const challenge = randomToken();
  await ctx.rpc("store_challenge_create", { p_hash: await hash(challenge), p_email: address, p_purpose: purpose });
  return cookie(CHALLENGE, challenge, 600);
}
function mask(address: string): string {
  const [name, domain] = address.split("@");
  return `${name.slice(0, 2)}${"•".repeat(Math.max(1, Math.min(6, name.length - 2)))}@${domain}`;
}
export async function getAccount(
  request: Request,
  ctx: AuthContext,
): Promise<Account | null> {
  if (!enabled(ctx)) return null;
  const raw = readCookie(request, SESSION);
  if (!tokenPattern.test(raw)) return null;
  try {
    // Resolves only for an active (not refunded) customer with a confirmed identity.
    const found = row(await ctx.rpc("store_session_get", { p_hash: await hash(raw) }));
    if (!found || !canonicalId(found.id) || typeof found.email !== "string" ||
      typeof found.password_set !== "boolean") return null;
    return { id: found.id, email: email(found.email), passwordSet: found.password_set };
  } catch {
    return null;
  }
}
function isAdmin(account: Account, ctx: AuthContext): boolean {
  return (ctx.env.SKAGER_ADMIN_EMAILS || "").split(",").map((s) => s.trim().toLowerCase())
    .filter(Boolean).includes(account.email);
}
const ROUTES = ["/api/auth/session", "/api/auth/login", "/api/auth/claim", "/api/auth/forgot",
  "/api/auth/verify", "/api/auth/password", "/api/auth/logout"];
export async function handleAuth(
  request: Request,
  ctx: AuthContext,
): Promise<Response | null> {
  const route = new URL(request.url).pathname;
  if (!ROUTES.includes(route)) return null;
  if (route === "/api/auth/session" && request.method === "GET") {
    const account = await getAccount(request, ctx);
    return response({ account: account ? { ...account, admin: isAdmin(account, ctx) } : null });
  }
  if (request.method !== "POST" || route === "/api/auth/session") {
    return response({ error: "Method not allowed." }, 405);
  }
  if (!mutationAllowed(request)) {
    return response({ error: "Request could not be verified." }, 403);
  }
  if (!enabled(ctx)) {
    return response({ error: "Sign-in is not available." }, 503);
  }
  try {
    if (route === "/api/auth/logout") {
      const raw = readCookie(request, SESSION);
      if (tokenPattern.test(raw)) await ctx.rpc("skager_auth_session_delete", { p_hash: await hash(raw) });
      const pending = readCookie(request, CHALLENGE);
      if (tokenPattern.test(pending)) await ctx.rpc("store_challenge_take", { p_hash: await hash(pending) });
      return response({ ok: true }, 200, [cookie(SESSION, "", 0), cookie(CHALLENGE, "", 0)]);
    }
    const data = await body(request);
    if (route === "/api/auth/login") {
      const address = email(data.email);
      if (Object.keys(data).some((key) => !["email", "password"].includes(key)) || !validEmail(address) ||
        typeof data.password !== "string") return response({ error: WRONG }, 400);
      if (!await sendThrottled(ctx, address)) {
        return response({ error: "Too many attempts. Please wait a few minutes and try again." }, 429);
      }
      const state = row(await ctx.rpc("store_customer_state", { p_email: address }));
      const known = !!state && (state.state === "active" || state.state === "revoked") && state.claimed === true;
      // The password is checked for every known account, so a closed account only
      // reveals itself to someone who already knows its password.
      if (!known || !await passwordValid(ctx, address, data.password)) return response({ error: WRONG }, 400);
      if (state!.state === "revoked") return response({ error: CLOSED }, 403);
      const challenge = await issueCode(request, ctx, address, "login");
      if (!challenge) return response({ error: "We could not send your code. Please try again." }, 503);
      return response({ ok: true, email: mask(address) }, 200, [challenge]);
    }
    if (route === "/api/auth/claim") {
      if (Object.keys(data).length !== 1 || !(typeof data.sessionId === "string" || typeof data.email === "string")) {
        return response({ error: "Request could not be completed." }, 400);
      }
      if (typeof data.sessionId === "string") {
        if (!/^cs_test_[A-Za-z0-9]{8,200}$/.test(data.sessionId)) return response({ state: "unknown" }, 404);
        const state = row(await ctx.rpc("store_claim_state", { p_session_id: data.sessionId, p_email: null }));
        if (!state || typeof state.state !== "string") throw new Error("claim unavailable");
        if (state.state === "pending") return response({ state: "pending" }, 202);
        if (["done", "revoked"].includes(state.state)) return response({ state: state.state });
        if (state.state !== "ready" || typeof state.email !== "string") return response({ state: "unknown" }, 404);
        if (!await sendThrottled(ctx, state.email)) return response({ state: "throttled" }, 429);
        const challenge = await issueCode(request, ctx, state.email, "setup");
        if (!challenge) return response({ error: "We could not send your code. Please try again." }, 503);
        return response({ state: "code_sent", email: mask(state.email) }, 200, [challenge]);
      }
      // By email: the same answer for everyone, so purchases cannot be discovered.
      const address = email(data.email);
      let challenge: string | null = null;
      if (validEmail(address) && await sendThrottled(ctx, address)) {
        const state = row(await ctx.rpc("store_claim_state", { p_session_id: null, p_email: address }));
        if (state?.state === "ready") challenge = await issueCode(request, ctx, address, "setup");
      }
      return response({ state: "code_sent_if_eligible", message: GENERIC }, 200,
        [challenge ?? cookie(CHALLENGE, randomToken(), 600)]);
    }
    if (route === "/api/auth/forgot") {
      const address = email(data.email);
      let challenge: string | null = null;
      if (Object.keys(data).length === 1 && validEmail(address) && await sendThrottled(ctx, address)) {
        const state = row(await ctx.rpc("store_customer_state", { p_email: address }));
        if (state?.state === "active" && state.claimed === true) challenge = await issueCode(request, ctx, address, "reset");
      }
      return response({ ok: true, message: GENERIC }, 200, [challenge ?? cookie(CHALLENGE, randomToken(), 600)]);
    }
    if (route === "/api/auth/password") {
      const account = await getAccount(request, ctx);
      if (!account) return response({ error: "Please sign in again." }, 401);
      if (Object.keys(data).some((key) => !["password", "current"].includes(key)) || !validPassword(data.password)) {
        return response({ error: "Choose a password of at least 10 characters." }, 400);
      }
      // Changing an existing password needs the current one; setup and reset follow a verified code.
      if (account.passwordSet && !await passwordValid(ctx, account.email, String(data.current ?? ""))) {
        return response({ error: "Your current password is incorrect." }, 400);
      }
      const updated = await provider(ctx, {
        method: "PUT", path: `admin/users/${account.id}`, payload: { password: data.password }, service: true,
      });
      if (!updated || updated.id !== account.id) return response({ error: "Your password could not be saved. Please try again." }, 503);
      await ctx.rpc("store_password_set", { p_account_id: account.id });
      return response({ ok: true });
    }
    // Verify a code.
    const pending = readCookie(request, CHALLENGE);
    const ipHash = ctx.clientIp ? await hash(ctx.clientIp) : null;
    if (
      !tokenPattern.test(pending) || typeof data.code !== "string" ||
      !/^\d{6,10}$/.test(data.code)
    ) return response({ error: INVALID }, 400);
    const allowed = await ctx.rpc("skager_auth_throttle", {
      p_keys: [
        "verify:challenge:" + await hash(pending),
        ...(ipHash ? ["verify:ip:" + ipHash] : []),
      ],
      p_limit: 10,
      p_seconds: 900,
    });
    if (allowed !== true) return response({ error: INVALID }, 400);
    // Consume before calling the provider: retries/races/replay cannot create another session.
    const challenge = row(await ctx.rpc("store_challenge_take", { p_hash: await hash(pending) }));
    if (
      !challenge || typeof challenge.email !== "string" ||
      !["login", "setup", "reset"].includes(String(challenge.purpose))
    ) return response({ error: INVALID }, 400, [cookie(CHALLENGE, "", 0)]);
    const verified = await provider(ctx, {
      method: "POST", path: "verify", payload: { email: challenge.email, token: data.code, type: "email" },
    });
    if (!verified || typeof verified.access_token !== "string") {
      return response({ error: INVALID }, 400, [cookie(CHALLENGE, "", 0)]);
    }
    const user = await provider(ctx, { method: "GET", path: "user", accessToken: verified.access_token });
    if (
      !user || !canonicalId(user.id) || user.email !== challenge.email ||
      user.aud !== "authenticated" || user.role !== "authenticated" ||
      user.is_anonymous !== false ||
      !confirmedTimestamp(user.email_confirmed_at)
    ) return response({ error: INVALID }, 400, [cookie(CHALLENGE, "", 0)]);
    // Binds the subject to the customer; a refunded (closed) customer is refused here.
    let linked: Record<string, unknown> | null;
    try {
      linked = row(await ctx.rpc("store_link_account", {
        p_email: challenge.email, p_account_id: user.id, p_purpose: challenge.purpose,
      }));
    } catch {
      return response({ error: CLOSED }, 403, [cookie(CHALLENGE, "", 0)]);
    }
    const session = randomToken();
    const old = readCookie(request, SESSION);
    if (tokenPattern.test(old)) await ctx.rpc("skager_auth_session_delete", { p_hash: await hash(old) });
    if (await ctx.rpc("skager_auth_session_create", { p_hash: await hash(session), p_subject: user.id }) !== true) {
      throw new Error("session unavailable");
    }
    const passwordSet = linked?.password_set === true;
    return response({ ok: true, next: passwordSet ? "/portal/" : "/portal/account/?setup=1" }, 200, [
      cookie(SESSION, session, 1200),
      cookie(CHALLENGE, "", 0),
    ]);
  } catch {
    // Provider/DB/body errors are deliberately never serialized or logged.
    if (route === "/api/auth/forgot" || route === "/api/auth/claim") {
      return response({ ok: true, message: GENERIC }, 200, [cookie(CHALLENGE, randomToken(), 600)]);
    }
    return response({
      error: route === "/api/auth/verify" ? INVALID : "Request could not be completed. Please try again.",
    }, route === "/api/auth/verify" ? 400 : 503);
  }
}
