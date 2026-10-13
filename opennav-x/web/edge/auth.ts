/** SCRUM-40: server-only Supabase email OTP and opaque, bounded sessions. */
export type Account = { id: string; email: string };
export type AuthContext = {
  env: Record<string, string | undefined>;
  rpc: (name: string, args: Record<string, unknown>) => Promise<unknown>;
  fetch?: typeof fetch;
  /** Trusted ingress metadata only; never read client-supplied forwarding headers. */
  clientIp?: string;
};
const SESSION = "__Host-skager_session";
const CHALLENGE = "__Host-skager_challenge";
const GENERIC = "If this address is eligible, a sign-in code has been sent.";
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
function invited(value: string, ctx: AuthContext): boolean {
  return !!value &&
    (ctx.env.SKAGER_INVITE_EMAILS || "").split(",").map((s) =>
      s.trim().toLowerCase()
    ).includes(value);
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
async function provider(
  ctx: AuthContext,
  path: "otp" | "verify" | "user" | "invite",
  payload?: unknown,
  accessToken?: string,
): Promise<Record<string, unknown> | null> {
  const base = new URL(ctx.env.SUPABASE_URL || "");
  if (base.href !== "https://kelwgubkuvditalsbqpr.supabase.co/") return null;
  const headers: Record<string, string> = {
    "apikey": ctx.env.SUPABASE_ANON_KEY || "",
    "content-type": "application/json",
    "accept-encoding": "identity",
  };
  if (!headers.apikey) return null;
  if (path === "invite") {
    const serviceKey = ctx.env.SUPABASE_SERVICE_ROLE_KEY;
    if (!serviceKey) return null;
    headers.authorization = `Bearer ${serviceKey}`;
  } else if (accessToken) headers.authorization = `Bearer ${accessToken}`;
  const result = await (ctx.fetch || fetch)(new URL(`/auth/v1/${path}`, base), {
    method: path === "user" ? "GET" : "POST",
    headers,
    ...(payload ? { body: JSON.stringify(payload) } : {}),
    redirect: "error",
    signal: AbortSignal.timeout(8000),
  });
  if (
    !result.ok || result.redirected ||
    (result.url && result.url !== new URL(`/auth/v1/${path}`, base).href)
  ) return null;
  return await readJson(result, 65536);
}
function row(value: unknown): Record<string, unknown> | null {
  const found = Array.isArray(value) ? value[0] : value;
  return found && typeof found === "object"
    ? found as Record<string, unknown>
    : null;
}
export async function getAccount(
  request: Request,
  ctx: AuthContext,
): Promise<Account | null> {
  if (!enabled(ctx)) return null;
  const raw = readCookie(request, SESSION);
  if (!tokenPattern.test(raw)) return null;
  try {
    const found = row(
      await ctx.rpc("skager_auth_session_get", { p_hash: await hash(raw) }),
    );
    if (
      !found || !canonicalId(found.id) ||
      typeof found.email !== "string" || !invited(email(found.email), ctx)
    ) return null;
    return { id: found.id, email: email(found.email) };
  } catch {
    return null;
  }
}
export async function handleAuth(
  request: Request,
  ctx: AuthContext,
): Promise<Response | null> {
  const route = new URL(request.url).pathname;
  if (
    ![
      "/api/auth/session",
      "/api/auth/login",
      "/api/auth/signup",
      "/api/auth/recover",
      "/api/auth/verify",
      "/api/auth/logout",
    ].includes(route)
  ) return null;
  if (route === "/api/auth/session" && request.method === "GET") {
    return response({ account: await getAccount(request, ctx) });
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
      if (tokenPattern.test(raw)) {
        await ctx.rpc("skager_auth_session_delete", {
          p_hash: await hash(raw),
        });
      }
      const pending = readCookie(request, CHALLENGE);
      if (tokenPattern.test(pending)) {
        await ctx.rpc("skager_auth_challenge_take", {
          p_hash: await hash(pending),
        });
      }
      return response({ ok: true }, 200, [
        cookie(SESSION, "", 0),
        cookie(CHALLENGE, "", 0),
      ]);
    }
    const data = await body(request);
    const pending = readCookie(request, CHALLENGE);
    const ipHash = ctx.clientIp ? await hash(ctx.clientIp) : null;
    if (route !== "/api/auth/verify") {
      const address = email(data.email);
      const challenge = randomToken();
      const globalAllowed = await ctx.rpc("skager_auth_throttle", {
        p_keys: ["send:global"],
        p_limit: 20,
        p_seconds: 900,
      });
      const keys = [
        "send:email:" + await hash(address),
        ...(ipHash ? ["send:ip:" + ipHash] : []),
      ];
      const allowed = globalAllowed === true &&
        await ctx.rpc("skager_auth_throttle", {
          p_keys: keys,
          p_limit: 5,
          p_seconds: 900,
        });
      // Every begin response is identical, including denied invitations/provider errors.
      if (
        allowed === true && invited(address, ctx) &&
        /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(address)
      ) {
        if (tokenPattern.test(pending)) {
          await ctx.rpc("skager_auth_challenge_take", {
            p_hash: await hash(pending),
          });
        }
        // Public Supabase signups remain disabled. A privileged invite sends a
        // confirmation code without auto-confirming email or granting a session.
        // Confirmed existing users may receive an ordinary OTP on the signup form.
        const invitation = route === "/api/auth/signup"
          ? await provider(ctx, "invite", { email: address })
          : null;
        const sent = invitation ||
          await provider(ctx, "otp", { email: address, create_user: false });
        if (sent) {
          await ctx.rpc("skager_auth_challenge_create", {
            p_hash: await hash(challenge),
            p_email: address,
          });
        }
      }
      return response({ ok: true, message: GENERIC }, 200, [
        cookie(CHALLENGE, challenge, 600),
      ]);
    }
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
    const challenge = row(
      await ctx.rpc("skager_auth_challenge_take", {
        p_hash: await hash(pending),
      }),
    );
    if (
      !challenge || typeof challenge.email !== "string" ||
      !invited(challenge.email, ctx)
    ) return response({ error: INVALID }, 400, [cookie(CHALLENGE, "", 0)]);
    const verified = await provider(ctx, "verify", {
      email: challenge.email,
      token: data.code,
      type: "email",
    });
    if (!verified || typeof verified.access_token !== "string") {
      return response({ error: INVALID }, 400, [cookie(CHALLENGE, "", 0)]);
    }
    const user = await provider(ctx, "user", undefined, verified.access_token);
    if (
      !user || !canonicalId(user.id) || user.email !== challenge.email ||
      user.aud !== "authenticated" || user.role !== "authenticated" ||
      user.is_anonymous !== false ||
      !confirmedTimestamp(user.email_confirmed_at)
    ) return response({ error: INVALID }, 400, [cookie(CHALLENGE, "", 0)]);
    const session = randomToken();
    const old = readCookie(request, SESSION);
    if (tokenPattern.test(old)) {
      await ctx.rpc("skager_auth_session_delete", { p_hash: await hash(old) });
    }
    if (
      await ctx.rpc("skager_auth_session_create", {
        p_hash: await hash(session),
        p_subject: user.id,
      }) !== true
    ) throw new Error("session unavailable");
    return response({ ok: true }, 200, [
      cookie(SESSION, session, 1200),
      cookie(CHALLENGE, "", 0),
    ]);
  } catch {
    // Provider/DB/body errors are deliberately never serialized or logged.
    if (
      ["/api/auth/login", "/api/auth/signup", "/api/auth/recover"].includes(
        route,
      )
    ) {
      return response({ ok: true, message: GENERIC }, 200, [
        cookie(CHALLENGE, randomToken(), 600),
      ]);
    }
    return response({
      error: route === "/api/auth/verify"
        ? INVALID
        : "Request could not be completed.",
    }, 400);
  }
}
