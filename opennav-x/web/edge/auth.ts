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
  /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i;
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
    (ctx.env.SUPABASE_INVITE_EMAILS || "").split(",").map((s) =>
      s.trim().toLowerCase()
    ).includes(value);
}
function enabled(ctx: AuthContext): boolean {
  return ctx.env.SUPABASE_AUTH_ENABLED === "true";
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
async function body(request: Request): Promise<Record<string, unknown>> {
  if (
    request.headers.get("content-type")?.split(";")[0].trim().toLowerCase() !==
      "application/json"
  ) throw new Error("invalid request");
  // Bound even chunked bodies; never include their content in exceptions.
  const reader = request.body?.getReader();
  if (!reader) throw new Error("invalid request");
  let total = 0;
  const chunks: Uint8Array[] = [];
  while (true) {
    const item = await reader.read();
    if (item.done) break;
    total += item.value.length;
    if (total > 2048) {
      await reader.cancel();
      throw new Error("invalid request");
    }
    chunks.push(item.value);
  }
  const data = new Uint8Array(total);
  let offset = 0;
  for (const part of chunks) {
    data.set(part, offset);
    offset += part.length;
  }
  const parsed = JSON.parse(new TextDecoder().decode(data));
  if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) {
    throw new Error("invalid request");
  }
  return parsed;
}
async function provider(
  ctx: AuthContext,
  path: "otp" | "verify" | "user" | "invite",
  payload?: unknown,
  accessToken?: string,
): Promise<Record<string, unknown> | null> {
  const base = new URL(ctx.env.SUPABASE_URL || "");
  if (
    base.protocol !== "https:" || !base.hostname.endsWith(".supabase.co") ||
    base.username || base.password || base.search || base.hash ||
    (base.pathname !== "/" && base.pathname !== "")
  ) return null;
  const headers: Record<string, string> = {
    "apikey": ctx.env.SUPABASE_ANON_KEY || "",
    "content-type": "application/json",
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
  if (!result.ok) return null;
  const parsed = await result.json();
  return parsed && typeof parsed === "object" && !Array.isArray(parsed)
    ? parsed as Record<string, unknown>
    : null;
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
      !found || typeof found.id !== "string" || !uuidPattern.test(found.id) ||
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
    const ipHash = await hash(ctx.clientIp || "unknown-ingress-global");
    if (route !== "/api/auth/verify") {
      const address = email(data.email);
      const challenge = randomToken();
      const keys = [
        "send:ip:" + ipHash,
        "send:email:" + await hash(address),
        "send:global",
      ];
      const allowed = await ctx.rpc("skager_auth_throttle", {
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
        "verify:ip:" + ipHash,
        "verify:challenge:" + await hash(pending),
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
      !user || typeof user.id !== "string" || !uuidPattern.test(user.id) ||
      email(user.email) !== challenge.email ||
      typeof user.email_confirmed_at !== "string"
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
