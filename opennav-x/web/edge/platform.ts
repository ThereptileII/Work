/** Supabase Edge boundary. Credentials exist only in platform secret bindings. */
export type RuntimeEnv = Record<string, string>;
export type Account = { id: string; email?: string; passwordSet?: boolean };
export type Context = {
  env: RuntimeEnv;
  fetch: typeof fetch;
  rpc: (name: string, args: Record<string, unknown>) => Promise<unknown>;
  getAccount: (request: Request) => Promise<Account | null>;
};

export const PUBLIC_ORIGIN = 'https://beta.skager.app';
export const SUPABASE_ORIGIN = 'https://kelwgubkuvditalsbqpr.supabase.co';
export const MAX_BODY = 262144;

export function json(value: unknown, status = 200): Response {
  return new Response(JSON.stringify(value), { status, headers: {
    'Content-Type': 'application/json; charset=utf-8',
    'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff',
    'Referrer-Policy': 'no-referrer',
  } });
}

export async function boundedBytes(response: Response, limit: number): Promise<Uint8Array> {
  const reader = response.body?.getReader();
  if (!reader) return new Uint8Array();
  const pieces: Uint8Array[] = [];
  let length = 0;
  try {
    while (true) {
      const part = await reader.read();
      if (part.done) break;
      length += part.value.length;
      if (length > limit) throw new Error('Response limit exceeded');
      pieces.push(part.value);
    }
  } finally {
    await reader.cancel().catch(() => {});
    reader.releaseLock();
  }
  const output = new Uint8Array(length);
  let offset = 0;
  for (const part of pieces) { output.set(part, offset); offset += part.length; }
  return output;
}

export function rpcClient(env: RuntimeEnv, transport: typeof fetch = fetch) {
  return async (name: string, args: Record<string, unknown>): Promise<unknown> => {
    if (env.SUPABASE_URL !== SUPABASE_ORIGIN || !env.SUPABASE_SERVICE_ROLE_KEY ||
        !/^(skager_auth_|commerce_|portal_|store_)[a-z0-9_]{1,80}$/.test(name)) {
      throw new Error('Backend unavailable');
    }
    try {
      const target = `${SUPABASE_ORIGIN}/rest/v1/rpc/${name}`;
      const response = await transport(target, {
        method: 'POST', redirect: 'error', signal: AbortSignal.timeout(8000),
        headers: { 'Content-Type': 'application/json',
          apikey: env.SUPABASE_SERVICE_ROLE_KEY,
          Authorization: `Bearer ${env.SUPABASE_SERVICE_ROLE_KEY}` },
        body: JSON.stringify(args),
      });
      if (!response.ok || (response.url && response.url !== target) ||
          !response.headers.get('content-type')?.startsWith('application/json')) {
        await response.body?.cancel();
        throw new Error();
      }
      return JSON.parse(new TextDecoder('utf-8', { fatal: true }).decode(await boundedBytes(response, 65536)));
    } catch { /* No provider response, request or credential is propagated. */ }
    throw new Error('Backend unavailable');
  };
}

export function cookie(request: Request, name: string): string | null {
  const candidates = (request.headers.get('cookie') || '').split(';')
    .map(part => part.trim()).filter(part => part.startsWith(`${name}=`));
  if (candidates.length !== 1) return null;
  return candidates[0].slice(name.length + 1);
}

export function newToken(): string {
  return Array.from(crypto.getRandomValues(new Uint8Array(32)), b => b.toString(16).padStart(2, '0')).join('');
}

export async function csrfValid(request: Request): Promise<boolean> {
  if (request.headers.get('origin') !== PUBLIC_ORIGIN) return false;
  const expected = cookie(request, '__Host-skager_csrf');
  const supplied = request.headers.get('X-CSRF-Token');
  if (!expected || !supplied || !/^[a-f0-9]{64}$/.test(expected) || !/^[a-f0-9]{64}$/.test(supplied)) return false;
  // Hash equal-sized strings before a fixed-length comparison.
  const [a, b] = await Promise.all([expected, supplied].map(v =>
    crypto.subtle.digest('SHA-256', new TextEncoder().encode(v)).then(v => new Uint8Array(v))));
  let difference = 0;
  for (let i = 0; i < a.length; ++i) difference |= a[i] ^ b[i];
  return difference === 0;
}

export async function bootstrapCsrf(request: Request, response: Response): Promise<Response> {
  if (!response.ok) return response;
  const existing = cookie(request, '__Host-skager_csrf');
  const token = existing && /^[a-f0-9]{64}$/.test(existing) ? existing : newToken();
  const payload = await response.json();
  const result = json({ ...payload, csrfToken: token });
  for (const value of response.headers.getSetCookie()) result.headers.append('Set-Cookie', value);
  result.headers.append('Set-Cookie', `__Host-skager_csrf=${token}; Secure; SameSite=Lax; Path=/; Max-Age=1200`);
  return result;
}
