// The real SKAGER Cloud request path for local integration tests and the browser
// preview: Cloudflare Worker -> skager-web edge router -> RPCs executed by psql in
// a disposable PostgreSQL that has the repository's SQL applied (PostgREST's
// named-argument RPC call is emulated). Never points at a hosted database.
import { execFileSync } from 'node:child_process';
import { readdirSync, readFileSync } from 'node:fs';
import { registerHooks } from 'node:module';
import { fileURLToPath } from 'node:url';

export const EDGE_DIR = fileURLToPath(new URL('../../../web/edge/', import.meta.url));
const SUPABASE = 'https://kelwgubkuvditalsbqpr.supabase.co';

/** SKAGER_E2E_PSQL: psql connection arguments, e.g. "-h /tmp/pg -p 55602 -U postgres -d scrum94_schema". */
export function psqlArgs(): string[] | null {
  const raw = process.env.SKAGER_E2E_PSQL;
  return raw ? raw.split(/\s+/).filter(Boolean) : null;
}
export function sql(text: string): string {
  return execFileSync(process.env.SKAGER_E2E_PSQL_BIN || 'psql', [...psqlArgs()!, '-qtAX', '-v', 'ON_ERROR_STOP=1', '-c', text],
    { encoding: 'utf8' }).trim();
}

const literal = (value: unknown): string => {
  if (value === null || value === undefined) return 'null';
  if (typeof value === 'boolean' || typeof value === 'number') return String(value);
  if (typeof value === 'string') return `$q$${value}$q$`;
  if (Array.isArray(value)) return value.every(v => typeof v === 'string') && value.length
    ? `ARRAY[${value.map(literal).join(',')}]::text[]` : `$q$${JSON.stringify(value)}$q$::jsonb`;
  return `$q$${JSON.stringify(value)}$q$::jsonb`;
};
const kinds = new Map<string, { set: boolean; void: boolean; args: Map<string, string> }>();
function postgrest(name: string, args: Record<string, unknown>): unknown {
  if (!kinds.has(name)) {
    const [set, rettype, names, types] = sql(`select proretset, prorettype::regtype, array_to_string(proargnames, ','),
      oidvectortypes(proargtypes) from pg_proc where proname = '${name}' and pronamespace = 'public'::regnamespace`).split('|');
    if (!rettype) throw new Error('No RPC ' + name);
    const typeList = (types || '').split(', ');
    kinds.set(name, { set: set === 't', void: rettype === 'void',
      args: new Map((names || '').split(',').filter(Boolean).map((n, i) => [n, typeList[i]])) });
  }
  const kind = kinds.get(name)!;
  const call = `public.${name}(${Object.entries(args).map(([k, v]) => {
    const type = kind.args.get(k);
    return `${k} => ${v === null ? `null::${type}` : type === 'jsonb' ? `$q$${JSON.stringify(v)}$q$::jsonb` : literal(v)}`;
  }).join(', ')})`;
  if (kind.void) { sql(`select ${call}`); return null; }
  const out = sql(kind.set ? `select coalesce(json_agg(t), '[]'::json) from ${call} t` : `select to_json(${call})`);
  return JSON.parse(out);
}
const supabase = async (url: string | URL | Request, init?: RequestInit) => {
  const name = String(url).replace(`${SUPABASE}/rest/v1/rpc/`, '');
  return new Response(JSON.stringify(postgrest(name, JSON.parse(String(init?.body ?? '{}')))), { headers: { 'content-type': 'application/json' } });
};

/** Push providers are replaced by a recorder: tests see exactly what would be sent. */
export const pushes: { url: string; body: any }[] = [];
const outbound = async (url: string | URL | Request, init?: RequestInit) => {
  const target = String(url);
  if (target.startsWith(`${SUPABASE}/rest/v1/rpc/`)) return supabase(url, init);
  if (target === 'https://oauth2.googleapis.com/token') return Response.json({ access_token: 'local', expires_in: 3600 });
  if (target.startsWith('https://fcm.googleapis.com/')) { pushes.push({ url: target, body: JSON.parse(String(init?.body)) }); return Response.json({ name: 'ok' }); }
  throw new Error('Unexpected outbound request ' + target);
};

export async function stack(extraEnv: Record<string, string> = {}) {
  const workerURL = new URL('cloudflare-worker.mjs', `file://${EDGE_DIR}`).href;
  // The member pages exactly as the packager copies them (web/edge/static).
  const assets = Object.fromEntries(readdirSync(`${EDGE_DIR}static`).filter(n => /\.(html|js|css)$/.test(n)).map(name => ['/' + name,
    { type: name.endsWith('.html') ? 'text/html' : name.endsWith('.css') ? 'text/css' : 'text/javascript',
      body: readFileSync(`${EDGE_DIR}static/${name}`).toString('base64') }]));
  const hook = registerHooks({ resolve(specifier, context, next) {
    if (specifier === './generated-assets.mjs' && context.parentURL === workerURL)
      return { url: 'data:text/javascript,' + encodeURIComponent(`export default ${JSON.stringify(assets)};`), shortCircuit: true };
    return next(specifier, context);
  } });
  const { handleWorker } = await import(workerURL);
  hook.deregister();
  const { handleRequest } = await import(new URL('index.ts', `file://${EDGE_DIR}`).href);
  const env = { SUPABASE_URL: SUPABASE, SUPABASE_SERVICE_ROLE_KEY: 'local-service-role', SKAGER_AUTH_ENABLED: 'true', ...extraEnv };
  const edge = async (url: string, init: RequestInit) => handleRequest(new Request(url, init), env, outbound as typeof fetch);
  /** One request through the Worker, as the phone (or browser) would send it to beta.skager.app. */
  const call = (path: string, init: RequestInit = {}) => handleWorker(new Request('https://beta.skager.app' + path, init), edge, {}) as Promise<Response>;
  /** Straight to the function, as pg_cron calls the scheduled job (store-schedule.sql). */
  const direct = (path: string, init: RequestInit = {}) => edge(`${SUPABASE}/functions/v1/skager-web${path}`, init) as Promise<Response>;
  return Object.assign(call, { direct });
}
