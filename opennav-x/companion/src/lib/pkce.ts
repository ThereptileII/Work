// Sign-in with the SKAGER account in the phone's browser (RFC 8252) using an
// authorization code bound to a PKCE S256 challenge (RFC 7636). The app never
// sees the password; the browser returns only a one-time code.
import { API_ORIGIN, APP_SCHEME } from './config.ts';

const encode = (bytes: Uint8Array) => btoa(String.fromCharCode(...bytes)).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
const random = (length: number) => encode(crypto.getRandomValues(new Uint8Array(length)));

export type Pending = { verifier: string; state: string; createdAt: number };

/** 32 random bytes → 43 characters, the minimum RFC 7636 allows and the server expects. */
export function newPending(now = Date.now()): Pending {
  return { verifier: random(32), state: random(24), createdAt: now };
}
export async function challengeFor(verifier: string): Promise<string> {
  return encode(new Uint8Array(await crypto.subtle.digest('SHA-256', new TextEncoder().encode(verifier))));
}
export async function authorizeUrl(pending: Pending): Promise<string> {
  return `${API_ORIGIN}/app/authorize/?challenge=${await challengeFor(pending.verifier)}&state=${pending.state}`;
}

export type Callback = { code: string } | { error: 'denied' | 'invalid' | 'expired' };
/** The browser returns to app.skager.companion://auth?code=..&state=.. (or error=access_denied). */
export function readCallback(url: string, pending: Pending | null, now = Date.now()): Callback | null {
  const prefix = `${APP_SCHEME}://auth?`;
  if (!url.startsWith(prefix)) return null;
  const params: Record<string, string> = {};
  for (const part of url.slice(prefix.length).split('&')) {
    const [key, value, extra] = part.split('=');
    if (extra !== undefined || !key || value === undefined || key in params) return { error: 'invalid' };
    params[key] = value;
  }
  if (!pending || params.state !== pending.state) return { error: 'invalid' };
  // The server's code lives two minutes; a sign-in left open longer starts again.
  if (now - pending.createdAt > 10 * 60_000) return { error: 'expired' };
  if (params.error === 'access_denied') return { error: 'denied' };
  return /^[a-f0-9]{64}$/.test(params.code ?? '') ? { code: params.code } : { error: 'invalid' };
}
