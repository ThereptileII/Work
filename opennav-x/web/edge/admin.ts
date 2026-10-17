/** Store administration for the addresses in SKAGER_ADMIN_EMAILS: products, prices,
 * images, checkout and download steps, orders (ship, refund), customers and shipping settings. */
import { boundedBytes, configuration, integer, object, requireValue, strictJson, type Configuration } from './commerce.ts';
import { adminRecipients, sendDue } from './email.ts';
import { MAX_IMAGE_BYTES, putImage } from './media.ts';
import { stripeRefund, type StoreContext } from './store.ts';

type Json = Record<string, any>;
const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-8][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/;
export const IMAGE_ROUTE = '/api/admin/product-image';
export const IMAGE_BODY_LIMIT = Math.ceil(MAX_IMAGE_BYTES * 4 / 3) + 4096;
const reply = (status: number, value: Json) => new Response(JSON.stringify(value), { status, headers: {
  'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'private, no-store, max-age=0',
  'X-Content-Type-Options': 'nosniff' } });
const plain = (value: unknown, maximum: number) => typeof value === 'string' && value.length <= maximum
  && !/[\u0000-\u0008\u000b\u000c\u000e-\u001f\u007f]/.test(value);

async function input(req: Request, maximum = 16384): Promise<Json> {
  requireValue(req.headers.get('Content-Type')?.split(';')[0].trim().toLowerCase() === 'application/json');
  return strictJson(await boundedBytes(req.body, maximum));
}

function slugify(name: string): string {
  const base = name.toLowerCase().normalize('NFKD').replace(/[^a-z0-9]+/g, '-').replace(/^-+|-+$/g, '').slice(0, 48) || 'product';
  return `${base.length < 2 ? base + '-x' : base}-${crypto.getRandomValues(new Uint8Array(3)).reduce((s, b) => s + b.toString(16).padStart(2, '0'), '')}`;
}

/** Validate an admin product edit; the database re-checks every constraint. */
export function productInput(value: Json): Json | null {
  const keys = ['id', 'name', 'summary', 'description', 'priceMinor', 'shippable', 'stepId', 'active', 'sort'];
  if (!object(value) || !Object.keys(value).every(key => keys.includes(key))) return null;
  if (value.id !== undefined && !(typeof value.id === 'string' && UUID.test(value.id))) return null;
  if (!plain(value.name, 80) || !value.name.trim() || (value.summary !== undefined && !plain(value.summary, 200))
    || (value.description !== undefined && !(typeof value.description === 'string' && value.description.length <= 4000))) return null;
  if (!integer(value.priceMinor) || value.priceMinor < 0 || value.priceMinor > 10_000_000) return null;
  for (const key of ['shippable', 'active']) if (value[key] !== undefined && typeof value[key] !== 'boolean') return null;
  if (value.stepId !== undefined && value.stepId !== null && !(typeof value.stepId === 'string' && UUID.test(value.stepId))) return null;
  if (value.sort !== undefined && (!integer(value.sort) || value.sort < 0 || value.sort > 10000)) return null;
  return { id: value.id ?? null, slug: value.id ? undefined : slugify(value.name), kind: 'hardware',
    name: value.name.trim(), summary: (value.summary ?? '').trim(), description: value.description ?? '',
    price_minor: value.priceMinor, shippable: value.shippable ?? true, step_id: value.stepId ?? null,
    active: value.active ?? true, sort: value.sort ?? 100 };
}

/** Validate a checkout or download stage; the database re-checks every constraint. */
export function stepInput(value: Json): Json | null {
  const keys = ['id', 'flow', 'title', 'summary', 'skipLabel', 'multiple', 'linkUrl', 'linkLabel', 'active', 'sort'];
  if (!object(value) || !Object.keys(value).every(key => keys.includes(key))) return null;
  if (value.id !== undefined && !(typeof value.id === 'string' && UUID.test(value.id))) return null;
  if (value.id === undefined && !['checkout', 'download'].includes(value.flow)) return null;
  if (!plain(value.title, 60) || !value.title.trim() || !plain(value.skipLabel, 80) || !value.skipLabel.trim()
    || (value.summary !== undefined && !plain(value.summary, 600))) return null;
  const link = typeof value.linkUrl === 'string' ? value.linkUrl.trim() : '';
  const label = typeof value.linkLabel === 'string' ? value.linkLabel.trim() : '';
  if ((value.linkUrl !== undefined && typeof value.linkUrl !== 'string') || (value.linkLabel !== undefined && !plain(value.linkLabel, 80))) return null;
  if (link) {
    let url: URL;
    try { url = new URL(link); } catch { return null; }
    if (url.protocol !== 'https:' || url.username || url.password || link.length > 500 || /[\s<>"]/.test(link) || !label) return null;
  } else if (label) return null;
  for (const key of ['multiple', 'active']) if (value[key] !== undefined && typeof value[key] !== 'boolean') return null;
  if (value.sort !== undefined && (!integer(value.sort) || value.sort < 0 || value.sort > 10000)) return null;
  return { id: value.id ?? null, flow: value.flow, title: value.title.trim(), summary: (value.summary ?? '').trim(),
    skip_label: value.skipLabel.trim(), multiple: value.multiple ?? false, link_url: link, link_label: label,
    active: value.active ?? true, sort: value.sort ?? 100 };
}

function base64(value: unknown): Uint8Array | null {
  if (typeof value !== 'string' || value.length > IMAGE_BODY_LIMIT || !/^[A-Za-z0-9+/]*={0,2}$/.test(value)) return null;
  try {
    const binary = atob(value);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
    return bytes.length > 0 && bytes.length <= MAX_IMAGE_BYTES ? bytes : null;
  } catch { return null; }
}

export async function handleAdmin(req: Request, ctx: StoreContext): Promise<Response | null> {
  const path = new URL(req.url).pathname;
  if (!path.startsWith('/api/admin/')) return null;
  const routes: Record<string, string> = { '/api/admin/overview': 'GET', '/api/admin/product': 'POST', '/api/admin/step': 'POST', [IMAGE_ROUTE]: 'POST',
    '/api/admin/ship': 'POST', '/api/admin/refund': 'POST', '/api/admin/settings': 'POST' };
  if (!routes[path]) return reply(404, { error: 'Not found' });
  if (req.method !== routes[path]) return reply(405, { error: `${routes[path]} required` });
  try {
    const account = await ctx.getAccount(req);
    if (!account || !account.email || account.passwordSet === false) return reply(401, { error: 'Sign in required.' });
    // Exact allowlist on the verified account email; nothing from the request decides.
    if (!adminRecipients(ctx.env).includes(account.email.toLowerCase())) return reply(403, { error: 'Not an administrator.' });
    const admin = account.email.toLowerCase();
    if (path === '/api/admin/overview') return reply(200, await ctx.rpc('store_admin_overview', {}));
    if (path === '/api/admin/product') {
      let product: Json | null = null;
      try { product = productInput(await input(req)); } catch { product = null; }
      if (!product) return reply(400, { error: 'Check the product fields.' });
      try { return reply(200, { product: await ctx.rpc('store_admin_save_product', { p_admin: admin, p_product: product }) }); }
      catch { return reply(400, { error: 'The product could not be saved.' }); }
    }
    if (path === '/api/admin/step') {
      let step: Json | null = null;
      try { step = stepInput(await input(req)); } catch { step = null; }
      if (!step) return reply(400, { error: 'Check the step fields. Links must start with https:// and need a button text.' });
      try { return reply(200, { step: await ctx.rpc('store_admin_save_step', { p_admin: admin, p_step: step }) }); }
      catch { return reply(400, { error: 'The step could not be saved.' }); }
    }
    if (path === IMAGE_ROUTE) {
      let data: Json;
      try { data = await input(req, IMAGE_BODY_LIMIT); } catch { return reply(400, { error: 'Send a PNG, JPEG or WebP image up to 2 MB.' }); }
      const bytes = base64(data.data);
      if (Object.keys(data).length !== 2 || typeof data.productId !== 'string' || !UUID.test(data.productId) || !bytes) {
        return reply(400, { error: 'Send a PNG, JPEG or WebP image up to 2 MB.' });
      }
      let key: string;
      try { key = await putImage(ctx.env, data.productId, bytes, ctx.fetch); }
      catch { return reply(400, { error: 'That image could not be stored. Use PNG, JPEG or WebP up to 2 MB.' }); }
      return reply(200, { product: await ctx.rpc('store_admin_set_image', { p_admin: admin, p_product_id: data.productId, p_key: key }) });
    }
    if (path === '/api/admin/settings') {
      let data: Json;
      try { data = await input(req); } catch { return reply(400, { error: 'Check the shipping settings.' }); }
      if (!integer(data.shippingMinor) || data.shippingMinor < 0 || data.shippingMinor > 100000 || !Array.isArray(data.countries)
        || data.countries.length < 1 || data.countries.length > 60 || !data.countries.every((c: unknown) => typeof c === 'string' && /^[A-Z]{2}$/.test(c))) {
        return reply(400, { error: 'Check the shipping settings.' });
      }
      return reply(200, { settings: await ctx.rpc('store_admin_settings', { p_admin: admin,
        p_shipping_minor: data.shippingMinor, p_countries: [...new Set(data.countries)] }) });
    }
    let data: Json;
    try { data = await input(req); } catch { return reply(400, { error: 'Choose an order.' }); }
    if (typeof data.orderId !== 'string' || !UUID.test(data.orderId)) return reply(400, { error: 'Choose an order.' });
    if (path === '/api/admin/ship') {
      if (data.tracking !== undefined && !plain(data.tracking, 200)) return reply(400, { error: 'Check the tracking reference.' });
      try {
        const order = await ctx.rpc('store_admin_ship', { p_admin: admin, p_order_id: data.orderId, p_tracking: data.tracking ?? '' });
        await sendDue(ctx, 5);
        return reply(200, { order });
      } catch { return reply(400, { error: 'This order cannot be marked as shipped.' }); }
    }
    let config: Configuration;
    try { config = configuration(ctx.env); } catch { return reply(503, { error: 'Payments are not configured.' }); }
    const target = await ctx.rpc('store_admin_refund_target', { p_admin: admin, p_order_id: data.orderId });
    requireValue(object(target));
    if (target.state === 'not_refundable') return reply(400, { error: 'This order cannot be refunded.' });
    if (target.state === 'requested') await stripeRefund(ctx, config, target);
    await sendDue(ctx, 5);
    return reply(200, { state: target.state });
  } catch { return reply(503, { error: 'Admin is temporarily unavailable.' }); }
}
