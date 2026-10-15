/** Transactional email through Resend. Messages come from the database outbox, so a
 * provider outage delays mail but never loses it; the job retries with backoff. */
import { PUBLIC_ORIGIN } from './platform.ts';

type Json = Record<string, any>;
export type EmailContext = {
  env: Record<string, string | undefined>;
  rpc: (name: string, args: Json) => Promise<any>;
  fetch?: typeof fetch;
};
type Rendered = { subject: string; heading: string; paragraphs: string[]; order?: Json; action?: [string, string] };

const RESEND = 'https://api.resend.com/emails';
const escape = (value: unknown) => String(value ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]!);
const money = (minor: unknown) => Number.isSafeInteger(minor) ? `USD ${(Number(minor) / 100).toFixed(2)}` : '';

export function adminRecipients(env: EmailContext['env']): string[] {
  return (env.SKAGER_ADMIN_EMAILS ?? '').split(',').map(value => value.trim().toLowerCase())
    .filter(value => /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(value));
}

export function render(template: string, data: Json): Rendered | null {
  const order = data?.order;
  switch (template) {
    case 'welcome': return { subject: 'Welcome aboard: finish setting up SKAGER', heading: 'Welcome aboard.',
      paragraphs: ['Thank you for buying SKAGER Public Beta Access. Your account is ready for you.',
        'Finish setting up in two short steps: confirm the code we send you, then choose a password. After that you can download SKAGER from your account.'],
      order, action: ['Finish setting up', `${PUBLIC_ORIGIN}/welcome/`] };
    case 'shop_receipt': return { subject: 'Your SKAGER order is confirmed', heading: 'Thank you for your order.',
      paragraphs: ['We have received your order and will let you know as soon as it ships.'], order,
      action: ['View your orders', `${PUBLIC_ORIGIN}/portal/orders/`] };
    case 'shipped': return { subject: 'Your SKAGER order has shipped', heading: 'It is on its way.',
      paragraphs: ['Your order has left us and is on its way to you.',
        ...(order?.tracking ? [`Tracking reference: ${order.tracking}`] : [])], order,
      action: ['View your orders', `${PUBLIC_ORIGIN}/portal/orders/`] };
    case 'refunded': return { subject: 'Your SKAGER refund', heading: 'Your refund is on its way.',
      paragraphs: ['We have refunded your order. Your bank may take a few days to show it.',
        ...(order?.kind === 'access' ? ['As requested, your SKAGER account has been closed. Your rights under the open-source licences of software you received are not affected.'] : [])],
      order };
    case 'cloud_started': return { subject: 'SKAGER Cloud is active', heading: 'Welcome to SKAGER Cloud.',
      paragraphs: ['Your subscription is active. New cloud services appear in your account as they launch.',
        'You can manage or cancel your subscription at any time from your account.'], order,
      action: ['Open SKAGER Cloud', `${PUBLIC_ORIGIN}/portal/cloud/`] };
    case 'cloud_ended': return { subject: 'Your SKAGER Cloud subscription has ended', heading: 'Your subscription has ended.',
      paragraphs: ['Your SKAGER Cloud subscription is no longer active. You can start it again from your account whenever you like.'],
      action: ['Open your account', `${PUBLIC_ORIGIN}/portal/cloud/`] };
    case 'admin_order': return { subject: `New order to ship: ${order?.id ?? ''}`, heading: 'A new order needs shipping.',
      paragraphs: ['A paid order contains items to ship.'], order, action: ['Open admin', `${PUBLIC_ORIGIN}/admin/orders/`] };
    default: return null;
  }
}

function orderTable(order: Json | undefined): { html: string; text: string } {
  if (!order || !Array.isArray(order.items)) return { html: '', text: '' };
  const rows = order.items.map((item: Json) => [`${item.name}${item.quantity > 1 ? ` × ${item.quantity}` : ''}`,
    money(item.unit_minor * item.quantity) + (item.interval ? ' / month' : '')]);
  if (order.discount_minor > 0) rows.push(['Discount', `−${money(order.discount_minor)}`]);
  if (order.shipping_minor > 0) rows.push(['Shipping', money(order.shipping_minor)]);
  if (Number.isSafeInteger(order.total_minor)) rows.push(['Total', money(order.total_minor)]);
  const address = order.shipping?.address;
  const ship = address ? [order.shipping.name, address.line1, address.line2, `${address.postal_code ?? ''} ${address.city ?? ''}`.trim(), address.country].filter(Boolean) : [];
  return {
    html: `<table role="presentation" style="width:100%;border-collapse:collapse;margin:24px 0">${rows.map(([a, b]: string[], i: number) =>
      `<tr><td style="padding:8px 0;border-top:1px solid #35464a;${i === rows.length - 1 ? 'font-weight:600' : ''}">${escape(a)}</td><td style="padding:8px 0;border-top:1px solid #35464a;text-align:right;${i === rows.length - 1 ? 'font-weight:600' : ''}">${escape(b)}</td></tr>`).join('')}</table>`
      + (ship.length ? `<p style="color:#aabdbd;margin:0 0 16px">Shipping to:<br>${ship.map(escape).join('<br>')}</p>` : ''),
    text: rows.map(([a, b]: string[]) => `${a}: ${b}`).join('\n') + (ship.length ? `\n\nShipping to:\n${ship.join('\n')}` : ''),
  };
}

export function compose(template: string, data: Json): { subject: string; html: string; text: string } | null {
  const message = render(template, data);
  if (!message) return null;
  const table = orderTable(message.order);
  const html = `<!doctype html><html><body style="margin:0;background:#0b1a1f;color:#f3f5ee;font-family:Inter,Segoe UI,Arial,sans-serif">
<div style="max-width:560px;margin:0 auto;padding:40px 28px"><p style="letter-spacing:.2em;font-weight:700;color:#b6efce;margin:0 0 28px">SKAGER</p>
<h1 style="font-size:28px;font-weight:600;margin:0 0 16px">${escape(message.heading)}</h1>
${message.paragraphs.map(p => `<p style="font-size:16px;line-height:1.6;color:#d6dedb;margin:0 0 14px">${escape(p)}</p>`).join('')}
${table.html}${message.action ? `<p style="margin:28px 0"><a href="${escape(message.action[1])}" style="display:inline-block;background:#6fe0ea;color:#04161b;text-decoration:none;font-weight:600;padding:14px 22px;border-radius:8px">${escape(message.action[0])}</a></p>` : ''}
<p style="font-size:13px;color:#7e9699;margin-top:36px">SKAGER · beta.skager.app · This is a service email about your SKAGER purchase.</p></div></body></html>`;
  const text = [message.heading, '', ...message.paragraphs, '', table.text,
    ...(message.action ? ['', `${message.action[0]}: ${message.action[1]}`] : []), '', 'SKAGER · beta.skager.app'].join('\n');
  return { subject: message.subject, html, text };
}

async function send(ctx: EmailContext, id: number, to: string[], message: { subject: string; html: string; text: string }): Promise<string | null> {
  const key = ctx.env.RESEND_API_KEY ?? '';
  if (!/^re_[A-Za-z0-9_]{10,200}$/.test(key) || !to.length) return null;
  const response = await (ctx.fetch ?? fetch)(RESEND, {
    method: 'POST', redirect: 'manual', signal: AbortSignal.timeout(10000),
    headers: { Authorization: `Bearer ${key}`, 'Content-Type': 'application/json', 'Idempotency-Key': `skager-email-${id}` },
    body: JSON.stringify({ from: ctx.env.SKAGER_EMAIL_FROM || 'SKAGER <orders@auth.skager.app>', to, ...message }),
  });
  if (response.status !== 200) { await response.body?.cancel().catch(() => {}); return null; }
  try {
    const result = await response.json();
    return typeof result?.id === 'string' ? result.id.slice(0, 100) : 'sent';
  } catch { return 'sent'; }
}

/** Claim due messages, send them, and record each outcome. Never throws. */
export async function sendDue(ctx: EmailContext, limit = 10): Promise<{ sent: number; failed: number }> {
  let sent = 0; let failed = 0;
  let due: Json[] = [];
  try { due = await ctx.rpc('store_email_claim', { p_limit: limit }); } catch { return { sent, failed }; }
  for (const item of Array.isArray(due) ? due : []) {
    let provider: string | null = null;
    try {
      const message = compose(item.template, item.data);
      const to = item.to === 'admin' ? adminRecipients(ctx.env) : [String(item.to)];
      if (message) provider = await send(ctx, Number(item.id), to, message);
    } catch { provider = null; }
    try { await ctx.rpc('store_email_result', { p_id: item.id, p_ok: !!provider, p_provider_id: provider }); } catch { /* retried by the job */ }
    if (provider) sent++; else failed++;
  }
  return { sent, failed };
}
