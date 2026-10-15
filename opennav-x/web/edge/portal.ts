/** Private staging portal. The root router checks CSRF before every mutation. */
export interface PortalContext {
  env: Record<string, string | undefined>;
  getAccount(request: Request): Promise<{ id: string; email?: string; passwordSet?: boolean } | null>;
  rpc(name: string, args: Record<string, unknown>): Promise<unknown>;
}

const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i;
const PRODUCT = "skager-beta";
const headers = {
  "Content-Type": "application/json; charset=utf-8",
  "Cache-Control": "private, no-store, max-age=0",
  "X-Content-Type-Options": "nosniff",
  "X-Robots-Tag": "noindex, nofollow",
};
function json(value: unknown, status = 200): Response {
  return new Response(JSON.stringify(value), { status, headers });
}
async function boundedBody(request: Request): Promise<string | null> {
  if (Number(request.headers.get("Content-Length")) > 4096) return null;
  const reader = request.body?.getReader();
  if (!reader) return "";
  const chunks: Uint8Array[] = [];
  let size = 0;
  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    size += value.byteLength;
    if (size > 4096) { await reader.cancel(); return null; }
    chunks.push(value);
  }
  const body = new Uint8Array(size);
  let offset = 0;
  for (const chunk of chunks) { body.set(chunk, offset); offset += chunk.byteLength; }
  return new TextDecoder("utf-8", { fatal: true }).decode(body);
}
function object(value: unknown): Record<string, unknown> {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("Invalid RPC response");
  return value as Record<string, unknown>;
}
function boat(value: unknown) {
  const row = object(value);
  if (typeof row.id !== "string" || !UUID.test(row.id) || typeof row.name !== "string"
      || typeof row.created_at !== "string") throw new Error("Invalid boat response");
  return { id: row.id, name: row.name, createdAt: row.created_at };
}
function boatList(value: unknown) {
  if (!Array.isArray(value)) throw new Error("Invalid boats response");
  return value.map(boat);
}
const ORDER_STATUS = ["pending", "paid", "refund_requested", "refunded", "expired"];
function item(value: unknown) {
  const row = object(value);
  if (typeof row.name !== "string" || !Number.isSafeInteger(row.quantity) || !Number.isSafeInteger(row.unit_minor)) {
    throw new Error("Invalid order item");
  }
  return { name: row.name, kind: row.kind, quantity: row.quantity, unitMinor: row.unit_minor,
    interval: row.interval ?? null };
}
function commerce(value: unknown) {
  const data = object(value);
  const entitlement = data.entitlement ? object(data.entitlement) : null;
  // Only a settled, non-refunded purchase grants access; a redirect or a pending order never does.
  const active = entitlement?.status === "active"
    && entitlement.environment === "staging" && entitlement.product_code === PRODUCT;
  const orders = (Array.isArray(data.orders) ? data.orders : []).map(value => {
    const row = object(value);
    if (typeof row.id !== "string" || !UUID.test(row.id) || typeof row.created_at !== "string"
        || !ORDER_STATUS.includes(String(row.status)) || !Array.isArray(row.items)) {
      throw new Error("Invalid order response");
    }
    return {
      id: row.id, kind: row.kind, createdAt: row.created_at, status: row.status, fulfillment: row.fulfillment,
      items: row.items.map(item), subtotalMinor: row.subtotal_minor, shippingMinor: row.shipping_minor,
      discountMinor: row.discount_minor, totalMinor: row.total_minor, tracking: row.tracking ?? null,
      shippedAt: row.shipped_at ?? null, refundable: row.refundable === true,
    };
  });
  const subscription = data.subscription ? object(data.subscription) : null;
  return {
    betaAccess: active ? "active" : "inactive", orders,
    subscription: subscription ? { status: subscription.status, currentPeriodEnd: subscription.current_period_end ?? null,
      cancelAtPeriodEnd: subscription.cancel_at_period_end === true } : null,
    hasBilling: data.customer ? object(data.customer).has_billing === true : false,
  };
}

export async function handlePortal(request: Request, ctx: PortalContext): Promise<Response | null> {
  const path = new URL(request.url).pathname;
  if (!["/api/portal/dashboard", "/api/portal/boats"].includes(path)) return null;
  if (ctx.env.PORTAL_ENABLED !== "true" || ctx.env.STRIPE_TEST_ENVIRONMENT !== "staging") {
    return json({ error: "Portal unavailable." }, 404);
  }
  const allowed = path.endsWith("/boats") ? ["GET", "POST"] : ["GET"];
  if (!allowed.includes(request.method)) {
    const response = json({ error: "Method not allowed." }, 405);
    response.headers.set("Allow", allowed.join(", "));
    return response;
  }
  try {
    const account = await ctx.getAccount(request);
    if (!account || !UUID.test(account.id)) return json({ error: "Sign in required." }, 401);
    // A new or reset account chooses its password before anything else.
    if (account.passwordSet === false) return json({ error: "Choose a password first.", setup: true }, 403);
    const args = { p_account_id: account.id };
    if (path === "/api/portal/dashboard") {
      const [access, boats] = await Promise.all([
        ctx.rpc("store_portal_access", args), ctx.rpc("portal_list_boats", args),
      ]);
      return json({
        account: { email: account.email || null }, environment: "staging",
        ...commerce(access), boats: boatList(boats),
        storeAvailable: ctx.env.STRIPE_TEST_CHECKOUT_ENABLED === "true",
      });
    }
    if (request.method === "GET") return json({ boats: boatList(await ctx.rpc("portal_list_boats", args)) });
    if (request.headers.get("Content-Type")?.split(";", 1)[0].trim().toLowerCase() !== "application/json") {
      return json({ error: "Send a boat name as JSON." }, 415);
    }
    const raw = await boundedBody(request);
    if (raw === null) return json({ error: "Request too large." }, 413);
    let body: Record<string, unknown>;
    try { body = object(JSON.parse(raw)); } catch { return json({ error: "Enter a boat name." }, 400); }
    if (Object.keys(body).some(key => key !== "name") || typeof body.name !== "string") {
      return json({ error: "Only a boat name may be submitted." }, 400);
    }
    const name = body.name.trim();
    if (!name || Array.from(name).length > 80 || /[\u0000-\u001f\u007f]/.test(name)) {
      return json({ error: "Enter a boat name of 1–80 characters without control characters." }, 400);
    }
    const created = await ctx.rpc("portal_create_boat", { ...args, p_name: name });
    return json({ boat: boat(Array.isArray(created) ? created[0] : created) }, 201);
  } catch {
    // No vendor bodies, service keys, database messages or account IDs cross this boundary.
    return json({ error: "Your account data is temporarily unavailable. Please try again." }, 503);
  }
}
