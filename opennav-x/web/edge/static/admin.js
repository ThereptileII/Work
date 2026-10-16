// Store administration. Server-side authorization decides; this page only renders.
import { escapeHTML as esc, money } from "./portal.js";
const $ = id => document.getElementById(id);
const content = $("admin-content"), message = $("admin-message");
const say = (text, kind = "") => { message.textContent = text; message.className = `message ${kind}`.trim(); };
const PAGES = { products: "/admin/", orders: "/admin/orders/", customers: "/admin/customers/", settings: "/admin/settings/" };
const pageFor = pathname => Object.keys(PAGES).find(key => PAGES[key] === pathname.replace(/\/?$/, "/")) || "products";
let page = pageFor(location.pathname);
// The overview lives in this page's memory only, so switching tabs renders at once.
let csrf = "", data = null, rendered = null;

export function productForm(product = {}) {
  const locked = product.kind === "access";
  return `<form id="product-form" class="form card" data-id="${esc(product.id || "")}">
    <h2>${product.id ? `Edit ${esc(product.name)}` : "New product"}</h2>
    <label>Name<input name="name" maxlength="80" required value="${esc(product.name || "")}"></label>
    <label>Short summary<input name="summary" maxlength="200" value="${esc(product.summary || "")}"></label>
    <label>Description<textarea name="description" maxlength="4000">${esc(product.description || "")}</textarea></label>
    <div class="row"><label>Price (USD)${locked ? " · set by the Stripe price" : ""}<input name="price" type="number" min="0" step="0.01" required value="${esc(((product.price_minor ?? 0) / 100).toFixed(2))}" ${locked ? "readonly" : ""}></label>
      <label>Sort order<input name="sort" type="number" min="0" max="10000" value="${esc(product.sort ?? 100)}"></label></div>
    ${product.kind && product.kind !== "hardware" ? "" : `<label class="check"><input name="shippable" type="checkbox" ${product.shippable !== false ? "checked" : ""}> Physical product (ships)</label>
    <label class="check"><input name="checkoutAddon" type="checkbox" ${product.checkout_addon ? "checked" : ""}> Offer as an add-on in the SKAGER checkout</label>`}
    ${locked ? "" : `<label class="check"><input name="active" type="checkbox" ${product.active !== false ? "checked" : ""}> Visible in the shop</label>`}
    ${product.id ? `<label>Image (PNG, JPEG or WebP, up to 2 MB)<input name="image" type="file" accept="image/png,image/jpeg,image/webp"></label>` : ""}
    <div class="actions"><button class="button" type="submit">Save</button><button class="button button-secondary" type="button" id="product-cancel">Cancel</button></div></form>`;
}

function renderProducts() {
  return `<div class="actions"><button class="button" type="button" id="product-new">Add product</button></div><div id="product-editor"></div>
  <div class="table-wrap after"><table><thead><tr><th></th><th>Product</th><th>Type</th><th>Price</th><th>Status</th><th></th></tr></thead><tbody>
  ${data.products.map(product => `<tr><td>${product.image_key ? `<img class="thumb" src="/api/media/${esc(product.image_key)}" alt="">` : '<span class="thumb"></span>'}</td>
    <td><strong>${esc(product.name)}</strong><div class="note">${esc(product.summary)}</div></td><td>${esc(product.kind)}${product.checkout_addon ? " · add-on" : ""}</td>
    <td>${esc(money(product.price_minor))}${product.interval ? " / month" : ""}</td><td>${product.active ? '<span class="badge ok">Visible</span>' : '<span class="badge off">Hidden</span>'}</td>
    <td><button class="button button-secondary" type="button" data-edit="${esc(product.id)}">Edit</button></td></tr>`).join("")}</tbody></table></div>`;
}
function address(order) {
  const a = order.shipping?.address;
  return a ? [order.shipping.name, a.line1, a.line2, `${a.postal_code ?? ""} ${a.city ?? ""}`.trim(), a.country].filter(Boolean).map(esc).join("<br>") : "";
}
function renderOrders() {
  if (!data.orders.length) return '<div class="empty">No orders yet.</div>';
  return `<div class="table-wrap"><table><thead><tr><th>Date</th><th>Customer</th><th>Items</th><th>Total</th><th>Status</th><th>Shipping</th><th></th></tr></thead><tbody>
  ${data.orders.map(order => `<tr><td>${esc(new Date(order.created_at).toLocaleString("en-GB"))}</td><td>${esc(order.email)}</td>
    <td>${order.items.map(item => `${esc(item.name)}${item.quantity > 1 ? ` × ${esc(item.quantity)}` : ""}`).join("<br>")}</td>
    <td>${esc(money(order.total_minor ?? order.subtotal_minor))}</td><td>${esc(order.status)}</td>
    <td>${order.fulfillment === "unfulfilled" ? `<div class="note">${address(order)}</div>` : esc(order.fulfillment === "shipped" ? `Shipped ${order.tracking || ""}` : "—")}</td>
    <td>${order.fulfillment === "unfulfilled" && order.status === "paid" ? `<input class="inline-input" data-tracking="${esc(order.id)}" placeholder="Tracking" maxlength="200"><button class="button button-secondary" type="button" data-ship="${esc(order.id)}">Mark shipped</button>` : ""}
      ${["paid", "refund_requested"].includes(order.status) ? `<button class="button button-danger" type="button" data-refund="${esc(order.id)}">Refund</button>` : ""}</td></tr>`).join("")}</tbody></table></div>`;
}
function renderCustomers() {
  if (!data.customers.length) return '<div class="empty">No customers yet.</div>';
  return `<div class="table-wrap"><table><thead><tr><th>Email</th><th>Status</th><th>Account</th><th>Since</th></tr></thead><tbody>
  ${data.customers.map(c => `<tr><td>${esc(c.email)}</td><td>${c.status === "active" ? '<span class="badge ok">Active</span>' : '<span class="badge bad">Closed</span>'}</td>
    <td>${c.claimed ? (c.password_set ? "Set up" : "Choosing password") : "Not set up yet"}</td><td>${esc(new Date(c.created_at).toLocaleDateString("en-GB"))}</td></tr>`).join("")}</tbody></table></div>`;
}
function renderSettings() {
  const s = data.settings;
  return `<form id="settings-form" class="form card narrow"><h2>Shipping</h2>
    <label>Flat shipping per order (USD)<input name="shipping" type="number" min="0" step="0.01" value="${esc((s.shipping_minor / 100).toFixed(2))}"></label>
    <label>Countries (ISO codes, comma separated)<textarea name="countries">${esc(s.shipping_countries.join(", "))}</textarea></label>
    <div class="actions"><button class="button" type="submit">Save shipping</button></div></form>`;
}

async function request(url, payload) {
  const response = await fetch(url, payload === undefined ? { credentials: "same-origin", cache: "no-store" } : {
    method: "POST", credentials: "same-origin", cache: "no-store",
    headers: { "Content-Type": "application/json", "X-CSRF-Token": csrf }, body: JSON.stringify(payload) });
  const body = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(body.error || "That did not work.");
  return body;
}
function render() {
  rendered = JSON.stringify([page, data]);
  content.innerHTML = { products: renderProducts, orders: renderOrders, customers: renderCustomers, settings: renderSettings }[page]();
  bind();
}
async function load() {
  data = await request("/api/admin/overview");
  render();
}
function mark() {
  for (const link of document.querySelectorAll("#admin-nav a")) {
    if (link.dataset.page === page) link.setAttribute("aria-current", "page"); else link.removeAttribute("aria-current");
  }
  $("admin-title").textContent = { products: "Products", orders: "Orders", customers: "Customers", settings: "Shipping" }[page];
}
/** Show tab p from memory, then refresh; re-render only when the data changed, so open forms stay open. */
async function show(p) {
  page = p; mark(); say("");
  render();
  try {
    const fresh = await request("/api/admin/overview");
    if (page !== p) return;
    data = fresh;
    if (JSON.stringify([page, data]) !== rendered) render();
  } catch (error) { if (page === p) say(error.message, "error"); }
}
const readFile = file => new Promise((resolve, reject) => {
  const reader = new FileReader();
  reader.onload = () => resolve(String(reader.result).split(",")[1] || "");
  reader.onerror = () => reject(new Error("The image could not be read."));
  reader.readAsDataURL(file);
});
function confirmTwice(button, label) {
  if (button.dataset.confirm === "yes") return true;
  button.dataset.confirm = "yes"; button.textContent = label; return false;
}
function bind() {
  const editor = $("product-editor");
  const openEditor = product => {
    editor.innerHTML = productForm(product);
    $("product-cancel").addEventListener("click", () => { editor.innerHTML = ""; });
    $("product-form").addEventListener("submit", async event => {
      event.preventDefault();
      const form = event.target, field = name => form.elements.namedItem(name);
      const payload = { name: field("name").value.trim(), summary: field("summary").value.trim(), description: field("description").value,
        priceMinor: Math.round(Number(field("price").value) * 100), sort: Number(field("sort").value) };
      if (product.id) payload.id = product.id;
      if (field("shippable")) payload.shippable = field("shippable").checked;
      if (field("checkoutAddon")) payload.checkoutAddon = field("checkoutAddon").checked;
      if (field("active")) payload.active = field("active").checked;
      try {
        const saved = await request("/api/admin/product", payload);
        const file = field("image")?.files?.[0];
        if (file) {
          if (file.size > 2 * 1024 * 1024) throw new Error("Images can be up to 2 MB.");
          await request("/api/admin/product-image", { productId: saved.product.id, data: await readFile(file) });
        }
        say("Saved.", "ok"); await load();
      } catch (error) { say(error.message, "error"); }
    });
  };
  $("product-new")?.addEventListener("click", () => openEditor({}));
  for (const button of content.querySelectorAll("[data-edit]")) {
    button.addEventListener("click", () => openEditor(data.products.find(p => p.id === button.dataset.edit)));
  }
  for (const button of content.querySelectorAll("[data-ship]")) {
    button.addEventListener("click", async () => {
      try {
        await request("/api/admin/ship", { orderId: button.dataset.ship, tracking: content.querySelector(`[data-tracking="${button.dataset.ship}"]`)?.value.trim() || undefined });
        say("Marked as shipped; the customer has been emailed.", "ok"); await load();
      } catch (error) { say(error.message, "error"); }
    });
  }
  for (const button of content.querySelectorAll("[data-refund]")) {
    button.addEventListener("click", async () => {
      if (!confirmTwice(button, "Press again to refund")) return;
      try { const result = await request("/api/admin/refund", { orderId: button.dataset.refund });
        say(result.state === "refunded" ? "Refunded." : "Refund sent to Stripe.", "ok"); await load();
      } catch (error) { say(error.message, "error"); }
    });
  }
  $("settings-form")?.addEventListener("submit", async event => {
    event.preventDefault();
    const form = event.target;
    try {
      await request("/api/admin/settings", { shippingMinor: Math.round(Number(form.elements.namedItem("shipping").value) * 100),
        countries: form.elements.namedItem("countries").value.split(/[\s,]+/).map(c => c.trim().toUpperCase()).filter(Boolean) });
      say("Shipping saved.", "ok"); await load();
    } catch (error) { say(error.message, "error"); }
  });
}
async function main() {
  try {
    const session = await request("/api/auth/session");
    if (!session.account) { location.assign("/auth/login/"); return; }
    if (!session.account.admin) { content.innerHTML = ""; say("This area is for SKAGER administrators.", "error"); return; }
    csrf = session.csrfToken;
    $("admin-nav").hidden = false;
    mark();
    await load(); say("");
    $("admin-nav").addEventListener("click", event => {
      const link = event.target.closest("a[data-page]");
      if (!link || data === null || event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
      event.preventDefault();
      if (link.dataset.page === page) return;
      history.pushState(null, "", link.getAttribute("href"));
      show(link.dataset.page);
    });
    addEventListener("popstate", () => { if (data !== null) show(pageFor(location.pathname)); });
  } catch (error) { if (data === null) content.innerHTML = ""; say(error.message, "error"); }
}
if (typeof document !== "undefined" && document.getElementById("admin-content")) main();
