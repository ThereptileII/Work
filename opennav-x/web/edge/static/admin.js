// Store administration. Server-side authorization decides; this page only renders.
import { escapeHTML as esc, money } from "./portal.js";
const $ = id => document.getElementById(id);
const content = $("admin-content"), message = $("admin-message");
const say = (text, kind = "") => { message.textContent = text; message.className = `message ${kind}`.trim(); };
const PAGES = { products: "/admin/", steps: "/admin/steps/", orders: "/admin/orders/", customers: "/admin/customers/", settings: "/admin/settings/" };
const pageFor = pathname => Object.keys(PAGES).find(key => PAGES[key] === pathname.replace(/\/?$/, "/")) || "products";
let page = pageFor(location.pathname);
// The overview lives in this page's memory only, so switching tabs renders at once.
let csrf = "", data = null, rendered = null;

export function productForm(product = {}, steps = []) {
  const locked = product.kind === "access";
  return `<form id="product-form" class="form card" data-id="${esc(product.id || "")}">
    <h2>${product.id ? `Edit ${esc(product.name)}` : "New product"}</h2>
    <label>Name<input name="name" maxlength="80" required value="${esc(product.name || "")}"></label>
    <label>Short summary<input name="summary" maxlength="200" value="${esc(product.summary || "")}"></label>
    <label>Description<textarea name="description" maxlength="4000">${esc(product.description || "")}</textarea></label>
    <div class="row"><label>Price (USD)${locked ? " · set by the Stripe price" : ""}<input name="price" type="number" min="0" step="0.01" required value="${esc(((product.price_minor ?? 0) / 100).toFixed(2))}" ${locked ? "readonly" : ""}></label>
      <label>Sort order<input name="sort" type="number" min="0" max="10000" value="${esc(product.sort ?? 100)}"></label></div>
    ${product.kind && product.kind !== "hardware" ? "" : `<label class="check"><input name="shippable" type="checkbox" ${product.shippable !== false ? "checked" : ""}> Physical product (ships)</label>
    <label>Checkout step<select name="stepId"><option value="">Not in the checkout (shop only)</option>${steps.filter(step => step.flow === "checkout").map(step => `<option value="${esc(step.id)}" ${product.step_id === step.id ? "selected" : ""}>${esc(step.title)}${step.active ? "" : " (hidden)"}</option>`).join("")}</select></label>`}
    ${locked ? "" : `<label class="check"><input name="active" type="checkbox" ${product.active !== false ? "checked" : ""}> Visible in the shop</label>`}
    ${product.id ? `<label>Image (PNG, JPEG or WebP, up to 2 MB)<input name="image" type="file" accept="image/png,image/jpeg,image/webp"></label>` : ""}
    <div class="actions"><button class="button" type="submit">Save</button><button class="button button-secondary" type="button" id="product-cancel">Cancel</button></div></form>`;
}

function renderProducts() {
  return `<div class="actions"><button class="button" type="button" id="product-new">Add product</button></div><div id="product-editor"></div>
  <div class="table-wrap after"><table><thead><tr><th></th><th>Product</th><th>Type</th><th>Price</th><th>Status</th><th></th></tr></thead><tbody>
  ${data.products.map(product => `<tr><td>${product.image_key ? `<img class="thumb" src="/api/media/${esc(product.image_key)}" alt="">` : '<span class="thumb"></span>'}</td>
    <td><strong>${esc(product.name)}</strong><div class="note">${esc(product.summary)}</div></td><td>${esc(product.kind)}${product.step_id ? ` · step: ${esc(data.steps?.find(step => step.id === product.step_id)?.title || "")}` : ""}</td>
    <td>${esc(money(product.price_minor))}${product.interval ? " / month" : ""}</td><td>${product.active ? '<span class="badge ok">Visible</span>' : '<span class="badge off">Hidden</span>'}</td>
    <td><button class="button button-secondary" type="button" data-edit="${esc(product.id)}">Edit</button></td></tr>`).join("")}</tbody></table></div>`;
}
const FLOWS = { checkout: ["Checkout", "Shown in order after the account step and before payment. Customers choose one of the step's products or the \u201cI already have\u201d answer."],
  download: ["Download page", "Shown in order before the SKAGER download in the customer account. Customers follow the link or answer that they already have it."],
  guide: ["Get going guide", "Shown in order after the SKAGER download: tips to get started, such as online services. The first line of the text is the introduction; each further line becomes a numbered step."] };
export function stepForm(step = {}) {
  const flow = step.flow || "checkout";
  return `<form id="step-form" class="form card" data-id="${esc(step.id || "")}">
    <h2>${step.id ? `Edit ${esc(step.title)}` : `New ${esc(FLOWS[flow][0].toLowerCase())} step`}</h2>
    ${step.id ? "" : `<label>Where<select name="flow"><option value="checkout" ${flow === "checkout" ? "selected" : ""}>Checkout</option><option value="download" ${flow === "download" ? "selected" : ""}>Download page</option><option value="guide" ${flow === "guide" ? "selected" : ""}>Get going guide</option></select></label>`}
    <label>Title<input name="title" maxlength="60" required value="${esc(step.title || "")}" placeholder="Your helm PC"></label>
    <label>Text<textarea name="summary" maxlength="1200" rows="6">${esc(step.summary || "")}</textarea></label>
    <label>${flow === "guide" ? "Skip button text" : "\u201cAlready have it\u201d answer"}<input name="skipLabel" maxlength="80" required value="${esc(step.skip_label || "")}" placeholder="${flow === "guide" ? "Skip for now" : "I already have a PC"}"></label>
    <div class="row"><label>Link (optional, https://)<input name="linkUrl" type="url" maxlength="500" value="${esc(step.link_url || "")}" placeholder="https://"></label>
      <label>Link button text<input name="linkLabel" maxlength="80" value="${esc(step.link_label || "")}" placeholder="Download OpenCPN"></label></div>
    <div class="row"><label>Sort order<input name="sort" type="number" min="0" max="10000" value="${esc(step.sort ?? 100)}"></label></div>
    <label class="check"${flow === "checkout" ? "" : " hidden"}><input name="multiple" type="checkbox" ${step.multiple ? "checked" : ""}> Customers may pick several products in this step</label>
    <label class="check"><input name="active" type="checkbox" ${step.active !== false ? "checked" : ""}> Show this step</label>
    <p class="note">${flow === "guide" ? "Each line of the text after the first is shown as a numbered step." : "Checkout steps list the products you assign to them on the Products tab."}</p>
    <div class="actions"><button class="button" type="submit">Save</button><button class="button button-secondary" type="button" id="step-cancel">Cancel</button></div></form>`;
}
function renderSteps() {
  const steps = data.steps || [];
  const products = id => data.products.filter(product => product.step_id === id).map(product => esc(product.name)).join(", ");
  return `<div class="actions"><button class="button" type="button" data-new-step="checkout">Add checkout step</button><button class="button button-secondary" type="button" data-new-step="download">Add download step</button><button class="button button-secondary" type="button" data-new-step="guide">Add guide step</button></div><div id="step-editor"></div>
  ${Object.entries(FLOWS).map(([flow, [title, note]]) => {
    const rows = steps.filter(step => step.flow === flow);
    const fixed = { checkout: ["Account (email)", "Review and pay"], download: [null, "Download SKAGER"], guide: ["Download SKAGER", "All set"] }[flow];
    return `<h2 class="section-title">${esc(title)}</h2><p class="note">${esc(note)}</p>
    <div class="table-wrap"><table><thead><tr><th>Order</th><th>Step</th><th>${flow === "checkout" ? "Products" : "Link"}</th><th>Status</th><th></th></tr></thead><tbody>
    ${fixed[0] ? `<tr><td>—</td><td><strong>${esc(fixed[0])}</strong><div class="note">${flow === "guide" ? "Comes before the guide" : "Always first"}</div></td><td></td><td><span class="badge ok">Fixed</span></td><td></td></tr>` : ""}
    ${rows.map(step => `<tr><td>${esc(step.sort)}</td><td><strong>${esc(step.title)}</strong><div class="note">${esc(step.skip_label)}</div></td>
      <td>${flow === "checkout" ? (products(step.id) || '<span class="note">No products yet</span>') : step.link_url ? `<span class="note">${esc(step.link_label)}</span>` : ""}</td>
      <td>${step.active ? '<span class="badge ok">Shown</span>' : '<span class="badge off">Hidden</span>'}</td>
      <td><button class="button button-secondary" type="button" data-edit-step="${esc(step.id)}">Edit</button></td></tr>`).join("")}
    <tr><td>—</td><td><strong>${esc(fixed[1])}</strong><div class="note">Always last</div></td><td></td><td><span class="badge ok">Fixed</span></td><td></td></tr>
    </tbody></table></div>`;
  }).join("")}`;
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
  content.innerHTML = { products: renderProducts, steps: renderSteps, orders: renderOrders, customers: renderCustomers, settings: renderSettings }[page]();
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
  $("admin-title").textContent = { products: "Products", steps: "Steps", orders: "Orders", customers: "Customers", settings: "Shipping" }[page];
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
    editor.innerHTML = productForm(product, data.steps || []);
    $("product-cancel").addEventListener("click", () => { editor.innerHTML = ""; });
    $("product-form").addEventListener("submit", async event => {
      event.preventDefault();
      const form = event.target, field = name => form.elements.namedItem(name);
      const payload = { name: field("name").value.trim(), summary: field("summary").value.trim(), description: field("description").value,
        priceMinor: Math.round(Number(field("price").value) * 100), sort: Number(field("sort").value) };
      if (product.id) payload.id = product.id;
      if (field("shippable")) payload.shippable = field("shippable").checked;
      if (field("stepId")) payload.stepId = field("stepId").value || null;
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
  const stepEditor = $("step-editor");
  const openStep = step => {
    stepEditor.innerHTML = stepForm(step);
    $("step-cancel").addEventListener("click", () => { stepEditor.innerHTML = ""; });
    // Changing "Where" redraws the form for that kind of step and keeps what was typed.
    $("step-form").elements.namedItem("flow")?.addEventListener("change", event => {
      const field = name => $("step-form").elements.namedItem(name);
      openStep({ flow: event.target.value, title: field("title").value, summary: field("summary").value, skip_label: field("skipLabel").value,
        link_url: field("linkUrl").value, link_label: field("linkLabel").value, sort: field("sort").value,
        multiple: field("multiple").checked, active: field("active").checked });
      $("step-form").elements.namedItem("flow").focus();
    });
    $("step-form").addEventListener("submit", async event => {
      event.preventDefault();
      const form = event.target, field = name => form.elements.namedItem(name), flow = step.id ? step.flow : field("flow").value;
      const payload = { title: field("title").value.trim(), summary: field("summary").value.trim(), skipLabel: field("skipLabel").value.trim(),
        linkUrl: field("linkUrl").value.trim(), linkLabel: field("linkLabel").value.trim(), sort: Number(field("sort").value),
        multiple: flow === "checkout" && field("multiple").checked, active: field("active").checked };
      if (step.id) payload.id = step.id; else payload.flow = flow;
      try { await request("/api/admin/step", payload); say("Step saved.", "ok"); await load(); }
      catch (error) { say(error.message, "error"); }
    });
  };
  for (const button of content.querySelectorAll("[data-new-step]")) button.addEventListener("click", () => openStep({ flow: button.dataset.newStep }));
  for (const button of content.querySelectorAll("[data-edit-step]")) {
    button.addEventListener("click", () => openStep(data.steps.find(step => step.id === button.dataset.editStep)));
  }
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
