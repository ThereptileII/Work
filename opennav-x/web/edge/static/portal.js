export function escapeHTML(value) {
  return String(value ?? "").replace(/[&<>"']/g, char => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[char]);
}
const esc = escapeHTML;
const date = value => {
  const parsed = new Date(value);
  return Number.isNaN(parsed.getTime()) ? "Date unavailable" : parsed.toLocaleDateString("en-GB", { timeZone: "UTC", day: "numeric", month: "short", year: "numeric" });
};
export function orderTotal(order) {
  if (order.fullyDiscounted) return 'Full discount<span class="portal-note">Zero total · no payment collected</span>';
  if (["USD", "EUR", "GBP", "SEK"].includes(order.currency)) return `${esc((order.amountMinor / 100).toFixed(2))} ${esc(order.currency)}<span class="portal-note">Test checkout total</span>`;
  return `Total recorded in ${esc(order.currency)}<span class="portal-note">Test checkout total</span>`;
}
const card = (eyebrow, title, body, extra = "") => `<section class="portal-card ${extra}"><p class="eyebrow">${eyebrow}</p><h2>${title}</h2>${body}</section>`;
const status = { pending: "Pending", fulfilled: "Completed", refunded: "Refunded", suspended: "Suspended", expired: "Expired", failed: "Failed" };
export function renderAccess(data) {
  return card("BETA ACCESS", data.betaAccess === "active" ? "Active in staging." : "Your next chapter.", data.betaAccess === "active"
    ? '<p>Your staging beta entitlement is active. Paid and fully discounted orders grant the same access.</p><a class="text-link" href="/portal/downloads/">View release availability ↗</a>'
    : '<p>No active beta access. A verified checkout must activate your entitlement. An order awaiting completion does not grant access.</p>' + (data.checkoutAvailable ? '<label for="discount-code">Discount code (optional)</label><input id="discount-code" name="discountCode" maxlength="64" autocomplete="off" placeholder="Your invitation code"><button class="button" id="checkout" type="button">Continue to test checkout ↗</button><p class="portal-note">Your approved discount is shown in checkout before you confirm. A full discount grants the same beta access.</p>' : '<p class="portal-note">Test checkout is currently unavailable.</p>') + '<a class="text-link" href="/pricing/">Beta pricing &amp; terms ↗</a>', "portal-access");
}
export function renderBoats(boats) {
  return boats.length ? `<ul class="portal-list">${boats.map(boat => `<li><strong>${esc(boat.name)}</strong><span class="portal-note">Added ${esc(date(boat.createdAt))}</span></li>`).join("")}</ul>`
    : '<div class="portal-empty"><p>No boats added yet.</p><p>Add a name to start your boat profile.</p></div>';
}
export function renderOrders(orders) {
  return orders.length ? `<div class="portal-table-wrap"><table class="portal-table"><caption>Your staging beta orders</caption><thead><tr><th scope="col">Order / date</th><th scope="col">Status</th><th scope="col">Checkout total</th></tr></thead><tbody>${orders.map(order => `<tr><th scope="row"><span class="portal-order-id">${esc(order.id)}</span><span class="portal-note">${esc(date(order.createdAt))}</span></th><td>${esc(status[order.status] || "Status unavailable")}</td><td>${orderTotal(order)}</td></tr>`).join("")}</tbody></table></div>`
    : '<div class="portal-empty"><p>No staging beta orders yet.</p><p>Your order will appear here after test checkout starts.</p></div>';
}
export function renderDashboard(data, page = "dashboard") {
  if (page === "account") return `<div class="portal-grid">${card("ACCOUNT DETAILS", "A place for your passages.", `<dl class="portal-details"><dt>Contact email</dt><dd>${esc(data.account.email || "Not provided")}</dd></dl><p class="portal-note">Profile editing is not available here yet. Sign-in uses a code sent to your email.</p>`)}${renderAccess(data)}</div>`;
  if (page === "boats") return `<div class="portal-grid">${card("YOUR FLEET", "It starts with a name.", renderBoats(data.boats) + '<p class="portal-note">Saved boat names only. Connected instruments, telemetry, position, renaming and removal are not available yet.</p>')}${card("ADD A BOAT", "Make it yours.", '<form id="boat-form" class="portal-form"><label for="boat-name">Boat name</label><input id="boat-name" name="name" required maxlength="80" autocomplete="off" placeholder="e.g. S/Y Aurora"><p class="portal-note">Up to 80 characters. Only your account can see this name.</p><button class="button" type="submit">Add boat +</button></form>')}</div>`;
  if (page === "orders") return card("YOUR PRIVATE STAGING HISTORY", "Every beta order, clearly.", '<p>Only your SKAGER beta test orders appear here. Test orders do not record real money received.</p>' + renderOrders(data.orders) + '<p class="portal-note">Invoice and receipt delivery are not available in this test portal.</p>');
  if (page === "downloads") return `<div class="portal-grid">${renderAccess(data)}${card("DOWNLOAD AVAILABILITY", "Release delivery is closed.", '<p>No installer or download link is available in this private test portal. Release qualification and secure delivery are pending.</p><p class="portal-note">Latest release and release notes are unavailable here until a qualified artifact is connected.</p><a class="text-link" href="/install/">Installation prerequisites ↗</a><a class="text-link" href="/open-source/">Open source &amp; licenses ↗</a>')}</div>`;
  return `<div class="portal-grid">${renderAccess(data)}${card("MY BOATS", "Built around your boat.", renderBoats(data.boats) + '<a class="text-link" href="/portal/boats/">Manage my boats ↗</a>')}${card("LATEST ORDERS", "Your beta journey.", renderOrders(data.orders.slice(0, 3)) + '<a class="text-link" href="/portal/orders/">Orders &amp; billing ↗</a>')}${card("SUPPORT &amp; FEEDBACK", "Help along the way.", '<p>Guides and frequently asked questions are available now. Customer bug reports, feature requests and their status are not available in this test area yet.</p><a class="text-link" href="/support/">Read help &amp; FAQ ↗</a>')}</div>`;
}

if (typeof document !== "undefined") {
  let csrfToken;
  const content = document.getElementById("portal-content");
  const message = document.getElementById("portal-message");
  const tell = text => { message.textContent = text; };
  const page = location.pathname.split("/").filter(Boolean)[1] || "dashboard";
  const titles = { dashboard: "Welcome aboard.", account: "Your account.", boats: "My boats.", orders: "Orders & billing.", downloads: "Your next passage." };
  document.querySelector("h1").textContent = titles[page] || titles.dashboard;
  document.querySelector(`[data-page="${Object.hasOwn(titles, page) ? page : "dashboard"}"]`)?.setAttribute("aria-current", "page");
  async function api(path, body) {
    const response = await fetch(path, { credentials: "same-origin", cache: "no-store", ...(body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": csrfToken }, body: JSON.stringify(body) }) });
    if (response.status === 401) { location.assign("/auth/login/"); throw new Error("Your session has ended. Please sign in."); }
    const data = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(data.error || "This service is temporarily unavailable. Please try again.");
    return data;
  }
  async function load() {
    try {
      const session = await api("/api/auth/session");
      if (!session.account) { location.assign("/auth/login/"); return; }
      if (typeof session.csrfToken !== "string" || !session.csrfToken) throw new Error("A secure session could not be started. Please reload this page.");
      csrfToken = session.csrfToken;
      const data = await api("/api/portal/dashboard");
      content.innerHTML = renderDashboard(data, page);
      content.setAttribute("aria-busy", "false");
      document.getElementById("logout").hidden = false;
      tell("");
      document.getElementById("boat-form")?.addEventListener("submit", async event => {
        event.preventDefault();
        const button = event.currentTarget.querySelector("button");
        button.disabled = true;
        try { await api("/api/portal/boats", { name: document.getElementById("boat-name").value }); await load(); tell("Your boat has been added."); }
        catch (error) { tell(error.message); button.disabled = false; }
      });
      document.getElementById("checkout")?.addEventListener("click", async event => {
        event.currentTarget.disabled = true;
        try {
          const checkout = await api("/api/checkout/start", { discountCode: document.getElementById("discount-code")?.value.trim() || "" });
          const target = new URL(checkout.url);
          if (target.protocol !== "https:" || target.hostname !== "checkout.stripe.com") throw new Error("Checkout is unavailable. Please try again.");
          location.assign(target.href);
        } catch (error) { tell(error.message); event.target.disabled = false; }
      });
    } catch (error) { content.setAttribute("aria-busy", "false"); tell(error.message); }
  }
  document.getElementById("logout").addEventListener("click", async event => {
    event.target.disabled = true;
    try { await api("/api/auth/logout", {}); location.assign("/auth/login/"); }
    catch (error) { tell(error.message); event.target.disabled = false; }
  });
  load();
}
