// Customer portal. Pure render functions are exported for tests; the browser
// bootstrap at the bottom only runs on the portal page. Every value from the
// server is escaped before it reaches innerHTML. No tokens are stored.
export function escapeHTML(value) {
  return String(value ?? "").replace(/[&<>"']/g, char => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[char]);
}
const esc = escapeHTML;
export const money = minor => Number.isSafeInteger(minor) ? `USD ${(minor / 100).toFixed(2)}` : "—";
const date = value => {
  const parsed = new Date(value);
  return Number.isNaN(parsed.getTime()) ? "—" : parsed.toLocaleDateString("en-GB", { timeZone: "UTC", day: "numeric", month: "short", year: "numeric" });
};
export const PAGES = { dashboard: "/portal/", downloads: "/portal/downloads/", shop: "/portal/shop/", cloud: "/portal/cloud/", orders: "/portal/orders/", account: "/portal/account/" };
const STATUS = { pending: ["Awaiting payment", "wait"], paid: ["Paid", "ok"], refund_requested: ["Refund in progress", "wait"], refunded: ["Refunded", "off"], expired: ["Not completed", "off"] };
const SUB_STATUS = { active: ["Active", "ok"], trialing: ["Active", "ok"], past_due: ["Payment due", "wait"], canceled: ["Ended", "off"], unpaid: ["Unpaid", "bad"], incomplete: ["Starting", "wait"], incomplete_expired: ["Not started", "off"], paused: ["Paused", "off"] };
const badge = ([label, kind]) => `<span class="badge ${kind}">${esc(label)}</span>`;

export function orderTotal(order) {
  if (order.totalMinor === 0 && order.discountMinor > 0) return `${money(0)}<span class="note"> · fully discounted, nothing charged</span>`;
  return money(order.totalMinor ?? order.subtotalMinor);
}

// ---------------------------------------------------------------- pages
export function renderOverview(data) {
  const active = data.betaAccess === "active";
  const sub = data.subscription;
  const latest = (data.orders || [])[0];
  return `<div class="grid grid-2">
    <section class="card feature"><p class="eyebrow">SKAGER for Windows</p><h2>${active ? "You are all set." : "Beta Access is not active."}</h2>
      <p class="note">${active ? "Download SKAGER, install it next to OpenCPN and you are ready to sail." : "Your purchase is not active on this account."}</p>
      <div class="actions">${active ? '<a class="button" href="/portal/downloads/">Go to downloads</a>' : ""}<a class="button button-secondary" href="/install/">Installation guide</a></div></section>
    <section class="card"><p class="eyebrow">SKAGER Cloud</p><h2>${sub && ["active", "trialing", "past_due"].includes(sub.status) ? "Cloud is active." : "Your boat, online."}</h2>
      <p class="note">Boat data, an online dashboard and web route planning for USD 2 a month.</p>
      <div class="actions"><a class="button button-secondary" href="/portal/cloud/">${sub && sub.status === "active" ? "Manage Cloud" : "Explore Cloud"}</a></div></section>
    <section class="card"><p class="eyebrow">Shop</p><h2>Upgrade your helm.</h2><p class="note">GPS modules, helm PCs and more, made for SKAGER.</p>
      <div class="actions"><a class="button button-secondary" href="/portal/shop/">Visit the shop</a></div></section>
    <section class="card"><p class="eyebrow">Latest order</p>${latest ? `<h2>${esc(latest.items.map(item => item.name).join(", ") || "Order")}</h2><p class="note">${esc(date(latest.createdAt))} · ${badge(STATUS[latest.status] || ["Unknown", "off"])}</p>` : "<h2>No orders yet.</h2>"}
      <div class="actions"><a class="button button-secondary" href="/portal/orders/">All orders</a></div></section>
  </div>`;
}

const PLATFORMS = [
  ["Windows on ARM", "Surface and other ARM64 PCs", "WIN"],
  ["Linux x64", "AppImage and .deb for Ubuntu, Debian and Fedora", "LNX"],
  ["Linux ARM64", "Raspberry Pi 5 and other ARM boards", "RPi"],
  ["macOS", "Apple silicon and Intel Macs", "MAC"],
];
const downloadUnavailable = "Downloads are being prepared. Please check back shortly.";
export function renderDownloads(downloads) {
  const coming = PLATFORMS.map(([name, detail, icon]) => `<section class="card disabled" aria-disabled="true"><div class="platform"><span class="platform-icon" aria-hidden="true">${icon}</span><div><h3>${esc(name)}</h3><p class="note">${esc(detail)}</p></div></div>${badge(["Coming", "soon"])}<div class="actions"><button class="button button-secondary" type="button" disabled>Coming</button></div></section>`).join("");
  const links = '<a class="text-link" href="/install/">Installation guide ↗</a><a class="text-link" href="/open-source/">Open source &amp; licenses ↗</a>';
  let windows;
  if (!downloads || downloads.available !== true || !Array.isArray(downloads.releases) || !downloads.releases.length) {
    windows = `<section class="card feature"><div class="platform"><span class="platform-icon" aria-hidden="true">WIN</span><div><p class="eyebrow">Windows 11 · x64</p><h2>Release delivery is closed.</h2></div></div><p class="note">${esc(downloads?.reason || downloadUnavailable)}</p><div class="actions">${links}</div></section>`;
  } else {
    // The catalog never contains bearer URLs. Each press obtains a new, short-lived
    // link after the server checks the account and entitlement again.
    const release = downloads.releases[0];
    const installer = release.artifacts.find(artifact => artifact.id === "installer");
    windows = `<section class="card feature"><div class="platform"><span class="platform-icon" aria-hidden="true">WIN</span><div><p class="eyebrow">Windows 11 · x64</p><h2>SKAGER ${esc(release.version)}</h2></div></div>
      <p class="note">OpenCPN is installed separately (5.12.4 for Windows). Read the installation guide before you install.</p>
      <div class="actions">${installer ? `<button class="button portal-download" type="button" data-release-id="${esc(release.id)}" data-artifact-id="${esc(installer.id)}">Download for Windows ↧</button>` : ""}${links}</div>
      <details><summary>All files, checksums and source revision</summary><p class="note">Source revision <code>${esc(release.sourceCommit)}</code></p><ul class="artifact-list">${release.artifacts.map(artifact => `<li><div><strong>${esc(artifact.name)}</strong><div class="meta">${esc(artifact.bytes.toLocaleString("en-GB"))} bytes · SHA-256 <code>${esc(artifact.sha256)}</code></div></div><button class="button button-secondary portal-download" type="button" data-release-id="${esc(release.id)}" data-artifact-id="${esc(artifact.id)}">Download ↧<span class="visually-hidden"> ${esc(artifact.name)}</span></button></li>`).join("")}</ul></details>
      <p class="note">Download links expire after a minute. If one expires, choose Download again. Download access does not restrict your rights under the software’s open-source licenses.</p></section>`;
  }
  return `<div class="grid">${windows}</div><h2 class="section-title">More platforms</h2><div class="grid grid-2">${coming}</div>`;
}

export function validateDownloads(data) {
  const downloadId = value => typeof value === "string" && /^[a-z0-9][a-z0-9._-]{0,95}$/.test(value);
  if (!data || data.available !== true) return { available: false, reason: downloadUnavailable, releases: [] };
  if (!Array.isArray(data.releases) || data.releases.length < 1 || data.releases.length > 20) throw new Error("Release information is unavailable.");
  for (const release of data.releases) {
    if (!downloadId(release.id) || typeof release.version !== "string" || release.version.length > 80
      || !/^[a-f0-9]{40}$/.test(release.sourceCommit) || release.channel !== "staging"
      || !Array.isArray(release.artifacts) || release.artifacts.length < 1 || release.artifacts.length > 10) throw new Error("Release information is unavailable.");
    for (const artifact of release.artifacts) {
      if (!downloadId(artifact.id) || typeof artifact.name !== "string" || artifact.name.length > 160
        || !Number.isSafeInteger(artifact.bytes) || artifact.bytes < 1 || !/^[a-f0-9]{64}$/.test(artifact.sha256)) throw new Error("Release information is unavailable.");
    }
    if (new Set(release.artifacts.map(artifact => artifact.id)).size !== release.artifacts.length) throw new Error("Release information is unavailable.");
  }
  if (new Set(data.releases.map(release => release.id)).size !== data.releases.length) throw new Error("Release information is unavailable.");
  return data;
}

export function downloadTarget(result, release, artifact, now = Date.now()) {
  const failure = () => { throw new Error("A secure download link could not be prepared. Please try again."); };
  if (!result || result.releaseId !== release.id || result.artifactId !== artifact.id || result.name !== artifact.name
    || result.bytes !== artifact.bytes || result.sha256 !== artifact.sha256 || typeof result.url !== "string") return failure();
  const expiry = Date.parse(result.expiresAt);
  if (!Number.isFinite(expiry) || expiry <= now || expiry > now + 60000) return failure();
  let url;
  try { url = new URL(result.url); } catch { return failure(); }
  const path = `/skager-private-releases/${release.channel}/${release.sourceCommit}/${release.id}/${artifact.id}/${artifact.sha256}/${artifact.name}`;
  const keys = ["response-content-disposition", "X-Amz-Expires", "X-Amz-Date", "X-Amz-Algorithm",
    "X-Amz-Credential", "X-Amz-SignedHeaders", "X-Amz-Signature"];
  const date = url.searchParams.get("X-Amz-Date") || "";
  const issued = /^\d{8}T\d{6}Z$/.test(date)
    ? Date.parse(`${date.slice(0, 4)}-${date.slice(4, 6)}-${date.slice(6, 8)}T${date.slice(9, 11)}:${date.slice(11, 13)}:${date.slice(13, 15)}Z`) : NaN;
  if (url.origin !== "https://7869adcf376c630b175c94729321e016.r2.cloudflarestorage.com"
    || url.username || url.password || url.hash || url.href !== result.url || url.pathname !== path
    || url.search.slice(1) !== url.searchParams.toString()
    || [...url.searchParams].length !== keys.length || keys.some(key => url.searchParams.getAll(key).length !== 1)
    || [...url.searchParams.keys()].some(key => !keys.includes(key))
    || url.searchParams.get("response-content-disposition") !== `attachment; filename="${artifact.name}"`
    || url.searchParams.get("X-Amz-Algorithm") !== "AWS4-HMAC-SHA256"
    || url.searchParams.get("X-Amz-Expires") !== "60" || issued + 60000 !== expiry || issued > now
    || !/^[0-9a-f]{32}\/\d{8}\/auto\/s3\/aws4_request$/.test(url.searchParams.get("X-Amz-Credential") || "")
    || url.searchParams.get("X-Amz-Credential").split("/")[1] !== date.slice(0, 8)
    || url.searchParams.get("X-Amz-SignedHeaders") !== "host"
    || !/^[0-9a-f]{64}$/.test(url.searchParams.get("X-Amz-Signature") || "")) return failure();
  return url.href;
}

export function renderShop(catalog) {
  const items = (catalog?.products || []).filter(product => product.kind === "hardware");
  if (!items.length) return '<div class="empty">The shop is being stocked. Please check back soon.</div>';
  return `<div class="grid grid-3">${items.map(product => `<section class="card product">
    <div class="product-image">${product.imageUrl && /^\/api\/media\/products\//.test(product.imageUrl) ? `<img src="${esc(product.imageUrl)}" alt="${esc(product.name)}" loading="lazy">` : "<span>SKAGER</span>"}</div>
    <div class="product-body"><h3>${esc(product.name)}</h3><p class="note">${esc(product.summary)}</p>${product.description ? `<details><summary>Details</summary><p class="note">${esc(product.description)}</p></details>` : ""}
      <p class="price">${esc(money(product.priceMinor))}</p>
      <div class="actions"><label class="qty">Qty <select data-qty="${esc(product.id)}">${[1, 2, 3, 4, 5].map(n => `<option>${n}</option>`).join("")}</select></label><button class="button" type="button" data-buy="${esc(product.id)}">Buy</button></div></div></section>`).join("")}</div>
    <p class="note after">Shipping ${esc(money(catalog.shippingMinor))} per order within the EU/EEA, the UK, Norway and Switzerland. Secure payment by Stripe.</p>`;
}

const CLOUD_FEATURES = [
  ["Boat data", "Your instruments, battery and position history, safely stored."],
  ["Online dashboard", "See your boat from anywhere: battery, bilge, shore power and more."],
  ["Web route planning", "Plan routes in your browser and send them to the boat."],
  ["Health and alerts", "Get notified when something on board needs attention."],
  ["Backup and sync", "Routes, waypoints and settings, safe and in sync."],
  ["Voyage log", "Every passage recorded, with tracks and statistics."],
];
export function renderCloud(data, catalog) {
  const product = (catalog?.products || []).find(item => item.kind === "subscription");
  const sub = data.subscription;
  const live = sub && ["active", "trialing", "past_due"].includes(sub.status);
  const status = sub ? `${badge(SUB_STATUS[sub.status] || ["Unknown", "off"])}${sub.currentPeriodEnd ? `<span class="note"> ${sub.cancelAtPeriodEnd ? "Ends" : "Renews"} ${esc(date(sub.currentPeriodEnd))}</span>` : ""}` : badge(["Not active", "off"]);
  const action = live
    ? '<button class="button button-secondary" type="button" id="cloud-manage">Manage or cancel</button>'
    : data.betaAccess === "active" && product
      ? `<button class="button" type="button" id="cloud-subscribe">Start SKAGER Cloud · ${esc(money(product.priceMinor))}/month</button>`
      : '<p class="note">SKAGER Cloud needs an active SKAGER Beta Access.</p>';
  return `<section class="card feature"><p class="eyebrow">SKAGER Cloud</p><h2>Your boat, wherever you are.</h2><div>${status}</div>
    <p class="note">One subscription for every cloud service. New services switch on for you as they launch. Cancel any time.</p><div class="actions">${action}${data.hasBilling && !live ? '<button class="button button-secondary" type="button" id="cloud-manage">Billing history</button>' : ""}</div></section>
    <h2 class="section-title">Included services</h2>
    <div class="feature-list">${CLOUD_FEATURES.map(([name, text]) => `<section class="card"><h3>${esc(name)}</h3><p class="note">${esc(text)}</p>${badge(["Coming", "soon"])}</section>`).join("")}</div>`;
}

export function renderOrders(data) {
  const orders = data.orders || [];
  if (!orders.length) return '<div class="empty">You have no orders yet.</div>';
  return `<div class="grid">${orders.map(order => {
    const lines = order.items.map(item => `<li><span>${esc(item.name)}${item.quantity > 1 ? ` × ${esc(item.quantity)}` : ""}</span><span>${esc(money(item.unitMinor * item.quantity))}${item.interval ? " / month" : ""}</span></li>`).join("");
    const shipping = order.shippingMinor > 0 ? `<li><span>Shipping</span><span>${esc(money(order.shippingMinor))}</span></li>` : "";
    const discount = order.discountMinor > 0 ? `<li><span>Discount</span><span>−${esc(money(order.discountMinor))}</span></li>` : "";
    const fulfilment = order.fulfillment === "shipped" ? ` ${badge(["Shipped", "ok"])}${order.tracking ? `<span class="note"> Tracking ${esc(order.tracking)}</span>` : ""}`
      : order.fulfillment === "unfulfilled" && order.status === "paid" ? ` ${badge(["Preparing to ship", "wait"])}` : "";
    const refund = order.refundable ? `<div class="actions"><button class="button button-danger" type="button" data-refund="${esc(order.id)}">Refund and close my account</button></div><p class="note">Within 14 days of purchase you get a full refund. Your account closes when the refund is made.</p>` : "";
    return `<section class="card order"><div class="order-head"><div><h3>${esc(order.kind === "access" ? "SKAGER Public Beta Access" : order.kind === "subscription" ? "SKAGER Cloud" : "Shop order")}</h3><p class="note">${esc(date(order.createdAt))}</p></div><div>${badge(STATUS[order.status] || ["Unknown", "off"])}${fulfilment}</div></div>
      <ul class="order-lines">${lines}${discount}${shipping}<li class="total"><span>Total</span><span>${orderTotal(order)}</span></li></ul>${refund}</section>`;
  }).join("")}</div>`;
}

export function renderAccount(data, setup) {
  if (setup) {
    return `<section class="card feature narrow"><p class="eyebrow">One last step</p><h2>Choose your password.</h2>
      <p class="note">You will use it with a code from your email each time you log in. At least 10 characters.</p>
      <form id="password-form" class="form"><label>New password<input id="new-password" type="password" autocomplete="new-password" minlength="10" maxlength="128" required></label>
      <label>Repeat password<input id="repeat-password" type="password" autocomplete="new-password" minlength="10" maxlength="128" required></label>
      <button class="button" type="submit">Save password and continue</button></form></section>`;
  }
  const boats = (data.boats || []).map(boat => `<li>${esc(boat.name)}</li>`).join("");
  return `<div class="grid grid-2">
    <section class="card"><p class="eyebrow">Account</p><h2>${esc(data.account?.email || "Your account")}</h2><p class="note">Log in with your password and a code from your email.</p>
      <form id="password-form" class="form"><label>Current password<input id="current-password" type="password" autocomplete="current-password" required maxlength="128"></label>
      <label>New password<input id="new-password" type="password" autocomplete="new-password" minlength="10" maxlength="128" required></label>
      <label>Repeat new password<input id="repeat-password" type="password" autocomplete="new-password" minlength="10" maxlength="128" required></label>
      <button class="button button-secondary" type="submit">Change password</button></form></section>
    <section class="card"><p class="eyebrow">Your boats</p><h2>${boats ? "Your fleet" : "Add your boat"}</h2>${boats ? `<ul class="order-lines">${boats}</ul>` : '<p class="note">Name your boat to personalise your account.</p>'}
      <form id="boat-form" class="form"><label>Boat name<input id="boat-name" maxlength="80" required></label><button class="button button-secondary" type="submit">Add boat</button></form></section></div>`;
}

export function renderDashboard(data, page = "dashboard", extra) {
  if (page === "downloads") return renderDownloads(extra);
  if (page === "shop") return renderShop(extra);
  if (page === "cloud") return renderCloud(data, extra);
  if (page === "orders") return renderOrders(data);
  if (page === "account") return renderAccount(data, false);
  return renderOverview(data);
}

export function stripeUrl(value, host) {
  try {
    const url = new URL(value);
    return url.protocol === "https:" && url.hostname === host && !url.username && !url.password ? url.href : null;
  } catch { return null; }
}

// ---------------------------------------------------------------- browser
const TITLES = { dashboard: "Welcome aboard.", downloads: "Downloads", shop: "Shop", cloud: "SKAGER Cloud", orders: "Orders", account: "Account", boats: "Account" };
if (typeof document !== "undefined" && document.getElementById("portal-content")) {
  let csrfToken;
  let sessionEnded = false;
  const $ = id => document.getElementById(id);
  const content = $("portal-content"), message = $("portal-message");
  const sessionMessage = "Your session has ended. Please sign in.";
  const tell = (text, kind = "") => { message.textContent = sessionEnded ? sessionMessage : text; message.className = `message ${sessionEnded ? "error" : kind}`.trim(); };
  const sessionError = () => Object.assign(new Error(sessionMessage), { status: 401 });
  const checkSession = () => { if (sessionEnded) throw sessionError(); };
  function endSession() {
    if (sessionEnded) return;
    sessionEnded = true;
    csrfToken = undefined;
    content.innerHTML = "";
    content.setAttribute("aria-busy", "false");
    $("logout").hidden = true;
    tell(sessionMessage);
    location.assign("/auth/login/");
  }
  const params = new URLSearchParams(location.search);
  const path = location.pathname.replace(/\/?$/, "/");
  // /portal/boats/ is kept as an alias for the account page.
  const page = path === "/portal/boats/" ? "boats" : Object.keys(PAGES).find(key => PAGES[key] === path) || "dashboard";
  document.querySelector("h1").textContent = TITLES[page];
  for (const link of $("portal-nav")?.querySelectorAll("a") || []) {
    if (link.dataset.page === (page === "boats" ? "account" : page)) link.setAttribute("aria-current", "page");
  }
  async function api(url, body) {
    checkSession();
    const response = await fetch(url, { credentials: "same-origin", cache: "no-store", ...(body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": csrfToken }, body: JSON.stringify(body) }) });
    if (response.status === 401) { endSession(); throw sessionError(); }
    const data = await response.json().catch(() => ({}));
    checkSession();
    if (response.status === 403 && data.setup === true) {
      location.assign("/portal/account/?setup=1");
      throw Object.assign(new Error("Choose a password to continue."), { status: 403 });
    }
    if (!response.ok) throw Object.assign(new Error(data.error || "This service is temporarily unavailable. Please try again."), { status: response.status });
    return data;
  }
  async function stripe(url, host, button) {
    button.disabled = true; tell("Opening secure Stripe page…");
    try {
      const target = stripeUrl((await api(url, button.dataset.buy ? { items: [{ productId: button.dataset.buy,
        quantity: Number(content.querySelector(`[data-qty="${button.dataset.buy}"]`)?.value || 1) }] } : {})).url, host);
      if (!target) throw new Error("Stripe could not be opened. Please try again.");
      checkSession();
      location.assign(target);
    } catch (error) { tell(error.message, "error"); button.disabled = false; }
  }
  function bindPassword(setup) {
    $("password-form")?.addEventListener("submit", async event => {
      event.preventDefault();
      const password = $("new-password").value;
      if (password !== $("repeat-password").value) { tell("The passwords do not match.", "error"); return; }
      try {
        await api("/api/auth/password", setup ? { password } : { password, current: $("current-password").value });
        if (setup) { location.assign("/portal/"); return; }
        for (const id of ["current-password", "new-password", "repeat-password"]) $(id).value = "";
        tell("Your password has been changed.", "ok");
      } catch (error) { tell(error.message, "error"); }
    });
  }
  function bind(data, extra) {
    if (page === "downloads") {
      for (const button of content.querySelectorAll(".portal-download")) {
        button.addEventListener("click", async () => {
          button.disabled = true;
          try {
            const release = extra.releases.find(item => item.id === button.dataset.releaseId);
            const artifact = release?.artifacts.find(item => item.id === button.dataset.artifactId);
            if (!artifact) throw new Error("Please refresh this page before downloading.");
            const result = await api("/api/portal/downloads", { releaseId: release.id, artifactId: artifact.id });
            checkSession();
            const link = document.createElement("a");
            link.href = downloadTarget(result, release, artifact);
            link.rel = "noreferrer";
            link.referrerPolicy = "no-referrer";
            link.download = artifact.name;
            document.body.appendChild(link);
            link.click();
            link.remove();
            tell("Your download is starting. If the link expires, choose Download again.", "ok");
          } catch (error) { tell(error.message, "error"); }
          finally { button.disabled = false; }
        });
      }
    }
    if (page === "shop") for (const button of content.querySelectorAll("[data-buy]")) button.addEventListener("click", () => stripe("/api/store/shop", "checkout.stripe.com", button));
    if (page === "cloud") {
      const subscribe = $("cloud-subscribe"), manage = $("cloud-manage");
      subscribe?.addEventListener("click", () => stripe("/api/store/cloud", "checkout.stripe.com", subscribe));
      manage?.addEventListener("click", () => stripe("/api/store/billing", "billing.stripe.com", manage));
    }
    if (page === "orders") {
      // A second press confirms; no browser dialogs.
      for (const button of content.querySelectorAll("[data-refund]")) {
        button.addEventListener("click", async () => {
          if (button.dataset.confirm !== "yes") { button.dataset.confirm = "yes"; button.textContent = "Press again to confirm the refund"; return; }
          button.disabled = true;
          try {
            const result = await api("/api/store/refund", { orderId: button.dataset.refund });
            if (result.state === "refunded") { sessionEnded = true; content.innerHTML = ""; location.assign("/?refunded=1"); return; }
            tell("Your refund is on its way. Your account closes when it completes.", "ok");
          } catch (error) { tell(error.message, "error"); button.disabled = false; }
        });
      }
    }
    if (page === "account" || page === "boats") {
      bindPassword(false);
      $("boat-form")?.addEventListener("submit", async event => {
        event.preventDefault();
        const button = event.currentTarget.querySelector("button");
        button.disabled = true;
        try { await api("/api/portal/boats", { name: $("boat-name").value }); if (await load()) { checkSession(); tell("Your boat has been added.", "ok"); } }
        catch (error) { tell(error.message, "error"); button.disabled = false; }
      });
    }
  }
  async function load() {
    try {
      const session = await api("/api/auth/session");
      checkSession();
      if (!session.account) { endSession(); return false; }
      if (typeof session.csrfToken !== "string" || !session.csrfToken) throw new Error("A secure session could not be started. Please reload this page.");
      csrfToken = session.csrfToken;
      if (session.account.admin === true && $("admin-link")) $("admin-link").hidden = false;
      if (session.account.passwordSet === false) {
        if (page !== "account") { location.assign("/portal/account/?setup=1"); return false; }
        content.innerHTML = renderAccount({}, true);
        content.setAttribute("aria-busy", "false");
        $("logout").hidden = false;
        tell("");
        bindPassword(true);
        return true;
      }
      const data = await api("/api/portal/dashboard");
      let extra = null;
      if (page === "downloads" && data.betaAccess === "active") {
        try { extra = validateDownloads(await api("/api/portal/downloads")); }
        catch (error) {
          if (error.status === 401) throw error;
          extra = { available: false, reason: error.status === 404 ? downloadUnavailable
            : error.status === 403 ? "Your SKAGER access could not be confirmed. Refresh this page before downloading."
            : "Release information is temporarily unavailable. Please try again later." };
        }
      } else if (page === "downloads") extra = { available: false, reason: "Downloads unlock with an active SKAGER purchase." };
      if (page === "shop" || page === "cloud") extra = await api("/api/store/catalog");
      // A second request can end the session after api resolves but before
      // this caller resumes. Guard each private effect at its await boundary.
      checkSession();
      content.innerHTML = renderDashboard(data, page === "boats" ? "account" : page, extra);
      content.setAttribute("aria-busy", "false");
      $("logout").hidden = false;
      if ($("portal-nav")) $("portal-nav").hidden = false;
      tell(page === "orders" && params.has("paid") ? "Thank you. New orders appear here as soon as Stripe confirms the payment."
        : page === "cloud" && params.has("activated") ? "Thank you. SKAGER Cloud switches on as soon as Stripe confirms the payment." : "", "ok");
      bind(data, extra);
      return true;
    } catch (error) { content.setAttribute("aria-busy", "false"); tell(error.message, "error"); return false; }
  }
  $("logout").addEventListener("click", async event => {
    event.target.disabled = true;
    try { await api("/api/auth/logout", {}); endSession(); }
    catch (error) { tell(error.message, "error"); event.target.disabled = false; }
  });
  load();
}
