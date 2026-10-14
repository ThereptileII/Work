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
const downloadUnavailable = "Release qualification and secure download delivery are pending.";
const downloadId = value => typeof value === "string" && /^[a-z0-9][a-z0-9._-]{0,95}$/.test(value);

// The catalog never contains bearer URLs. Each press obtains a new, short-lived
// link after the server checks the account and entitlement again.
export function renderDownloads(downloads) {
  if (!downloads || downloads.available !== true || !Array.isArray(downloads.releases) || !downloads.releases.length) {
    return card("DOWNLOAD AVAILABILITY", "Release delivery is closed.", `<p>${esc(downloads?.reason || downloadUnavailable)}</p><p class="portal-note">No installer link is available here yet.</p><a class="text-link" href="/install/">Installation prerequisites ↗</a><a class="text-link" href="/open-source/">Open source &amp; licenses ↗</a>`);
  }
  return card("PRIVATE STAGING RELEASES", "Ready for your next test.", '<p>OpenCPN is installed separately. Read the installation guide and release notes before updating.</p><a class="text-link" href="/install/">Installation guide ↗</a><a class="text-link" href="/open-source/">Open source &amp; licenses ↗</a>'
    + downloads.releases.map(release => `<section class="portal-release"><h3>${esc(release.version)} <span class="portal-note">${esc(release.channel)}</span></h3><details><summary>Source revision</summary><code class="portal-note">${esc(release.sourceCommit)}</code></details><ul class="portal-list">${release.artifacts.map(artifact => `<li><strong>${esc(artifact.name)}</strong><span class="portal-note">${esc(artifact.bytes.toLocaleString("en-GB"))} bytes</span><details><summary>SHA-256 checksum</summary><code class="portal-note">${esc(artifact.sha256)}</code></details><button class="button button-secondary portal-download" type="button" data-release-id="${esc(release.id)}" data-artifact-id="${esc(artifact.id)}">Download ↧<span class="visually-hidden"> ${esc(artifact.name)}</span></button></li>`).join("")}</ul></section>`).join("")
    + '<p class="portal-note">Download links expire quickly. If a link expires, choose Download again. Download access does not restrict your rights under the software’s open-source licenses.</p>');
}

export function validateDownloads(data) {
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
export function renderDashboard(data, page = "dashboard", downloads = null) {
  if (page === "account") return `<div class="portal-grid">${card("ACCOUNT DETAILS", "A place for your passages.", `<dl class="portal-details"><dt>Contact email</dt><dd>${esc(data.account.email || "Not provided")}</dd></dl><p class="portal-note">Profile editing is not available here yet. Sign-in uses a code sent to your email.</p>`)}${renderAccess(data)}</div>`;
  if (page === "boats") return `<div class="portal-grid">${card("YOUR FLEET", "It starts with a name.", renderBoats(data.boats) + '<p class="portal-note">Saved boat names only. Connected instruments, telemetry, position, renaming and removal are not available yet.</p>')}${card("ADD A BOAT", "Make it yours.", '<form id="boat-form" class="portal-form"><label for="boat-name">Boat name</label><input id="boat-name" name="name" required maxlength="80" autocomplete="off" placeholder="e.g. S/Y Aurora"><p class="portal-note">Up to 80 characters. Only your account can see this name.</p><button class="button" type="submit">Add boat +</button></form>')}</div>`;
  if (page === "orders") return card("YOUR PRIVATE STAGING HISTORY", "Every beta order, clearly.", '<p>Only your SKAGER beta test orders appear here. Test orders do not record real money received.</p>' + renderOrders(data.orders) + '<p class="portal-note">Invoice and receipt delivery are not available in this test portal.</p>');
  if (page === "downloads") return `<div class="portal-grid">${renderAccess(data)}${renderDownloads(downloads)}</div>`;
  return `<div class="portal-grid">${renderAccess(data)}${card("MY BOATS", "Built around your boat.", renderBoats(data.boats) + '<a class="text-link" href="/portal/boats/">Manage my boats ↗</a>')}${card("LATEST ORDERS", "Your beta journey.", renderOrders(data.orders.slice(0, 3)) + '<a class="text-link" href="/portal/orders/">Orders &amp; billing ↗</a>')}${card("SUPPORT &amp; FEEDBACK", "Help along the way.", '<p>Guides and frequently asked questions are available now. Customer bug reports, feature requests and their status are not available in this test area yet.</p><a class="text-link" href="/support/">Read help &amp; FAQ ↗</a>')}</div>`;
}

if (typeof document !== "undefined") {
  let csrfToken;
  let sessionEnded = false;
  const content = document.getElementById("portal-content");
  const message = document.getElementById("portal-message");
  const tell = text => { message.textContent = text; };
  const sessionError = () => Object.assign(new Error("Your session has ended. Please sign in."), { status: 401 });
  function endSession() {
    if (sessionEnded) return;
    sessionEnded = true;
    csrfToken = undefined;
    content.innerHTML = "";
    content.setAttribute("aria-busy", "false");
    document.getElementById("logout").hidden = true;
    tell("Your session has ended. Please sign in.");
    location.assign("/auth/login/");
  }
  const page = location.pathname.split("/").filter(Boolean)[1] || "dashboard";
  const titles = { dashboard: "Welcome aboard.", account: "Your account.", boats: "My boats.", orders: "Orders & billing.", downloads: "Your next passage." };
  document.querySelector("h1").textContent = titles[page] || titles.dashboard;
  document.querySelector(`[data-page="${Object.hasOwn(titles, page) ? page : "dashboard"}"]`)?.setAttribute("aria-current", "page");
  async function api(path, body) {
    if (sessionEnded) throw sessionError();
    const response = await fetch(path, { credentials: "same-origin", cache: "no-store", ...(body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": csrfToken }, body: JSON.stringify(body) }) });
    if (response.status === 401) { endSession(); throw sessionError(); }
    const data = await response.json().catch(() => ({}));
    if (sessionEnded) throw sessionError();
    if (!response.ok) throw Object.assign(new Error(data.error || "This service is temporarily unavailable. Please try again."), { status: response.status });
    return data;
  }
  async function load() {
    try {
      const session = await api("/api/auth/session");
      if (!session.account) { endSession(); return false; }
      if (typeof session.csrfToken !== "string" || !session.csrfToken) throw new Error("A secure session could not be started. Please reload this page.");
      csrfToken = session.csrfToken;
      const data = await api("/api/portal/dashboard");
      let downloads = null;
      if (page === "downloads" && data.betaAccess === "active") {
        try { downloads = validateDownloads(await api("/api/portal/downloads")); }
        catch (error) {
          if (error.status === 401) throw error;
          downloads = { available: false, reason: error.status === 404 ? downloadUnavailable
            : error.status === 403 ? "Your beta access could not be confirmed. Refresh your account before downloading."
            : "Release information is temporarily unavailable. Please try again later." };
        }
      }
      content.innerHTML = renderDashboard(data, page, downloads);
      content.setAttribute("aria-busy", "false");
      document.getElementById("logout").hidden = false;
      tell("");
      for (const button of content.querySelectorAll(".portal-download")) {
        button.addEventListener("click", async () => {
          button.disabled = true;
          try {
            const release = downloads.releases.find(item => item.id === button.dataset.releaseId);
            const artifact = release?.artifacts.find(item => item.id === button.dataset.artifactId);
            if (!artifact) throw new Error("Please refresh release information before downloading.");
            const result = await api("/api/portal/downloads", { releaseId: release.id, artifactId: artifact.id });
            const link = document.createElement("a");
            link.href = downloadTarget(result, release, artifact);
            link.rel = "noreferrer";
            link.referrerPolicy = "no-referrer";
            link.download = artifact.name;
            document.body.appendChild(link);
            link.click();
            link.remove();
            tell("Your download is starting. If the link expires, choose Download again.");
          } catch (error) { tell(error.message); }
          finally { button.disabled = false; }
        });
      }
      document.getElementById("boat-form")?.addEventListener("submit", async event => {
        event.preventDefault();
        const button = event.currentTarget.querySelector("button");
        button.disabled = true;
        try { await api("/api/portal/boats", { name: document.getElementById("boat-name").value }); if (await load()) tell("Your boat has been added."); }
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
      return true;
    } catch (error) { content.setAttribute("aria-busy", "false"); tell(error.message); return false; }
  }
  document.getElementById("logout").addEventListener("click", async event => {
    event.target.disabled = true;
    try { await api("/api/auth/logout", {}); endSession(); }
    catch (error) { tell(error.message); event.target.disabled = false; }
  });
  load();
}
