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
export const PAGES = { dashboard: "/portal/", downloads: "/portal/downloads/", shop: "/portal/shop/", cloud: "/portal/cloud/", orders: "/portal/orders/", account: "/portal/account/", link: "/link/" };
/** Placeholder cards shown while a tab's data is on its way (same markup as portal.html). */
export const SKELETON = '<div class="skeleton-grid" aria-hidden="true"><div class="skeleton-card"></div><div class="skeleton-card"></div><div class="skeleton-card wide"></div></div>';
/** Portal page for a path; /portal/boats/ is kept as an alias for the account page. */
export function pageForPath(pathname) {
  const path = pathname.replace(/\/?$/, "/");
  return path === "/portal/boats/" ? "boats" : Object.keys(PAGES).find(key => PAGES[key] === path) || "dashboard";
}
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
      <p class="note">Automatic updates, boat data and your boat online for USD 2 a month.</p>
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
/** Admin-defined steps of the download page: "download" steps (such as installing OpenCPN) come
 * before SKAGER, "guide" steps (such as setting up GRIBstream or AISstream) after it. */
export function downloadSteps(catalog, flow = "download") {
  return (Array.isArray(catalog?.steps) ? catalog.steps : []).filter(step => step && step.flow === flow
    && typeof step.title === "string" && typeof step.skipLabel === "string");
}
const safeLink = value => {
  try { const url = new URL(value); return url.protocol === "https:" && !url.username && !url.password ? url.href : null; } catch { return null; }
};
/** First line of a step's text is the introduction; each further line becomes a numbered step. */
function stepText(summary) {
  const [intro, ...rest] = String(summary || "").split("\n").map(line => line.trim()).filter(Boolean);
  return `${intro ? `<p class="lead">${esc(intro)}</p>` : ""}${rest.length ? `<ol class="guide-list">${rest.map(line => `<li>${esc(line)}</li>`).join("")}</ol>` : ""}`;
}
/** One step at a time: each prerequisite, the SKAGER download itself, then the get-going guide. */
export function renderSetup(downloads, steps = [], current = steps.length, used = [], guide = []) {
  if (!steps.length && !guide.length) return renderDownloads(downloads);
  const download = steps.length, total = steps.length + 1 + guide.length;
  const at = Math.max(0, Math.min(current, total));
  const names = [...steps.map(step => step.title), "Download SKAGER", ...guide.map(step => step.title)];
  const nav = `<ol class="setup-steps" aria-label="Install steps">${names.map((name, index) => `<li${index === at ? ' aria-current="step"' : index < at ? ' class="is-done"' : ""}><button type="button" data-setup-step="${index}"><span aria-hidden="true">${index < at ? "✓" : index + 1}</span>${esc(name)}</button></li>`).join("")}</ol>`;
  const back = at > 0 ? `<div class="actions"><button class="text-link" type="button" data-setup-step="${at - 1}">← Back</button></div>` : "";
  if (at === download) {
    return `${nav}<p class="eyebrow step-of">Step ${at + 1} of ${names.length}</p><h2 class="visually-hidden" id="setup-title" tabindex="-1">Download SKAGER</h2>${renderDownloads(downloads)}
      <div class="actions after">${at > 0 ? `<button class="button button-secondary" type="button" data-setup-step="${at - 1}">← Back</button>` : ""}${guide.length ? `<button class="button" type="button" data-setup-step="${at + 1}">Next: get going →</button>` : ""}</div>`;
  }
  if (at === total) {
    return `${nav}<section class="card feature setup-card"><p class="eyebrow">All set</p><h2 id="setup-title" tabindex="-1">You are ready to go</h2>
      <p class="lead">SKAGER is installed. These steps stay here on the Downloads page whenever you need them again.</p>
      <div class="actions"><button class="button button-secondary" type="button" data-setup-step="0">Start again</button></div>${back}</section>`;
  }
  const step = at < download ? steps[at] : guide[at - download - 1];
  const link = step.linkUrl && step.linkLabel ? safeLink(step.linkUrl) : null, clicked = used.includes(at);
  const linkButton = link ? `<a class="button${clicked ? " button-secondary" : ""}" href="${esc(link)}" target="_blank" rel="noopener noreferrer" data-setup-link="${at}">${esc(step.linkLabel)} ↗</a>` : "";
  const next = `<button class="button${link && !clicked ? " button-secondary" : ""}" type="button" data-setup-step="${at + 1}">${clicked ? (at === total - 1 ? "Done, finish →" : "Done, next step →") : esc(step.skipLabel)}</button>`;
  return `${nav}<section class="card feature setup-card"><p class="eyebrow">${at > download ? "Get going · " : ""}Step ${at + 1} of ${names.length}</p><h2 id="setup-title" tabindex="-1">${esc(step.title)}</h2>
    ${stepText(step.summary)}<div class="actions">${linkButton}${next}</div>
    ${back}</section>`;
}
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
  ["Automatic updates", "New SKAGER versions install themselves on your helm PC.", true],
  ["Boat data", "Your instruments, battery and position, with 30 days of history.", true],
  ["Online dashboard", "See your boat from anywhere: battery, bilge, shore power and more."],
  ["Web route planning", "Plan routes in your browser and send them to the boat."],
  ["Health and alerts", "Get notified when something on board needs attention."],
  ["Backup and sync", "Routes, waypoints and settings, safe and in sync."],
  ["Voyage log", "Every passage recorded, with tracks and statistics."],
];
// ---------------------------------------------------------------- SKAGER Cloud: boats and pairing
const ago = seconds => seconds < 90 ? `${Math.max(0, Math.round(seconds))} s ago` : seconds < 5400 ? `${Math.round(seconds / 60)} min ago`
  : seconds < 172800 ? `${Math.round(seconds / 3600)} h ago` : `${Math.round(seconds / 86400)} days ago`;
const since = (value, now) => { const t = Date.parse(value); return Number.isFinite(t) ? (now - t) / 1000 : null; };
const GEAR = ["Forward", "Neutral", "Reverse"], REGEN = ["Off", "One bar", "Two bars"];
const DEGREES = (v, pos, neg) => { const a = Math.abs(v), d = Math.floor(a); return `${d}° ${((a - d) * 60).toFixed(3)}′ ${v >= 0 ? pos : neg}`; };
/** A value in its unit as the boat reported it. Never a default: missing stays missing. */
export function formatReading(key, value, unit) {
  if (typeof value !== "number" || !Number.isFinite(value)) return "—";
  if (key === "navigation.latitude") return DEGREES(value, "N", "S");
  if (key === "navigation.longitude") return DEGREES(value, "E", "W");
  if (key === "propulsion.gear") return GEAR[Math.round(value)] ?? "—";
  if (key === "propulsion.regeneration") return REGEN[Math.round(value)] ?? "—";
  const fixed = { deg: 0, kn: 1, m: 1, C: 1, hPa: 0, "%": 0, V: 2, A: 1, kW: 1, kWh: 1, rpm: 0, "msg/s": 0 }[unit] ?? 1;
  const suffix = { deg: "°", C: " °C", "%": " %" }[unit] ?? ` ${unit}`;
  return `${value.toFixed(fixed)}${suffix}`;
}
const GROUPS = [["navigation", "Position and motion"], ["wind", "Wind"], ["environment", "Water and weather"],
  ["battery", "Battery"], ["propulsion", "Motor"], ["tanks", "Tanks"], ["steering", "Steering"], ["connectivity", "Instruments"]];
const QUALITY = { live: null, aging: ["Aging", "wait"], stale: ["Stale", "off"], estimated: ["Estimated", "wait"], uncertain: ["Uncertain", "wait"] };
/** Inline SVG sparkline (no script, no style attribute; CSP-safe). */
export function sparkline(points, width = 280, height = 56) {
  const clean = (Array.isArray(points) ? points : []).filter(p => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1]));
  if (clean.length < 2) return '<p class="note">Not enough history yet.</p>';
  const xs = clean.map(p => p[0]), ys = clean.map(p => p[1]);
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  const sx = x => ((x - x0) / Math.max(1, x1 - x0)) * (width - 4) + 2, sy = y => height - 2 - ((y - y0) / Math.max(1e-9, y1 - y0)) * (height - 4);
  const path = clean.map(([x, y], i) => `${i ? "L" : "M"}${sx(x).toFixed(1)},${sy(y).toFixed(1)}`).join("");
  return `<svg class="spark" viewBox="0 0 ${width} ${height}" width="${width}" height="${height}" role="img" aria-hidden="true"><path d="${path}" fill="none" stroke="currentColor" stroke-width="2"/></svg>`;
}
/** Latest snapshot of one boat. Old data is labelled as old; nothing is shown as live without its age. */
export function renderBoat(boat, keys, now = Date.now()) {
  if (!boat || !boat.device) return "";
  const received = since(boat.receivedAt, now);
  const items = boat.items && typeof boat.items === "object" ? boat.items : {};
  const head = received === null
    ? `<p class="note">${boat.device.telemetryEnabled ? "No boat data received yet." : "Boat data sharing is off in SKAGER (Settings → System → SKAGER Cloud)."}</p>`
    : `<p class="note">Last update ${esc(ago(received))}${received > 120 ? " · SKAGER is offline or not sharing right now" : ""}</p>`;
  const groups = GROUPS.map(([prefix, title]) => {
    const rows = Object.keys(keys || {}).filter(key => key.startsWith(prefix + ".") && items[key]).map(key => {
      const item = items[key], spec = keys[key];
      const age = received === null ? null : received + (Number(item.age) || 0);
      const quality = QUALITY[item.q] || null;
      return `<li><span>${esc(spec.label)}</span><span><strong>${esc(formatReading(key, item.v, spec.unit))}</strong>${quality ? ` ${badge(quality)}` : ""}<span class="meta">${age === null ? "" : esc(ago(age))}${item.src ? ` · ${esc(item.src)}` : ""}</span></span></li>`;
    }).join("");
    return rows ? `<section class="card"><h3>${esc(title)}</h3><ul class="order-lines readings">${rows}</ul></section>` : "";
  }).join("");
  const lat = items["navigation.latitude"]?.v, lon = items["navigation.longitude"]?.v;
  const map = Number.isFinite(lat) && Number.isFinite(lon)
    ? `<a class="text-link" href="https://www.openstreetmap.org/?mlat=${lat.toFixed(5)}&amp;mlon=${lon.toFixed(5)}#map=13/${lat.toFixed(5)}/${lon.toFixed(5)}" target="_blank" rel="noopener noreferrer">Show position on a map ↗</a>` : "";
  const series = Object.entries(boat.series || {}).filter(([key]) => keys?.[key]).map(([key, points]) =>
    `<section class="card"><h3>${esc(keys[key].label)} · last 24 h</h3>${sparkline(points)}</section>`).join("");
  return `<div class="boat-view">${head}${map ? `<div class="actions">${map}</div>` : ""}<div class="grid grid-2">${groups}${series}</div></div>`;
}
/** Linked SKAGER installations, each with its latest data and an Unlink action. */
export function renderDevices(cloud, boat, now = Date.now()) {
  if (!cloud) return "";
  const devices = Array.isArray(cloud.devices) ? cloud.devices : [];
  const intro = '<p class="note">Link SKAGER under Settings → System → SKAGER Cloud. Linked computers receive automatic updates and can share boat data while SKAGER Cloud is active.</p>';
  if (!devices.length) return `<h2 class="section-title">Your boats</h2><section class="card">${intro}<div class="actions"><a class="button button-secondary" href="/link/">Enter a pairing code</a></div></section>`;
  return `<h2 class="section-title">Your boats</h2>${intro}<div class="grid">${devices.map(device => {
    const seen = since(device.lastSeenAt, now), selected = boat?.device?.id === device.id;
    return `<section class="card"><div class="order-head"><div><h3>${esc(device.name)}</h3><p class="note">SKAGER ${esc(device.appVersion)} · ${esc(device.platform === "linux" ? "Linux" : "Windows")}${seen === null ? "" : ` · seen ${esc(ago(seen))}`}</p></div>
      <div>${badge(device.telemetryEnabled ? ["Sharing boat data", "ok"] : ["Not sharing", "off"])}</div></div>
      <div class="actions"><button class="button${selected ? " button-secondary" : ""}" type="button" data-boat="${esc(device.id)}">${selected ? "Refresh" : "Show boat data"}</button><button class="button button-danger" type="button" data-unlink="${esc(device.id)}">Unlink</button></div>
      ${selected ? renderBoat(boat, cloud.telemetryKeys, now) : ""}</section>`;
  }).join("")}</div>`;
}
/** The page the SKAGER app sends people to: enter its code, check the computer, approve. */
export function renderLink(state = {}) {
  if (state.done) {
    return `<section class="card feature narrow"><p class="eyebrow">Linked</p><h2>${esc(state.done.name)} is linked.</h2>
      <p class="note">Go back to SKAGER. It shows your account within a few seconds.</p><div class="actions"><a class="button button-secondary" href="/portal/cloud/">Your boats</a></div></section>`;
  }
  if (state.found) {
    const f = state.found;
    return `<section class="card feature narrow"><p class="eyebrow">Link SKAGER</p><h2>Link ${esc(f.name)}?</h2>
      <p class="note">SKAGER ${esc(f.appVersion || "")} on ${esc(f.platform === "linux" ? "Linux" : "Windows")}. Only approve if this code is on your own SKAGER screen right now.</p>
      <div class="actions"><button class="button" type="button" id="link-approve">Link to my account</button><button class="button button-secondary" type="button" id="link-decline">This is not my computer</button></div></section>`;
  }
  return `<section class="card feature narrow"><p class="eyebrow">Link SKAGER</p><h2>Enter the code from SKAGER.</h2>
    <p class="note">In SKAGER, open Settings → System → SKAGER Cloud and choose Link to my account. The 8-character code appears there.</p>
    <form id="link-form" class="form"><label>Pairing code<input id="link-code" autocomplete="off" autocapitalize="characters" spellcheck="false" maxlength="9" required placeholder="BCDF-2345"></label>
    <button class="button" type="submit">Continue</button></form></section>`;
}

export function renderCloud(data, catalog, cloud, boat) {
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
    ${renderDevices(cloud, boat)}
    <h2 class="section-title">Included services</h2>
    <div class="feature-list">${CLOUD_FEATURES.map(([name, text, live]) => `<section class="card"><h3>${esc(name)}</h3><p class="note">${esc(text)}</p>${badge(live ? ["Available", "ok"] : ["Coming", "soon"])}</section>`).join("")}</div>`;
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

export function renderDashboard(data, page = "dashboard", extra, setup, cloud) {
  if (page === "downloads") return setup ? renderSetup(extra, setup.steps, setup.current, setup.used, setup.guide) : renderDownloads(extra);
  if (page === "shop") return renderShop(extra);
  if (page === "cloud") return renderCloud(data, extra, cloud?.overview, cloud?.boat);
  if (page === "link") return renderLink(cloud?.link);
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
const TITLES = { dashboard: "Welcome aboard.", downloads: "Downloads", shop: "Shop", cloud: "SKAGER Cloud", orders: "Orders", account: "Account", boats: "Account", link: "Link SKAGER" };
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
    if ($("portal-nav")) $("portal-nav").hidden = true;
    tell(sessionMessage);
    location.assign(page === "link" ? "/auth/login/?next=/link/" : "/auth/login/");
  }
  let page = pageForPath(location.pathname);
  // Portal data lives in this page's memory only (never in browser storage), so
  // switching tabs renders at once and refreshes in the background.
  const cache = {};
  // Where the customer is in the download steps; memory only, like the rest.
  const setup = { current: 0, used: [] };
  // SKAGER Cloud: linked devices, the boat being viewed, and the pairing page.
  const cloudView = { overview: undefined, boat: null, link: {} };
  const setupFor = p => p === "downloads" && cache.dashboard?.betaAccess === "active"
    ? { steps: downloadSteps(cache.catalog), guide: downloadSteps(cache.catalog, "guide"), current: setup.current, used: setup.used } : undefined;
  function mark() {
    document.querySelector("h1").textContent = TITLES[page];
    for (const link of $("portal-nav")?.querySelectorAll("a") || []) {
      if (link.dataset.page === (page === "boats" ? "account" : page)) link.setAttribute("aria-current", "page");
      else link.removeAttribute("aria-current");
    }
  }
  mark();
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
      for (const button of content.querySelectorAll("[data-setup-step]")) {
        button.addEventListener("click", () => {
          setup.current = Number(button.dataset.setupStep);
          render();
          $("setup-title")?.focus({ preventScroll: true });
          if (content.getBoundingClientRect().top < 0) content.scrollIntoView({ block: "start", behavior: "smooth" });
        });
      }
      for (const link of content.querySelectorAll("[data-setup-link]")) {
        link.addEventListener("click", () => {
          if (!setup.used.includes(Number(link.dataset.setupLink))) setup.used.push(Number(link.dataset.setupLink));
          // After the link opens, the next step becomes the main action.
          setTimeout(() => { render(); content.querySelector(`[data-setup-step="${setup.current + 1}"]`)?.focus(); }, 0);
        });
      }
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
      for (const button of content.querySelectorAll("[data-boat]")) {
        button.addEventListener("click", async () => {
          button.disabled = true;
          try {
            const deviceId = button.dataset.boat;
            const keys = Object.keys(cloudView.overview?.telemetryKeys || {});
            const latest = await api("/api/cloud/boat", { deviceId });
            const chart = ["navigation.sog", "battery.soc", "environment.depth", "wind.true_speed"].filter(key => keys.includes(key) && latest.items?.[key]);
            const series = chart.length ? (await api("/api/cloud/boat/series", { deviceId, keys: chart, hours: 24 })).series : {};
            checkSession();
            cloudView.boat = { ...latest, series };
            render();
          } catch (error) { tell(error.message, "error"); button.disabled = false; }
        });
      }
      for (const button of content.querySelectorAll("[data-unlink]")) {
        button.addEventListener("click", async () => {
          // A second press confirms; no browser dialogs.
          if (button.dataset.confirm !== "yes") { button.dataset.confirm = "yes"; button.textContent = "Press again to unlink"; return; }
          button.disabled = true;
          try {
            await api("/api/cloud/devices/revoke", { deviceId: button.dataset.unlink });
            if (cloudView.boat?.device?.id === button.dataset.unlink) cloudView.boat = null;
            cloudView.overview = await api("/api/cloud/overview");
            checkSession();
            render();
            tell("Unlinked. That SKAGER no longer receives automatic updates or shares boat data.", "ok");
          } catch (error) { tell(error.message, "error"); button.disabled = false; }
        });
      }
    }
    if (page === "link") {
      const decide = approve => async () => {
        for (const id of ["link-approve", "link-decline"]) if ($(id)) $(id).disabled = true;
        try {
          const result = await api("/api/cloud/link/decide", { code: cloudView.link.code, approve });
          checkSession();
          cloudView.link = approve && result.state === "approved" ? { done: { name: result.name } } : {};
          render();
          if (!approve) tell("Declined. That computer was not linked.", "ok");
        } catch (error) { cloudView.link = {}; render(); tell(error.message, "error"); }
      };
      $("link-approve")?.addEventListener("click", decide(true));
      $("link-decline")?.addEventListener("click", decide(false));
      $("link-form")?.addEventListener("submit", async event => {
        event.preventDefault();
        const button = event.currentTarget.querySelector("button");
        button.disabled = true;
        try {
          const code = $("link-code").value;
          const found = await api("/api/cloud/link/find", { code });
          checkSession();
          cloudView.link = { code, found };
          render();
        } catch (error) { tell(error.message, "error"); button.disabled = false; }
      });
      $("link-code")?.focus();
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
  const shopPage = p => p === "shop" || p === "cloud";
  async function downloadsCatalog() {
    try { return validateDownloads(await api("/api/portal/downloads")); }
    catch (error) {
      if (error.status === 401) throw error;
      return { available: false, reason: error.status === 404 ? downloadUnavailable
        : error.status === 403 ? "Your SKAGER access could not be confirmed. Refresh this page before downloading."
        : "Release information is temporarily unavailable. Please try again later." };
    }
  }
  function extraFor(p) {
    if (p === "downloads") return cache.dashboard.betaAccess === "active" ? cache.downloads : { available: false, reason: "Downloads unlock with an active SKAGER purchase." };
    return shopPage(p) ? cache.catalog : null;
  }
  const cached = p => cache.dashboard !== undefined && extraFor(p) !== undefined && (p !== "cloud" || cloudView.overview !== undefined);
  // What the visible tab was last rendered from, so a refresh re-renders only on change.
  let rendered = null;
  const shownFrom = () => JSON.stringify([page, cache.dashboard, extraFor(page), setupFor(page), page === "cloud" || page === "link" ? cloudView : null]);
  /** Fetch what page p shows into the cache. */
  async function refresh(p) {
    // The download steps are guidance: without the catalog the download itself still shows.
    const [dashboard, catalog] = await Promise.all([api("/api/portal/dashboard"), shopPage(p) ? api("/api/store/catalog")
      : p === "downloads" ? api("/api/store/catalog").catch(error => { if (error.status === 401) throw error; return cache.catalog; }) : cache.catalog]);
    const downloads = p === "downloads" && dashboard.betaAccess === "active" ? await downloadsCatalog() : cache.downloads;
    // Linked devices are part of the Cloud page; failing to load them never hides the subscription.
    const overview = p === "cloud" ? await api("/api/cloud/overview").catch(error => { if (error.status === 401) throw error; return cloudView.overview ?? null; }) : cloudView.overview;
    // A second request can end the session after api resolves but before
    // this caller resumes. Guard each private effect at its await boundary.
    checkSession();
    Object.assign(cache, { dashboard, catalog, downloads });
    cloudView.overview = overview;
  }
  function render() {
    rendered = shownFrom();
    content.innerHTML = renderDashboard(cache.dashboard, page === "boats" ? "account" : page, extraFor(page), setupFor(page), cloudView);
    content.setAttribute("aria-busy", "false");
    $("logout").hidden = false;
    if ($("portal-nav")) $("portal-nav").hidden = false;
    const params = new URLSearchParams(location.search);
    tell(page === "orders" && params.has("paid") ? "Thank you. New orders appear here as soon as Stripe confirms the payment."
      : page === "cloud" && params.has("activated") ? "Thank you. SKAGER Cloud switches on as soon as Stripe confirms the payment." : "", "ok");
    bind(cache.dashboard, extraFor(page));
  }
  /** Show page p from memory when possible, then revalidate and re-render only on change. */
  async function show(p) {
    page = p;
    mark();
    if (cached(p)) render();
    else { rendered = null; content.innerHTML = SKELETON; content.setAttribute("aria-busy", "true"); tell(""); }
    try {
      await refresh(p);
      if (page !== p) return false;
      if (shownFrom() !== rendered) render();
      return true;
    } catch (error) {
      if (page === p) { if (rendered === null) content.innerHTML = ""; content.setAttribute("aria-busy", "false"); tell(error.message, "error"); }
      return false;
    }
  }
  /** Load the other tabs' data once in the background so the first visit is instant too. */
  function prefetch() {
    const jobs = [];
    if (cache.catalog === undefined) jobs.push(api("/api/store/catalog").then(value => { cache.catalog ??= value; }));
    if (cache.downloads === undefined && cache.dashboard?.betaAccess === "active") jobs.push(downloadsCatalog().then(value => { cache.downloads ??= value; }));
    Promise.all(jobs).catch(() => {});
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
        if ($("portal-nav")) $("portal-nav").hidden = true;
        content.innerHTML = renderAccount({}, true);
        content.setAttribute("aria-busy", "false");
        $("logout").hidden = false;
        tell("");
        bindPassword(true);
        return true;
      }
    } catch (error) { content.innerHTML = ""; content.setAttribute("aria-busy", "false"); tell(error.message, "error"); return false; }
    const shown = await show(page);
    if (shown) prefetch();
    return shown;
  }
  $("portal-nav")?.addEventListener("click", event => {
    const link = event.target.closest("a[data-page]");
    if (!link || cache.dashboard === undefined || sessionEnded || event.button !== 0
      || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
    event.preventDefault();
    if (link.dataset.page === page) return;
    history.pushState(null, "", link.getAttribute("href"));
    show(link.dataset.page);
  });
  addEventListener("popstate", () => { if (cache.dashboard !== undefined && !sessionEnded) show(pageForPath(location.pathname)); });
  $("logout").addEventListener("click", async event => {
    event.target.disabled = true;
    try { await api("/api/auth/logout", {}); endSession(); }
    catch (error) { tell(error.message, "error"); event.target.disabled = false; }
  });
  load();
}
