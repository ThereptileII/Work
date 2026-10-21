// Sign-in for existing customers. Accounts are only ever created by a purchase.
const mode = ["login", "forgot", "verify"].find(value => location.pathname.replace(/\/$/, "") === `/auth/${value}`) || "login";
const $ = id => document.getElementById(id);
const form = $("auth-form"), message = $("auth-message"), submit = $("auth-submit");
const emailInput = $("auth-email"), passwordInput = $("auth-password"), codeInput = $("auth-code");
const copy = {
  login: ["Welcome back.", "Log in with your password. We will then email you a code.", "Log in"],
  forgot: ["Forgot your password?", "Enter your email. We will send you a code, then you choose a new password.", "Send code"],
  verify: ["Check your email.", "Enter the code we sent you. Keep this browser open.", "Verify"],
};
let step = mode === "verify" ? "code" : "start";
let csrfToken;
function show() {
  const [title, intro, action] = step === "code" ? copy.verify : copy[mode];
  $("auth-title").textContent = title;
  $("auth-intro").textContent = intro;
  submit.textContent = action;
  document.title = `${title} — SKAGER`;
  $("email-field").hidden = step === "code";
  $("password-field").hidden = step === "code" || mode !== "login";
  $("code-field").hidden = step !== "code";
  $("link-login").hidden = mode === "login" && step !== "code";
  $("link-forgot").hidden = mode === "forgot";
  emailInput.required = step !== "code";
  passwordInput.required = step !== "code" && mode === "login";
  codeInput.required = step === "code";
}
// Where to continue after signing in: the portal, or the SKAGER pairing page.
const returnTo = /[?&]next=(?:\/|%2F)link(?:\/|%2F)(?:&|$)/i.test(String(location.search || "")) ? "/link/" : null;
function say(text, kind = "") { message.textContent = text; message.className = `message ${kind}`.trim(); }
async function post(path, payload) {
  const response = await fetch(path, { method: "POST", credentials: "same-origin", cache: "no-store",
    headers: { "Content-Type": "application/json", "X-CSRF-Token": csrfToken }, body: JSON.stringify(payload) });
  const data = await response.json().catch(() => ({}));
  return { response, data };
}
async function bootstrap() {
  try {
    const response = await fetch("/api/auth/session", { credentials: "same-origin", cache: "no-store" });
    if (!response.ok) throw new Error("Your account is temporarily unavailable. Please try again shortly.");
    const session = await response.json();
    if (session.account) { location.assign(returnTo || "/portal/"); return; }
    if (typeof session.csrfToken !== "string" || !session.csrfToken) throw new Error("Please reload this page.");
    csrfToken = session.csrfToken;
    form.hidden = false; show(); say("");
  } catch (error) { say(error.message, "error"); }
}
form.addEventListener("submit", async event => {
  event.preventDefault();
  submit.disabled = true; say("One moment…");
  try {
    if (step === "code") {
      const { response, data } = await post("/api/auth/verify", { code: codeInput.value.trim() });
      if (!response.ok) throw new Error(data.error || "That code did not work. Request a new one and try again.");
      // A new account still has to choose its password first.
      const next = typeof data.next === "string" && data.next.startsWith("/portal/") ? data.next : "/portal/";
      location.assign(returnTo && next === "/portal/" ? returnTo : next);
      return;
    }
    const payload = mode === "login"
      ? { email: emailInput.value.trim(), password: passwordInput.value }
      : { email: emailInput.value.trim() };
    const { response, data } = await post(`/api/auth/${mode}`, payload);
    if (!response.ok) throw new Error(data.error || "That did not work. Please try again.");
    passwordInput.value = "";
    step = "code"; show();
    say(mode === "login" ? `We sent a code to ${data.email || "your email"}.` : (data.message || "Check your email for a code."), "ok");
    codeInput.focus?.();
  } catch (error) { say(error.message, "error"); }
  finally { submit.disabled = false; }
});
bootstrap();
