const mode = ["login", "signup", "recover", "verify"].find(value => location.pathname.replace(/\/$/, "") === `/auth/${value}`) || "login";
const copy = {
  login: ["Welcome back.", "Sign in securely with a code sent to your email.", "Send sign-in code ↗"],
  signup: ["Come aboard.", "Create your private test account using a code sent to your email.", "Send account code ↗"],
  recover: ["Find your way back.", "Request a code to recover access to your account.", "Send recovery code ↗"],
  verify: ["One more step.", "Enter the code from your email. Keep this browser open while verifying.", "Verify code ↗"],
};
const form = document.getElementById("auth-form");
const input = document.getElementById("auth-input");
const submit = document.getElementById("auth-submit");
const message = document.getElementById("auth-message");
document.getElementById("auth-title").textContent = copy[mode][0];
document.getElementById("auth-intro").textContent = copy[mode][1];
submit.textContent = copy[mode][2];
document.title = `${copy[mode][0]} — SKAGER`;
if (mode === "verify") {
  document.getElementById("auth-label").textContent = "Email verification code";
  input.name = "code"; input.type = "text"; input.autocomplete = "one-time-code";
  input.inputMode = "numeric"; input.maxLength = 10; input.pattern = "[0-9]{6,10}";
}
let csrfToken;
async function bootstrap() {
  try {
    const response = await fetch("/api/auth/session", { credentials: "same-origin", cache: "no-store" });
    if (!response.ok) throw new Error("Account access is currently unavailable. Please try again later.");
    const session = await response.json();
    if (session.account) { location.assign("/portal/"); return; }
    if (typeof session.csrfToken !== "string" || !session.csrfToken) throw new Error("A secure session could not be started. Please reload this page.");
    csrfToken = session.csrfToken; form.hidden = false; message.textContent = "";
  } catch (error) { message.textContent = error.message; }
}
form.addEventListener("submit", async event => {
  event.preventDefault(); submit.disabled = true; message.textContent = "Connecting securely…";
  try {
    const response = await fetch(`/api/auth/${mode}`, {
      method: "POST", credentials: "same-origin", cache: "no-store",
      headers: { "Content-Type": "application/json", "X-CSRF-Token": csrfToken },
      body: JSON.stringify(mode === "verify" ? { code: input.value.trim() } : { email: input.value.trim() }),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(data.error || "This request could not be completed. Please try again.");
    if (mode === "verify") { location.assign("/portal/"); return; }
    message.textContent = "If this request is eligible, a code has been sent. Check your email, then enter the code in this browser.";
    form.hidden = true;
    const link = document.createElement("a"); link.href = "/auth/verify/";
    link.className = "button"; link.textContent = "Enter email code ↗"; form.after(link);
  } catch (error) { message.textContent = error.message; submit.disabled = false; }
});
bootstrap();
