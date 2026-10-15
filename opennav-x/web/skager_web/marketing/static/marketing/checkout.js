/* Checkout (/checkout/) and account setup after payment (/welcome/).
   Prices and totals are decided by the server and Stripe; this page only
   previews them. Server text is always inserted as text, never as HTML. */
(() => {
  const $ = (id) => document.getElementById(id);
  const money = (minor) => `USD ${(minor / 100).toFixed(2)}`;
  let csrf = '';

  const say = (node, text, kind = '', link) => {
    node.textContent = text;
    node.className = `order-message ${kind}`.trim();
    if (link) {
      const a = document.createElement('a');
      a.href = link.href; a.textContent = link.label;
      node.append(' ', a);
    }
  };
  async function session() {
    const response = await fetch('/api/auth/session', { credentials: 'same-origin', cache: 'no-store' });
    if (!response.ok) throw new Error('We could not connect securely. Please reload the page.');
    const data = await response.json();
    if (typeof data.csrfToken !== 'string' || !data.csrfToken) throw new Error('Please reload the page.');
    csrf = data.csrfToken;
    return data;
  }
  async function post(path, payload) {
    const response = await fetch(path, { method: 'POST', credentials: 'same-origin', cache: 'no-store',
      headers: { 'Content-Type': 'application/json', 'X-CSRF-Token': csrf }, body: JSON.stringify(payload) });
    const data = await response.json().catch(() => ({}));
    return { status: response.status, ok: response.ok, data };
  }
  const validEmail = (value) => /^[^\s@]{1,64}@[^\s@]+\.[^\s@]{2,}$/.test(value) && value.length <= 254;

  /* ---------------------------------------------------------------- checkout */
  const form = $('checkout-form');
  if (form) {
    const message = $('checkout-message'), submit = $('checkout-submit'), totalNode = $('order-total');
    const wanted = new URLSearchParams(location.search).get('add');
    let catalog = { products: [], shippingMinor: 0 };
    let access = 2000;
    const chosen = () => [...form.querySelectorAll('input[name="addon"]:checked')].map((input) => input.value);
    const refresh = () => {
      const picks = catalog.products.filter((product) => chosen().includes(product.id));
      const shipping = picks.some((product) => product.shippable) ? catalog.shippingMinor : 0;
      const total = access + picks.reduce((sum, product) => sum + product.priceMinor, 0) + shipping;
      totalNode.textContent = money(total);
      const ship = $('addon-ship');
      ship.hidden = !shipping;
      ship.textContent = shipping ? `Shipping ${money(shipping)}. You add your delivery address on the payment page.` : '';
    };
    const renderAddons = () => {
      const addons = catalog.products.filter((product) => product.checkoutAddon && product.kind === 'hardware');
      const list = $('addon-list');
      list.replaceChildren(...addons.map((product) => {
        const label = document.createElement('label');
        label.className = 'addon';
        const input = Object.assign(document.createElement('input'), { type: 'checkbox', name: 'addon', value: product.id });
        input.checked = wanted === product.slug;
        let thumb;
        if (product.imageUrl && /^\/api\/media\/products\/[\w./-]+$/.test(product.imageUrl)) {
          thumb = Object.assign(document.createElement('img'), { className: 'addon-thumb', src: product.imageUrl, alt: '', loading: 'lazy' });
        } else {
          thumb = Object.assign(document.createElement('span'), { className: 'addon-thumb chip', textContent: 'GPS' });
          thumb.setAttribute('aria-hidden', 'true');
        }
        const text = document.createElement('span');
        const name = document.createElement('b'); name.textContent = product.name;
        const summary = document.createElement('small'); summary.textContent = product.summary || '';
        text.append(name, summary);
        const price = Object.assign(document.createElement('span'), { className: 'addon-price', textContent: `+ ${money(product.priceMinor)}` });
        label.append(input, thumb, text, price);
        input.addEventListener('change', refresh);
        return label;
      }));
      $('addons').hidden = addons.length === 0;
      refresh();
    };
    (async () => {
      try {
        const [me, response] = await Promise.all([session(), fetch('/api/store/catalog', { cache: 'no-store' })]);
        if (!response.ok) throw new Error('Checkout is temporarily unavailable. Please try again shortly.');
        catalog = await response.json();
        const product = catalog.products.find((item) => item.kind === 'access');
        if (product) access = product.priceMinor;
        renderAddons();
        if (!catalog.checkoutOpen) { say(message, 'Checkout is temporarily unavailable. Please try again shortly.', 'error'); return; }
        if (me.account) say(message, 'You are logged in and already own SKAGER.', '', { href: '/portal/', label: 'Go to your account →' });
        submit.disabled = false;
      } catch (error) { say(message, error.message, 'error'); }
    })();
    form.addEventListener('submit', async (event) => {
      event.preventDefault();
      const email = $('checkout-email').value.trim();
      if (!validEmail(email)) { say(message, 'Enter the email address for your SKAGER account.', 'error'); $('checkout-email').focus(); return; }
      const discountCode = $('discount-code').value.trim();
      submit.disabled = true;
      say(message, 'Opening secure payment…');
      try {
        const { ok, data } = await post('/api/store/checkout', { email, addons: chosen(), ...(discountCode ? { discountCode } : {}) });
        if (!ok) {
          const link = data.state === 'exists' ? { href: '/auth/login/', label: 'Log in →' }
            : data.state === 'setup' ? { href: '/welcome/', label: 'Finish setup →' } : undefined;
          say(message, data.error || 'Checkout could not be opened. Please try again.', 'error', link);
          submit.disabled = false;
          return;
        }
        const target = new URL(data.url);
        if (target.protocol !== 'https:' || target.hostname !== 'checkout.stripe.com') throw new Error('Checkout could not be opened. Please try again.');
        location.assign(target.href);
      } catch (error) { say(message, error.message || 'Checkout could not be opened. Please try again.', 'error'); submit.disabled = false; }
    });
  }

  /* ---------------------------------------------------------------- welcome */
  const welcome = $('welcome');
  if (welcome) {
    const text = $('welcome-text'), message = $('welcome-message');
    const emailForm = $('welcome-email'), codeForm = $('welcome-code');
    const step = (name) => {
      let done = true;
      welcome.querySelectorAll('[data-step]').forEach((node) => {
        const current = node.dataset.step === name;
        if (current) done = false;
        node.classList.toggle('is-current', current);
        node.classList.toggle('is-done', done && !current);
      });
    };
    const askCode = (masked) => {
      step('code');
      emailForm.hidden = true; codeForm.hidden = false;
      text.textContent = masked ? `We sent a code to ${masked}. Enter it below to verify your email.` : 'If that email belongs to a SKAGER purchase, a code is on its way. Enter it below.';
      say(message, '');
      $('welcome-input').focus();
    };
    const askEmail = (why) => {
      step('code');
      codeForm.hidden = true; emailForm.hidden = false;
      text.textContent = why;
    };
    const params = new URLSearchParams(location.search);
    const sessionId = params.get('session_id') || '';
    // Keep the Checkout reference out of history once it has been read.
    if (sessionId) history.replaceState(null, '', '/welcome/');
    let attempts = 0;
    const claim = async () => {
      const { status, data } = await post('/api/auth/claim', { sessionId });
      if (status === 202 && attempts++ < 30) { text.textContent = 'Confirming your payment with Stripe…'; setTimeout(() => claim().catch(fail), 2000); return; }
      if (data.state === 'code_sent') { askCode(data.email); return; }
      if (data.state === 'done') { step('sail'); text.textContent = 'Your account is ready.'; say(message, '', '', { href: '/auth/login/', label: 'Log in to SKAGER →' }); return; }
      if (data.state === 'revoked') { text.textContent = 'This purchase was refunded and its account is closed.'; return; }
      if (status === 429) { askEmail('We have sent several codes already. Wait a few minutes, then send a new one.'); return; }
      if (status === 202) { askEmail('Your payment is still being confirmed. Enter your email to get your code as soon as it is done.'); return; }
      askEmail('Enter the email address you used at checkout and we will send your code.');
    };
    const fail = (error) => say(message, error.message || 'Something went wrong. Please try again.', 'error');
    (async () => {
      try {
        const me = await session();
        if (me.account) { location.assign(me.account.passwordSet === false ? '/portal/account/?setup=1' : '/portal/'); return; }
        if (/^cs_test_[A-Za-z0-9]{8,200}$/.test(sessionId)) await claim();
        else askEmail('Enter the email address you used at checkout and we will send your code.');
      } catch (error) { fail(error); }
    })();
    emailForm.addEventListener('submit', async (event) => {
      event.preventDefault();
      const email = $('welcome-address').value.trim();
      if (!validEmail(email)) { say(message, 'Enter a valid email address.', 'error'); return; }
      try {
        const { ok, data } = await post('/api/auth/claim', { email });
        if (!ok) throw new Error(data.error || 'We could not send a code. Please try again.');
        askCode(null);
      } catch (error) { fail(error); }
    });
    codeForm.addEventListener('submit', async (event) => {
      event.preventDefault();
      const button = codeForm.querySelector('button');
      button.disabled = true;
      try {
        const { ok, data } = await post('/api/auth/verify', { code: $('welcome-input').value.trim() });
        if (!ok) throw new Error(data.error || 'That code did not work. Request a new one and try again.');
        step('password');
        location.assign(typeof data.next === 'string' && data.next.startsWith('/portal/') ? data.next : '/portal/account/?setup=1');
      } catch (error) { fail(error); button.disabled = false; }
    });
  }
})();
