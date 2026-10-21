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
  // One step per requirement: the account, then each admin-defined stage (helm PC,
  // GPS, ...) where the customer picks a product or answers "I already have one",
  // then review and pay.
  const form = $('checkout-form');
  if (form) {
    const message = $('checkout-message'), submit = $('checkout-submit');
    const nav = $('step-nav'), holder = $('option-steps'), stepTotal = $('step-total');
    const wanted = new URLSearchParams(location.search).get('add');
    const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/;
    let catalog = { products: [], steps: [], shippingMinor: 0 };
    let access = { name: 'SKAGER Public Beta Access', priceMinor: 2000 };
    let stages = [{ key: 'account', label: 'Account' }, { key: 'review', label: 'Pay' }];
    let current = 0, reached = 0, open = false;
    const el = (tag, props = {}) => Object.assign(document.createElement(tag), props);
    const section = (key) => [...form.querySelectorAll('[data-step]')].find((node) => node.dataset.step === key);
    const picked = (stage) => stage.step ? [...section(stage.key).querySelectorAll('input:checked')].map((input) => input.value) : [];
    const chosen = () => stages.flatMap((stage) => picked(stage).filter((value) => value !== 'own'));
    const productById = (id) => catalog.products.find((product) => product.id === id);

    const totals = () => {
      const picks = chosen().map(productById).filter(Boolean);
      const shipping = picks.some((product) => product.shippable) ? catalog.shippingMinor : 0;
      return { picks, shipping, total: access.priceMinor + picks.reduce((sum, product) => sum + product.priceMinor, 0) + shipping };
    };
    const refresh = () => {
      const { picks, shipping, total } = totals();
      $('order-total').textContent = money(total);
      const stage = stages[current];
      stepTotal.hidden = !stage?.step;
      stepTotal.replaceChildren(el('span', { textContent: 'Your order so far' }), el('b', { textContent: money(total) }));
      const lines = [[access.name, money(access.priceMinor)]];
      stages.filter((s) => s.step).forEach((s) => {
        const values = picked(s);
        values.filter((value) => value !== 'own').map(productById).filter(Boolean).forEach((product) => lines.push([product.name, money(product.priceMinor)]));
        if (values.includes('own')) lines.push([`${s.step.title}: ${s.step.skipLabel}`, '', 'own']);
      });
      if (shipping) lines.push(['Shipping (address on the payment page)', money(shipping)]);
      $('review-lines').replaceChildren(...lines.map(([name, price, kind]) => {
        const li = el('li', kind ? { className: kind } : {});
        li.append(el('span', { textContent: name }), el('span', { textContent: price }));
        return li;
      }));
      $('review-email').textContent = $('checkout-email').value.trim();
      stages.forEach((s) => {
        const next = section(s.key).querySelector('[data-next]');
        if (next && s.step) next.disabled = picked(s).length === 0;
      });
      submit.disabled = !open;
      return picks;
    };
    const renderNav = () => {
      nav.replaceChildren(...stages.map((stage, index) => {
        const li = el('li');
        if (index === current) li.setAttribute('aria-current', 'step');
        else if (index <= reached) li.className = 'is-done';
        const button = el('button', { type: 'button', disabled: index > reached, title: stage.label });
        button.append(el('span', { textContent: stage.label }));
        button.addEventListener('click', () => go(index));
        li.append(button);
        return li;
      }));
    };
    // A step can be left once its requirement is met.
    const ready = (index) => {
      const stage = stages[index];
      if (stage.key === 'account') {
        if (validEmail($('checkout-email').value.trim())) return true;
        say(message, 'Enter the email address for your SKAGER account.', 'error');
        $('checkout-email').focus();
        return false;
      }
      if (stage.step && picked(stage).length === 0) { say(message, 'Choose one of the options to continue.', 'error'); return false; }
      return true;
    };
    function go(index, focus = true) {
      if (index > current) for (let i = current; i < index; i++) if (!ready(i)) { show(i, false); return; }
      show(index, focus);
    }
    function show(index, focus = true) {
      current = Math.max(0, Math.min(index, stages.length - 1));
      reached = Math.max(reached, current);
      stages.forEach((stage, i) => {
        const node = section(stage.key);
        node.hidden = i !== current;
        const count = node.querySelector('.step-count');
        if (count) count.textContent = `Step ${i + 1} of ${stages.length}`;
      });
      if (message.classList.contains('error')) say(message, '');
      renderNav();
      refresh();
      if (focus) {
        section(stages[current].key).querySelector('h2').focus({ preventScroll: true });
        if (form.getBoundingClientRect().top < 0) form.scrollIntoView({ block: 'start', behavior: 'smooth' });
      }
    }

    const optionCard = (type, name, value, thumb, title, detail, price) => {
      const label = el('label', { className: value === 'own' ? 'option option-own' : 'option' });
      const input = el('input', { type, name, value });
      const text = el('span');
      text.append(el('b', { textContent: title }));
      if (detail) text.append(el('small', { textContent: detail }));
      if (price) text.append(el('span', { className: 'option-price', textContent: price }));
      label.append(input, thumb, text);
      return label;
    };
    const thumbFor = (product) => {
      if (product.imageUrl && /^\/api\/media\/products\/[\w./-]+$/.test(product.imageUrl)) {
        return el('img', { className: 'option-thumb', src: product.imageUrl, alt: '', loading: 'lazy' });
      }
      const chip = el('span', { className: 'option-thumb chip', textContent: 'SKAGER' });
      chip.setAttribute('aria-hidden', 'true');
      return chip;
    };
    const buildStep = (step, products) => {
      const key = `opt-${step.id}`;
      const node = el('section', { className: 'step', hidden: true });
      node.dataset.step = key;
      const heading = el('h2', { id: `step-${step.id}-title`, tabIndex: -1, textContent: step.title });
      node.setAttribute('aria-labelledby', heading.id);
      node.append(el('p', { className: 'step-count' }), heading);
      if (step.summary) node.append(el('p', { className: 'step-summary', textContent: step.summary }));
      const set = el('fieldset', { className: 'options' });
      set.append(el('legend', { className: 'visually-hidden', textContent: step.title }));
      const type = step.multiple ? 'checkbox' : 'radio', name = `step-${step.id}`;
      products.forEach((product) => set.append(optionCard(type, name, product.id, thumbFor(product), product.name, product.summary, `+ ${money(product.priceMinor)}`)));
      const own = el('span', { className: 'option-thumb chip', textContent: '✓' });
      own.setAttribute('aria-hidden', 'true');
      set.append(optionCard(type, name, 'own', own, step.skipLabel, '', ''));
      set.addEventListener('change', (event) => {
        // "I already have one" and the products exclude each other.
        if (step.multiple && event.target.checked) {
          set.querySelectorAll('input').forEach((input) => {
            if ((event.target.value === 'own') !== (input.value === 'own')) input.checked = false;
          });
        }
        if (message.classList.contains('error')) say(message, '');
        refresh();
      });
      node.append(set);
      if (step.linkUrl && /^https:\/\//.test(step.linkUrl) && step.linkLabel) {
        node.append(el('a', { className: 'step-link', href: step.linkUrl, target: '_blank', rel: 'noopener noreferrer', textContent: `${step.linkLabel} ↗` }));
      }
      const actions = el('div', { className: 'step-actions' });
      const back = el('button', { className: 'back-button', type: 'button', textContent: '← Back' });
      back.dataset.back = '';
      const next = el('button', { className: 'cta order-cta', type: 'button', disabled: true });
      next.dataset.next = '';
      const arrow = el('span', { textContent: '→' });
      arrow.setAttribute('aria-hidden', 'true');
      next.append('Continue ', arrow);
      actions.append(back, next);
      node.append(actions);
      const preset = products.find((product) => wanted && product.slug === wanted);
      if (preset) [...set.querySelectorAll('input')].find((input) => input.value === preset.id).checked = true;
      return { key, label: step.title, step, node };
    };
    const buildStages = () => {
      const addons = catalog.products.filter((product) => product.kind === 'hardware' && product.checkoutAddon);
      let steps = (catalog.steps || []).filter((step) => step.flow === 'checkout' && UUID.test(step.id));
      // Before stages exist (older catalog), all add-ons share one optional step.
      if (!Array.isArray(catalog.steps)) steps = addons.length ? [{ id: '00000000-0000-4000-8000-000000000000', title: 'Add to your helm', summary: '', skipLabel: 'No thanks', multiple: true }] : [];
      const built = steps.map((step) => buildStep(step, addons.filter((product) => !Array.isArray(catalog.steps) || product.stepId === step.id)))
        .filter((stage) => stage.node.querySelectorAll('input').length > 1);
      holder.replaceChildren(...built.map((stage) => stage.node));
      stages = [stages[0], ...built, stages[stages.length - 1]];
    };

    form.addEventListener('click', (event) => {
      const target = event.target.closest('[data-next],[data-back],[data-goto]');
      if (!target) return;
      if (target.matches('[data-next]')) go(current + 1);
      else if (target.matches('[data-back]')) go(current - 1);
      else go(stages.findIndex((stage) => stage.key === target.dataset.goto));
    });
    $('checkout-email').addEventListener('input', () => { if (message.classList.contains('error')) say(message, ''); });

    (async () => {
      try {
        const [me, response] = await Promise.all([session(), fetch('/api/store/catalog', { cache: 'no-store' })]);
        if (!response.ok) throw new Error('Checkout is temporarily unavailable. Please try again shortly.');
        catalog = await response.json();
        const product = catalog.products.find((item) => item.kind === 'access');
        if (product) { access = product; $('access-price').textContent = money(product.priceMinor).replace('.00', ''); }
        buildStages();
        show(0, false);
        if (!catalog.checkoutOpen) { say(message, 'Checkout is temporarily unavailable. Please try again shortly.', 'error'); return; }
        if (me.account) say(message, 'You are logged in and already own SKAGER.', '', { href: '/portal/', label: 'Go to your account →' });
        open = true;
        refresh();
      } catch (error) { show(0, false); say(message, error.message, 'error'); }
    })();
    form.addEventListener('submit', async (event) => {
      event.preventDefault();
      // Enter in a field moves on until the last step.
      if (stages[current].key !== 'review') { go(current + 1); return; }
      for (let i = 0; i < current; i++) if (!ready(i)) { show(i); return; }
      const email = $('checkout-email').value.trim();
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
