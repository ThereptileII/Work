(() => {
  const search = document.querySelector('form[data-help-search]');
  if (search) {
    const query = search.querySelector('input[name="q"]');
    const items = [...document.querySelectorAll('.help-item[data-search]')];
    const status = document.querySelector('[data-search-status]');
    const empty = document.querySelector('[data-search-empty]');

    if (query && items.length && status && empty) {
      query.maxLength = 120;
      const normalize = (text) => text.toLocaleLowerCase('en').replace(/\s+/g, ' ').trim();
      const filter = () => {
        const terms = normalize(query.value.slice(0, 120)).split(' ').filter(Boolean);
        let shown = 0;
        items.forEach((item) => {
          const text = normalize(item.dataset.search);
          const matches = terms.every((term) => text.includes(term));
          item.hidden = !matches;
          if (matches) shown += 1;
        });
        empty.hidden = shown !== 0;
        status.textContent = `${shown} guide${shown === 1 ? '' : 's'} ${terms.length ? 'found' : 'available'}.`;
      };
      search.addEventListener('submit', (event) => {
        event.preventDefault();
        filter();
      });
      query.addEventListener('input', filter);
      if (query.value) filter();
    }
  }

  const toggle = document.querySelector('.menu-toggle');
  const navigation = document.querySelector('#site-navigation');
  if (!toggle || !navigation) return;

  document.documentElement.classList.add('js');
  toggle.hidden = false;

  const setOpen = (open) => {
    toggle.setAttribute('aria-expanded', String(open));
    navigation.classList.toggle('is-open', open);
    toggle.querySelector('span').textContent = open ? 'Close' : 'Menu';
  };

  toggle.addEventListener('click', () => {
    setOpen(toggle.getAttribute('aria-expanded') !== 'true');
  });

  navigation.addEventListener('click', (event) => {
    if (event.target.closest('a')) setOpen(false);
  });

  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape' && toggle.getAttribute('aria-expanded') === 'true') {
      setOpen(false);
      toggle.focus();
    }
  });

  window.matchMedia('(min-width: 801px)').addEventListener('change', () => setOpen(false));
})();
