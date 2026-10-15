(() => {
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
