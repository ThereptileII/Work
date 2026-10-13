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
        status.hidden = false;
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

  const scene = document.querySelector('[data-marine-scene]');
  if (scene) {
    const controls = scene.querySelector('[data-scene-controls]');
    const motionButton = scene.querySelector('[data-motion-toggle]');
    const status = scene.querySelector('[data-scene-status]');
    const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)');
    const smallScreen = window.matchMedia('(max-width: 800px)');
    const connection = navigator.connection;
    let requestedMotion = true;
    let visible = true;
    const needsStatic = () => reducedMotion.matches || smallScreen.matches ||
      Boolean(connection && (connection.saveData || ['slow-2g', '2g'].includes(connection.effectiveType))) ||
      (navigator.hardwareConcurrency && navigator.hardwareConcurrency <= 2);
    const updateMotion = () => {
      const fallback = needsStatic();
      const playing = !fallback && requestedMotion && visible && !document.hidden;
      scene.dataset.motion = fallback ? 'static' : playing ? 'playing' : 'paused';
      motionButton.disabled = fallback;
      motionButton.setAttribute('aria-pressed', String(!fallback && requestedMotion));
      motionButton.textContent = fallback ? 'Static scene' : requestedMotion ? 'Pause scene' : 'Play scene';
      status.hidden = !fallback;
      status.textContent = fallback ? 'Static illustration for reduced motion, mobile, data-saving or lower-performance devices. Day, dusk and night views remain available.' : '';
    };
    if (controls && motionButton && status) {
      controls.hidden = false;
      motionButton.addEventListener('click', () => {
        requestedMotion = !requestedMotion;
        updateMotion();
      });
      scene.querySelectorAll('[data-scene-mode]').forEach((button) => {
        button.addEventListener('click', () => {
          scene.dataset.mode = button.dataset.sceneMode;
          scene.querySelectorAll('[data-scene-mode]').forEach((mode) => {
            mode.setAttribute('aria-pressed', String(mode === button));
          });
        });
      });
      reducedMotion.addEventListener('change', updateMotion);
      smallScreen.addEventListener('change', updateMotion);
      if (connection && connection.addEventListener) connection.addEventListener('change', updateMotion);
      document.addEventListener('visibilitychange', updateMotion);
      if ('IntersectionObserver' in window) {
        const observer = new IntersectionObserver((entries) => {
          visible = entries[0].isIntersecting;
          updateMotion();
        }, { threshold: 0 });
        observer.observe(scene);
      }
      updateMotion();
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
