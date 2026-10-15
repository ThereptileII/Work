/* SKAGER homepage: a living chart scene, the scroll tour and the feature visuals.
 * One invented archipelago is shared by the hero and the tour. Everything runs on
 * requestAnimationFrame, pauses off-screen, and renders a still frame for
 * prefers-reduced-motion. No dependencies (the CSP is script-src 'self'). */
(() => {
  'use strict';

  const root = document.documentElement;
  const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)');
  const smallScreen = window.matchMedia('(max-width: 800px)');
  const still = () => reducedMotion.matches;
  root.classList.add('home-js');

  /* ---------- colour ---------- */
  const hex = (value) => [1, 3, 5].map((i) => parseInt(value.slice(i, i + 2), 16));
  const palette = (spec) => Object.fromEntries(Object.entries(spec).map(([k, v]) => [k, hex(v)]));
  const PALETTES = {
    hero: palette({ water: '#06171c', shallow: '#0d2d35', land: '#14352f', shore: '#3f7d74', contour: '#1d5059',
      route: '#6fe0ea', chartText: '#3c6d6e', ais: '#f0a6c8', own: '#ffffff', wind: '#8fdde8' }),
  };
  const mixRGB = (a, b, t) => [0, 1, 2].map((i) => a[i] + (b[i] - a[i]) * t);
  const rgba = (c, a = 1) => `rgba(${c[0] | 0},${c[1] | 0},${c[2] | 0},${a})`;

  /* ---------- maths ---------- */
  const clamp = (v, lo = 0, hi = 1) => Math.min(hi, Math.max(lo, v));
  const smooth = (e0, e1, x) => { const t = clamp((x - e0) / (e1 - e0)); return t * t * (3 - 2 * t); };
  const approach = (current, target, dt, rate = 5) => current + (target - current) * (1 - Math.exp(-dt * rate));
  const noise = (t, seed) => Math.sin(t * 0.71 + seed) * 0.5 + Math.sin(t * 1.37 + seed * 2.1) * 0.3 + Math.sin(t * 2.9 + seed * 3.7) * 0.2;
  let seedState = 20261015;
  const random = () => { seedState = (seedState * 1664525 + 1013904223) >>> 0; return seedState / 4294967296; };

  /* ---------- the world: invented islands, one looping passage ---------- */
  const WORLD = (() => {
    const ISLANDS = [[2200, 1420, 250], [1880, 1760, 120], [2680, 1480, 150], [2330, 1020, 95], [680, 1120, 320],
      [1120, 560, 210], [2000, 170, 250], [3000, 280, 230], [3720, 760, 260], [3920, 1720, 270], [3300, 2480, 220],
      [2000, 2800, 250], [970, 2650, 230], [300, 1950, 250], [1540, 1600, 70], [2900, 1100, 60], [1980, 2000, 55],
      [-260, 700, 330], [4400, 1200, 300], [600, 300, 200], [3500, 3000, 260], [1400, 3150, 240], [4300, 2500, 230]];
    const WAYPOINTS = [[800, 1950], [1250, 1520], [1520, 1020], [2080, 700], [2820, 760], [3330, 1230], [3200, 1880],
      [2450, 2160], [1600, 2250]];
    const BANDS = [170, 95, 40];

    const outline = (x, y, r, grow) => {
      const harmonics = [2, 3, 4, 5, 7].map((k) => [k, (random() * 0.22) / Math.sqrt(k - 1), random() * Math.PI * 2]);
      return (extra) => {
        const path = new Path2D();
        const steps = 72;
        const points = [];
        for (let i = 0; i < steps; i += 1) {
          const a = (i / steps) * Math.PI * 2;
          let k = 1;
          harmonics.forEach(([n, amp, phase]) => { k += amp * Math.sin(n * a + phase); });
          const rr = r * k * grow + extra;
          points.push([x + Math.cos(a) * rr, y + Math.sin(a) * rr]);
        }
        points.forEach((p, i) => {
          const next = points[(i + 1) % steps];
          const mid = [(p[0] + next[0]) / 2, (p[1] + next[1]) / 2];
          if (i === 0) path.moveTo(mid[0], mid[1]); else path.quadraticCurveTo(p[0], p[1], mid[0], mid[1]);
        });
        const first = points[0];
        const second = points[1];
        path.quadraticCurveTo(first[0], first[1], (first[0] + second[0]) / 2, (first[1] + second[1]) / 2);
        path.closePath();
        return path;
      };
    };

    const islands = ISLANDS.map(([x, y, r]) => {
      const shape = outline(x, y, r, 1);
      return { x, y, r, reach: r * 1.3 + BANDS[0], land: shape(0), bands: BANDS.map((d) => shape(d)) };
    });
    // A few rocks and islets for texture.
    for (let i = 0; i < 26; i += 1) {
      const x = random() * 4400 - 200;
      const y = random() * 3200 - 200;
      const r = 14 + random() * 26;
      const free = islands.every((is) => Math.hypot(is.x - x, is.y - y) > is.r * 1.4 + r + 60);
      if (!free) continue;
      const shape = outline(x, y, r, 1);
      islands.push({ x, y, r, reach: r * 1.3 + 60, land: shape(0), bands: [shape(55), shape(30), shape(12)] });
    }

    // Closed Catmull-Rom spline through the waypoints, sampled by arc length.
    const n = WAYPOINTS.length;
    const spline = [];
    const catmull = (p0, p1, p2, p3, t) => [0, 1].map((i) => 0.5 * ((2 * p1[i]) + (-p0[i] + p2[i]) * t +
      (2 * p0[i] - 5 * p1[i] + 4 * p2[i] - p3[i]) * t * t + (-p0[i] + 3 * p1[i] - 3 * p2[i] + p3[i]) * t * t * t));
    const waypointArc = [];
    let length = 0;
    for (let i = 0; i < n; i += 1) {
      for (let k = 0; k < 48; k += 1) {
        const p = catmull(WAYPOINTS[(i - 1 + n) % n], WAYPOINTS[i], WAYPOINTS[(i + 1) % n], WAYPOINTS[(i + 2) % n], k / 48);
        if (spline.length) { const q = spline[spline.length - 1]; length += Math.hypot(p[0] - q[0], p[1] - q[1]); }
        if (k === 0) waypointArc.push(length);
        spline.push([p[0], p[1], length]);
      }
    }
    length += Math.hypot(spline[0][0] - spline[spline.length - 1][0], spline[0][1] - spline[spline.length - 1][1]);

    const wrap = (s) => ((s % length) + length) % length;
    const at = (s) => {
      s = wrap(s);
      let lo = 0;
      let hi = spline.length - 1;
      while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (spline[mid][2] <= s) lo = mid; else hi = mid - 1; }
      const a = spline[lo];
      const b = spline[(lo + 1) % spline.length];
      const segment = (lo + 1 === spline.length ? length : b[2]) - a[2];
      const t = segment > 0 ? (s - a[2]) / segment : 0;
      const dx = b[0] - a[0];
      const dy = b[1] - a[1];
      const d = Math.hypot(dx, dy) || 1;
      return { x: a[0] + dx * t, y: a[1] + dy * t, nx: dx / d, ny: dy / d };
    };
    const trace = (ctx, from, to, step = 18) => {
      let p = at(from);
      ctx.moveTo(p.x, p.y);
      for (let s = from + step; s < to; s += step) { p = at(s); ctx.lineTo(p.x, p.y); }
      p = at(to);
      ctx.lineTo(p.x, p.y);
    };

    // Depth soundings in open water.
    const soundings = [];
    for (let i = 0; soundings.length < 260 && i < 4000; i += 1) {
      const x = random() * 4800 - 400;
      const y = random() * 3600 - 400;
      const nearest = Math.min(...islands.map((is) => Math.hypot(is.x - x, is.y - y) - is.r * 1.3));
      if (nearest < 30) continue;
      soundings.push({ x, y, label: (2 + Math.min(nearest, 900) / 28 + random() * 3).toFixed(1) });
    }
    const depthAt = (x, y) => {
      const nearest = Math.min(...islands.map((is) => Math.hypot(is.x - x, is.y - y) - is.r * 1.3));
      return clamp(2.4 + nearest / 34, 2.4, 38);
    };

    // AIS traffic shares the fairway: lane offset, direction and speed vary.
    const TRAFFIC = [
      { name: 'BALTIC TERN', type: 'Cargo', offset: 0.18, lane: 120, speed: -58 },
      { name: 'ÅLAND STAR', type: 'Passenger', offset: 0.52, lane: 130, speed: -74 },
      { name: 'SVALAN', type: 'Sailing', offset: 0.07, lane: -60, speed: 22 },
      { name: 'NORDVIK', type: 'Tug', offset: 0.72, lane: -110, speed: 30 },
      { name: 'MARE LIBERA', type: 'Sailing', offset: 0.36, lane: 70, speed: 18 },
    ];

    return { islands, soundings, waypointArc, waypoints: WAYPOINTS, length, at, wrap, trace, depthAt, TRAFFIC };
  })();

  /* ---------- the scene renderer ---------- */
  const BOAT_SPEED = 32; // world units per second

  class Scene {
    constructor(canvas, options) {
      this.canvas = canvas;
      this.ctx = canvas.getContext('2d', { alpha: false });
      this.options = options;
      this.time = options.startTime || 0;
      this.params = { contour: 1, route: 1, ais: 1, wind: 1, scale: options.scale || 1 };
      this.palette = options.palette;
      this.wake = [];
      this.particles = [];
      this.camera = null;
      this.width = 0;
      this.height = 0;
      this.dpr = 1;
      this.running = false;
      this.visible = true;
      this.last = 0;
      this.onFrame = options.onFrame || null;
      this.resize();
      this.prime();
    }

    resize(width, height) {
      const rect = this.canvas.getBoundingClientRect();
      this.width = Math.max(1, Math.round(width || rect.width));
      this.height = Math.max(1, Math.round(height || rect.height));
      this.dpr = Math.min(window.devicePixelRatio || 1, this.options.maxDpr || 1.5) * (this.options.pixelScale || 1);
      this.canvas.width = Math.round(this.width * this.dpr);
      this.canvas.height = Math.round(this.height * this.dpr);
      const density = smallScreen.matches ? 20000 : 10500;
      const count = Math.min(this.options.maxParticles || 280, Math.round((this.width * this.height) / density));
      while (this.particles.length < count) this.particles.push(this.spawn(true));
      this.particles.length = count;
    }

    spawn(anywhere) {
      return { x: random() * this.width, y: random() * this.height, trail: [], clock: 0,
        age: anywhere ? random() * 3 : 0, life: 4 + random() * 4, seed: random() * 100 };
    }

    boat() {
      const s = this.time * BOAT_SPEED + (this.options.arcOffset || 0);
      const p = WORLD.at(s);
      return { s, x: p.x, y: p.y, heading: Math.atan2(p.ny, p.nx) };
    }

    state() {
      const boat = this.boat();
      const t = this.time;
      return {
        sog: 6.3 + noise(t * 0.4, 1) * 0.35,
        cog: ((boat.heading * 180) / Math.PI + 90 + 360) % 360,
        depth: WORLD.depthAt(boat.x, boat.y) + noise(t, 4) * 0.15,
        wind: 14.2 + noise(t * 0.5, 7) * 1.6,
      };
    }

    start() {
      if (this.running) return;
      this.running = true;
      this.last = performance.now();
      const tick = (now) => {
        if (!this.running) return;
        const dt = Math.min(0.05, (now - this.last) / 1000);
        this.last = now;
        this.time += dt;
        if (this.options.update) this.options.update(this, dt);
        this.render(dt);
        if (this.onFrame) this.onFrame(this, dt);
        this.raf = requestAnimationFrame(tick);
      };
      this.raf = requestAnimationFrame(tick);
    }

    stop() {
      this.running = false;
      cancelAnimationFrame(this.raf);
    }

    render(dt = 0) {
      const { ctx, width: w, height: h, dpr, params } = this;
      const pal = this.palette;
      const boat = this.boat();
      const scale = params.scale;
      const anchor = this.options.anchor ? this.options.anchor(this) : [0.5, 0.5];
      const target = { x: boat.x - (anchor[0] - 0.5) * w / scale, y: boat.y - (anchor[1] - 0.5) * h / scale };
      const previous = this.camera || target;
      this.camera = target;
      const camDx = (target.x - previous.x) * scale;
      const camDy = (target.y - previous.y) * scale;

      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = rgba(pal.water);
      ctx.fillRect(0, 0, w, h);

      const left = target.x - w / 2 / scale;
      const top = target.y - h / 2 / scale;
      const right = target.x + w / 2 / scale;
      const bottom = target.y + h / 2 / scale;
      ctx.setTransform(dpr * scale, 0, 0, dpr * scale, dpr * (w / 2 - target.x * scale), dpr * (h / 2 - target.y * scale));
      const visible = WORLD.islands.filter((is) => is.x + is.reach > left && is.x - is.reach < right &&
        is.y + is.reach > top && is.y - is.reach < bottom);

      // Shallow-water bands, deepest first.
      [0.34, 0.62, 1].forEach((k, band) => {
        ctx.fillStyle = rgba(mixRGB(pal.water, pal.shallow, k), params.contour);
        visible.forEach((is) => ctx.fill(is.bands[band]));
      });
      ctx.lineWidth = 1.1 / scale;
      ctx.strokeStyle = rgba(pal.contour, 0.9 * params.contour);
      visible.forEach((is) => is.bands.forEach((band) => ctx.stroke(band)));
      ctx.fillStyle = rgba(pal.land);
      ctx.strokeStyle = rgba(pal.shore);
      ctx.lineWidth = 1.4 / scale;
      visible.forEach((is) => { ctx.fill(is.land); ctx.stroke(is.land); });

      // Soundings.
      if (scale > 0.42) {
        ctx.fillStyle = rgba(pal.chartText, 0.55 * params.contour);
        ctx.font = `${10.5 / scale}px Inter, "Segoe UI", Arial, sans-serif`;
        ctx.textAlign = 'center';
        WORLD.soundings.forEach((p) => {
          if (p.x > left && p.x < right && p.y > top && p.y < bottom) ctx.fillText(p.label, p.x, p.y);
        });
      }

      // Route: travelled, ahead (drawn progressively), waypoints.
      const ahead = 2600 * params.route;
      if (params.route > 0.01) {
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.beginPath();
        WORLD.trace(ctx, boat.s - 1400, boat.s);
        ctx.strokeStyle = rgba(pal.route, 0.28 * params.route);
        ctx.lineWidth = 3 / scale;
        ctx.stroke();
        ctx.beginPath();
        WORLD.trace(ctx, boat.s, boat.s + ahead);
        ctx.strokeStyle = rgba(pal.route, 0.16);
        ctx.lineWidth = 12 / scale;
        ctx.stroke();
        ctx.strokeStyle = rgba(pal.route, 0.95);
        ctx.lineWidth = 2.6 / scale;
        ctx.setLineDash([16 / scale, 10 / scale]);
        ctx.lineDashOffset = -this.time * 22 / scale;
        ctx.stroke();
        ctx.setLineDash([]);
        const s = WORLD.wrap(boat.s);
        WORLD.waypointArc.forEach((arc, i) => {
          let d = arc - s;
          if (d < 0) d += WORLD.length;
          if (d > ahead) return;
          const [x, y] = WORLD.waypoints[i];
          ctx.beginPath();
          ctx.arc(x, y, 7 / scale, 0, Math.PI * 2);
          ctx.fillStyle = rgba(pal.water);
          ctx.fill();
          ctx.lineWidth = 2.4 / scale;
          ctx.strokeStyle = rgba(pal.route);
          ctx.stroke();
        });
      }

      // AIS traffic.
      if (params.ais > 0.01) {
        WORLD.TRAFFIC.forEach((ship) => {
          const s = ship.offset * WORLD.length + this.time * ship.speed;
          const p = WORLD.at(s);
          const x = p.x - p.ny * ship.lane;
          const y = p.y + p.nx * ship.lane;
          if (x < left - 100 || x > right + 100 || y < top - 100 || y > bottom + 100) return;
          const dir = ship.speed < 0 ? -1 : 1;
          const heading = Math.atan2(p.ny * dir, p.nx * dir);
          ctx.save();
          ctx.translate(x, y);
          ctx.rotate(heading);
          ctx.scale(1 / scale, 1 / scale);
          ctx.globalAlpha = params.ais;
          ctx.strokeStyle = rgba(pal.ais, 0.7);
          ctx.lineWidth = 1.4;
          ctx.setLineDash([4, 5]);
          ctx.beginPath();
          ctx.moveTo(12, 0);
          ctx.lineTo(12 + Math.abs(ship.speed) * 1.3, 0);
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.beginPath();
          ctx.moveTo(13, 0);
          ctx.lineTo(-9, -7.5);
          ctx.lineTo(-9, 7.5);
          ctx.closePath();
          ctx.fillStyle = rgba(pal.ais, 0.22);
          ctx.fill();
          ctx.lineWidth = 1.8;
          ctx.strokeStyle = rgba(pal.ais);
          ctx.stroke();
          ctx.restore();
          if (scale > 0.5 && this.options.aisLabels) {
            ctx.save();
            ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            const sx = (x - left) * scale;
            const sy = (y - top) * scale;
            ctx.globalAlpha = params.ais * 0.8;
            ctx.fillStyle = rgba(pal.ais);
            ctx.font = '600 9.5px Inter, "Segoe UI", Arial, sans-serif';
            ctx.textAlign = 'left';
            ctx.fillText(ship.name, sx + 14, sy - 12);
            ctx.restore();
          }
        });
      }

      // Own vessel: wake, heading line, pulse, hull.
      this.wake.push([boat.x, boat.y]);
      if (this.wake.length > 90) this.wake.shift();
      ctx.lineCap = 'round';
      for (let i = 1; i < this.wake.length; i += 1) {
        ctx.beginPath();
        ctx.moveTo(this.wake[i - 1][0], this.wake[i - 1][1]);
        ctx.lineTo(this.wake[i][0], this.wake[i][1]);
        ctx.strokeStyle = rgba(pal.own, (i / this.wake.length) * 0.35);
        ctx.lineWidth = (1 + (i / this.wake.length) * 4) / scale;
        ctx.stroke();
      }
      ctx.save();
      ctx.translate(boat.x, boat.y);
      ctx.scale(1 / scale, 1 / scale);
      const pulse = (this.time % 3.2) / 3.2;
      ctx.beginPath();
      ctx.arc(0, 0, 14 + pulse * 46, 0, Math.PI * 2);
      ctx.strokeStyle = rgba(pal.route, (1 - pulse) * 0.55);
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.rotate(boat.heading);
      ctx.beginPath();
      ctx.moveTo(18, 0);
      ctx.lineTo(150, 0);
      ctx.strokeStyle = rgba(pal.own, 0.55);
      ctx.lineWidth = 1.3;
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(19, 0);
      ctx.lineTo(-12, -10);
      ctx.lineTo(-6, 0);
      ctx.lineTo(-12, 10);
      ctx.closePath();
      ctx.fillStyle = rgba(pal.own);
      ctx.fill();
      ctx.restore();

      // Wind streamlines in screen space, anchored to the moving chart.
      this.advect(dt, camDx, camDy);
      if (params.wind > 0.01) {
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.lineWidth = 1.25;
        this.particles.forEach((p) => {
          const n = p.trail.length;
          if (n < 6) return;
          const fade = Math.min(1, p.age / 0.8, (p.life - p.age) / 0.8) * params.wind;
          // Three passes so the tail tapers away behind the head.
          const third = Math.max(2, Math.floor(n / 6) * 2);
          [[0, third, 0.12], [third, third * 2, 0.3], [third * 2, n, 0.6]].forEach(([from, to, alpha]) => {
            if (to - from < 2) return;
            ctx.strokeStyle = rgba(pal.wind, alpha * fade);
            ctx.beginPath();
            ctx.moveTo(p.trail[from], p.trail[from + 1]);
            for (let k = from + 2; k < to; k += 2) ctx.lineTo(p.trail[k], p.trail[k + 1]);
            if (to === n) ctx.lineTo(p.x, p.y); else ctx.lineTo(p.trail[to], p.trail[to + 1]);
            ctx.stroke();
          });
        });
      }
    }

    advect(dt, camDx = 0, camDy = 0) {
      const { width: w, height: h } = this;
      const base = ((225 + 180 - 90) * Math.PI) / 180; // blowing from the south-west
      const t = this.time;
      this.particles.forEach((p, i) => {
        const angle = base + Math.sin(p.x * 0.0058 + t * 0.12) * 0.55 + Math.cos(p.y * 0.0067 - t * 0.09) * 0.45;
        const speed = 52 + Math.sin(p.seed + t * 0.5) * 14;
        p.x += Math.cos(angle) * speed * dt - camDx;
        p.y += Math.sin(angle) * speed * dt - camDy;
        for (let k = 0; k < p.trail.length; k += 2) { p.trail[k] -= camDx; p.trail[k + 1] -= camDy; }
        p.age += dt;
        p.clock += dt;
        if (p.clock > 0.05) {
          p.clock = 0;
          p.trail.push(p.x, p.y);
          if (p.trail.length > 56) p.trail.splice(0, 2);
        }
        if (p.age > p.life || p.x < -60 || p.x > w + 60 || p.y < -60 || p.y > h + 60) this.particles[i] = this.spawn(false);
      });
    }

    prime(seconds = 2.4) {
      // Give every streamline its tail before the first visible frame.
      for (let s = 0; s < seconds; s += 0.045) { this.time += 0.045; this.advect(0.045); }
      this.time -= seconds;
    }

  }

  /* ---------- visibility helper ---------- */
  const whenVisible = (element, callback, margin = '120px') => {
    if (!('IntersectionObserver' in window)) { callback(true); return; }
    new IntersectionObserver((entries) => entries.forEach((e) => callback(e.isIntersecting)),
      { rootMargin: margin }).observe(element);
  };
  const bindRunState = (scene, element) => {
    const update = () => {
      if (!still() && scene.visible && !document.hidden) scene.start(); else scene.stop();
      if (still()) { scene.render(0); if (scene.onFrame) scene.onFrame(scene, 1); }
    };
    whenVisible(element, (visible) => { scene.visible = visible; update(); });
    document.addEventListener('visibilitychange', update);
    reducedMotion.addEventListener('change', update);
    update();
  };
  const text = (node, value) => { if (node && node.textContent !== value) node.textContent = value; };

  /* ---------- header ---------- */
  const header = document.querySelector('.site-header');
  if (header) {
    const onScroll = () => header.classList.toggle('is-scrolled', window.scrollY > 24);
    window.addEventListener('scroll', onScroll, { passive: true });
    onScroll();
  }

  /* ---------- hero ---------- */
  const heroCanvas = document.querySelector('[data-hero-canvas]');
  if (heroCanvas) {
    const hud = Object.fromEntries([...document.querySelectorAll('[data-hero-value]')].map((n) => [n.dataset.heroValue, n]));
    let intro = still() ? 1 : 0;
    let hudClock = 0;
    const hero = new Scene(heroCanvas, {
      palette: PALETTES.hero,
      startTime: 38,
      maxDpr: 1.5,
      maxParticles: 320,
      aisLabels: true,
      scale: smallScreen.matches ? 0.62 : 0.86,
      anchor: () => (smallScreen.matches ? [0.62, 0.78] : [0.66, 0.56]),
      update(scene, dt) {
        intro = Math.min(1, intro + dt / 3.2);
        scene.params.contour = smooth(0, 0.45, intro);
        scene.params.route = smooth(0.25, 0.85, intro);
        scene.params.ais = smooth(0.55, 1, intro);
        scene.params.wind = smooth(0.4, 1, intro);
        scene.params.scale = (smallScreen.matches ? 0.62 : 0.86) * (1 + 0.04 * Math.sin(scene.time / 14));
      },
      onFrame(scene, dt) {
        hudClock += dt;
        if (hudClock < 0.25) return;
        hudClock = 0;
        const s = scene.state();
        text(hud.sog, s.sog.toFixed(1));
        text(hud.depth, s.depth.toFixed(1));
        text(hud.wind, s.wind.toFixed(0));
        text(hud.cog, `${Math.round(s.cog).toString().padStart(3, '0')}°`);
      },
    });
    if (still()) { hero.params = { contour: 1, route: 1, ais: 1, wind: 0.8, scale: hero.params.scale }; }
    window.addEventListener('resize', () => { hero.resize(); if (!hero.running) hero.render(0); });
    bindRunState(hero, heroCanvas);
  }

  /* ---------- interface fragments are laid out at their real size, then scaled ---------- */
  const setLead = (node, value) => {
    if (!node) return;
    const first = node.firstChild;
    if (first && first.nodeType === 3) { if (first.nodeValue !== value) first.nodeValue = value; } else node.prepend(value);
  };
  // Chromium keeps SVG text laid out at the scale it first saw. After a scale change, nudge the
  // text so it is laid out again at the new size (otherwise labels drift away from their islands).
  const relayoutText = (root) => {
    const texts = [...root.querySelectorAll('svg text')];
    texts.forEach((node) => { const x = node.getAttribute('x'); if (x !== null) node.setAttribute('x', String(Number(x) + 0.01)); });
    requestAnimationFrame(() => texts.forEach((node) => {
      const x = node.getAttribute('x');
      if (x !== null) node.setAttribute('x', String(Math.round((Number(x) - 0.01) * 10) / 10));
    }));
  };
  const fit = (box) => {
    const [w, h] = (box.dataset.fit || '1014x630').split('x').map(Number);
    const k = box.clientWidth / w;
    const value = k.toFixed(4);
    if (box.style.getPropertyValue('--k') === value) return;
    box.style.setProperty('--k', value);
    if (box.classList.contains('fit')) box.style.height = `${Math.round(h * k)}px`;
    relayoutText(box);
  };
  const fitted = [...document.querySelectorAll('[data-fit], .fragment-chart')];
  fitted.forEach(fit);
  if ('ResizeObserver' in window) { const ro = new ResizeObserver((entries) => entries.forEach((e) => fit(e.target))); fitted.forEach((b) => ro.observe(b)); }
  window.addEventListener('resize', () => fitted.forEach(fit));

  // The feature charts reuse the tour's real chart artwork rather than shipping a second copy.
  const chartSource = document.querySelector('[data-tour-screen] #ps-chartSvg');
  // Chromium misplaces scaled, centre-anchored SVG text, so give it an explicit left edge measured
  // in the visitor's own font. The chart looks exactly as before.
  if (chartSource && chartSource.getBBox) {
    chartSource.querySelectorAll('text').forEach((node) => {
      const anchor = getComputedStyle(node).textAnchor;
      if (anchor === 'start' || node.querySelector('tspan')) return;
      try {
        const box = node.getBBox();
        if (!box.width) return;
        node.setAttribute('x', box.x.toFixed(1));
        node.setAttribute('text-anchor', 'start');
        node.style.textAnchor = 'start';
      } catch (error) { /* unrendered text keeps its anchor */ }
    });
  }
  const clones = {};
  document.querySelectorAll('[data-chart-clone]').forEach((host) => {
    if (!chartSource) return;
    const svg = chartSource.cloneNode(true);
    svg.removeAttribute('id');
    const defs = svg.querySelector('defs');
    if (defs) defs.remove();
    svg.querySelectorAll('[id]').forEach((n) => { n.dataset.layer = n.id.replace(/^ps-/, ''); n.removeAttribute('id'); });
    host.append(svg);
    clones[host.dataset.chartClone] = svg;
  });

  /* ---------- the tour: scroll drives the real SKAGER screen ---------- */
  const tour = document.querySelector('[data-tour]');
  if (tour) {
    const stage = tour.querySelector('[data-tour-stage]');
    const screen = tour.querySelector('[data-tour-screen]');
    const beats = [...tour.querySelectorAll('[data-beat]')];
    const steps = [...tour.querySelectorAll('[data-step]')];
    const q = (selector) => screen.querySelector(selector);
    const map = q('#ps-mapTransform');
    const ship = q('#ps-ownShip');
    const shipHeading = q('#ps-ownShipHeading');
    const routeLines = [...screen.querySelectorAll('#ps-routeLayer .chart-route, #ps-routeLayer .chart-route-under, #ps-routeLayer .chart-route-corridor')];
    const track = q('#ps-routeLayer .chart-track');
    const routeMarks = [...screen.querySelectorAll('#ps-routeLayer .map-waypoint, #ps-routeLayer .route-point-label')];
    const aisLayer = q('#ps-aisLayer');
    const wind = q('#ps-windLayer');
    const turnCard = q('.next-turn');
    const turnSub = q('.next-turn .turn-sub');
    const drawer = screen.querySelector(':scope > .drawer');
    const freja = q('#ps-ais-0');
    const metrics = [...screen.querySelectorAll('.data-rail .metric-value')];
    const spark = q('.data-rail .sparkline path');
    const nowLine = q('.timeline-event.now small');
    const footerCog = q('.statusbar .footer-middle b');
    const position = q('#ps-positionLabel');
    const themeButton = q('#ps-themeButton');
    const icons = Object.fromEntries([...document.querySelectorAll('template[data-theme-icon]')].map((t) => [t.dataset.themeIcon, t.innerHTML]));

    // The prototype's passage: own vessel at the start of the Långholmen leg.
    const ROUTE = [[500, 420], [596, 330], [795, 278], [820, 134], [884, 149]];
    const LEG = Math.hypot(ROUTE[1][0] - ROUTE[0][0], ROUTE[1][1] - ROUTE[0][1]);
    const KNOT = 0.12; // chart units per second per knot (time-lapse)
    const CYCLE = 150;
    const TRAFFIC = { 'ps-ais-0': 11.2, 'ps-ais-1': 5.1, 'ps-ais-2': 8.6, 'ps-ais-3': 6.8 };
    const ships = [...screen.querySelectorAll('.ais-ship')].map((node) => {
      const at = (node.getAttribute('transform') || '').match(/translate\(([-\d.]+)[ ,]+([-\d.]+)\)/);
      const rot = (node.querySelector('g') && node.querySelector('g').getAttribute('transform') || '').match(/rotate\(([-\d.]+)/);
      return { node, x: at ? +at[1] : 0, y: at ? +at[2] : 0, heading: rot ? +rot[1] : 0, knots: TRAFFIC[node.id] || 6 };
    });
    if (wind) wind.classList.add('wind-live');

    let progress = 0;
    let auto = smallScreen.matches;
    let clock = 0;
    let autoClock = 0;
    let readout = 0;
    let theme = 'day';
    let geometryClock = 1;
    let theming = 0;
    const lastStyle = new WeakMap();
    const setStyle = (node, prop, value) => {
      if (!node) return;
      const key = lastStyle.get(node) || {};
      if (key[prop] === value) return;
      key[prop] = value;
      lastStyle.set(node, key);
      node.style[prop] = value;
    };
    const current = { pull: still() ? 1 : 0, route: 0, ais: 0, wind: 0, light: 0 };
    const targetsFor = (p) => ({ pull: smooth(0, 0.12, p), route: smooth(0.13, 0.26, p), ais: smooth(0.35, 0.45, p),
      wind: smooth(0.57, 0.67, p), light: smooth(0.8, 0.97, p) });
    const beatFor = (p) => (p < 0.12 ? 0 : p < 0.34 ? 1 : p < 0.56 ? 2 : p < 0.78 ? 3 : 4);
    const autoProgress = () => {
      const c = autoClock % 30;
      return c < 2 ? 0.1 : c < 8 ? 0.28 : c < 14 ? 0.5 : c < 20 ? 0.72 : 0.78 + ((c - 20) / 10) * 0.22;
    };
    const fmt = (v, n = 1) => v.toFixed(n);

    const render = (dt) => {
      clock += dt;
      if (auto) autoClock += dt;
      const p = auto ? autoProgress() : progress;
      const target = targetsFor(p);
      Object.keys(current).forEach((k) => { current[k] = dt ? approach(current[k], target[k], dt, 6) : target[k]; });

      // Own vessel advances along the leg; the chart follows it, as in "Following Reptil".
      // It moves under a pixel a second, so ten geometry updates a second are plenty.
      const t = clock % CYCLE;
      geometryClock += dt;
      const moveNow = geometryClock >= 0.1 || !dt;
      if (moveNow) geometryClock = 0;
      const d = Math.min(LEG * 0.85, t * 6.3 * KNOT);
      const bx = ROUTE[0][0] + (ROUTE[1][0] - ROUTE[0][0]) * (d / LEG);
      const by = ROUTE[0][1] + (ROUTE[1][1] - ROUTE[0][1]) * (d / LEG);
      const dx = bx - ROUTE[0][0];
      const dy = by - ROUTE[0][1];
      if (moveNow) {
      if (map) map.setAttribute('transform', `translate(${fmt(-dx, 2)} ${fmt(-dy, 2)}) translate(500 315) scale(1) rotate(0) translate(-500 -315)`);
      if (ship) ship.setAttribute('transform', `translate(${fmt(bx, 2)} ${fmt(by, 2)})`);
      if (shipHeading) shipHeading.setAttribute('transform', `rotate(${fmt(41 + noise(clock * 0.3, 2) * 1.6)})`);
      const tail = ROUTE.slice(1).map((pt) => pt.join(' ')).join(' ');
      routeLines.forEach((n) => n.setAttribute('d', `M${fmt(bx, 2)} ${fmt(by, 2)} ${tail}`));
      if (track) track.setAttribute('d', `M373 673 405 552Q430 495 500 420L${fmt(bx, 2)} ${fmt(by, 2)}`);
      ships.forEach((s) => {
        const a = (s.heading * Math.PI) / 180;
        const run = t * s.knots * KNOT;
        s.node.setAttribute('transform', `translate(${fmt(s.x + Math.sin(a) * run, 2)} ${fmt(s.y - Math.cos(a) * run, 2)})`);
      });
      }
      const fade = t > CYCLE - 0.8 ? (CYCLE - t) / 0.8 : t < 0.8 && clock > 1 ? t / 0.8 : 1;
      setStyle(map, 'opacity', fade.toFixed(2));

      // Story layers. The route draws by stroke length so each line keeps its own styling.
      routeLines.forEach((n) => {
        if (current.route > 0.995) { setStyle(n, 'strokeDasharray', ''); return; }
        const length = n.getTotalLength();
        setStyle(n, 'strokeDasharray', `${fmt(current.route * length)} ${fmt(length + 1)}`);
      });
      routeMarks.forEach((n) => setStyle(n, 'opacity', fmt(smooth(0.6, 1, current.route), 2)));
      setStyle(aisLayer, 'opacity', fmt(current.ais, 2));
      setStyle(wind, 'opacity', fmt(current.wind, 2));
      if (turnCard) turnCard.classList.toggle('is-shown', p > 0.14);
      if (drawer) drawer.classList.toggle('is-shown', p > 0.39 && p < 0.58);
      if (freja) freja.classList.toggle('is-selected', p > 0.39 && p < 0.58);
      const next = current.light < 0.34 ? 'day' : current.light < 0.67 ? 'dusk' : 'night';
      if (theming > 0) { theming -= dt; if (theming <= 0) screen.classList.remove('is-theming'); }
      if (next !== theme) {
        theme = next;
        screen.classList.add('is-theming');
        theming = 1.1;
        screen.dataset.theme = theme;
        if (themeButton && icons[theme]) themeButton.innerHTML = icons[theme];
      }
      if (lastStyle.get(stage) !== fmt(current.pull, 3)) { lastStyle.set(stage, fmt(current.pull, 3)); stage.style.setProperty('--pull', fmt(current.pull, 3)); }
      const beat = beatFor(p);
      beats.forEach((node) => node.classList.toggle('is-active', Number(node.dataset.beat) === beat));
      steps.forEach((node) => node.classList.toggle('is-active', Number(node.dataset.step) <= beat && beat > 0));

      // Live readouts, a few times a second.
      readout += dt;
      if (readout < 0.3 && dt) return;
      readout = 0;
      const sog = 6.3 + noise(clock * 0.4, 1) * 0.25;
      const cog = Math.round(43 + noise(clock * 0.3, 2) * 1.4);
      if (metrics[0]) setLead(metrics[0], fmt(sog));
      if (metrics[1]) setLead(metrics[1], fmt(8.4 + noise(clock * 0.2, 4) * 1.1));
      if (metrics[2]) setLead(metrics[2], fmt(14.3 + noise(clock * 0.5, 7) * 1.2));
      if (spark) {
        const pts = Array.from({ length: 16 }, (_, i) => `${i * 10} ${fmt(14 - noise((clock + i * 1.3) * 0.35, 1) * 9)}`);
        spark.setAttribute('d', `M${pts.join(' ')}`);
      }
      const left = Math.hypot(ROUTE[1][0] - bx, ROUTE[1][1] - by) / 190;
      if (turnSub) turnSub.textContent = `${fmt(left)} nm · in ${Math.max(1, Math.round((left / sog) * 60))} min · new course 075°`;
      if (nowLine) nowLine.textContent = `${String(cog).padStart(3, '0')}° COG · ${fmt(sog)} kn`;
      if (footerCog) footerCog.textContent = `${String(cog).padStart(3, '0')}°`;
      if (position) position.textContent = `58° ${(20.462 - dy * 0.0045).toFixed(3)}′ N   016° ${(48.218 + dx * 0.0085).toFixed(3)}′ E`;
    };

    let running = false;
    let raf = 0;
    let last = 0;
    const tick = (now) => {
      const dt = Math.min(0.05, (now - last) / 1000);
      last = now;
      render(dt);
      if (running) raf = requestAnimationFrame(tick);
    };
    let visible = true;
    const update = () => {
      const on = visible && !document.hidden && !still();
      if (on && !running) { running = true; last = performance.now(); raf = requestAnimationFrame(tick); }
      if (!on && running) { running = false; cancelAnimationFrame(raf); }
    };
    const measure = () => {
      auto = smallScreen.matches || still();
      tour.classList.toggle('is-auto', auto);
      const rect = tour.getBoundingClientRect();
      const span = rect.height - window.innerHeight;
      progress = span > 0 ? clamp(-rect.top / span) : 0;
    };
    const fitStage = () => {
      const value = (stage.clientWidth / 1280).toFixed(4);
      if (screen.style.getPropertyValue('--k') === value) return;
      screen.style.setProperty('--k', value);
      relayoutText(screen);
    };
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(() => relayoutText(screen));
    window.addEventListener('scroll', measure, { passive: true });
    window.addEventListener('resize', () => { measure(); fitStage(); });
    smallScreen.addEventListener('change', measure);
    if ('ResizeObserver' in window) new ResizeObserver(fitStage).observe(stage);
    measure();
    fitStage();
    if (still()) {
      tour.classList.add('is-still');
      progress = 0.3;
      auto = false;
      clock = 20;
      render(0);
    } else {
      render(0);
    }
    whenVisible(stage, (v) => { visible = v; update(); });
    document.addEventListener('visibilitychange', update);
    reducedMotion.addEventListener('change', update);
    update();
  }

  /* ---------- reveal on scroll ---------- */
  const revealables = [...document.querySelectorAll('[data-reveal]')];
  if ('IntersectionObserver' in window && !still()) {
    const observer = new IntersectionObserver((entries) => entries.forEach((entry) => {
      if (!entry.isIntersecting) return;
      entry.target.classList.add('is-revealed');
      observer.unobserve(entry.target);
    }), { rootMargin: '0px 0px -12% 0px' });
    revealables.forEach((node) => observer.observe(node));
  } else {
    revealables.forEach((node) => node.classList.add('is-revealed'));
  }

  /* ---------- feature visuals ---------- */
  const loop = (element, step) => {
    let running = false;
    let raf = 0;
    let last = 0;
    let time = 0;
    const tick = (now) => {
      const dt = Math.min(0.05, (now - last) / 1000);
      last = now;
      time += dt;
      step(time, dt);
      if (running) raf = requestAnimationFrame(tick);
    };
    const set = (on) => {
      if (on && !running) { running = true; last = performance.now(); raf = requestAnimationFrame(tick); }
      if (!on && running) { running = false; cancelAnimationFrame(raf); }
    };
    let visible = false;
    const update = () => set(visible && !document.hidden && !still());
    whenVisible(element, (v) => { visible = v; update(); }, '0px');
    document.addEventListener('visibilitychange', update);
    reducedMotion.addEventListener('change', update);
    step(still() ? 6 : 0, 0);
  };

  // Navigation: the real Chart presentation panel switches layers off, one by one.
  const declutter = document.querySelector('[data-declutter]');
  if (declutter && clones.layers) {
    const svg = clones.layers;
    const layer = (name) => [...svg.querySelectorAll(`[data-layer="${name}"]`)];
    const cls = (name) => [...svg.querySelectorAll(name)];
    const TARGETS = {
      'Depth soundings': () => layer('depthLayer'),
      'Chart symbols': () => [...layer('chartSymbolLayer'), ...layer('chartLightSectorLayer')],
      'Symbol labels': () => cls('.chart-symbol-label'),
      'Depth contours': () => layer('contourLayer'),
      'Route corridor': () => cls('.chart-route-corridor'),
      'AIS vessels': () => layer('aisLayer'),
      'Wind vectors': () => layer('windLayer'),
    };
    const switches = Object.fromEntries([...declutter.querySelectorAll('.toggle[role=switch]')]
      .map((n) => [n.getAttribute('aria-label'), n]).filter(([name]) => TARGETS[name]));
    Object.entries(switches).forEach(([name, node]) => {
      const off = node.getAttribute('aria-checked') !== 'true';
      TARGETS[name]().forEach((el) => { el.dataset.layer = el.dataset.layer || name; el.classList.toggle('is-off', off); });
    });
    const ORDER = ['Depth soundings', 'Symbol labels', 'Chart symbols', 'Depth contours'].filter((n) => switches[n]);
    const set = (name, on) => {
      switches[name].setAttribute('aria-checked', String(on));
      TARGETS[name]().forEach((el) => { el.dataset.layer = el.dataset.layer || name; el.classList.toggle('is-off', !on); });
    };
    let stepIndex = -1;
    let wait = 1.2;
    loop(declutter, (time, dt) => {
      if (!dt) { if (still()) ORDER.forEach((n) => set(n, false)); return; }
      wait -= dt;
      if (wait > 0) return;
      stepIndex = (stepIndex + 1) % (ORDER.length * 2 + 2);
      if (stepIndex < ORDER.length) { set(ORDER[stepIndex], false); wait = 1.3; }
      else if (stepIndex === ORDER.length) { wait = 2.6; }
      else if (stepIndex < ORDER.length * 2 + 1) { set(ORDER[stepIndex - ORDER.length - 1], true); wait = 0.5; }
      else { wait = 2.2; }
    });
  }

  // Instruments: the real Instruments view, live.
  const instruments = document.querySelector('[data-instruments]');
  if (instruments) {
    const tiles = [...instruments.querySelectorAll('.instrument-tile b')];
    const stats = [...instruments.querySelectorAll('.stats-grid strong')];
    const arrows = [...instruments.querySelectorAll('.wind-rose path[transform*="-72"]')];
    const BASE = [6.3, 43, 6.1, 8.4, 17.1, 54, 8.2, 16.8];
    const SWING = [0.25, 1.5, 0.2, 0.9, 1.4, 3, 1.2, 0.05];
    const DIGITS = [1, 0, 1, 1, 1, 0, 1, 1];
    let clock = 0;
    loop(instruments, (time, dt) => {
      clock += dt;
      if (clock < 0.35 && dt) return;
      clock = 0;
      tiles.forEach((b, i) => {
        const v = BASE[i] + noise(time * 0.4, i * 1.7) * SWING[i];
        setLead(b, i === 1 ? String(Math.round(v)).padStart(3, '0') : v.toFixed(DIGITS[i]));
      });
      if (stats[0]) setLead(stats[0], (14.3 + noise(time * 0.5, 7) * 1.2).toFixed(1));
      const twa = Math.round(72 + noise(time * 0.3, 9) * 4);
      if (stats[1]) setLead(stats[1], String(twa));
      arrows.forEach((n) => n.setAttribute('transform', `rotate(${-twa} 140 140)`));
    });
  }

  // Autopilot: the real helm-control panel, adjusting the commanded heading.
  const pilot = document.querySelector('[data-pilot]');
  if (pilot) {
    const value = pilot.querySelector('.dial-value span');
    const pointer = pilot.querySelector('.heading-dial path[fill]');
    const keys = Object.fromEntries([...pilot.querySelectorAll('[data-heading]')].map((n) => [n.dataset.heading, n]));
    const SCRIPT = ['10', '1', '1', '-10', '-1', '-1'];
    let heading = 41;
    let shown = 41;
    let index = -1;
    let wait = 1.4;
    loop(pilot, (time, dt) => {
      if (!dt) return;
      wait -= dt;
      if (wait <= 0) {
        index = (index + 1) % SCRIPT.length;
        const key = keys[SCRIPT[index]];
        heading = (heading + Number(SCRIPT[index]) + 360) % 360;
        if (key) { key.classList.add('is-pressed'); setTimeout(() => key.classList.remove('is-pressed'), 380); }
        wait = 1.8;
      }
      let diff = heading - shown;
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      shown = (shown + diff * (1 - Math.exp(-dt * 3)) + 360) % 360;
      text(value, `${String(Math.round(shown) % 360).padStart(3, '0')}°`);
      if (pointer) pointer.setAttribute('transform', `rotate(${shown.toFixed(1)} 100 100)`);
    });
  }

  // Weather: the chart's wind vectors veer as the forecast hour advances.
  const weather = document.querySelector('[data-weather]');
  if (weather && clones.wind) {
    const vectors = [...clones.wind.querySelectorAll('[data-layer="windLayer"] path')].map((node) => {
      const m = (node.getAttribute('d') || '').match(/^m([-\d.]+) ([-\d.]+)/);
      return { node, x: m ? +m[1] + 9 : 0, y: m ? +m[2] - 4 : 0, i: Math.random() * 10 };
    });
    vectors.forEach((v) => { v.node.setAttribute('opacity', '.75'); v.node.setAttribute('stroke-width', '1.6'); });
    const hourNode = weather.querySelector('[data-forecast-hour]');
    const speedNode = weather.querySelector('[data-forecast-speed]');
    loop(weather, (time) => {
      const k = (time % 12) / 12;
      text(hourNode, `${12 + Math.floor(k * 6)}:00`);
      text(speedNode, String(Math.round(11 + Math.sin(k * Math.PI) * 6)));
      vectors.forEach((v) => {
        const veer = -12 + k * 30 + Math.sin(time * 0.8 + v.i) * 4;
        v.node.setAttribute('transform', `rotate(${veer.toFixed(1)} ${v.x} ${v.y})`);
      });
    });
  }
})();
