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
    day: palette({ water: '#e2eeee', shallow: '#b7d5d9', land: '#eeeee2', shore: '#a3b8a5', contour: '#9fc3c7',
      route: '#267c76', chartText: '#687b7a', ais: '#b9578a', own: '#16262a', wind: '#2f8a84' }),
    dusk: palette({ water: '#304a54', shallow: '#3f606b', land: '#4e615d', shore: '#748779', contour: '#5a7e86',
      route: '#b0dfc8', chartText: '#adbbb1', ais: '#d9a0bb', own: '#f3f5ee', wind: '#b0dfc8' }),
    night: palette({ water: '#101b21', shallow: '#17272e', land: '#25342f', shore: '#46574a', contour: '#2c444c',
      route: '#91bca2', chartText: '#758579', ais: '#a77f8a', own: '#b8b5a7', wind: '#7f9f9a' }),
  };
  // Interface chrome colours from the SKAGER design tokens (day / dusk / night).
  const CHROME = {
    day: palette({ bg: '#152326', surface: '#1d2d31', raised: '#26393d', border: '#35464a', text: '#f3f5ee',
      secondary: '#aabdbd', muted: '#7e9699', active: '#b6efce', instrument: '#7bc8d7' }),
    dusk: palette({ bg: '#1d282e', surface: '#25343b', raised: '#30444b', border: '#405059', text: '#e2e5db',
      secondary: '#acb9b7', muted: '#819394', active: '#9bc5b1', instrument: '#82b0c1' }),
    night: palette({ bg: '#0c1115', surface: '#141c21', raised: '#1d282f', border: '#29353b', text: '#b8b5a7',
      secondary: '#91988e', muted: '#747d77', active: '#85a995', instrument: '#78989c' }),
  };
  const mixRGB = (a, b, t) => [0, 1, 2].map((i) => a[i] + (b[i] - a[i]) * t);
  const mixPalette = (a, b, t) => Object.fromEntries(Object.keys(a).map((k) => [k, mixRGB(a[k], b[k], t)]));
  const threeWay = (set, t) => (t < 0.5 ? mixPalette(set.day, set.dusk, t * 2) : mixPalette(set.dusk, set.night, (t - 0.5) * 2));
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
      this.params = { contour: 1, route: 1, ais: 1, wind: 1, select: 0, scale: options.scale || 1 };
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
      const s = WORLD.wrap(boat.s);
      let next = WORLD.waypointArc.findIndex((arc) => arc > s + 1);
      if (next < 0) next = 0;
      const count = WORLD.waypoints.length;
      const wp = WORLD.waypoints[next];
      const before = WORLD.waypoints[(next - 1 + count) % count];
      const after = WORLD.waypoints[(next + 1) % count];
      const turn = (wp[0] - before[0]) * (after[1] - wp[1]) - (wp[1] - before[1]) * (after[0] - wp[0]);
      const outgoing = (Math.atan2(after[1] - wp[1], after[0] - wp[0]) * 180) / Math.PI + 90;
      const distance = Math.hypot(wp[0] - boat.x, wp[1] - boat.y);
      const bearing = (Math.atan2(wp[1] - boat.y, wp[0] - boat.x) * 180) / Math.PI + 90;
      const t = this.time;
      return {
        sog: 6.3 + noise(t * 0.4, 1) * 0.35,
        cog: ((boat.heading * 180) / Math.PI + 90 + 360) % 360,
        depth: WORLD.depthAt(boat.x, boat.y) + noise(t, 4) * 0.15,
        wind: 14.2 + noise(t * 0.5, 7) * 1.6,
        windDir: 225 + noise(t * 0.2, 9) * 12,
        battery: 68 - ((t / 60) % 25),
        nextDistance: distance / 240,
        nextBearing: (bearing + 360) % 360,
        turnSide: turn > 0 ? 'Starboard' : 'Port',
        nextCourse: (outgoing + 360) % 360,
        traffic: this.nearestTraffic(),
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
        const selected = params.select > 0.01 ? this.nearestTraffic() : null;
        if (selected) {
          ctx.save();
          ctx.translate(selected.x, selected.y);
          ctx.scale(1 / scale, 1 / scale);
          ctx.strokeStyle = rgba(pal.ais, params.select);
          ctx.lineWidth = 2;
          const r = 24 + (1 - params.select) * 16;
          [0, 1, 2, 3].forEach((q) => {
            ctx.beginPath();
            ctx.arc(0, 0, r, q * Math.PI / 2 + 0.25, q * Math.PI / 2 + Math.PI / 2 - 0.25);
            ctx.stroke();
          });
          ctx.restore();
        }
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

    nearestTraffic() {
      const boat = this.boat();
      let best = null;
      WORLD.TRAFFIC.forEach((ship) => {
        const p = WORLD.at(ship.offset * WORLD.length + this.time * ship.speed);
        const x = p.x - p.ny * ship.lane;
        const y = p.y + p.nx * ship.lane;
        const d = Math.hypot(x - boat.x, y - boat.y);
        if (!best || d < best.distance) {
          const ahead = (x - boat.x) * Math.cos(boat.heading) + (y - boat.y) * Math.sin(boat.heading) > 0;
          best = { ship, x, y, distance: d, ahead };
        }
      });
      return best;
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

  /* ---------- the tour: scroll drives the product story ---------- */
  const tour = document.querySelector('[data-tour]');
  if (tour) {
    const stage = tour.querySelector('[data-tour-stage]');
    const screen = tour.querySelector('[data-tour-screen]');
    const canvas = tour.querySelector('[data-tour-canvas]');
    const beats = [...tour.querySelectorAll('[data-beat]')];
    const steps = [...tour.querySelectorAll('[data-step]')];
    const cards = Object.fromEntries([...tour.querySelectorAll('[data-card]')].map((n) => [n.dataset.card, n]));
    const values = Object.fromEntries([...tour.querySelectorAll('[data-value]')].map((n) => [n.dataset.value, n]));
    const modes = [...tour.querySelectorAll('[data-mode]')];
    const DESIGN = { width: 1280, height: 800, chart: 800 - 52 - 64 };
    let progress = 0;
    let auto = smallScreen.matches;
    let autoClock = 0;
    let readout = 0;
    let lastChrome = '';
    const current = { pull: still() ? 1 : 0, route: 0, ais: 0, wind: 0, light: 0, select: 0 };

    const targetsFor = (p) => ({
      pull: smooth(0, 0.12, p),
      route: smooth(0.13, 0.26, p),
      ais: smooth(0.35, 0.45, p),
      wind: smooth(0.57, 0.67, p),
      light: smooth(0.8, 0.97, p),
      select: p > 0.39 && p < 0.58 ? 1 : 0,
    });
    const beatFor = (p) => (p < 0.12 ? 0 : p < 0.34 ? 1 : p < 0.56 ? 2 : p < 0.78 ? 3 : 4);
    const autoProgress = () => {
      // Phones: cycle the beats on a clock instead of pinning the scroll.
      const cycle = autoClock % 30;
      return cycle < 2 ? 0.1 : cycle < 8 ? 0.28 : cycle < 14 ? 0.5 : cycle < 20 ? 0.72 : 0.78 + ((cycle - 20) / 10) * 0.22;
    };

    const scene = new Scene(canvas, {
      palette: PALETTES.day,
      startTime: 12,
      maxDpr: 2,
      maxParticles: 160,
      aisLabels: true,
      arcOffset: 2100,
      scale: 1,
      anchor: () => [0.45, 0.55],
      update(sc, dt) {
        if (auto) autoClock += dt;
        const p = auto ? autoProgress() : progress;
        const target = targetsFor(p);
        Object.keys(current).forEach((k) => { current[k] = approach(current[k], target[k], dt, 6); });
        sc.params.contour = 1;
        sc.params.route = current.route;
        sc.params.ais = current.ais;
        sc.params.wind = current.wind;
        sc.params.select = current.select;
        sc.params.scale = 1.55 - current.pull * 0.6;
        sc.palette = threeWay(PALETTES, current.light);
        applyStage(p);
      },
      onFrame(sc, dt) {
        readout += dt;
        if (readout < 0.25) return;
        readout = 0;
        const s = sc.state();
        text(values.sog, s.sog.toFixed(1));
        text(values.depth, s.depth.toFixed(1));
        text(values.wind, s.wind.toFixed(1));
        text(values.battery, Math.round(s.battery).toString());
        text(values.heading, `${Math.round(s.cog).toString().padStart(3, '0')}°`);
        text(values.dtw, s.nextDistance.toFixed(1));
        text(values.btw, `${Math.round(s.nextBearing).toString().padStart(3, '0')}°`);
        text(values.next, s.nextDistance.toFixed(1));
        text(values.side, s.turnSide);
        text(values.course, `${Math.round(s.nextCourse).toString().padStart(3, '0')}°`);
        if (s.traffic) {
          text(values.aisName, s.traffic.ship.name);
          text(values.aisType, s.traffic.ship.type);
          text(values.aisRange, (s.traffic.distance / 240).toFixed(1));
          text(values.aisPass, s.traffic.ahead ? 'Crossing ahead' : 'Passing astern');
        }
      },
    });

    const applyStage = (p) => {
      const beat = beatFor(p);
      beats.forEach((node) => node.classList.toggle('is-active', Number(node.dataset.beat) === beat));
      steps.forEach((node) => node.classList.toggle('is-active', Number(node.dataset.step) <= beat && beat > 0));
      if (cards.turn) cards.turn.classList.toggle('is-shown', p > 0.16 && p < 0.36);
      if (cards.ais) cards.ais.classList.toggle('is-shown', p > 0.39 && p < 0.58);
      if (cards.weather) cards.weather.classList.toggle('is-shown', p > 0.6 && p < 0.8);
      const lightName = current.light < 0.25 ? 'day' : current.light < 0.75 ? 'dusk' : 'night';
      modes.forEach((node) => node.classList.toggle('is-active', node.dataset.mode === lightName));
      stage.style.setProperty('--pull', current.pull.toFixed(4));
      const chrome = threeWay(CHROME, current.light);
      const key = Object.values(chrome).map((c) => c.map(Math.round).join(',')).join(';');
      if (key !== lastChrome) {
        lastChrome = key;
        Object.entries(chrome).forEach(([name, colour]) => screen.style.setProperty(`--ui-${name}`, rgba(colour)));
      }
    };

    const fit = () => {
      const k = stage.clientWidth / DESIGN.width;
      screen.style.setProperty('--k', k.toFixed(5));
      stage.style.setProperty('--stage-height', `${Math.round(DESIGN.height * k)}px`);
      scene.options.pixelScale = Math.max(0.5, k);
      scene.resize(DESIGN.width, DESIGN.chart);
      if (!scene.running) scene.render(0);
    };
    const measure = () => {
      auto = smallScreen.matches || still();
      tour.classList.toggle('is-auto', auto);
      const rect = tour.getBoundingClientRect();
      const span = rect.height - window.innerHeight;
      progress = span > 0 ? clamp(-rect.top / span) : 0;
    };
    window.addEventListener('scroll', measure, { passive: true });
    window.addEventListener('resize', () => { measure(); fit(); });
    smallScreen.addEventListener('change', () => { measure(); fit(); });
    if ('ResizeObserver' in window) new ResizeObserver(fit).observe(stage);
    measure();
    fit();
    if (still()) {
      Object.assign(current, { pull: 1, route: 1, ais: 1, wind: 1, light: 0 });
      scene.params = { contour: 1, route: 1, ais: 1, wind: 1, scale: 0.95 };
      applyStage(0.5);
      tour.classList.add('is-still');
    }
    bindRunState(scene, stage);
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

  // Instruments: live tiles with a speed sparkline.
  const rail = document.querySelector('[data-rail]');
  if (rail) {
    const v = Object.fromEntries([...rail.querySelectorAll('[data-rail-value]')].map((n) => [n.dataset.railValue, n]));
    const spark = rail.querySelector('[data-spark]');
    const meter = rail.querySelector('[data-depth-meter]');
    const needle = rail.querySelector('[data-wind-needle]');
    const battery = rail.querySelector('[data-battery-bar]');
    const history = Array.from({ length: 40 }, (_, i) => 6.3 + noise(i * 0.3, 1) * 0.4);
    let clock = 0;
    loop(rail, (t, dt) => {
      clock += dt;
      if (clock < 0.12 && dt) return;
      clock = 0;
      const sog = 6.3 + noise(t * 0.5, 1) * 0.45;
      const depth = 8.4 + noise(t * 0.25, 3) * 2.6;
      const wind = 14 + noise(t * 0.6, 5) * 2.2;
      const dir = 225 + noise(t * 0.3, 8) * 14;
      const soc = 68 - (t % 40) * 0.05;
      history.push(sog);
      history.shift();
      text(v.sog, sog.toFixed(1));
      text(v.depth, depth.toFixed(1));
      text(v.wind, wind.toFixed(1));
      text(v.dir, `${Math.round(dir)}°`);
      text(v.battery, soc.toFixed(0));
      if (spark) spark.setAttribute('points', history.map((y, i) => `${(i / 39) * 120},${30 - (y - 5.6) * 18}`).join(' '));
      if (meter) meter.style.setProperty('--level', clamp(depth / 20).toFixed(3));
      if (needle) needle.style.setProperty('--angle', `${dir + 180}deg`);
      if (battery) battery.style.setProperty('--level', (soc / 100).toFixed(3));
    });
  }

  // Autopilot: a short loop of adjustments.
  const pilot = document.querySelector('[data-pilot]');
  if (pilot) {
    const headingNode = pilot.querySelector('[data-pilot-heading]');
    const rose = pilot.querySelector('[data-pilot-rose]');
    const keys = Object.fromEntries([...pilot.querySelectorAll('[data-key]')].map((n) => [n.dataset.key, n]));
    const SCRIPT = [['auto', 0], ['+10', 10], ['+1', 1], ['+1', 1], ['-10', -10], ['-1', -1]];
    let heading = 143;
    let shown = 143;
    let index = -1;
    let wait = 0;
    loop(pilot, (t, dt) => {
      if (!dt) { text(headingNode, '143°'); return; }
      wait -= dt;
      if (wait <= 0) {
        index = (index + 1) % SCRIPT.length;
        const [key, delta] = SCRIPT[index];
        heading = (heading + delta + 360) % 360;
        Object.values(keys).forEach((k) => k.classList.remove('is-pressed'));
        if (keys[key]) {
          keys[key].classList.add('is-pressed');
          setTimeout(() => keys[key].classList.remove('is-pressed'), 420);
        }
        wait = key === 'auto' ? 2.2 : 1.8;
      }
      let diff = heading - shown;
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      shown = (shown + diff * (1 - Math.exp(-dt * 3)) + 360) % 360;
      text(headingNode, `${Math.round(shown).toString().padStart(3, '0')}°`);
      if (rose) rose.style.setProperty('--angle', `${-shown}deg`);
    });
  }

  // Energy: charge drains as the boat crosses the passage.
  const energy = document.querySelector('[data-energy]');
  if (energy) {
    const dot = energy.querySelector('[data-energy-boat]');
    const path = energy.querySelector('[data-energy-path]');
    const now = energy.querySelector('[data-energy-now]');
    const bar = energy.querySelector('[data-energy-bar]');
    const total = path ? path.getTotalLength() : 0;
    loop(energy, (t) => {
      const cycle = (t % 9) / 7.5;
      const k = clamp(cycle);
      if (path && dot) {
        const p = path.getPointAtLength(total * k);
        dot.setAttribute('transform', `translate(${p.x.toFixed(1)} ${p.y.toFixed(1)})`);
        energy.style.setProperty('--travelled', k.toFixed(3));
      }
      const soc = 92 - k * 49;
      text(now, soc.toFixed(0));
      if (bar) bar.style.setProperty('--level', (soc / 100).toFixed(3));
    });
  }

  // Weather: wind chevrons swing as the forecast hour advances.
  const weather = document.querySelector('[data-weather]');
  if (weather) {
    const arrows = [...weather.querySelectorAll('[data-chevron]')];
    const hourNode = weather.querySelector('[data-forecast-hour]');
    const speedNode = weather.querySelector('[data-forecast-speed]');
    const handle = weather.querySelector('[data-forecast-handle]');
    loop(weather, (t) => {
      const k = (t % 12) / 12;
      const hour = 12 + k * 6;
      text(hourNode, `${Math.floor(hour)}:00`);
      text(speedNode, (11 + Math.sin(k * Math.PI) * 7).toFixed(0));
      if (handle) handle.style.setProperty('--at', k.toFixed(3));
      arrows.forEach((node) => {
        const x = Number(node.dataset.x);
        const y = Number(node.dataset.y);
        const veer = 200 + k * 60 + Math.sin(x * 0.9 + k * 4) * 10 + Math.cos(y * 0.8) * 8;
        const strength = 0.75 + Math.sin(k * Math.PI + x * 0.3) * 0.25;
        node.style.setProperty('--angle', `${veer + 180}deg`);
        node.style.setProperty('--strength', strength.toFixed(3));
      });
    });
  }
})();
