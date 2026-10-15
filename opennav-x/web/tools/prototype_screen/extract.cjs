#!/usr/bin/env node
/* Derive the homepage's SKAGER interface from the unchanged v8 HTML prototype.
 *
 * The prototype (docs/design/prototype/index.html) is the authoritative visual
 * reference and is never edited. This tool opens it read-only in headless
 * Chromium at its 1280 x 800 reference size, captures the states the homepage
 * shows, keeps only the CSS rules those states use, scopes everything under
 * .proto-screen and applies the approved SKAGER identity.
 *
 * Usage (playwright-core and a Chromium are tool-only, not site dependencies):
 *   NODE_PATH=/path/to/node_modules node extract.cjs --chrome /path/to/chrome
 */
'use strict';
const crypto = require('crypto');
const fs = require('fs');
const path = require('path');
const { chromium } = require('playwright-core');

const DOCS = path.resolve(__dirname, '../../../docs/design');
const MARKETING = path.resolve(__dirname, '../../skager_web/marketing');
const PROTOTYPE = path.join(DOCS, 'prototype/index.html');
const INCLUDES = path.join(MARKETING, 'templates/marketing/includes');
const CSS_OUT = path.join(MARKETING, 'static/marketing/skager-screen.css');

const arg = (name) => { const i = process.argv.indexOf(name); return i > 0 ? process.argv[i + 1] : undefined; };

async function main() {
  const manifest = JSON.parse(fs.readFileSync(path.join(DOCS, 'prototype-original.json'), 'utf8'));
  const sha = crypto.createHash('sha256').update(fs.readFileSync(PROTOTYPE)).digest('hex');
  if (sha !== manifest.htmlSha256) throw new Error(`Prototype hash ${sha} does not match the original manifest`);

  const browser = await chromium.launch({ executablePath: arg('--chrome') });
  const page = await (await browser.newContext({ viewport: { width: 1280, height: 800 }, deviceScaleFactor: 1 })).newPage();
  await page.goto('file://' + PROTOTYPE, { waitUntil: 'load' });
  await page.waitForTimeout(1500);

  // Rule usage is gathered in every captured state; capture() also returns markup.
  await page.evaluate(() => {
    window.__used = new Set();
    const strip = (s) => s.replace(/::?(before|after|placeholder|selection|marker|hover|focus-visible|focus-within|focus|active|visited|disabled|enabled|checked|first-letter|first-line|-webkit-[a-z-]+|-moz-[a-z-]+)(\([^)]*\))?/g, '');
    const split = (text) => { const out = []; let depth = 0; let cur = ''; for (const ch of text) { if (ch === '(') depth++; if (ch === ')') depth--; if (ch === ',' && !depth) { out.push(cur.trim()); cur = ''; } else cur += ch; } out.push(cur.trim()); return out; };
    window.__split = split;
    window.__markUsage = (roots) => {
      const inside = (el) => roots.some((r) => r === el || r.contains(el));
      const visit = (rules, prefix) => [...rules].forEach((rule, i) => {
        const key = prefix + i;
        if (rule.cssRules && !rule.selectorText) { visit(rule.cssRules, key + '/'); return; }
        if (!rule.selectorText) return;
        split(rule.selectorText).forEach((sel, j) => {
          if (/^(:root|html|body)\b/.test(sel) && !/\s/.test(sel.replace(/^(:root|html|body)/, '').trim())) { __used.add(key + ':' + j); return; }
          let hits;
          try { hits = [...document.querySelectorAll(strip(sel) || '*')]; } catch { __used.add(key + ':' + j); return; }
          if (hits.some(inside)) __used.add(key + ':' + j);
        });
      });
      [...document.styleSheets].forEach((sheet, s) => visit(sheet.cssRules, s + '/'));
    };
    window.__capture = (el) => {
      // Drop anything not rendered, round geometry, fold inline styles into classes.
      const live = [el, ...el.querySelectorAll('*')];
      live.forEach((n) => { const cs = getComputedStyle(n); if (n.hidden || cs.display === 'none') n.setAttribute('data-x-drop', ''); });
      const clone = el.cloneNode(true);
      live.forEach((n) => n.removeAttribute('data-x-drop'));
      clone.querySelectorAll('[data-x-drop]').forEach((n) => n.remove());
      const round = /^(d|points|transform|x|y|x1|x2|y1|y2|cx|cy|r|rx|ry|width|height|viewBox|stroke-dasharray|offset|font-size|stroke-width|dx|dy)$/;
      [clone, ...clone.querySelectorAll('*')].forEach((n) => [...n.attributes].forEach((a) => {
        if (round.test(a.name)) n.setAttribute(a.name, a.value.replace(/(-?\d+\.\d)\d+/g, '$1').replace(/(\d)\.0(?!\d)/g, '$1'));
      }));
      return clone.outerHTML;
    };
  });

  const click = async (selector) => { await page.click(selector, { force: true }); await page.waitForTimeout(700); };
  const close = async () => { await page.keyboard.press('Escape'); await page.waitForTimeout(500); };
  const usage = (selectors) => page.evaluate((list) => __markUsage(list.flatMap((s) => [...document.querySelectorAll(s)])), selectors);

  // Main chart view with the prototype's wind-vector layer switched on.
  await click('[aria-label="Chart layers"]');
  const layers = await page.evaluate(() => __capture(document.querySelector('#drawer')));
  await usage(['#drawer']);
  await click('[data-action="toggleWind"]');
  await close();
  const SCREEN_PARTS = ['.topbar', '.sidebar', '#workspace', '.data-rail', '.statusbar'];
  // The prototype's own theme button cycles day -> dusk -> night and swaps its icon.
  const themeIcons = {};
  for (const theme of ['day', 'dusk', 'night', 'day']) {
    if (theme !== 'day' || Object.keys(themeIcons).length) { await click('#themeButton'); await page.waitForTimeout(2600); }
    const current = await page.evaluate(() => document.querySelector('#app').dataset.theme);
    if (current !== theme) throw new Error(`Expected ${theme} theme, found ${current}`);
    themeIcons[theme] = await page.evaluate(() => document.querySelector('#themeButton').innerHTML);
    await usage(['#app']);
  }
  const screen = await page.evaluate((parts) => parts.map((s) => __capture(document.querySelector(s))).join('\n'), SCREEN_PARTS);

  // AIS target details (Freja).
  await click('[data-target="0"]');
  const ais = await page.evaluate(() => __capture(document.querySelector('#drawer')));
  await usage(['#drawer']);
  await close();

  // Autopilot with control enabled and Auto engaged.
  await click('.autopilot-summary');
  await click('[data-action="enableControl"]');
  if (await page.$('[data-action="confirmControl"]')) await click('[data-action="confirmControl"]');
  await click('[data-ap-mode="Auto"]');
  await page.waitForTimeout(3500);
  const autopilot = await page.evaluate(() => __capture(document.querySelector('#drawer')));
  await usage(['#drawer']);
  await close();
  await close();

  // Energy and Instruments full views.
  const views = {};
  for (const view of ['energy', 'instruments']) {
    await click(`.sidebar [data-view="${view}"]`);
    views[view] = await page.evaluate(() => ({ html: __capture(document.querySelector('#fullView')), cls: document.querySelector('#app').className }));
    await usage(['#fullView']);
    await close();
  }

  // Serialize the used rules.
  const css = await page.evaluate(() => {
    const out = [];
    const keyframes = new Map();
    const visit = (rules, prefix, wrap) => [...rules].forEach((rule, i) => {
      const key = prefix + i;
      if (rule.type === CSSRule.KEYFRAMES_RULE) { keyframes.set(rule.name, rule.cssText); return; }
      if (rule.type === CSSRule.MEDIA_RULE) {
        // The screen is always laid out at 1280 x 800: viewport-size queries do not apply.
        if (/width|height|orientation|pointer|hover/.test(rule.conditionText)) return;
        visit(rule.cssRules, key + '/', rule.conditionText);
        return;
      }
      if (!rule.selectorText) return;
      const sels = __split(rule.selectorText).filter((_, j) => __used.has(key + ':' + j));
      const text = rule.cssText;
      if (sels.length) out.push({ sels, body: text.slice(text.indexOf('{') + 1, text.lastIndexOf('}')).trim(), wrap });
    });
    [...document.styleSheets].forEach((sheet, s) => visit(sheet.cssRules, s + '/'));
    return { rules: out, keyframes: [...keyframes] };
  });
  await browser.close();

  /* ---------- post-process ---------- */
  const fragments = { screen, ais, autopilot, layers, energy: views.energy.html, instruments: views.instruments.html };
  const ids = new Set();
  Object.values(fragments).forEach((html) => { for (const m of html.matchAll(/\sid="([^"]+)"/g)) ids.add(m[1]); });
  ids.add('app');
  const renameIds = (text) => text
    .replace(/\sid="([^"]+)"/g, (_, id) => ` id="ps-${id}"`)
    .replace(/url\(#([\w-]+)\)/g, (m, id) => (ids.has(id) ? `url(#ps-${id})` : m))
    .replace(/(href|aria-labelledby|aria-describedby|aria-controls|for)="#?([\w-]+)"/g, (m, attr, id) => (ids.has(id) ? `${attr}="${attr === 'href' ? '#' : ''}ps-${id}"` : m));

  const inlineStyles = new Map();
  const foldStyles = (html) => html.replace(/\sstyle="([^"]*)"/g, (_, decl) => {
    if (!inlineStyles.has(decl)) inlineStyles.set(decl, `ps-s${inlineStyles.size}`);
    return ` data-ps-style="${inlineStyles.get(decl)}"`;
  }).replace(/(<[^>]*?)\sclass="([^"]*)"([^>]*?)\sdata-ps-style="([^"]+)"/g, '$1 class="$2 $4"$3')
    .replace(/(<[^>]*?)\sdata-ps-style="([^"]+)"/g, '$1 class="$2"');

  const brand = (html) => html
    .replace(/<button class="brand"[\s\S]*?<\/button>/, '<span class="brand skager-brand"><span class="skager-mark" role="img" aria-label="SKAGER App"></span></span>')
    .replace(/<span class="prototype-tag">[\s\S]*?<\/span>/g, '')
    .replace(/<span class="map-disclaimer">[\s\S]*?<\/span>/g, '')
    .replace(/AIS target · mock broadcast/gi, 'AIS target')
    // Prototype-only design notes and review controls are not product UI.
    .replace(/<p class="note">[\s\S]*?<\/p>/g, '')
    .replace(/<details class="sensor-details">[\s\S]*?<\/details>/g, '')
    .replace(/Raster concept/g, 'Raster')
    .replace(/Heading · mock/g, 'Heading')
    .replace(/Good · calibrated mock model/g, 'Good')
    .replace(/aria-label="Illustrative /g, 'aria-label="')
    .replace(/;?\s*this preview does not determine a collision-avoidance manoeuvre\.?/gi, '.')
    .replace(/\s(data-action|data-panel|data-view|data-target|data-select-target|data-map-waypoint|data-chart-object|data-chart-symbol|data-set-theme|data-orientation|title|tabindex)="[^"]*"/g, '');

  // CSS that targets an id targets a class instead, so fragments can repeat on one page.
  const styledIds = new Set();
  css.rules.forEach(({ sels }) => sels.forEach((sel) => { for (const m of sel.matchAll(/#([\w-]+)/g)) if (ids.has(m[1])) styledIds.add(m[1]); }));
  const idClasses = (html) => html.replace(/<([a-zA-Z]+)([^>]*?)\sid="ps-([\w-]+)"([^>]*)>/g, (m, tag, before, id, after) => {
    if (!styledIds.has(id)) return m;
    const attrs = before + after;
    if (/\sclass="/.test(attrs)) return `<${tag}${before} id="ps-${id}"${after}>`.replace(/\sclass="([^"]*)"/, ` class="$1 ps-id-${id}"`);
    return `<${tag}${before} id="ps-${id}" class="ps-id-${id}"${after}>`;
  });
  const stripIds = (html) => html.replace(/\s(id|aria-labelledby|aria-describedby|aria-controls)="[^"]*"/g, '');
  const finish = (html, keepIds) => { const out = idClasses(brand(foldStyles(renameIds(html)))); return keepIds ? out : stripIds(out); };
  const header = `{# Generated by web/tools/prototype_screen/extract.cjs from docs/design/prototype/index.html (sha256 ${sha.slice(0, 16)}). Do not edit by hand. #}\n`;
  fs.mkdirSync(INCLUDES, { recursive: true });
  const write = (name, html, keepIds = false) => fs.writeFileSync(path.join(INCLUDES, name), header + finish(html, keepIds) + '\n');
  write('skager_screen.html', fragments.screen, true);
  write('skager_ais.html', fragments.ais);
  write('skager_autopilot.html', fragments.autopilot);
  write('skager_layers.html', fragments.layers);
  write('skager_energy.html', fragments.energy);
  write('skager_instruments.html', fragments.instruments);
  fs.writeFileSync(path.join(INCLUDES, 'skager_theme_icons.html'), header +
    Object.entries(themeIcons).map(([t, svg]) => `<template data-theme-icon="${t}">${svg}</template>`).join('\n') + '\n');

  const usedFrames = new Set();
  const scopeSelector = (sel) => {
    let s = sel.replace(/#app\b/g, '.proto-screen').replace(/^(:root|html|body)(?=[\s.[:>]|$)/, '.proto-screen');
    s = s.replace(/#([\w-]+)/g, (m, id) => (ids.has(id) ? `.ps-id-${id}` : m));
    if (!s.startsWith('.proto-screen')) s = `.proto-screen ${s}`;
    return s;
  };
  const scopeBody = (body) => body
    .replace(/url\(#([\w-]+)\)/g, (m, id) => (ids.has(id) ? `url(#ps-${id})` : m))
    .replace(/(font-family:[^;]*?)Arial/g, '$1Inter, Arial')
    .replace(/(animation(?:-name)?:\s*)([^;]+)/g, (m, prop, value) => prop + value.replace(/\b([a-zA-Z][\w-]*)\b/g, (w) => {
      if (fragmentsFrames.has(w)) { usedFrames.add(w); return `ps-${w}`; }
      return w;
    }));
  const fragmentsFrames = new Set(css.keyframes.map(([name]) => name));
  // Drop custom properties nothing references (the prototype's symbol-atlas sprite is ~780 KB).
  const referenced = new Set();
  const allText = css.rules.map((r) => r.body).join(';') + [...inlineStyles.keys()].join(';');
  for (const m of allText.matchAll(/var\((--[\w-]+)/g)) referenced.add(m[1]);
  css.rules.forEach((r) => {
    r.body = r.body.split(/;(?![^(]*\))/).filter((decl) => {
      const name = decl.trim().split(':')[0];
      return !name.startsWith('--') || referenced.has(name);
    }).join(';');
  });
  const lines = css.rules.filter(({ body }) => body.trim()).map(({ sels, body, wrap }) => {
    const rule = `${sels.map(scopeSelector).join(',')}{${scopeBody(body)}}`;
    return wrap ? `@media ${wrap}{${rule}}` : rule;
  });
  inlineStyles.forEach((cls, decl) => lines.push(`.proto-screen .${cls}{${scopeBody(decl)}}`));
  css.keyframes.forEach(([name, text]) => { if (usedFrames.has(name)) lines.push(text.replace(`@keyframes ${name}`, `@keyframes ps-${name}`)); });
  fs.writeFileSync(CSS_OUT, `/* Generated by web/tools/prototype_screen/extract.cjs from the v8 prototype (sha256 ${sha.slice(0, 16)}). Do not edit by hand. */\n${lines.join('\n')}\n`);
  const sizes = Object.fromEntries(fs.readdirSync(INCLUDES).map((f) => [f, fs.statSync(path.join(INCLUDES, f)).size]));
  console.log(JSON.stringify({ prototype: sha, rules: lines.length, css: fs.statSync(CSS_OUT).size, includes: sizes, energyClass: views.energy.cls, instrumentsClass: views.instruments.cls }, null, 2));
}

main().catch((error) => { console.error(error); process.exit(1); });
