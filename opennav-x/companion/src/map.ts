// The map: an open base map (OpenFreeMap vector tiles, OpenMapTiles schema)
// coloured with the SKAGER chart palette, plus what the boat reported: last
// position, track and the anchor watch circle. Not a chart and not for navigation.
import * as maplibregl from 'maplibre-gl';
import type { Anchor, TrackPoint } from './lib/api.ts';
import { MAP_GLYPHS, MAP_TILES } from './lib/config.ts';
import { bounds, circle, trackSegments } from './lib/geo.ts';
import { PALETTES } from './theme.ts';
import type { ThemeName } from './theme.ts';

const ATTRIBUTION = '<a href="https://openfreemap.org" target="_blank" rel="noopener">OpenFreeMap</a> '
  + '<a href="https://www.openmaptiles.org/" target="_blank" rel="noopener">© OpenMapTiles</a> '
  + 'Data from <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap</a>';

function style(theme: ThemeName): maplibregl.StyleSpecification {
  const p = PALETTES[theme];
  const text = p.chartText;
  return {
    version: 8,
    glyphs: MAP_GLYPHS,
    sources: { base: { type: 'vector', url: MAP_TILES, attribution: ATTRIBUTION } },
    layers: [
      { id: 'land', type: 'background', paint: { 'background-color': p.land } },
      { id: 'landcover', type: 'fill', source: 'base', 'source-layer': 'landcover', paint: { 'fill-color': p.shore, 'fill-opacity': 0.35 } },
      { id: 'landuse', type: 'fill', source: 'base', 'source-layer': 'landuse', paint: { 'fill-color': p.shore, 'fill-opacity': 0.18 } },
      { id: 'water', type: 'fill', source: 'base', 'source-layer': 'water', paint: { 'fill-color': p.water } },
      { id: 'waterway', type: 'line', source: 'base', 'source-layer': 'waterway', paint: { 'line-color': p.water, 'line-width': 1.2 } },
      { id: 'shoreline', type: 'line', source: 'base', 'source-layer': 'water', minzoom: 9,
        paint: { 'line-color': p.contour, 'line-width': 0.8 } },
      { id: 'roads', type: 'line', source: 'base', 'source-layer': 'transportation', minzoom: 10,
        filter: ['in', ['get', 'class'], ['literal', ['motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'minor']]],
        paint: { 'line-color': p.shore, 'line-width': ['interpolate', ['linear'], ['zoom'], 10, 0.4, 16, 2.2] } },
      { id: 'ferries', type: 'line', source: 'base', 'source-layer': 'transportation', filter: ['==', ['get', 'class'], 'ferry'],
        paint: { 'line-color': p.contour, 'line-width': 1, 'line-dasharray': [3, 3] } },
      { id: 'buildings', type: 'fill', source: 'base', 'source-layer': 'building', minzoom: 14, paint: { 'fill-color': p.shore, 'fill-opacity': 0.5 } },
      { id: 'boundaries', type: 'line', source: 'base', 'source-layer': 'boundary', filter: ['<=', ['get', 'admin_level'], 2],
        paint: { 'line-color': text, 'line-width': 0.8, 'line-dasharray': [4, 3] } },
      { id: 'water-names', type: 'symbol', source: 'base', 'source-layer': 'water_name',
        layout: { 'text-field': ['coalesce', ['get', 'name:latin'], ['get', 'name']], 'text-font': ['Noto Sans Italic'], 'text-size': 12 },
        paint: { 'text-color': text, 'text-halo-color': p.water, 'text-halo-width': 1.2 } },
      { id: 'places', type: 'symbol', source: 'base', 'source-layer': 'place',
        filter: ['in', ['get', 'class'], ['literal', ['city', 'town', 'village', 'hamlet', 'island', 'islet']]],
        layout: { 'text-field': ['coalesce', ['get', 'name:latin'], ['get', 'name']], 'text-font': ['Noto Sans Regular'],
          'text-size': ['match', ['get', 'class'], 'city', 15, 'town', 13, 12] },
        paint: { 'text-color': text, 'text-halo-color': p.land, 'text-halo-width': 1.2 } },
    ],
  };
}

type Overlay = { position: [number, number] | null; current: boolean; track: TrackPoint[]; anchor: Anchor | null; alarm: boolean;
  anchorStale: boolean };

export class BoatMap {
  readonly element = document.createElement('div');
  private map: maplibregl.Map | null = null;
  private ready = false;
  private theme: ThemeName = 'day';
  private overlay: Overlay = { position: null, current: false, track: [], anchor: null, alarm: false, anchorStale: false };
  private fittedKey = '';
  private failed = false;

  constructor() { this.element.className = 'map'; }

  /** Moves the single map into the screen's slot; MapLibre keeps its state across tabs. */
  attach(slot: Element | null, theme: ThemeName): void {
    if (!slot) return;
    if (this.element.parentElement !== slot) slot.appendChild(this.element);
    if (!this.map && !this.failed) this.create(theme);
    else if (theme !== this.theme) { this.theme = theme; this.ready = false; this.map?.setStyle(style(theme)); }
    requestAnimationFrame(() => this.map?.resize());
  }

  private create(theme: ThemeName): void {
    this.theme = theme;
    try {
      maplibregl.setWorkerUrl(new URL('vendor/maplibre-gl-worker.mjs', location.href).href);
      this.map = new maplibregl.Map({ container: this.element, style: style(theme), center: [18.07, 59.33], zoom: 9,
        attributionControl: { compact: true }, pitchWithRotate: false, dragRotate: false, maxPitch: 0, fadeDuration: 0 });
      this.map.touchZoomRotate.disableRotation();
      this.map.addControl(new maplibregl.NavigationControl({ showCompass: false }), 'bottom-right');
      this.map.on('style.load', () => { this.ready = true; this.draw(); });
    } catch {
      // No WebGL: the rest of the app works; the slot explains it.
      this.failed = true;
      this.element.innerHTML = '<p class="note map-fallback">The map cannot be shown on this device.</p>';
    }
  }

  update(overlay: Overlay, fitKey: string): void {
    this.overlay = overlay;
    this.draw();
    if (fitKey !== this.fittedKey && this.ready) this.fit(fitKey);
  }

  private fit(key: string): void {
    const o = this.overlay;
    const points: [number, number][] = o.track.map(([, lat, lon]) => [lon, lat]);
    if (o.position) points.push(o.position);
    if (o.anchor?.lat !== undefined && o.anchor.lon !== undefined) {
      points.push(...circle(o.anchor.lat, o.anchor.lon, Math.abs(o.anchor.radius_m ?? 30), 8));
    }
    const box = bounds(points);
    if (!box || !this.map) return;
    this.fittedKey = key;
    this.map.fitBounds(box, { padding: 48, maxZoom: 16, duration: 0 });
  }

  private draw(): void {
    const map = this.map;
    if (!map || !this.ready) return;
    const p = PALETTES[this.theme];
    const o = this.overlay;
    const source = (id: string, data: Parameters<maplibregl.GeoJSONSource['setData']>[0]) => {
      const existing = map.getSource(id) as maplibregl.GeoJSONSource | undefined;
      if (existing) existing.setData(data); else map.addSource(id, { type: 'geojson', data });
    };
    const layer = (spec: maplibregl.AddLayerObject) => { if (!map.getLayer(spec.id)) map.addLayer(spec); };
    source('track', { type: 'FeatureCollection', features: trackSegments(o.track).filter(s => s.length > 1)
      .map(coordinates => ({ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates } })) });
    const anchor = o.anchor?.armed && o.anchor.lat !== undefined && o.anchor.lon !== undefined ? o.anchor : null;
    source('anchor', { type: 'FeatureCollection', features: anchor ? [
      ...(anchor.radius_m ? [{ type: 'Feature' as const, properties: {}, geometry: { type: 'Polygon' as const,
        coordinates: [circle(anchor.lat!, anchor.lon!, anchor.radius_m)] } }] : []),
      { type: 'Feature', properties: { point: true }, geometry: { type: 'Point', coordinates: [anchor.lon!, anchor.lat!] } }] : [] });
    source('boat', { type: 'FeatureCollection', features: o.position
      ? [{ type: 'Feature', properties: {}, geometry: { type: 'Point', coordinates: o.position } }] : [] });
    const ring = o.anchorStale ? p.muted : o.alarm ? p.critical : p.advisory;
    layer({ id: 'track-line', type: 'line', source: 'track', layout: { 'line-cap': 'round', 'line-join': 'round' },
      paint: { 'line-color': p.route, 'line-width': 2.4, 'line-opacity': 0.9 } });
    layer({ id: 'anchor-fill', type: 'fill', source: 'anchor', filter: ['==', ['geometry-type'], 'Polygon'],
      paint: { 'fill-color': ring, 'fill-opacity': 0.12 } });
    layer({ id: 'anchor-casing', type: 'line', source: 'anchor', filter: ['==', ['geometry-type'], 'Polygon'],
      paint: { 'line-color': p.chartText, 'line-width': 4, 'line-opacity': 0.35 } });
    layer({ id: 'anchor-ring', type: 'line', source: 'anchor', filter: ['==', ['geometry-type'], 'Polygon'],
      paint: { 'line-color': ring, 'line-width': 2.5, 'line-dasharray': [2, 1.5] } });
    layer({ id: 'anchor-point', type: 'circle', source: 'anchor', filter: ['==', ['geometry-type'], 'Point'],
      paint: { 'circle-radius': 5, 'circle-color': ring, 'circle-stroke-color': p.chartSurface, 'circle-stroke-width': 2 } });
    layer({ id: 'boat-halo', type: 'circle', source: 'boat', paint: { 'circle-radius': 14, 'circle-color': p.active, 'circle-opacity': 0.25 } });
    layer({ id: 'boat-dot', type: 'circle', source: 'boat', paint: { 'circle-radius': 7, 'circle-color': p.active,
      'circle-stroke-color': p.background, 'circle-stroke-width': 2.5 } });
    // A last-known position is drawn without the "current" halo, dimmed.
    map.setPaintProperty('boat-halo', 'circle-opacity', o.current ? 0.25 : 0);
    map.setPaintProperty('boat-dot', 'circle-color', o.current ? p.active : p.muted);
    for (const id of ['anchor-fill', 'anchor-ring', 'anchor-point']) {
      map.setPaintProperty(id, id === 'anchor-fill' ? 'fill-color' : id === 'anchor-ring' ? 'line-color' : 'circle-color', ring);
    }
  }
}
