// Display geometry only. The anchor alarm is OpenCPN's decision on board; the
// circle drawn here just shows the radius it reported.
const EARTH_M = 6_371_008.8;
const rad = (d: number) => d * Math.PI / 180, deg = (r: number) => r * 180 / Math.PI;

/** Closed ring of `steps` points at `radius` metres around a position (GeoJSON order: lon, lat). */
export function circle(lat: number, lon: number, radius: number, steps = 72): [number, number][] {
  const ring: [number, number][] = [];
  const d = Math.abs(radius) / EARTH_M, φ1 = rad(lat), λ1 = rad(lon);
  for (let i = 0; i <= steps; i++) {
    const θ = (i % steps) / steps * 2 * Math.PI;
    const φ2 = Math.asin(Math.sin(φ1) * Math.cos(d) + Math.cos(φ1) * Math.sin(d) * Math.cos(θ));
    const λ2 = λ1 + Math.atan2(Math.sin(θ) * Math.sin(d) * Math.cos(φ1), Math.cos(d) - Math.sin(φ1) * Math.sin(φ2));
    ring.push([((deg(λ2) + 540) % 360) - 180, deg(φ2)]);
  }
  return ring;
}

export function distanceM(lat1: number, lon1: number, lat2: number, lon2: number): number {
  const a = Math.sin(rad(lat2 - lat1) / 2) ** 2 + Math.cos(rad(lat1)) * Math.cos(rad(lat2)) * Math.sin(rad(lon2 - lon1) / 2) ** 2;
  return 2 * EARTH_M * Math.asin(Math.min(1, Math.sqrt(a)));
}

/** [[west, south], [east, north]] around points, padded so a lone point still gets a sensible view. */
export function bounds(points: [number, number][], padMetres = 150): [[number, number], [number, number]] | null {
  const valid = points.filter(([lon, lat]) => Number.isFinite(lon) && Number.isFinite(lat));
  if (!valid.length) return null;
  let [w, s, e, n] = [Infinity, Infinity, -Infinity, -Infinity];
  for (const [lon, lat] of valid) { w = Math.min(w, lon); e = Math.max(e, lon); s = Math.min(s, lat); n = Math.max(n, lat); }
  const dLat = deg(padMetres / EARTH_M), dLon = dLat / Math.max(0.05, Math.cos(rad((s + n) / 2)));
  return [[w - dLon, s - dLat], [e + dLon, n + dLat]];
}

/** Splits a track where the boat stopped reporting, so a gap is not drawn as a straight passage. */
export function trackSegments(points: [number, number, number, number | null][], gapSeconds = 600): [number, number][][] {
  const segments: [number, number][][] = [];
  let current: [number, number][] = [];
  let last: number | null = null;
  for (const [t, lat, lon] of points) {
    if (last !== null && t - last > gapSeconds && current.length) { segments.push(current); current = []; }
    current.push([lon, lat]);
    last = t;
  }
  if (current.length) segments.push(current);
  return segments;
}
