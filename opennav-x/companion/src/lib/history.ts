// History charts from SKAGER Cloud's one-minute samples (averaged into buckets by
// the server). Gaps stay gaps; nothing is interpolated across missing data.

export type Metric = { key: string; label: string; unit: string };
/** What the History tab offers, in order. Only metrics the boat has reported are shown. */
export const METRICS: Metric[] = [
  { key: 'battery.voltage', label: 'Battery voltage', unit: 'V' },
  { key: 'battery.soc', label: 'State of charge', unit: '%' },
  { key: 'battery.net_discharge', label: 'Battery power', unit: 'kW' },
  { key: 'battery.current', label: 'Battery current', unit: 'A' },
  { key: 'propulsion.electrical_power', label: 'Motor power', unit: 'kW' },
  { key: 'propulsion.motor_temperature', label: 'Motor temperature', unit: 'C' },
  { key: 'navigation.sog', label: 'Speed over ground', unit: 'kn' },
  { key: 'wind.true_speed', label: 'True wind', unit: 'kn' },
  { key: 'wind.apparent_speed', label: 'Apparent wind', unit: 'kn' },
  { key: 'environment.depth', label: 'Depth', unit: 'm' },
  { key: 'environment.water_temperature', label: 'Water temperature', unit: 'C' },
];
export const RANGES = [{ hours: 6, label: '6 h' }, { hours: 24, label: '24 h' }, { hours: 72, label: '3 d' },
  { hours: 168, label: '7 d' }, { hours: 720, label: '30 d' }];

export type Point = [number, number];
export type Stats = { min: number; max: number; mean: number; last: number; count: number };

export function clean(points: unknown): Point[] {
  return (Array.isArray(points) ? points : [])
    .filter((p): p is Point => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1]))
    .sort((a, b) => a[0] - b[0]);
}

export function stats(points: Point[]): Stats | null {
  if (!points.length) return null;
  const values = points.map(p => p[1]);
  return { min: Math.min(...values), max: Math.max(...values), mean: values.reduce((a, b) => a + b, 0) / values.length,
    last: values[values.length - 1], count: values.length };
}

/** Energy drawn from the battery (kWh) by integrating average battery power over the buckets
 * that have data. Charging (negative power) is reported separately. An estimate. */
export function energy(points: Point[], bucketSeconds: number): { used: number; charged: number; coveredHours: number } | null {
  if (!points.length || !(bucketSeconds > 0)) return null;
  let used = 0, charged = 0;
  for (const [, kw] of points) {
    const kwh = kw * bucketSeconds / 3600;
    if (kwh >= 0) used += kwh; else charged -= kwh;
  }
  return { used, charged, coveredHours: points.length * bucketSeconds / 3600 };
}

/** SVG path data with a break wherever buckets are missing (more than 1.5 buckets apart). */
export function chartPath(points: Point[], bucketSeconds: number, width: number, height: number,
  domain?: { t0: number; t1: number; y0: number; y1: number }): string {
  if (!points.length) return '';
  const t0 = domain?.t0 ?? points[0][0], t1 = domain?.t1 ?? points[points.length - 1][0];
  const ys = points.map(p => p[1]);
  const y0 = domain?.y0 ?? Math.min(...ys), y1 = domain?.y1 ?? Math.max(...ys);
  const x = (t: number) => (t - t0) / Math.max(1, t1 - t0) * width;
  const y = (v: number) => height - (v - y0) / Math.max(1e-9, y1 - y0) * height;
  let path = '', previous: number | null = null;
  for (const [t, v] of points) {
    const jump = previous === null || t - previous > bucketSeconds * 1.5;
    path += `${jump ? 'M' : 'L'}${x(t).toFixed(1)},${y(v).toFixed(1)}`;
    previous = t;
  }
  return path;
}

/** A readable y-axis: rounded bounds with a little headroom; flat data gets a band. */
export function niceDomain(points: Point[]): { y0: number; y1: number } {
  const ys = points.map(p => p[1]);
  let lo = Math.min(...ys), hi = Math.max(...ys);
  if (!Number.isFinite(lo)) return { y0: 0, y1: 1 };
  if (hi - lo < 1e-6) { lo -= Math.max(1, Math.abs(lo) * 0.05); hi += Math.max(1, Math.abs(hi) * 0.05); }
  const pad = (hi - lo) * 0.08;
  return { y0: lo - pad, y1: hi + pad };
}
