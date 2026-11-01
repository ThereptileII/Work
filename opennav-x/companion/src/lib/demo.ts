// The demo: SKAGER Companion without an account (app review, and anyone curious).
// Two simulated boats answer the same calls as SKAGER Cloud (lib/api.ts): one
// sailing off Marstrand, one at anchor. Values follow smooth functions of time, so
// the dashboard, map, history and alarms all agree. Nothing leaves the phone.
import type { Alarm, AlarmInput, AlarmKey, Boat, BoatEvent, Boats, Device, Item, Series, TelemetryKey, TrackPoint } from './api.ts';
import { ServiceError } from './api.ts';
import { distanceM } from './geo.ts';

export const DEMO_EMAIL = 'demo@skager.app';
const SAILING = '00000000-0000-4000-8000-00000000d001';
const ANCHORED = '00000000-0000-4000-8000-00000000d002';

/** The boat data SKAGER Cloud describes (web/edge/cloud.ts TELEMETRY_KEYS), for the keys the demo boats report. */
export const DEMO_KEYS: Record<string, TelemetryKey> = {
  'navigation.latitude': { label: 'Latitude', unit: 'deg', min: -90, max: 90 },
  'navigation.longitude': { label: 'Longitude', unit: 'deg', min: -180, max: 180 },
  'navigation.sog': { label: 'Speed over ground', unit: 'kn', min: 0, max: 100 },
  'navigation.cog': { label: 'Course over ground', unit: 'deg', min: 0, max: 360 },
  'navigation.heading': { label: 'Heading', unit: 'deg', min: 0, max: 360 },
  'navigation.stw': { label: 'Speed through water', unit: 'kn', min: 0, max: 100 },
  'wind.apparent_speed': { label: 'Apparent wind speed', unit: 'kn', min: 0, max: 200 },
  'wind.apparent_angle': { label: 'Apparent wind angle', unit: 'deg', min: -180, max: 360 },
  'wind.true_speed': { label: 'True wind speed', unit: 'kn', min: 0, max: 200 },
  'wind.true_angle': { label: 'True wind angle', unit: 'deg', min: -180, max: 360 },
  'environment.depth': { label: 'Depth below transducer', unit: 'm', min: 0, max: 12000 },
  'environment.water_temperature': { label: 'Water temperature', unit: 'C', min: -5, max: 50 },
  'environment.pressure': { label: 'Air pressure', unit: 'hPa', min: 800, max: 1100 },
  'steering.rudder': { label: 'Rudder', unit: 'deg', min: -90, max: 90 },
  'steering.heel': { label: 'Heel', unit: 'deg', min: -90, max: 90 },
  'propulsion.electrical_power': { label: 'Motor power', unit: 'kW', min: -1000, max: 1000 },
  'propulsion.rpm': { label: 'Motor speed', unit: 'rpm', min: -20000, max: 20000 },
  'propulsion.motor_temperature': { label: 'Motor temperature', unit: 'C', min: -50, max: 250 },
  'battery.voltage': { label: 'Battery voltage', unit: 'V', min: 0, max: 1500 },
  'battery.current': { label: 'Battery current', unit: 'A', min: -5000, max: 5000 },
  'battery.soc': { label: 'State of charge', unit: '%', min: 0, max: 100 },
  'battery.soh': { label: 'State of health', unit: '%', min: 0, max: 100 },
  'battery.net_discharge': { label: 'Net discharge', unit: 'kW', min: -1000, max: 1000 },
  'tanks.fresh_water': { label: 'Fresh water', unit: '%', min: 0, max: 100 },
  'tanks.waste': { label: 'Waste', unit: '%', min: 0, max: 100 },
};
/** What an alarm can watch (web/edge/companion.ts ALARM_KEYS), for the demo boats' data. */
const ALARM_UNITS: Record<string, string> = {
  'navigation.sog': 'kn', 'navigation.stw': 'kn', 'wind.true_speed': 'kn', 'wind.apparent_speed': 'kn', 'environment.depth': 'm',
  'environment.water_temperature': '°C', 'environment.pressure': 'hPa', 'steering.heel': '°', 'propulsion.electrical_power': 'kW',
  'propulsion.rpm': 'rpm', 'propulsion.motor_temperature': '°C', 'battery.voltage': 'V', 'battery.current': 'A', 'battery.soc': '%',
  'battery.net_discharge': 'kW', 'tanks.fresh_water': '%', 'tanks.waste': '%',
};
const ALARM_KEYS: Record<string, AlarmKey> = Object.fromEntries(Object.entries(ALARM_UNITS).map(([key, unit]) =>
  [key, { label: DEMO_KEYS[key].label, unit, min: DEMO_KEYS[key].min, max: DEMO_KEYS[key].max }]));

// ---------------------------------------------------------------- the simulation (t in epoch seconds)
const TAU = Math.PI * 2;
const wave = (t: number, period: number, phase = 0) => Math.sin(TAU * t / period + phase);
const norm360 = (d: number) => ((d % 360) + 360) % 360;
const signed = (d: number) => { const a = norm360(d); return a > 180 ? a - 360 : a; };
const M_PER_DEG = 111_320;

/** Wind over both boats: a sea breeze from the south-west, building in the afternoon. */
function wind(t: number): { from: number; speed: number } {
  const hour = (t / 3600 + 1) % 24;  // Swedish local time, near enough
  return { from: norm360(225 + 12 * wave(t, 5400) + 5 * wave(t, 900, 1)),
    speed: Math.max(3, 9 + 5 * Math.sin(TAU * (hour - 9) / 24) + 1.5 * wave(t, 1200) + 0.8 * wave(t, 180, 2)) };
}

/** A repeatable 0..1 number per day, so no two days of the demo look alike. */
const daily = (day: number, salt: number) => { const x = Math.sin(day * 12.9898 + salt * 78.233) * 43758.5453; return x - Math.floor(x); };
const ease = (x: number) => { const c = Math.min(1, Math.max(0, x)); return c * c * (3 - 2 * c); };

/** Battery that is used by day (more on some days than others) and charges overnight on shore power. */
function battery(t: number, use: number, load: number): Record<string, number> {
  const local = t / 3600 + 1;  // Swedish local time, near enough
  const day = Math.floor(local / 24), hour = local - day * 24;
  const top = (d: number) => 93 + 5 * daily(d, 1);
  const low = (d: number) => top(d) - use * (0.45 + 0.75 * daily(d, 2));
  const soc = hour < 7 ? low(day - 1) + (top(day) - low(day - 1)) * ease(hour / 7)
    : hour < 21 ? top(day) - (top(day) - low(day)) * ease((hour - 7) / 14) : low(day);
  const net = hour < 7 ? -(1.4 + 0.3 * wave(t, 1800)) : hour < 21 ? load : 0.12;
  const voltage = 48 + 4.6 * soc / 100 - net * 0.18;
  return { 'battery.soc': soc, 'battery.net_discharge': net, 'battery.voltage': voltage,
    'battery.current': -net * 1000 / voltage, 'battery.soh': 97 };
}

/** The sailing boat: a loop south-west of Marstrand, two nautical miles across, at about 5 knots. */
const LOOP = { lat: 57.858, lon: 11.47, radius: 1800, period: 4400 };
function sailing(t: number): Record<string, number> {
  const a = TAU * (t % LOOP.period) / LOOP.period;
  const lat = LOOP.lat + LOOP.radius * Math.cos(a) / M_PER_DEG;
  const lon = LOOP.lon + LOOP.radius * Math.sin(a) / (M_PER_DEG * Math.cos(LOOP.lat * Math.PI / 180));
  const cog = norm360(90 + a * 180 / Math.PI);  // clockwise around the loop
  const sog = TAU * LOOP.radius / LOOP.period / 0.5144 + 0.4 * wave(t, 600);
  const heading = norm360(cog - 3 + 2 * wave(t, 300));
  const w = wind(t);
  const twa = signed(w.from - heading);
  // Apparent wind: true wind plus the headwind from the boat's own speed.
  const tx = w.speed * Math.cos(twa * Math.PI / 180) + sog, ty = w.speed * Math.sin(twa * Math.PI / 180);
  const motor = 1.6 + 0.5 * wave(t, 900, 0.5);
  return {
    'navigation.latitude': lat, 'navigation.longitude': lon, 'navigation.sog': sog, 'navigation.cog': cog,
    'navigation.heading': heading, 'navigation.stw': sog - 0.2 + 0.1 * wave(t, 1300),
    'wind.true_speed': w.speed, 'wind.true_angle': twa,
    'wind.apparent_speed': Math.hypot(tx, ty), 'wind.apparent_angle': Math.atan2(ty, tx) * 180 / Math.PI,
    'environment.depth': 24 + 9 * wave(t, LOOP.period, 1) + 2 * wave(t, 500), 'environment.water_temperature': 13.6 + 0.6 * wave(t, 86400),
    'environment.pressure': 1014 + 6 * wave(t, 3.5 * 86400) + 0.4 * wave(t, 10800),
    'steering.rudder': 4 + 3 * wave(t, 40), 'steering.heel': -Math.sign(twa) * Math.min(22, w.speed * 0.9) * Math.abs(Math.sin(twa * Math.PI / 180)),
    'propulsion.electrical_power': motor, 'propulsion.rpm': 520 + 180 * motor, 'propulsion.motor_temperature': 36 + 4 * motor,
    ...battery(t, 60, motor + 0.35),
    'tanks.fresh_water': 70 - 25 * ((t / 86400) % 4) / 4, 'tanks.waste': 12 + 20 * ((t / 86400) % 4) / 4,
  };
}

/** The anchored boat: off Koön's north shore in Marstrandsfjorden, swinging to the wind on 30 m of chain. */
const ANCHOR = { lat: 57.9028, lon: 11.5945, radius: 45 };
function anchored(t: number): Record<string, number> {
  const w = wind(t);
  // Downwind of the anchor, swinging a little either side.
  const bearing = norm360(w.from + 180 + 25 * wave(t, 840));
  const dist = 28 + 5 * wave(t, 600, 1);
  const lat = ANCHOR.lat + dist * Math.cos(bearing * Math.PI / 180) / M_PER_DEG;
  const lon = ANCHOR.lon + dist * Math.sin(bearing * Math.PI / 180) / (M_PER_DEG * Math.cos(ANCHOR.lat * Math.PI / 180));
  const heading = norm360(w.from + 10 * wave(t, 840));
  const awa = signed(w.from - heading);
  return {
    'navigation.latitude': lat, 'navigation.longitude': lon, 'navigation.sog': 0.1 + 0.08 * Math.abs(wave(t, 420)),
    'navigation.heading': heading, 'wind.apparent_speed': w.speed, 'wind.apparent_angle': awa,
    'wind.true_speed': w.speed, 'wind.true_angle': awa,
    'environment.depth': 7.2 + 0.3 * wave(t, 12.4 * 3600), 'environment.water_temperature': 14.1 + 0.5 * wave(t, 86400),
    'environment.pressure': 1014 + 6 * wave(t, 3.5 * 86400) + 0.4 * wave(t, 10800),
    ...battery(t, 25, 0.32 + 0.1 * wave(t, 3600)),
    'tanks.fresh_water': 54, 'tanks.waste': 31,
  };
}

const MODEL: Record<string, (t: number) => Record<string, number>> = { [SAILING]: sailing, [ANCHORED]: anchored };

// ---------------------------------------------------------------- the demo service
const iso = (t: number) => new Date(t * 1000).toISOString();
const round = (v: number) => Math.round(v * 1e6) / 1e6;

/** Answers Companion's calls the way SKAGER Cloud does, from the simulation; changes stay in memory. */
export class DemoApi {
  private readonly now: () => number;
  private readonly alarmList: Alarm[];
  private nextAlarm = 3;

  constructor(now: () => number = () => Date.now() / 1000) {
    this.now = now;
    this.alarmList = [
      this.makeAlarm(1, SAILING, 'battery.soc', 'below', 30),
      this.makeAlarm(2, ANCHORED, 'wind.true_speed', 'above', 25),
    ];
  }

  private devices(): Device[] {
    const t = Math.floor(this.now());
    const at = iso(t - 4);
    return [
      { id: SAILING, name: 'Aurora', platform: 'windows', appVersion: '0.4.0-beta2.10', telemetryEnabled: true,
        createdAt: iso(t - 40 * 86400), lastSeenAt: at, lastDataAt: at, anchor: { armed: false }, lastEventId: 6 },
      { id: ANCHORED, name: 'Selma', platform: 'windows', appVersion: '0.4.0-beta2.10', telemetryEnabled: true,
        createdAt: iso(t - 90 * 86400), lastSeenAt: at, lastDataAt: at, anchor: this.anchor(ANCHORED, t), lastEventId: 5 },
    ];
  }

  private anchor(device: string, t: number) {
    if (device !== ANCHORED) return { armed: false };
    const v = anchored(t);
    return { armed: true, alarm: false, lat: ANCHOR.lat, lon: ANCHOR.lon, radius_m: ANCHOR.radius,
      distance_m: Math.round(distanceM(ANCHOR.lat, ANCHOR.lon, v['navigation.latitude'], v['navigation.longitude'])), age: 2, src: 'GPS' };
  }

  private makeAlarm(id: number, device: string, key: string, direction: 'above' | 'below', threshold: number): Alarm {
    const value = MODEL[device](this.now())[key] ?? null;
    const breached = value !== null && (direction === 'above' ? value > threshold : value < threshold);
    return { id, deviceId: device, key, label: DEMO_KEYS[key].label, unit: ALARM_UNITS[key], direction, threshold, enabled: true,
      state: breached ? 'alarm' : 'ok', value: value === null ? null : Math.round(value * 100) / 100, changedAt: iso(this.now() - 3 * 86400) };
  }

  private known(device: string): void {
    if (!MODEL[device]) throw new ServiceError('Boat not found.', 404);
  }

  async session() {
    return { account: { email: DEMO_EMAIL }, cloud: this.cloud(), push: false };
  }
  private cloud() {
    return { active: true, status: 'active', renewsAt: iso(this.now() + 200 * 86400), cancelAtPeriodEnd: false };
  }
  async boats(): Promise<Boats> {
    return { cloud: this.cloud(), devices: this.devices(), telemetryKeys: DEMO_KEYS };
  }
  async boat(device: string, _etag?: string): Promise<{ boat: Boat | null; etag: string | null }> {
    this.known(device);
    const t = Math.floor(this.now()) - 4;
    const values = MODEL[device](t);
    const items: Record<string, Item> = Object.fromEntries(Object.entries(values).map(([key, v]) =>
      [key, { v: round(v), age: key.startsWith('tanks.') ? 30 : 1, q: 'live', src: key.startsWith('navigation.l') ? 'GPS' : 'NMEA 2000' }]));
    return { boat: { device: this.devices().find(d => d.id === device)!, receivedAt: iso(t), items,
      anchor: device === ANCHORED ? this.anchor(device, t) : null, snapshotAge: 4 }, etag: null };
  }
  async series(device: string, keys: string[], hours: number): Promise<Series> {
    this.known(device);
    // The bucket sizes SKAGER Cloud uses: about 120 points per chart.
    const bucket = hours <= 1 ? 60 : hours <= 6 ? 300 : hours <= 24 ? 900 : hours <= 72 ? 1800 : hours <= 168 ? 3600 : 21600;
    const end = Math.floor(this.now() / bucket) * bucket;
    const series: Record<string, [number, number][]> = {};
    for (const key of keys) {
      const points: [number, number][] = [];
      for (let t = end - hours * 3600 + bucket; t <= end; t += bucket) {
        const v = MODEL[device](t + bucket / 2)[key];
        if (v !== undefined) points.push([t, Math.round(v * 1000) / 1000]);
      }
      series[key] = points;
    }
    return { bucketSeconds: bucket, series };
  }
  async track(device: string, hours: number): Promise<TrackPoint[]> {
    this.known(device);
    const step = device === ANCHORED ? 60 : Math.max(30, hours * 3600 / 600);
    const end = Math.floor(this.now()), points: TrackPoint[] = [];
    // The sailing boat's last loop reads clearly; older loops would overlap.
    const span = device === SAILING ? Math.min(hours * 3600, LOOP.period) : hours * 3600;
    for (let t = end - span; t <= end; t += step) {
      const v = MODEL[device](t);
      points.push([Math.floor(t), round(v['navigation.latitude']), round(v['navigation.longitude']), Math.round(v['navigation.sog'] * 10) / 10]);
    }
    return points;
  }
  async events(options: { device?: string; before?: number; limit?: number } = {}): Promise<BoatEvent[]> {
    const t = Math.floor(this.now());
    const all: BoatEvent[] = ([
      [6, SAILING, 'threshold_cleared', 'info', 'State of charge back to normal', 'Now 34 %.', 2.6 * 86400],
      [5, ANCHORED, 'anchor_armed', 'info', 'Anchor watch on', 'Radius 45 m.', 9 * 3600],
      [4, SAILING, 'threshold_alarm', 'critical', 'State of charge below 30 %', 'Now 29.6 %. Your alarm is set at 30.', 3 * 86400],
      [3, ANCHORED, 'anchor_alarm_cleared', 'info', 'Anchor alarm ended', 'The boat is back inside the radius.', 4 * 86400 - 600],
      [2, ANCHORED, 'anchor_alarm', 'critical', 'Anchor alarm', 'The boat is 52 m from the anchor, outside the 45 m radius.', 4 * 86400],
      [1, ANCHORED, 'connection_lost', 'critical', 'No contact with the boat', 'SKAGER on board has not reported for 10 minutes.', 6 * 86400],
    ] as const).map(([id, deviceId, kind, severity, title, detail, ago]) => ({ id, deviceId, boat: deviceId === SAILING ? 'Aurora' : 'Selma',
      kind, severity, title, detail, data: {}, occurredAt: iso(t - ago), receivedAt: iso(t - ago + 3) }));
    return all.filter(e => (!options.device || e.deviceId === options.device) && (!options.before || e.id < options.before))
      .slice(0, options.limit ?? 50);
  }
  async registerPush(): Promise<void> {}
  async alarms(device: string): Promise<{ alarms: Alarm[]; keys: Record<string, AlarmKey> }> {
    this.known(device);
    return { alarms: this.alarmList.filter(a => a.deviceId === device)
      .map(a => ({ ...this.makeAlarm(a.id, a.deviceId, a.key, a.direction, a.threshold), enabled: a.enabled })), keys: ALARM_KEYS };
  }
  async saveAlarm(input: AlarmInput): Promise<Alarm> {
    this.known(input.device);
    const spec = ALARM_KEYS[input.key];
    if (!spec || !Number.isFinite(input.threshold) || input.threshold < spec.min || input.threshold > spec.max) throw new ServiceError('Invalid alarm.', 400);
    const alarm = { ...this.makeAlarm(input.id ?? this.nextAlarm++, input.device, input.key, input.direction, input.threshold), enabled: input.enabled };
    const i = this.alarmList.findIndex(a => a.id === alarm.id);
    if (i >= 0) this.alarmList[i] = alarm; else this.alarmList.push(alarm);
    return alarm;
  }
  async deleteAlarm(id: number): Promise<void> {
    const i = this.alarmList.findIndex(a => a.id === id);
    if (i >= 0) this.alarmList.splice(i, 1);
  }
  async signOut(): Promise<void> {}
}
