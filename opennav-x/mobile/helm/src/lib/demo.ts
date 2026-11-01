// The demo: SKAGER Helm without a boat (app review, and anyone curious before pairing).
// Screens captured from SKAGER with a simulated boat are streamed through the same
// screen protocol and drawing code as a real session; taps on SKAGER's side rail
// switch screens. Nothing leaves the phone.
import type { Boat, ControlMessage, InputEvent } from './client.ts';
import { MSG_BEGIN, MSG_END, MSG_TILE } from './protocol.ts';

export const DEMO_ID = 'demo';
export const isDemo = (boat: Boat | null | undefined) => boat?.serverId === DEMO_ID;
export const demoBoat = (): Boat =>
  ({ serverId: DEMO_ID, name: 'Demo boat', deviceId: DEMO_ID, addresses: [], pairedAt: new Date().toISOString() });

export const DEMO_WIDTH = 1600, DEMO_HEIGHT = 1000;
export type DemoScreen = 'chart' | 'passage' | 'instruments' | 'anchor' | 'settings';
type Area = { x: number; y: number; w: number; h: number; to: DemoScreen };
/** Where SKAGER's buttons are on the captured screens, in screen pixels. */
const AREAS: Area[] = [
  { x: 0, y: 128, w: 136, h: 84, to: 'chart' },
  { x: 0, y: 214, w: 136, h: 84, to: 'passage' },
  { x: 0, y: 488, w: 136, h: 84, to: 'instruments' },
  { x: 0, y: 574, w: 136, h: 84, to: 'anchor' },
  { x: 0, y: 828, w: 136, h: 84, to: 'settings' },
];
/** The Close button of an open SKAGER panel. */
const CLOSE: Omit<Area, 'to'> = { x: 1080, y: 180, w: 150, h: 64 };
const inside = (a: Omit<Area, 'to'>, x: number, y: number) => x >= a.x && x < a.x + a.w && y >= a.y && y < a.y + a.h;

/** The screen a tap at normalized (x, y) on `current` leads to. */
export function demoTarget(current: DemoScreen, x: number, y: number): DemoScreen {
  const px = x * DEMO_WIDTH, py = y * DEMO_HEIGHT;
  if (current !== 'chart' && inside(CLOSE, px, py)) return 'chart';
  return AREAS.find(a => inside(a, px, py))?.to ?? current;
}

/** One whole-screen frame in the stream format SKAGER sends (begin, one JPEG tile, end). */
export function demoFrame(frame: number, jpeg: Uint8Array): ArrayBuffer[] {
  const begin = new DataView(new ArrayBuffer(12));
  begin.setUint8(0, MSG_BEGIN); begin.setUint32(1, frame, true);
  begin.setUint16(5, DEMO_WIDTH, true); begin.setUint16(7, DEMO_HEIGHT, true);
  begin.setUint8(9, 1); begin.setUint16(10, 1, true);
  const tile = new Uint8Array(13 + jpeg.length);
  const head = new DataView(tile.buffer);
  head.setUint8(0, MSG_TILE); head.setUint32(1, frame, true);
  head.setUint16(5, 0, true); head.setUint16(7, 0, true);
  head.setUint16(9, DEMO_WIDTH, true); head.setUint16(11, DEMO_HEIGHT, true);
  tile.set(jpeg, 13);
  const end = new DataView(new ArrayBuffer(9));
  end.setUint8(0, MSG_END); end.setUint32(1, frame, true);
  return [begin.buffer, tile.buffer, end.buffer];
}

export type DemoHandlers = { screen(data: ArrayBuffer): void; control(msg: ControlMessage): void; closed(reason: string): void };

/** Stands in for Connection: the same members main.ts uses. */
export class DemoConnection {
  readonly address = 'demo';
  readonly by = '';
  rtt: number | null = 3;
  private open = true;
  private frame = 0;
  private screen: DemoScreen = 'chart';
  private readonly images = new Map<DemoScreen, Uint8Array>();
  private readonly handlers: DemoHandlers;
  private readonly load: (name: DemoScreen) => Promise<Uint8Array>;
  private stats: ReturnType<typeof setInterval> | undefined;

  constructor(handlers: DemoHandlers, load: (name: DemoScreen) => Promise<Uint8Array> = fetchScreen) {
    this.handlers = handlers;
    this.load = load;
  }

  get live(): boolean { return this.open; }

  async start(): Promise<void> {
    this.handlers.control({ type: 'hello', name: 'Demo boat', server_id: DEMO_ID, software: 'SKAGER (demo screens)', capabilities: [] });
    await this.show('chart');
    this.stats = setInterval(() => this.handlers.control({ type: 'stats',
      stats: { fps: 1, kbps: 0, capture_ms: 0, encode_ms: 0, width: DEMO_WIDTH, height: DEMO_HEIGHT } }), 2000);
  }

  async show(name: DemoScreen): Promise<void> {
    this.screen = name;
    let jpeg = this.images.get(name);
    if (!jpeg) { jpeg = await this.load(name); this.images.set(name, jpeg); }
    if (!this.open || this.screen !== name) return;
    for (const message of demoFrame(++this.frame, jpeg)) this.handlers.screen(message);
  }

  send(ev: InputEvent): boolean {
    if (!this.open) return false;
    if (ev.type === 'pointer_up' && ev.button !== 2) {
      const next = demoTarget(this.screen, ev.x, ev.y);
      if (next !== this.screen) void this.show(next);
    }
    return true;
  }

  sendControl(msg: { type: string; [k: string]: unknown }): void {
    if (msg.type === 'refresh') void this.show(this.screen);
  }

  close(): void {
    this.open = false;
    clearInterval(this.stats);
  }
}

async function fetchScreen(name: DemoScreen): Promise<Uint8Array> {
  const response = await fetch(`demo/${name}.jpg`);
  return new Uint8Array(await response.arrayBuffer());
}
