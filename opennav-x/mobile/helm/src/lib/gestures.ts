// Turns touches on the phone into input for SKAGER. Pure logic (no DOM), so it
// is unit-tested; the screen view feeds it pointer events and a clock.
//
//   tap                → click
//   drag               → press, move, release (pans the chart)
//   touch and hold     → secondary click (context menu)
//   pinch              → wheel at the pinch centre (zooms the chart)
//   two fingers, magnifier on → zoom and pan this phone's view of the screen
import type { InputEvent } from './client.ts';

export type Point = { x: number; y: number };
/** Screen px → normalized frame position; null outside the frame unless clamp is set. */
export type Mapper = (p: Point, clamp?: boolean) => Point | null;

export type ViewChange = { kind: 'view'; scale: number; dx: number; dy: number; centre: Point };

export type GestureOptions = { slop: number; longPressMs: number; pinchStep: number; moveIntervalMs: number };
export const DEFAULT_GESTURES: GestureOptions = { slop: 10, longPressMs: 550, pinchStep: 1.2, moveIntervalMs: 16 };

type Touch = { id: number; start: Point; at: Point; t0: number };

export class Gestures {
  opts: GestureOptions;
  map: Mapper;
  emit: (ev: InputEvent) => void;
  view: (v: ViewChange) => void;
  feedback: (kind: 'secondary') => void;
  magnifier = false;
  touches = new Map<number, Touch>();
  mode: 'idle' | 'pending' | 'drag' | 'done' | 'pinch' = 'idle';
  pressedAt: Point | null = null;
  lastMoveAt = 0;
  pendingMove: Point | null = null;
  pinchDist = 0;
  pinchRef = 0;
  pinchCentre: Point = { x: 0, y: 0 };
  timer: ReturnType<typeof setTimeout> | undefined;

  constructor(map: Mapper, emit: (ev: InputEvent) => void, view: (v: ViewChange) => void = () => {},
    feedback: (kind: 'secondary') => void = () => {}, opts: GestureOptions = DEFAULT_GESTURES) {
    this.map = map;
    this.emit = emit;
    this.view = view;
    this.feedback = feedback;
    this.opts = opts;
  }

  down(id: number, p: Point, now: number): void {
    this.touches.set(id, { id, start: p, at: p, t0: now });
    if (this.touches.size === 1) {
      this.mode = 'pending';
      clearTimeout(this.timer);
      this.timer = setTimeout(() => this.longPress(), this.opts.longPressMs);
      return;
    }
    if (this.touches.size === 2) {
      clearTimeout(this.timer);
      if (this.mode === 'drag') this.release();
      this.mode = 'pinch';
      const [a, b] = [...this.touches.values()];
      this.pinchDist = this.pinchRef = dist(a.at, b.at);
      this.pinchCentre = mid(a.at, b.at);
    }
  }

  move(id: number, p: Point, now: number): void {
    const t = this.touches.get(id);
    if (!t) return;
    t.at = p;
    if (this.mode === 'pending' && dist(t.start, p) > this.opts.slop) {
      clearTimeout(this.timer);
      const start = this.map(t.start);
      if (!start) { this.mode = 'done'; return; }
      this.mode = 'drag';
      this.press(start, 0);
      this.dragTo(p, now);
    } else if (this.mode === 'drag') {
      this.dragTo(p, now);
    } else if (this.mode === 'pinch' && this.touches.size >= 2) {
      const [a, b] = [...this.touches.values()];
      const d = dist(a.at, b.at), c = mid(a.at, b.at);
      if (this.magnifier) {
        this.view({ kind: 'view', scale: d / Math.max(1, this.pinchDist), dx: c.x - this.pinchCentre.x, dy: c.y - this.pinchCentre.y, centre: c });
        this.pinchDist = d;
        this.pinchCentre = c;
        return;
      }
      const at = this.map(c);
      while (at && d / this.pinchRef >= this.opts.pinchStep) {
        this.emit({ type: 'wheel', x: at.x, y: at.y, delta: 1 });
        this.pinchRef *= this.opts.pinchStep;
      }
      while (at && this.pinchRef / d >= this.opts.pinchStep) {
        this.emit({ type: 'wheel', x: at.x, y: at.y, delta: -1 });
        this.pinchRef /= this.opts.pinchStep;
      }
    }
  }

  up(id: number, p: Point, now: number): void {
    const t = this.touches.get(id);
    if (!t) return;
    t.at = p;
    this.touches.delete(id);
    if (this.mode === 'pending') {
      clearTimeout(this.timer);
      const at = this.map(t.start);
      if (at) {
        this.emit({ type: 'pointer_down', x: at.x, y: at.y, button: 0 });
        this.emit({ type: 'pointer_up', x: at.x, y: at.y, button: 0 });
      }
      this.mode = 'done';
    } else if (this.mode === 'drag') {
      this.dragTo(p, now, true);
      this.release();
      this.mode = 'done';
    }
    if (this.touches.size === 0) this.mode = 'idle';
  }

  /** The OS took the touches (or the connection dropped): never leave a button held. */
  cancel(): void {
    clearTimeout(this.timer);
    if (this.pressedAt) this.release();
    this.touches.clear();
    this.mode = 'idle';
  }

  longPress(): void {
    if (this.mode !== 'pending' || this.touches.size !== 1) return;
    const [t] = [...this.touches.values()];
    const at = this.map(t.start);
    this.mode = 'done';
    if (!at) return;
    this.feedback('secondary');
    this.emit({ type: 'pointer_down', x: at.x, y: at.y, button: 2 });
    this.emit({ type: 'pointer_up', x: at.x, y: at.y, button: 2 });
  }

  press(at: Point, button: 0 | 2): void {
    this.pressedAt = at;
    this.emit({ type: 'pointer_down', x: at.x, y: at.y, button });
  }

  dragTo(p: Point, now: number, flush = false): void {
    const at = this.map(p, true); // dragging past the edge keeps the press at the edge
    if (!at) return;
    this.pendingMove = at;
    if (!flush && now - this.lastMoveAt < this.opts.moveIntervalMs) return;
    this.lastMoveAt = now;
    this.emit({ type: 'pointer_move', x: at.x, y: at.y });
    this.pressedAt = at;
    this.pendingMove = null;
  }

  release(): void {
    const at = this.pendingMove ?? this.pressedAt;
    if (at) this.emit({ type: 'pointer_up', x: at.x, y: at.y, button: 0 });
    this.pressedAt = null;
    this.pendingMove = null;
  }
}

const dist = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y);
const mid = (a: Point, b: Point) => ({ x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 });
