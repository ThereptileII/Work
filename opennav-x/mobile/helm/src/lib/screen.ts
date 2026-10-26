// Draws the SKAGER screen from the tile stream and maps touches back onto it.
import { parseStreamMessage } from './protocol.ts';
import type { Point } from './gestures.ts';

export type Rect = { x: number; y: number; w: number; h: number };

/** Fits the frame in the host (letterboxed), then applies the magnifier zoom and pan. */
export function fitRect(hostW: number, hostH: number, frameW: number, frameH: number, zoom = 1, panX = 0, panY = 0): Rect {
  if (frameW <= 0 || frameH <= 0 || hostW <= 0 || hostH <= 0) return { x: 0, y: 0, w: 0, h: 0 };
  const s = Math.min(hostW / frameW, hostH / frameH) * zoom;
  const w = frameW * s, h = frameH * s;
  return { x: (hostW - w) / 2 + panX, y: (hostH - h) / 2 + panY, w, h };
}

/** Keeps a magnified frame covering the host where it can (no empty margins revealed by panning). */
export function clampPan(hostW: number, hostH: number, frameW: number, frameH: number, zoom: number, panX: number, panY: number): [number, number] {
  const r = fitRect(hostW, hostH, frameW, frameH, zoom, 0, 0);
  const limX = Math.max(0, (r.w - hostW) / 2), limY = Math.max(0, (r.h - hostH) / 2);
  return [Math.min(limX, Math.max(-limX, panX)), Math.min(limY, Math.max(-limY, panY))];
}

/** Host px → normalized frame position. */
export function toFrame(r: Rect, p: Point, clamp = false): Point | null {
  if (r.w <= 0 || r.h <= 0) return null;
  let x = (p.x - r.x) / r.w, y = (p.y - r.y) / r.h;
  if (clamp) { x = Math.min(1, Math.max(0, x)); y = Math.min(1, Math.max(0, y)); }
  return x >= 0 && x <= 1 && y >= 0 && y <= 1 ? { x, y } : null;
}

type PendingTile = { x: number; y: number; bitmap: Promise<ImageBitmap | HTMLImageElement> };

async function decode(data: Uint8Array): Promise<ImageBitmap | HTMLImageElement> {
  const blob = new Blob([data as Uint8Array<ArrayBuffer>], { type: 'image/jpeg' });
  if (typeof createImageBitmap === 'function') return createImageBitmap(blob);
  const url = URL.createObjectURL(blob);
  try {
    const img = new Image();
    img.src = url;
    await img.decode();
    return img;
  } finally {
    URL.revokeObjectURL(url);
  }
}

export type FrameInfo = { frame: number; width: number; height: number; decodeMs: number };

export class ScreenView {
  host: HTMLElement;
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  frameW = 0;
  frameH = 0;
  zoom = 1;
  panX = 0;
  panY = 0;
  rect: Rect = { x: 0, y: 0, w: 0, h: 0 };
  chain: Promise<void> = Promise.resolve();
  pending: { frame: number; started: number; tiles: PendingTile[] } | null = null;
  drawn = 0;
  onFrame: (info: FrameInfo) => void;
  onBroken: (frame: number) => void;
  onResize: () => void = () => {};

  constructor(host: HTMLElement, canvas: HTMLCanvasElement, onFrame: (info: FrameInfo) => void, onBroken: (frame: number) => void) {
    this.host = host;
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { alpha: false })!;
    this.onFrame = onFrame;
    this.onBroken = onBroken;
    new ResizeObserver(() => this.layout()).observe(host);
  }

  get hasFrame(): boolean { return this.drawn > 0; }

  layout(): void {
    const hw = this.host.clientWidth, hh = this.host.clientHeight;
    [this.panX, this.panY] = clampPan(hw, hh, this.frameW, this.frameH, this.zoom, this.panX, this.panY);
    this.rect = fitRect(hw, hh, this.frameW, this.frameH, this.zoom, this.panX, this.panY);
    const s = this.canvas.style;
    s.width = `${this.rect.w}px`;
    s.height = `${this.rect.h}px`;
    s.transform = `translate(${this.rect.x}px, ${this.rect.y}px)`;
    this.onResize();
  }

  map = (p: Point, clamp = false): Point | null => {
    const box = this.host.getBoundingClientRect();
    return toFrame(this.rect, { x: p.x - box.left, y: p.y - box.top }, clamp);
  };

  /** Magnifier: scale by factor around a screen point and pan by (dx, dy). */
  magnify(factor: number, centre: Point, dx: number, dy: number): void {
    const box = this.host.getBoundingClientRect();
    const cx = centre.x - box.left - this.host.clientWidth / 2, cy = centre.y - box.top - this.host.clientHeight / 2;
    const zoom = Math.min(4, Math.max(1, this.zoom * factor));
    const k = zoom / this.zoom;
    this.panX = cx - (cx - this.panX) * k + dx;
    this.panY = cy - (cy - this.panY) * k + dy;
    this.zoom = zoom;
    this.layout();
  }

  resetView(): void { this.zoom = 1; this.panX = 0; this.panY = 0; this.layout(); }

  handle(data: ArrayBuffer): void {
    const msg = parseStreamMessage(data);
    if (!msg) return;
    if (msg.kind === 'begin') {
      this.pending = { frame: msg.frame, started: performance.now(), tiles: [] };
      if (msg.width !== this.frameW || msg.height !== this.frameH) {
        const [w, h] = [msg.width, msg.height];
        this.chain = this.chain.then(() => {
          this.frameW = w;
          this.frameH = h;
          this.canvas.width = w;
          this.canvas.height = h;
          this.layout();
        });
      }
    } else if (msg.kind === 'tile' && this.pending?.frame === msg.frame) {
      this.pending.tiles.push({ x: msg.x, y: msg.y, bitmap: decode(msg.data) });
    } else if (msg.kind === 'end' && this.pending?.frame === msg.frame) {
      const frame = this.pending;
      this.pending = null;
      this.chain = this.chain.then(async () => {
        try {
          const bitmaps = await Promise.all(frame.tiles.map(t => t.bitmap));
          frame.tiles.forEach((t, i) => {
            this.ctx.drawImage(bitmaps[i], t.x, t.y);
            const b = bitmaps[i];
            if ('close' in b) b.close();
          });
          this.drawn++;
          this.onFrame({ frame: frame.frame, width: this.frameW, height: this.frameH, decodeMs: performance.now() - frame.started });
        } catch {
          this.onBroken(frame.frame);
        }
      });
    }
  }

  clear(): void {
    this.pending = null;
    this.drawn = 0;
  }
}
