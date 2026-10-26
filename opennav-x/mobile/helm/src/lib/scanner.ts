// Reads the pairing QR code from the camera (getUserMedia + jsQR, on the phone).
import jsQR from 'jsqr';

export class Scanner {
  video: HTMLVideoElement;
  stream: MediaStream | null = null;
  canvas = document.createElement('canvas');
  running = false;
  timer: ReturnType<typeof setTimeout> | undefined;

  constructor(video: HTMLVideoElement) { this.video = video; }

  /** Starts the back camera and calls found() with the first QR code text read. */
  async start(found: (text: string) => void): Promise<void> {
    if (!navigator.mediaDevices?.getUserMedia) throw new Error('unsupported');
    this.stream = await navigator.mediaDevices.getUserMedia({ audio: false,
      video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 720 } } });
    this.video.srcObject = this.stream;
    this.video.setAttribute('playsinline', '');
    this.video.muted = true;
    await this.video.play();
    this.running = true;
    const ctx = this.canvas.getContext('2d', { willReadFrequently: true })!;
    const scan = () => {
      if (!this.running) return;
      const w = this.video.videoWidth, h = this.video.videoHeight;
      if (w && h) {
        // Read the middle of the picture at a moderate size: fast and enough for the code on a screen.
        const side = Math.min(w, h), size = Math.min(side, 720);
        this.canvas.width = this.canvas.height = size;
        ctx.drawImage(this.video, (w - side) / 2, (h - side) / 2, side, side, 0, 0, size, size);
        const code = jsQR(ctx.getImageData(0, 0, size, size).data, size, size, { inversionAttempts: 'attemptBoth' });
        if (code?.data) { found(code.data); return; }
      }
      this.timer = setTimeout(scan, 120);
    };
    scan();
  }

  stop(): void {
    this.running = false;
    clearTimeout(this.timer);
    this.stream?.getTracks().forEach(t => t.stop());
    this.stream = null;
    this.video.srcObject = null;
  }
}
