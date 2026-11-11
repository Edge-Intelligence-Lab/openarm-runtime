// Both server streams carry bounded Content-Length multipart JPEG/PNG frames.
export class MultipartFrames {
  constructor(limit = 8_000_000) {
    this.limit = limit;
    this.buffer = new Uint8Array(0);
    this.pending = null;
  }
  push(chunk) {
    const joined = new Uint8Array(this.buffer.length + chunk.length);
    joined.set(this.buffer); joined.set(chunk, this.buffer.length);
    this.buffer = joined;
    const frames = [];
    while (true) {
      if (!this.pending) {
        let end = -1;
        for (let i = 0; i + 3 < this.buffer.length; ++i) {
          if (this.buffer[i] === 13 && this.buffer[i+1] === 10 &&
              this.buffer[i+2] === 13 && this.buffer[i+3] === 10) { end = i; break; }
        }
        if (end < 0) {
          if (this.buffer.length > 16384) throw new Error('视频帧头过大');
          break;
        }
        if (end > 16384) throw new Error('视频帧头过大');
        const header = new TextDecoder().decode(this.buffer.subarray(0, end));
        const length = /^Content-Length:\s*(\d+)\s*$/im.exec(header);
        const type = /^Content-Type:\s*(image\/(?:jpeg|png))\s*$/im.exec(header);
        if (!length || !type || +length[1] < 1 || +length[1] > this.limit)
          throw new Error('视频帧协议无效');
        this.pending = {length: +length[1], type: type[1].toLowerCase()};
        this.buffer = this.buffer.slice(end + 4);
      }
      if (this.buffer.length < this.pending.length) break;
      frames.push({bytes: this.buffer.slice(0, this.pending.length), type: this.pending.type});
      this.buffer = this.buffer.slice(this.pending.length);
      this.pending = null;
    }
    if (this.buffer.length > this.limit + 16384) throw new Error('视频缓冲超限');
    return frames;
  }
}

export default {
  template: `<div class="relative w-full">
    <canvas ref="canvas" class="block w-full" style="height:auto;min-height:100px"></canvas>
    <span v-if="status" class="absolute bottom-1 left-1 rounded px-2 py-1 text-xs"
      style="background:rgba(0,0,0,.65);color:white">{{ status }}</span>
  </div>`,
  props: {src: String, reconnect: {type: Boolean, default: true}},
  data() { return {status: '连接中…', running: false}; },
  mounted() { this.generation = 0; this.connect(); },
  beforeUnmount() {
    ++this.generation;
    this.controller?.abort();
    clearTimeout(this.watchdog);
    // Also wake a pending retry immediately, without leaving timer callbacks.
    clearTimeout(this.retryTimer);
    this.retryWake?.();
  },
  watch: {
    src() { this.connect(); },
    reconnect(value) { if (value && !this.running) this.connect(); },
  },
  methods: {
    async connect() {
      const generation = ++this.generation;
      this.controller?.abort();
      clearTimeout(this.watchdog);
      clearTimeout(this.retryTimer); this.retryWake?.();
      this.running = true;
      let delay = 500;
      while (generation === this.generation) {
        const controller = new AbortController();
        this.controller = controller;
        let reader;
        let watchdog;
        const armWatchdog = () => {
          clearTimeout(watchdog);
          watchdog = setTimeout(() => controller.abort(), 5000);
          this.watchdog = watchdog;
        };
        try {
          const url = new URL(this.src, window.location.href);
          if (url.origin !== window.location.origin) throw new Error('仅允许同源视频');
          armWatchdog();
          const response = await fetch(url, {signal: controller.signal, cache: 'no-store', credentials: 'same-origin'});
          if (!response.ok || !response.headers.get('content-type')?.includes('multipart/x-mixed-replace'))
            throw new Error('视频源未就绪');
          reader = response.body.getReader();
          const parser = new MultipartFrames();
          while (generation === this.generation) {
            const {value, done} = await reader.read();
            if (done) break;
            for (const frame of parser.push(value)) {
              const image = await createImageBitmap(new Blob([frame.bytes], {type: frame.type}));
              try {
                if (generation !== this.generation) break;
                const canvas = this.$refs.canvas;
                if (canvas.width !== image.width || canvas.height !== image.height) {
                  canvas.width = image.width; canvas.height = image.height;
                }
                canvas.getContext('2d').drawImage(image, 0, 0);
                this.status = ''; delay = 500; armWatchdog();
              } finally { image.close(); }
            }
          }
        } catch (_) {
          if (generation === this.generation) this.status = '视频断开，重连中…';
        } finally {
          clearTimeout(watchdog);
          controller.abort();
          try { await reader?.cancel(); } catch (_) {}
        }
        if (generation !== this.generation) return;
        if (!this.reconnect) { this.status = '本轮输入已结束'; break; }
        this.status = '视频断开，重连中…';
        await new Promise(resolve => {
          this.retryWake = resolve;
          this.retryTimer = setTimeout(resolve, delay);
        });
        if (generation !== this.generation) return;
        this.retryWake = null;
        delay = Math.min(delay * 2, 8000);
      }
      if (generation === this.generation) this.running = false;
    },
  },
};
