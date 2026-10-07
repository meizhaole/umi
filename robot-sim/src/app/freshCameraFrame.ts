export interface CameraFrameSample {
  cameraFrameIndex: number;
  cameraTimestampMs: number;
  imageAvailable: boolean;
}

export interface FreshCameraFrameWaitOptions {
  previousCameraFrameIndex: number;
  previousCameraTimestampMs: number;
  timeoutMs: number;
  signal: AbortSignal;
}

export interface FreshCameraFrameWaitResult {
  cameraFrameIndex: number;
  cameraTimestampMs: number;
  waitDurationMs: number;
}

export type FreshCameraFrameWaitFailureReason = 'timeout' | 'aborted' | 'busy' | 'reset';

export class FreshCameraFrameWaitError extends Error {
  constructor(
    readonly reason: FreshCameraFrameWaitFailureReason,
    readonly previousCameraFrameIndex: number,
    readonly previousCameraTimestampMs: number,
    readonly latestCameraFrameIndex: number | null,
    readonly latestCameraTimestampMs: number | null,
    readonly waitDurationMs: number,
  ) {
    super(`等待新相机帧失败：${reason}`);
    this.name = 'FreshCameraFrameWaitError';
  }
}

interface PendingWaiter {
  options: FreshCameraFrameWaitOptions;
  startedAtMs: number;
  resolve: (sample: FreshCameraFrameWaitResult) => void;
  reject: (error: FreshCameraFrameWaitError) => void;
  timeoutId: ReturnType<typeof setTimeout>;
  abortListener: () => void;
}

const isFreshFrame = (
  sample: CameraFrameSample | null,
  options: FreshCameraFrameWaitOptions,
): sample is CameraFrameSample =>
  Boolean(
    sample?.imageAvailable &&
    Number.isInteger(sample.cameraFrameIndex) &&
    sample.cameraFrameIndex > options.previousCameraFrameIndex &&
    Number.isFinite(sample.cameraTimestampMs) &&
    sample.cameraTimestampMs > options.previousCameraTimestampMs,
  );

export class FreshCameraFrameWaiter {
  private latestFrame: CameraFrameSample | null = null;

  private pendingWaiter: PendingWaiter | null = null;

  publish(sample: CameraFrameSample): void {
    this.latestFrame = { ...sample };
    if (this.pendingWaiter && isFreshFrame(this.latestFrame, this.pendingWaiter.options)) {
      this.resolvePendingWaiter(this.latestFrame);
    }
  }

  reset(): void {
    this.latestFrame = null;
    this.cancelPendingWaiter('reset');
  }

  waitForFreshFrame(options: FreshCameraFrameWaitOptions): Promise<FreshCameraFrameWaitResult> {
    const startedAtMs = performance.now();
    if (this.pendingWaiter) {
      return Promise.reject(this.createError('busy', options, startedAtMs));
    }
    if (options.signal.aborted) {
      return Promise.reject(this.createError('aborted', options, startedAtMs));
    }
    if (isFreshFrame(this.latestFrame, options)) {
      return Promise.resolve({
        cameraFrameIndex: this.latestFrame.cameraFrameIndex,
        cameraTimestampMs: this.latestFrame.cameraTimestampMs,
        waitDurationMs: performance.now() - startedAtMs,
      });
    }

    return new Promise((resolve, reject) => {
      const abortListener = () => this.cancelPendingWaiter('aborted');
      const timeoutId = setTimeout(() => this.cancelPendingWaiter('timeout'), options.timeoutMs);
      this.pendingWaiter = {
        options,
        startedAtMs,
        resolve,
        reject,
        timeoutId,
        abortListener,
      };
      options.signal.addEventListener('abort', abortListener, { once: true });
    });
  }

  private resolvePendingWaiter(sample: CameraFrameSample): void {
    const waiter = this.pendingWaiter;
    if (!waiter) return;
    this.cleanupPendingWaiter(waiter);
    waiter.resolve({
      cameraFrameIndex: sample.cameraFrameIndex,
      cameraTimestampMs: sample.cameraTimestampMs,
      waitDurationMs: performance.now() - waiter.startedAtMs,
    });
  }

  private cancelPendingWaiter(reason: FreshCameraFrameWaitFailureReason): void {
    const waiter = this.pendingWaiter;
    if (!waiter) return;
    this.cleanupPendingWaiter(waiter);
    waiter.reject(this.createError(reason, waiter.options, waiter.startedAtMs));
  }

  private cleanupPendingWaiter(waiter: PendingWaiter): void {
    if (this.pendingWaiter !== waiter) return;
    this.pendingWaiter = null;
    clearTimeout(waiter.timeoutId);
    waiter.options.signal.removeEventListener('abort', waiter.abortListener);
  }

  private createError(
    reason: FreshCameraFrameWaitFailureReason,
    options: FreshCameraFrameWaitOptions,
    startedAtMs: number,
  ): FreshCameraFrameWaitError {
    return new FreshCameraFrameWaitError(
      reason,
      options.previousCameraFrameIndex,
      options.previousCameraTimestampMs,
      this.latestFrame?.cameraFrameIndex ?? null,
      this.latestFrame?.cameraTimestampMs ?? null,
      performance.now() - startedAtMs,
    );
  }
}
