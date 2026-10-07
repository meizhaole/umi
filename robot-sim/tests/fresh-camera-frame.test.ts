import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { FreshCameraFrameWaitError, FreshCameraFrameWaiter } from '../src/app/freshCameraFrame';

const baseline = { cameraFrameIndex: 100, cameraTimestampMs: 500, imageAvailable: true };
const waitOptions = (signal = new AbortController().signal) => ({
  previousCameraFrameIndex: 100,
  previousCameraTimestampMs: 500,
  timeoutMs: 1000,
  signal,
});

describe('FreshCameraFrameWaiter', () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it('缓存仍是 baseline 时保持等待，callback 同时推进 index 和 timestamp 后 resolve', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const pending = waiter.waitForFreshFrame(waitOptions());
    let settled = false;
    void pending.then(() => {
      settled = true;
    });

    await Promise.resolve();
    expect(settled).toBe(false);
    waiter.publish({ cameraFrameIndex: 101, cameraTimestampMs: 510, imageAvailable: true });

    await expect(pending).resolves.toMatchObject({
      cameraFrameIndex: 101,
      cameraTimestampMs: 510,
    });
    expect(vi.getTimerCount()).toBe(0);
  });

  it('只推进 frame index 时继续等待', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const pending = waiter.waitForFreshFrame(waitOptions());
    waiter.publish({ cameraFrameIndex: 101, cameraTimestampMs: 500, imageAvailable: true });

    let settled = false;
    void pending.then(() => {
      settled = true;
    });
    await Promise.resolve();
    expect(settled).toBe(false);

    waiter.publish({ cameraFrameIndex: 102, cameraTimestampMs: 510, imageAvailable: true });
    await expect(pending).resolves.toMatchObject({ cameraFrameIndex: 102 });
  });

  it('只推进 timestamp 时继续等待', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const pending = waiter.waitForFreshFrame(waitOptions());
    waiter.publish({ cameraFrameIndex: 100, cameraTimestampMs: 510, imageAvailable: true });

    let settled = false;
    void pending.then(() => {
      settled = true;
    });
    await Promise.resolve();
    expect(settled).toBe(false);

    waiter.publish({ cameraFrameIndex: 101, cameraTimestampMs: 520, imageAvailable: true });
    await expect(pending).resolves.toMatchObject({ cameraFrameIndex: 101 });
  });

  it('index 和 timestamp 必须都严格增加', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    waiter.publish({ cameraFrameIndex: 101, cameraTimestampMs: 510, imageAvailable: true });

    await expect(waiter.waitForFreshFrame(waitOptions())).resolves.toMatchObject({
      cameraFrameIndex: 101,
      cameraTimestampMs: 510,
      waitDurationMs: 0,
    });
    expect(vi.getTimerCount()).toBe(0);
  });

  it('timeout 错误提供 baseline、latest sample 和等待时长', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const pending = waiter.waitForFreshFrame(waitOptions());
    const assertion = expect(pending).rejects.toMatchObject({
      reason: 'timeout',
      previousCameraFrameIndex: 100,
      previousCameraTimestampMs: 500,
      latestCameraFrameIndex: 100,
      latestCameraTimestampMs: 500,
      waitDurationMs: 1000,
    });

    await vi.advanceTimersByTimeAsync(1000);
    await assertion;
    expect(vi.getTimerCount()).toBe(0);
  });

  it('timestamp 非有限时继续等待并在 timeout evidence 中保留异常值', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const pending = waiter.waitForFreshFrame(waitOptions());
    waiter.publish({ cameraFrameIndex: 101, cameraTimestampMs: Number.NaN, imageAvailable: true });
    const assertion = expect(pending).rejects.toSatisfy(
      (error: FreshCameraFrameWaitError) =>
        error.reason === 'timeout' &&
        error.latestCameraFrameIndex === 101 &&
        Number.isNaN(error.latestCameraTimestampMs) &&
        error.waitDurationMs === 1000,
    );

    await vi.advanceTimersByTimeAsync(1000);
    await assertion;
    expect(vi.getTimerCount()).toBe(0);
  });

  it('abort 立即取消 waiter 并清理 timeout', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const controller = new AbortController();
    const pending = waiter.waitForFreshFrame(waitOptions(controller.signal));
    const assertion = expect(pending).rejects.toBeInstanceOf(FreshCameraFrameWaitError);

    controller.abort();

    await assertion;
    expect(vi.getTimerCount()).toBe(0);
  });

  it('同一 camera stream 不接受并发 waiter', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const first = waiter.waitForFreshFrame(waitOptions());
    const second = waiter.waitForFreshFrame(waitOptions());
    const assertion = expect(second).rejects.toMatchObject({ reason: 'busy' });

    waiter.publish({ cameraFrameIndex: 101, cameraTimestampMs: 510, imageAvailable: true });

    await Promise.all([first, assertion]);
    expect(vi.getTimerCount()).toBe(0);
  });

  it('没有有效图像的 camera callback 不满足 freshness', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const pending = waiter.waitForFreshFrame(waitOptions());
    waiter.publish({ cameraFrameIndex: 101, cameraTimestampMs: 510, imageAvailable: false });

    let settled = false;
    void pending.then(() => {
      settled = true;
    });
    await Promise.resolve();
    expect(settled).toBe(false);

    waiter.publish({ cameraFrameIndex: 101, cameraTimestampMs: 510, imageAvailable: true });
    await expect(pending).resolves.toMatchObject({ cameraFrameIndex: 101 });
  });

  it('camera cache reset 会取消 waiter 并移除 timeout', async () => {
    const waiter = new FreshCameraFrameWaiter();
    waiter.publish(baseline);
    const pending = waiter.waitForFreshFrame(waitOptions());
    const assertion = expect(pending).rejects.toMatchObject({ reason: 'reset' });

    waiter.reset();

    await assertion;
    expect(vi.getTimerCount()).toBe(0);
  });
});
