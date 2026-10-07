// 验证 UMI action 时间戳筛选、回退与 observation TCP 位姿引用。
// @vitest-environment jsdom
import { describe, expect, it } from 'vitest';
import type { Pose } from '../src/core/types';
import { OFFICIAL_FRAME_LATENCY_SECONDS, scheduleUmiActions } from '../src/app/umiActionScheduler';
import { decodeActionPose } from '../src/workers/umiIk.worker';

const makeSchedule = (overrides: Partial<Parameters<typeof scheduleUmiActions>[0]> = {}) =>
  scheduleUmiActions({
    actionCount: 16,
    observationTimestampMs: 1000,
    inferenceRequestSentTimestampMs: 1000,
    inferenceResponseReceivedTimestampMs: 1000,
    episodeStartTimestampMs: 1000 + OFFICIAL_FRAME_LATENCY_SECONDS * 1000,
    iterationIndex: 0,
    ...overrides,
  });

describe('UMI 官方式 action 时间戳调度', () => {
  it('10 Hz 为 16 个动作生成间隔 100 ms 的时间戳', () => {
    const schedule = makeSchedule();

    expect(schedule.actionTimestampsMs).toEqual(
      Array.from({ length: 16 }, (_, index) => 1000 + index * 100),
    );
    expect(schedule.selectedActionIndexes).toEqual(
      Array.from({ length: 15 }, (_, index) => index + 1),
    );
  });

  it('230 ms 推理耗时会过期前三项并选择 index 3 至 15', () => {
    const schedule = makeSchedule({
      inferenceResponseReceivedTimestampMs: 1230,
    });

    expect(schedule.expiredActionIndexes).toEqual([0, 1, 2]);
    expect(schedule.selectedActionIndexes).toEqual(
      Array.from({ length: 13 }, (_, index) => index + 3),
    );
  });

  it('全部动作过期时按 eval_t_start 栅格回退到最后一项', () => {
    const schedule = makeSchedule({
      inferenceResponseReceivedTimestampMs: 2600,
    });

    expect(schedule.expiredActionIndexes).toEqual(Array.from({ length: 16 }, (_, index) => index));
    expect(schedule.executableActionIndexes).toEqual([]);
    expect(schedule.fallbackActionIndex).toBe(15);
    expect(schedule.selectedActionIndexes).toEqual([15]);
    expect(schedule.fallbackTimestampMs).toBeCloseTo(2616.6666667);
    expect(schedule.actionTimings[15].state).toBe('FALLBACK');
  });

  it('steps_per_inference 只推进下一次重规划时间，不裁剪到前六项', () => {
    const schedule = makeSchedule({
      inferenceResponseReceivedTimestampMs: 1010,
    });

    expect(schedule.stepsPerInference).toBe(6);
    expect(schedule.selectedActionIndexes).not.toEqual([0, 1, 2, 3, 4, 5]);
    expect(schedule.nextReplanTimestampMs).toBeCloseTo(
      1000 + OFFICIAL_FRAME_LATENCY_SECONDS * 1000 + 600 - OFFICIAL_FRAME_LATENCY_SECONDS * 1000,
    );
  });

  it('仿真 command latency 为 0 时不平移 waypoint 时间戳', () => {
    const schedule = makeSchedule({
      inferenceResponseReceivedTimestampMs: 1020,
      actionExecLatencySeconds: 0,
      simulatedCommandLatencySeconds: 0,
    });

    expect(schedule.simulatedCommandLatencySeconds).toBe(0);
    expect(schedule.actionTimings[1].effectiveScheduledTimestampMs).toBe(1100);
  });

  it('非零 synthetic latency 按严格大于 cutoff 的公式筛选', () => {
    const schedule = makeSchedule({
      inferenceResponseReceivedTimestampMs: 1230,
      actionExecLatencySeconds: 0.071,
      simulatedCommandLatencySeconds: 0.04,
    });

    expect(schedule.timingDiagnostics.staleCutoffTimestampMs).toBe(1301);
    expect(schedule.expiredActionIndexes).toEqual([0, 1, 2, 3]);
    expect(schedule.selectedActionIndexes[0]).toBe(4);
    expect(schedule.actionTimings[4].effectiveScheduledTimestampMs).toBe(1360);
  });

  it('观测时间、推理时间与当前时间用同一个单调时钟域计算', () => {
    const schedule = makeSchedule({
      observationTimestampMs: 5000,
      inferenceRequestSentTimestampMs: 5100,
      inferenceResponseReceivedTimestampMs: 5200,
      schedulerCurrentTimestampMs: 5204,
      episodeStartTimestampMs: 5016.6666667,
    });

    expect(schedule.timingDiagnostics.clockDomain).toBe('window.performance.now');
    expect(schedule.timingDiagnostics.schedulerCurrentTimestampMs).toBe(5204);
    expect(schedule.timingDiagnostics.staleCutoffTimestampMs).toBe(5214);
    expect(schedule.timingDiagnostics.inferenceRoundTripMs).toBe(100);
    expect(schedule.expiredActionIndexes).toEqual([0, 1, 2]);
    expect(schedule.selectedActionIndexes[0]).toBe(3);
  });

  it('所有相对 action pose 都以原 observation TCP 合成，不逐项累加', () => {
    const observationTcp: Pose = {
      position: [1, 2, 3],
      orientation: [0, 0, Math.SQRT1_2, Math.SQRT1_2],
    };
    const targets = Array.from({ length: 16 }, (_, index) =>
      decodeActionPose('relative', observationTcp, {
        position: [index / 100, 0, 0],
        orientation: [0, 0, 0, 1],
      }),
    );
    const schedule = makeSchedule({
      inferenceResponseReceivedTimestampMs: 1230,
    });

    schedule.selectedActionIndexes.forEach((actionIndex) => {
      expect(targets[actionIndex].position[0]).toBeCloseTo(1);
      expect(targets[actionIndex].position[1]).toBeCloseTo(2 + actionIndex / 100);
    });
  });
});
