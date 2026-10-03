// 验证动作位姿表示必填、relative 解码及不支持表示的快速拒绝。
// @vitest-environment jsdom
import { describe, expect, it } from 'vitest';
import type { IkWorkerRequest } from '../src/app/inferenceProtocol';
import { isActionChunk } from '../src/app/inferenceProtocol';
import type { Pose } from '../src/core/types';
import { parseUrdf } from '../src/utils/urdfParser';
import { decodeActionPose, runIkSolve } from '../src/workers/umiIk.worker';

const makeWorkerRequest = (actionPoseRepr: string): IkWorkerRequest => ({
  type: 'solve',
  request_id: 'request-action-pose-repr',
  episode_index: 0,
  frame_index: 1,
  description: parseUrdf('<robot name="test"><link name="base" /><link name="tool" /></robot>'),
  startPose: { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
  initialJointValues: {},
  actions: [[0, 0, 0, 0, 0, 0, 0]],
  predictionCount: 1,
  configuredExecutionHorizon: 6,
  executionCount: 1,
  selectedActionIndexes: [0],
  action_pose_repr: actionPoseRepr,
  traceAllIterations: false,
});

describe('action_pose_repr 协议与解码', () => {
  it('Browser action chunk 必须携带表示字段，并保留未知字符串交给 decoder 拒绝', () => {
    const chunk = {
      type: 'action_chunk',
      request_id: 'request-1',
      episode_index: 0,
      frame_index: 1,
      last_chunk: false,
      actions: [[0, 0, 0, 0, 0, 0, 0]],
      action_pose_repr: 'relative',
    };

    expect(isActionChunk(chunk)).toBe(true);
    expect(isActionChunk({ ...chunk, action_pose_repr: 'pose-v2' })).toBe(true);
    expect(isActionChunk({ ...chunk, action_pose_repr: undefined })).toBe(false);
  });

  it('relative 在非单位 startPose 下保持原有位姿合成结果', () => {
    const startPose: Pose = {
      position: [1, 2, 3],
      orientation: [0, 0, Math.SQRT1_2, Math.SQRT1_2],
    };
    const relativePose: Pose = {
      position: [0.1, 0, 0],
      orientation: [0, 0, Math.sin(Math.PI / 12), Math.cos(Math.PI / 12)],
    };

    const decoded = decodeActionPose('relative', startPose, relativePose);

    expect(decoded.position[0]).toBeCloseTo(1);
    expect(decoded.position[1]).toBeCloseTo(2.1);
    expect(decoded.position[2]).toBeCloseTo(3);
    expect(decoded.orientation[0]).toBeCloseTo(0);
    expect(decoded.orientation[1]).toBeCloseTo(0);
    expect(decoded.orientation[2]).toBeCloseTo(Math.sin(Math.PI / 3));
    expect(decoded.orientation[3]).toBeCloseTo(Math.cos(Math.PI / 3));
  });

  it.each(['abs', 'rel', 'delta', 'pose-v2'])('%s 在 IK worker 中明确 fail-fast', (repr) => {
    const messages: Array<{ type: string; error?: { code: string } }> = [];

    runIkSolve(makeWorkerRequest(repr), (message) => messages.push(message));

    expect(messages).toEqual([
      expect.objectContaining({
        type: 'error',
        error: expect.objectContaining({ code: 'UNSUPPORTED_ACTION_POSE_REPR' }),
      }),
    ]);
  });
});
