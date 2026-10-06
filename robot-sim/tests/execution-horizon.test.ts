// 验证完整预测与执行前缀分离，并保持单步人工确认。
// @vitest-environment jsdom
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { act, createElement } from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import type {
  IKActionDebugRecord,
  IkWorkerRequest,
  IkWorkerResponse,
} from '../src/app/inferenceProtocol';
import { findRobotConfig } from '../src/app/config';
import { selectExecutionActions } from '../src/app/executionHorizon';
import { useInferenceReplay } from '../src/app/useInferenceReplay';
import type { InferenceObservation } from '../src/app/inferenceProtocol';
import type { JointValues, Pose } from '../src/core/types';
import { Kinematics } from '../src/core/Kinematics';
import { parseUrdf } from '../src/utils/urdfParser';
import { quaternionToRotationVector } from '../src/utils/math';
import { runIkSolve } from '../src/workers/umiIk.worker';

const planarUrdf = `
  <robot name="planar">
    <link name="base" />
    <link name="arm" />
    <link name="tool" />
    <joint name="shoulder" type="revolute">
      <parent link="base" />
      <child link="arm" />
      <axis xyz="0 0 1" />
      <limit lower="-2" upper="2" effort="10" velocity="2" />
    </joint>
    <joint name="elbow" type="revolute">
      <origin xyz="1 0 0" />
      <parent link="arm" />
      <child link="tool" />
      <axis xyz="0 0 1" />
      <limit lower="-2" upper="2" effort="10" velocity="2" />
    </joint>
  </robot>
`;

const description = parseUrdf(planarUrdf);
const seed = { shoulder: 0, elbow: 0 };
const startPose = new Kinematics(description).forwardKinematics(seed);
const cadConfig = findRobotConfig('UR5_CAD');
const cadDescription = parseUrdf(readFileSync(resolve('public/robot', cadConfig.file), 'utf8'), {
  tipLink: cadConfig.tipLink,
});
const cadSeed = cadConfig.initialJoints;
const cadStartPose = new Kinematics(cadDescription).forwardKinematics(cadSeed);
const cadStartRotation = quaternionToRotationVector(cadStartPose.orientation);
const makeActions = (count: number) =>
  Array.from({ length: count }, (_, index) => [index, 0, 0, 0, 0, 0, 0]);

class FakeWebSocket {
  static readonly OPEN = 1;

  static readonly CLOSING = 2;

  static sockets: FakeWebSocket[] = [];

  readyState = 0;

  onopen: ((event: Event) => void) | null = null;

  onmessage: ((event: MessageEvent<string>) => void) | null = null;

  onerror: ((event: Event) => void) | null = null;

  onclose: ((event: CloseEvent) => void) | null = null;

  sent: string[] = [];

  constructor(readonly url: string) {
    FakeWebSocket.sockets.push(this);
  }

  send(data: string) {
    this.sent.push(data);
  }

  close() {
    this.readyState = FakeWebSocket.CLOSING;
  }

  emitMessage(message: unknown) {
    this.onmessage?.({ data: JSON.stringify(message) } as MessageEvent<string>);
  }
}

class FakeWorker {
  static workers: FakeWorker[] = [];

  static nextResponse: IkWorkerResponse | null = null;

  onmessage: ((event: MessageEvent<IkWorkerResponse>) => void) | null = null;

  onerror: ((event: ErrorEvent) => void) | null = null;

  onmessageerror: ((event: MessageEvent) => void) | null = null;

  request: IkWorkerRequest | null = null;

  requests: IkWorkerRequest[] = [];

  messages: IkWorkerResponse[] = [];

  constructor() {
    FakeWorker.workers.push(this);
  }

  postMessage(request: IkWorkerRequest) {
    this.request = request;
    this.requests.push(request);
    if (FakeWorker.nextResponse) {
      const response = FakeWorker.nextResponse;
      FakeWorker.nextResponse = null;
      this.messages.push(response);
      this.onmessage?.({ data: response } as MessageEvent<IkWorkerResponse>);
      return;
    }
    runIkSolve(request, (message) => {
      this.messages.push(message);
      this.onmessage?.({ data: message } as MessageEvent<IkWorkerResponse>);
    });
  }

  terminate() {}
}

const createSingleActionReviewHarness = async (requestId: string) => {
  let observation: InferenceObservation = {
    frame_index: 7,
    camera_frame_timestamp_ms: 50,
    camera0_rgb: 'AAAA',
    robot0_eef_pos: [cadStartPose.position, cadStartPose.position],
    robot0_eef_rot_axis_angle: [cadStartRotation, cadStartRotation],
    robot0_gripper_width: [[0], [0]],
  };
  const context = {
    robotId: 'UR5_CAD' as const,
    robotConfig: cadConfig,
    loadedRobotId: 'UR5_CAD' as const,
    loadedRobotUrdfFile: cadConfig.file,
    description: cadDescription,
    jointValues: cadSeed,
    tcpPose: cadStartPose,
    getObservation: () => observation,
    singleActionDebugEnabled: true,
  };
  let replay!: ReturnType<typeof useInferenceReplay>;
  const Probe = () => {
    replay = useInferenceReplay(context);
    return null;
  };
  const container = document.createElement('div');
  document.body.appendChild(container);
  const root = createRoot(container);

  await act(async () => {
    root.render(createElement(Probe));
  });
  act(() => replay.step());
  const socket = FakeWebSocket.sockets[0];
  socket.readyState = FakeWebSocket.OPEN;
  act(() => socket.emitMessage({ type: 'status', state: 'ready' }));

  const prediction = Array.from({ length: 16 }, () => [0.001, 0, 0, 1, 0, 0, 0, 1, 0, 0.06]);
  const actions = Array.from({ length: 16 }, () => [0.001, 0, 0, 0, 0, 0, 0.06]);
  await act(async () => {
    socket.emitMessage({
      type: 'action_chunk',
      request_id: requestId,
      episode_index: 0,
      frame_index: 7,
      last_chunk: false,
      actions,
      action_pose_repr: 'relative',
      prediction,
      prediction_keys: ['action', 'action_pred'],
      action_pred_shape: [1, 16, 10],
      policy_tensor_shapes: { camera0_rgb: [1, 1, 3, 224, 224] },
      input_image_shape: [224, 224, 3],
    });
    await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
  });

  return {
    get replay() {
      return replay;
    },
    setObservation(nextObservation: InferenceObservation) {
      observation = nextObservation;
    },
    socket,
    root,
    container,
  };
};

const createAfterActionSnapshot = () => ({
  joint_values: cadSeed,
  tcp_pose: cadStartPose,
  gripper_opening_m: 0.06,
  observation: {
    frame_index: 8,
    camera_frame_timestamp_ms: 100,
    camera0_rgb: 'AAAA',
    robot0_eef_pos: [cadStartPose.position, cadStartPose.position],
    robot0_eef_rot_axis_angle: [cadStartRotation, cadStartRotation],
    robot0_gripper_width: [[0.06], [0.06]],
  } satisfies InferenceObservation,
});

const makeAutoLoopObservation = (
  frameIndex: number,
  cameraFrameIndex: number,
  cameraTimestamp: number,
  tcpPose: Pose,
  gripperOpening: number,
): InferenceObservation => {
  const rotation = quaternionToRotationVector(tcpPose.orientation);
  return {
    frame_index: frameIndex,
    camera_frame_index: cameraFrameIndex,
    camera_frame_timestamp_ms: cameraTimestamp,
    camera0_rgb: `frame-${cameraFrameIndex}`,
    camera_image_shape: [224, 224, 3],
    robot0_eef_pos: [tcpPose.position, tcpPose.position],
    robot0_eef_rot_axis_angle: [rotation, rotation],
    robot0_gripper_width: [[gripperOpening], [gripperOpening]],
  };
};

const createAutoLoopHarness = async () => {
  let observation = makeAutoLoopObservation(7, 10, 50, cadStartPose, 0.06);
  const context = {
    robotId: 'UR5_CAD' as const,
    robotConfig: cadConfig,
    loadedRobotId: 'UR5_CAD' as const,
    loadedRobotUrdfFile: cadConfig.file,
    description: cadDescription,
    jointValues: cadSeed,
    tcpPose: cadStartPose,
    getObservation: () => observation,
    singleActionDebugEnabled: true,
  };
  let replay!: ReturnType<typeof useInferenceReplay>;
  const Probe = () => {
    replay = useInferenceReplay(context);
    return null;
  };
  const container = document.createElement('div');
  document.body.appendChild(container);
  const root = createRoot(container);
  const postedReviews: Record<string, unknown>[] = [];
  vi.stubGlobal(
    'fetch',
    vi.fn(async (_url: string, init: RequestInit) => {
      const review = JSON.parse(init.body as string) as Record<string, unknown>;
      postedReviews.push(review);
      if (review.review_status === 'COMPLETED') {
        const actualAfterAction = review.actual_after_action as {
          observation_frame_index: number;
          camera_frame_index: number;
          camera_frame_timestamp_ms: number;
          actual_joints: Record<string, number>;
          actual_tcp: Pose;
          actual_gripper_opening_m: number;
        };
        observation = makeAutoLoopObservation(
          actualAfterAction.observation_frame_index + 1,
          actualAfterAction.camera_frame_index + 1,
          actualAfterAction.camera_frame_timestamp_ms + 10,
          actualAfterAction.actual_tcp,
          actualAfterAction.actual_gripper_opening_m,
        );
        context.jointValues = actualAfterAction.actual_joints as JointValues;
        context.tcpPose = actualAfterAction.actual_tcp;
        root.render(createElement(Probe));
      }
      return {
        ok: true,
        status: 200,
        json: async () => ({ saved: true, path: '/tmp/auto-loop-step.json' }),
      };
    }),
  );

  await act(async () => {
    root.render(createElement(Probe));
  });
  act(() => replay.startAutoLoop());
  const socket = FakeWebSocket.sockets[0];
  socket.readyState = FakeWebSocket.OPEN;
  await act(async () => {
    socket.emitMessage({ type: 'status', state: 'ready' });
    await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
  });

  return {
    get replay() {
      return replay;
    },
    get observation() {
      return observation;
    },
    postedReviews,
    socket,
    root,
    container,
    setObservation(nextObservation: InferenceObservation) {
      observation = nextObservation;
      root.render(createElement(Probe));
    },
  };
};

const makeAutoLoopActionChunk = (
  observation: InferenceObservation,
  requestId: string,
  translationX = 0.001,
) => ({
  type: 'action_chunk',
  request_id: requestId,
  episode_index: 0,
  frame_index: observation.frame_index,
  last_chunk: false,
  actions: Array.from({ length: 16 }, () => [translationX, 0, 0, 0, 0, 0, 0.06]),
  action_pose_repr: 'relative',
  prediction: Array.from({ length: 16 }, () => [translationX, 0, 0, 1, 0, 0, 0, 1, 0, 0.06]),
  prediction_keys: ['action', 'action_pred'],
  action_pred_shape: [1, 16, 10],
  policy_tensor_shapes: { camera0_rgb: [1, 1, 3, 224, 224] },
  input_image_shape: [224, 224, 3],
});

const sendAutoLoopActionChunk = async (
  harness: Awaited<ReturnType<typeof createAutoLoopHarness>>,
  requestId: string,
  translationX = 0.001,
) => {
  await act(async () => {
    harness.socket.emitMessage(
      makeAutoLoopActionChunk(harness.observation, requestId, translationX),
    );
    await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
  });
};

const completeAutoLoopPlayback = async (
  harness: Awaited<ReturnType<typeof createAutoLoopHarness>>,
  options: { cameraFrameIndex?: number; gripperOpening?: number } = {},
) => {
  const review = harness.replay.reviewData as Record<string, unknown>;
  const targetTcp = review.target_tcp as Pose;
  const targetJoints = review.target_joints as Record<string, number>;
  const targetGripper = review.target_gripper_opening_m as number;
  const cameraFrameIndex =
    options.cameraFrameIndex ?? (harness.observation.camera_frame_index as number) + 1;
  const actualGripper = options.gripperOpening ?? targetGripper;
  const observation = makeAutoLoopObservation(
    harness.observation.frame_index + 1,
    cameraFrameIndex,
    harness.observation.camera_frame_timestamp_ms + 10,
    targetTcp,
    actualGripper,
  );
  const token = harness.replay.playback?.token as number;
  await act(async () => {
    harness.replay.completePlayback(token, {
      joint_values: targetJoints,
      tcp_pose: targetTcp,
      gripper_opening_m: actualGripper,
      observation,
    });
    await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
  });
};

describe('execution horizon', () => {
  beforeEach(() => {
    FakeWebSocket.sockets = [];
    FakeWorker.workers = [];
    FakeWorker.nextResponse = null;
    vi.stubGlobal('WebSocket', FakeWebSocket);
    vi.stubGlobal('Worker', FakeWorker);
    Object.assign(globalThis, { IS_REACT_ACT_ENVIRONMENT: true });
  });

  afterEach(() => {
    vi.unstubAllGlobals();
    Object.assign(globalThis, { IS_REACT_ACT_ENVIRONMENT: false });
  });

  it('16 项预测保留完整，只选择 6 项执行', () => {
    const predictedActions = makeActions(16);
    const selection = selectExecutionActions(predictedActions);

    expect(predictedActions).toHaveLength(16);
    expect(selection).toEqual({
      actions: predictedActions.slice(0, 6),
      predictionCount: 16,
      configuredExecutionHorizon: 6,
      executionCount: 6,
      selectedActionIndexes: [0, 1, 2, 3, 4, 5],
    });
  });

  it('prediction 比 execution horizon 短时只选择已有动作', () => {
    const predictedActions = makeActions(4);
    const selection = selectExecutionActions(predictedActions, 6);

    expect(selection.executionCount).toBe(4);
    expect(selection.selectedActionIndexes).toEqual([0, 1, 2, 3]);
    expect(predictedActions).toHaveLength(4);
  });

  it('execution horizon 为 1 时只选择 action 0', () => {
    const selection = selectExecutionActions(makeActions(16), 1);

    expect(selection.executionCount).toBe(1);
    expect(selection.selectedActionIndexes).toEqual([0]);
    expect(selection.actions).toHaveLength(1);
  });

  it('action 8 的 IK failure 不阻止 0 到 5 求解和播放，ACK 后等待手动下一步', () => {
    const observation: InferenceObservation = {
      frame_index: 7,
      camera_frame_timestamp_ms: 50,
      camera0_rgb: 'AAAA',
      robot0_eef_pos: [startPose.position, startPose.position],
      robot0_eef_rot_axis_angle: [
        [0, 0, 0],
        [0, 0, 0],
      ],
      robot0_gripper_width: [[0], [0]],
    };
    const liveTcpPose = {
      position: [startPose.position[0] + 0.25, 0.5, -0.5] as [number, number, number],
      orientation: [0, 0, Math.SQRT1_2, Math.SQRT1_2] as [number, number, number, number],
    };
    const context = {
      robotId: 'UR5' as const,
      robotConfig: {
        ...findRobotConfig('UR5'),
        ikOptions: { positionTolerance: 0.0005, orientationTolerance: 0.001 },
      },
      loadedRobotId: 'UR5' as const,
      loadedRobotUrdfFile: findRobotConfig('UR5').file,
      description,
      jointValues: seed,
      tcpPose: liveTcpPose,
      getObservation: () => observation,
      singleActionDebugEnabled: false,
    };
    let replay!: ReturnType<typeof useInferenceReplay>;
    const Probe = () => {
      replay = useInferenceReplay(context);
      return null;
    };
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);

    act(() => root.render(createElement(Probe)));
    act(() => replay.step());
    const socket = FakeWebSocket.sockets[0];
    socket.readyState = FakeWebSocket.OPEN;
    act(() => socket.emitMessage({ type: 'status', state: 'ready' }));

    const predictedActions = Array.from({ length: 16 }, () => [0, 0, 0, 0, 0, 0, 0]);
    predictedActions[8] = [5, 5, 5, 0, 0, 0, 0];
    act(() =>
      socket.emitMessage({
        type: 'action_chunk',
        request_id: 'request-horizon',
        episode_index: 0,
        frame_index: 7,
        last_chunk: false,
        actions: predictedActions,
        action_pose_repr: 'relative',
        prediction: Array.from({ length: 16 }, () => [0, 0, 0, 1, 0, 0, 0, 1, 0, 0]),
        prediction_keys: ['action', 'action_pred'],
        action_pred_shape: [1, 16, 10],
        policy_tensor_shapes: { camera0_rgb: [1, 1, 3, 224, 224] },
        input_image_shape: [224, 224, 3],
      }),
    );

    const worker = FakeWorker.workers[0];
    expect(worker.request).toMatchObject({
      request_id: 'request-horizon',
      episode_index: 0,
      frame_index: 7,
      startPose,
      predictionCount: 16,
      configuredExecutionHorizon: 6,
      executionCount: 6,
      selectedActionIndexes: [0, 1, 2, 3, 4, 5],
      action_pose_repr: 'relative',
      ikOptions: { positionTolerance: 0.0005, orientationTolerance: 0.001 },
      actions: predictedActions.slice(0, 6),
    });
    expect(predictedActions).toHaveLength(16);
    expect(replay.playback?.actions).toHaveLength(6);
    expect(replay.playback?.actions.map((action) => action.actionIndex)).toEqual([
      0, 1, 2, 3, 4, 5,
    ]);

    const resultRecords = worker.messages.flatMap((message) =>
      message.type === 'ik_record' && message.record.record_phase === 'result'
        ? [message.record as IKActionDebugRecord]
        : [],
    );
    expect(resultRecords.map((record) => record.action_index)).toEqual([0, 1, 2, 3, 4, 5]);
    expect(resultRecords.every((record) => record.action_pose_repr === 'relative')).toBe(true);
    expect(resultRecords[0]).toMatchObject({
      request_id: 'request-horizon',
      episode_index: 0,
      frame_index: 7,
      prediction_count: 16,
      configured_execution_horizon: 6,
      execution_count: 6,
      selected_action_indexes: [0, 1, 2, 3, 4, 5],
    });
    expect(resultRecords.some((record) => record.action_index === 8)).toBe(false);

    const playbackToken = replay.playback?.token as number;
    act(() => replay.completePlayback(playbackToken));
    expect(replay.status).toBe('waiting');
    const sentBeforeManualStep = socket.sent.map((message) => JSON.parse(message).type);
    expect(sentBeforeManualStep.filter((type) => type === 'observation')).toHaveLength(1);
    expect(sentBeforeManualStep).toContain('ack');

    act(() => replay.step());
    const sentAfterManualStep = socket.sent.map((message) => JSON.parse(message).type);
    expect(sentAfterManualStep.filter((type) => type === 'observation')).toHaveLength(2);

    act(() => root.unmount());
    container.remove();
  });

  it('single-action review 只让非零 IK 目标进入人工审批', async () => {
    vi.stubGlobal('fetch', async () => ({
      ok: true,
      status: 200,
      json: async () => ({ saved: true, path: '/tmp/single-action-review.json' }),
    }));
    const observation: InferenceObservation = {
      frame_index: 7,
      camera_frame_timestamp_ms: 50,
      camera0_rgb: 'AAAA',
      robot0_eef_pos: [cadStartPose.position, cadStartPose.position],
      robot0_eef_rot_axis_angle: [cadStartRotation, cadStartRotation],
      robot0_gripper_width: [[0], [0]],
    };
    const context = {
      robotId: 'UR5_CAD' as const,
      robotConfig: cadConfig,
      loadedRobotId: 'UR5_CAD' as const,
      loadedRobotUrdfFile: cadConfig.file,
      description: cadDescription,
      jointValues: cadSeed,
      tcpPose: cadStartPose,
      getObservation: () => observation,
      singleActionDebugEnabled: true,
    };
    const results = [];

    for (const [requestId, translationX] of [
      ['request-zero-target', 0],
      ['request-nonzero-target', 0.001],
    ] as const) {
      FakeWebSocket.sockets = [];
      FakeWorker.workers = [];
      let replay!: ReturnType<typeof useInferenceReplay>;
      const Probe = () => {
        replay = useInferenceReplay(context);
        return null;
      };
      const container = document.createElement('div');
      document.body.appendChild(container);
      const root = createRoot(container);

      await act(async () => {
        root.render(createElement(Probe));
      });
      act(() => replay.step());
      const socket = FakeWebSocket.sockets[0];
      socket.readyState = FakeWebSocket.OPEN;
      act(() => socket.emitMessage({ type: 'status', state: 'ready' }));

      const prediction = Array.from({ length: 16 }, () => [
        translationX,
        0,
        0,
        1,
        0,
        0,
        0,
        1,
        0,
        0.06,
      ]);
      const actions = Array.from({ length: 16 }, () => [translationX, 0, 0, 0, 0, 0, 0.06]);
      await act(async () => {
        socket.emitMessage({
          type: 'action_chunk',
          request_id: requestId,
          episode_index: 0,
          frame_index: 7,
          last_chunk: false,
          actions,
          action_pose_repr: 'relative',
          prediction,
          prediction_keys: ['action', 'action_pred'],
          action_pred_shape: [1, 16, 10],
          policy_tensor_shapes: { camera0_rgb: [1, 1, 3, 224, 224] },
          input_image_shape: [224, 224, 3],
        });
        await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
      });

      results.push({
        requestId,
        status: replay.status,
        reviewData: replay.reviewData,
        canExecuteAction: replay.canExecuteAction,
        playback: replay.playback,
      });
      act(() => root.unmount());
      container.remove();
    }

    expect(results[0]).toMatchObject({
      requestId: 'request-zero-target',
      reviewData: {
        arm_motion_classification: 'ZERO_WITHIN_TOLERANCE',
        safety_verdict: 'BLOCKED',
        review_status: 'NOT_READY_FOR_EXECUTION',
        persistence_status: 'saved',
      },
      canExecuteAction: false,
      playback: null,
    });
    expect(results[1]).toMatchObject({
      requestId: 'request-nonzero-target',
      status: 'review',
      reviewData: {
        arm_motion_classification: 'NONZERO_IK_TARGET',
        safety_verdict: 'READY_FOR_HUMAN_APPROVAL',
        review_status: 'WAITING_FOR_HUMAN_APPROVAL',
        persistence_status: 'saved',
      },
      canExecuteAction: true,
      playback: null,
    });
  });

  it('执行后只有实际状态保存成功才将 review 标记为 COMPLETED', async () => {
    const postedReviews: Record<string, unknown>[] = [];
    let resolveCompletionSave!: (response: {
      ok: boolean;
      status: number;
      json: () => Promise<{ saved?: boolean; path?: string; detail?: string }>;
    }) => void;
    vi.stubGlobal(
      'fetch',
      vi.fn((_url: string, init: RequestInit) => {
        const review = JSON.parse(init.body as string) as Record<string, unknown>;
        postedReviews.push(review);
        if (review.actual_after_action) {
          return new Promise((resolvePromise) => {
            resolveCompletionSave = resolvePromise;
          });
        }
        return Promise.resolve({
          ok: true,
          status: 200,
          json: async () => ({ saved: true, path: '/tmp/single-action-review.json' }),
        });
      }),
    );
    const harness = await createSingleActionReviewHarness('request-review-completion');

    await act(async () => {
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });
    expect(harness.replay.reviewData).toMatchObject({
      review_status: 'WAITING_FOR_HUMAN_APPROVAL',
      persistence_status: 'saved',
    });
    let executed = false;
    act(() => {
      executed = harness.replay.executePendingAction();
    });
    expect(executed).toBe(true);

    expect(harness.replay.playback).toBeTruthy();
    const token = harness.replay.playback?.token as number;
    act(() => harness.replay.completePlayback(token, createAfterActionSnapshot()));

    expect(postedReviews).toHaveLength(2);
    expect(postedReviews[1]).toMatchObject({
      request_id: 'request-review-completion',
      review_status: 'COMPLETED',
      actual_after_action: { observation_frame_index: 8 },
    });
    expect(harness.replay.reviewData).toMatchObject({
      review_status: 'WAITING_FOR_HUMAN_APPROVAL',
      persistence_status: 'saving',
      actual_after_action: { observation_frame_index: 8 },
    });
    expect(harness.replay.canStart).toBe(false);
    expect(
      harness.socket.sent
        .map((message) => JSON.parse(message).type)
        .filter((type) => type === 'ack'),
    ).toHaveLength(1);
    act(() => harness.replay.step());
    expect(FakeWebSocket.sockets).toHaveLength(1);

    await act(async () => {
      resolveCompletionSave({
        ok: true,
        status: 200,
        json: async () => ({ saved: true, path: '/tmp/single-action-review.json' }),
      });
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });

    expect(harness.replay.status).toBe('complete');
    expect(harness.replay.reviewData).toMatchObject({
      review_status: 'COMPLETED',
      persistence_status: 'saved',
      actual_after_action: { observation_frame_index: 8 },
    });
    expect(harness.replay.canStart).toBe(true);
    harness.setObservation({
      frame_index: 9,
      camera_frame_timestamp_ms: 150,
      camera0_rgb: 'BBBB',
      robot0_eef_pos: [cadStartPose.position, cadStartPose.position],
      robot0_eef_rot_axis_angle: [cadStartRotation, cadStartRotation],
      robot0_gripper_width: [[0.06], [0.06]],
    });
    act(() => harness.replay.step());

    expect(FakeWebSocket.sockets).toHaveLength(1);
    expect(harness.socket.readyState).toBe(FakeWebSocket.OPEN);
    expect(
      harness.socket.sent
        .map((message) => JSON.parse(message).type)
        .filter((messageType) => messageType === 'observation'),
    ).toHaveLength(2);
    expect(
      harness.socket.sent
        .map((message) => JSON.parse(message))
        .filter((message) => message.type === 'observation')
        .at(-1),
    ).toMatchObject({ episode_index: 0, frame_index: 9, camera0_rgb: 'BBBB' });
    expect(harness.replay.reviewData).toBeNull();
    expect(
      harness.socket.sent
        .map((message) => JSON.parse(message).type)
        .filter((messageType) => messageType === 'ack'),
    ).toHaveLength(1);
    expect(FakeWorker.workers).toHaveLength(1);

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('执行后状态快照缺失时不保存为 COMPLETED', async () => {
    const postedReviews: Record<string, unknown>[] = [];
    vi.stubGlobal(
      'fetch',
      vi.fn(async (_url: string, init: RequestInit) => {
        postedReviews.push(JSON.parse(init.body as string) as Record<string, unknown>);
        return {
          ok: true,
          status: 200,
          json: async () => ({ saved: true, path: '/tmp/single-action-review.json' }),
        };
      }),
    );
    const harness = await createSingleActionReviewHarness('request-missing-after-state');

    await act(async () => {
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });
    let executed = false;
    act(() => {
      executed = harness.replay.executePendingAction();
    });
    expect(executed).toBe(true);
    const token = harness.replay.playback?.token as number;

    act(() => harness.replay.completePlayback(token));

    expect(harness.replay.status).toBe('error');
    expect(harness.replay.error?.code).toBe('ACTUAL_AFTER_ACTION_MISSING');
    expect(harness.replay.reviewData).toMatchObject({
      review_status: 'WAITING_FOR_HUMAN_APPROVAL',
      persistence_status: 'saved',
    });
    expect(harness.replay.reviewData).not.toHaveProperty('actual_after_action');
    expect(postedReviews).toHaveLength(1);
    expect(harness.socket.readyState).toBe(FakeWebSocket.CLOSING);
    expect(
      harness.socket.sent
        .map((message) => JSON.parse(message).type)
        .filter((type) => type === 'ack'),
    ).toHaveLength(1);

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('执行后状态保存失败时保持未完成并报告错误', async () => {
    const postedReviews: Record<string, unknown>[] = [];
    vi.stubGlobal(
      'fetch',
      vi.fn(async (_url: string, init: RequestInit) => {
        const review = JSON.parse(init.body as string) as Record<string, unknown>;
        postedReviews.push(review);
        return review.actual_after_action
          ? {
              ok: false,
              status: 500,
              json: async () => ({ detail: 'disk write failed' }),
            }
          : {
              ok: true,
              status: 200,
              json: async () => ({ saved: true, path: '/tmp/single-action-review.json' }),
            };
      }),
    );
    const harness = await createSingleActionReviewHarness('request-save-failure');

    await act(async () => {
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });
    let executed = false;
    act(() => {
      executed = harness.replay.executePendingAction();
    });
    expect(executed).toBe(true);
    const token = harness.replay.playback?.token as number;

    act(() => harness.replay.completePlayback(token, createAfterActionSnapshot()));
    await act(async () => {
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });

    expect(postedReviews).toHaveLength(2);
    expect(postedReviews[1]).toMatchObject({
      review_status: 'COMPLETED',
      actual_after_action: { observation_frame_index: 8 },
    });
    expect(harness.replay.status).toBe('error');
    expect(harness.replay.reviewData).toMatchObject({
      review_status: 'WAITING_FOR_HUMAN_APPROVAL',
      persistence_status: 'failed',
      persistence_error: 'disk write failed',
      actual_after_action: { observation_frame_index: 8 },
    });
    expect(harness.socket.readyState).toBe(FakeWebSocket.CLOSING);

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('三步自动闭环每步重新推理，只执行 action 0，并在同一 episode 内停止', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-0123456789' });
    const harness = await createAutoLoopHarness();

    for (let step = 1; step <= 3; step += 1) {
      if (step === 1) {
        expect(harness.replay.startAutoLoop()).toBe(false);
        act(() => harness.replay.step());
        expect(
          harness.socket.sent.filter((message) => JSON.parse(message).type === 'observation'),
        ).toHaveLength(1);
      }
      await sendAutoLoopActionChunk(harness, `request-auto-${step}`);

      expect(harness.replay.reviewData).toMatchObject({
        safety_verdict: 'READY_FOR_HUMAN_APPROVAL',
        review_status: 'WAITING_FOR_HUMAN_APPROVAL',
      });
      expect(harness.replay.error).toBeNull();
      expect(harness.replay.status).toBe('playing');
      expect(harness.replay.canExecuteAction).toBe(false);
      expect(harness.replay.playback?.actions.map((action) => action.actionIndex)).toEqual([0]);
      expect(harness.replay.reviewData).toMatchObject({
        auto_loop_run_id: 'run-auto-0123456789',
        auto_loop_step: step,
        action_index: 0,
        selected_action_indexes: [0],
        safety_verdict: 'READY_FOR_HUMAN_APPROVAL',
        review_status: 'WAITING_FOR_HUMAN_APPROVAL',
        persistence_status: 'saved',
        single_action_execution_horizon: 1,
      });
      expect(
        (harness.replay.reviewData?.action_pred as number[][]).map((action) => action.length),
      ).toEqual(Array.from({ length: 16 }, () => 10));

      await completeAutoLoopPlayback(harness, {
        gripperOpening: [0.0607, 0.0612, 0.0618][step - 1],
      });
    }

    const sentObservations = harness.socket.sent
      .map((message) => JSON.parse(message))
      .filter((message) => message.type === 'observation');
    const completedReviews = harness.postedReviews.filter(
      (review) => review.review_status === 'COMPLETED',
    );
    expect(sentObservations).toHaveLength(3);
    expect(sentObservations.map((message) => message.episode_index)).toEqual([0, 0, 0]);
    expect(sentObservations.map((message) => message.frame_index)).toEqual([7, 9, 11]);
    expect(sentObservations.map((message) => message.camera_frame_index)).toEqual([10, 12, 14]);
    expect(sentObservations.map((message) => message.robot0_gripper_width.at(-1)?.[0])).toEqual([
      0.06, 0.0607, 0.0612,
    ]);
    expect(completedReviews).toHaveLength(3);
    expect(completedReviews.map((review) => review.request_id)).toEqual([
      'request-auto-1',
      'request-auto-2',
      'request-auto-3',
    ]);
    expect(completedReviews.every((review) => review.review_status === 'COMPLETED')).toBe(true);
    expect(completedReviews[1]).toMatchObject({
      current_gripper_opening_m: 0.0607,
      camera_frame_index: 12,
    });
    completedReviews.forEach((review) => {
      expect(review.action_start_pose).toEqual(review.observation_tcp_snapshot);
    });
    expect((completedReviews[1].observation_tcp_snapshot as Pose).position[0]).not.toBe(
      (completedReviews[0].observation_tcp_snapshot as Pose).position[0],
    );
    expect(completedReviews.map((review) => review.action_index)).toEqual([0, 0, 0]);
    expect(FakeWebSocket.sockets).toHaveLength(1);
    expect(FakeWorker.workers).toHaveLength(1);
    expect(
      harness.socket.sent.filter((message) => JSON.parse(message).type === 'ack'),
    ).toHaveLength(3);
    expect(harness.replay.autoLoop).toMatchObject({
      step: 3,
      maxSteps: 3,
      completedSteps: 3,
      phase: 'COMPLETED',
    });
    expect(harness.socket.readyState).toBe(FakeWebSocket.CLOSING);

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('第二步 inference 失败时不会启动第三步', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-inference-error' });
    const harness = await createAutoLoopHarness();
    await sendAutoLoopActionChunk(harness, 'request-auto-first');
    await completeAutoLoopPlayback(harness);

    await act(async () => {
      harness.socket.emitMessage({
        type: 'error',
        code: 'POLICY_FAILED',
        stage: 'inference',
        message: 'mock inference failure',
      });
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });

    const sentObservations = harness.socket.sent
      .map((message) => JSON.parse(message))
      .filter((message) => message.type === 'observation');
    expect(sentObservations).toHaveLength(2);
    expect(harness.replay.autoLoop.phase).toBe('ERROR');
    expect(harness.replay.playback).toBeNull();

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('用户在动作播放期间停止时，等待当前动作落盘后不再推理', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-user-stop' });
    const harness = await createAutoLoopHarness();
    await sendAutoLoopActionChunk(harness, 'request-auto-stop');

    act(() => harness.replay.stopAutoLoop());
    expect(harness.replay.autoLoop.phase).toBe('STOPPED');
    expect(harness.replay.playback).toBeTruthy();
    await completeAutoLoopPlayback(harness);

    const sentObservations = harness.socket.sent
      .map((message) => JSON.parse(message))
      .filter((message) => message.type === 'observation');
    expect(sentObservations).toHaveLength(1);
    expect(
      harness.postedReviews.filter((review) => review.review_status === 'COMPLETED'),
    ).toHaveLength(1);
    expect(harness.replay.autoLoop.phase).toBe('STOPPED');
    expect(harness.socket.readyState).toBe(FakeWebSocket.CLOSING);

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('没有动作正在播放时，停止立即关闭自动闭环', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-stop-before-action' });
    const harness = await createAutoLoopHarness();

    act(() => harness.replay.stopAutoLoop());

    expect(harness.replay.autoLoop.phase).toBe('STOPPED');
    expect(harness.replay.status).toBe('stopped');
    expect(harness.socket.readyState).toBe(FakeWebSocket.CLOSING);
    expect(
      harness.socket.sent.filter((message) => JSON.parse(message).type === 'observation'),
    ).toHaveLength(1);

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('执行后相机帧没有推进时不完成 review，也不启动下一步', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-stale-camera' });
    const harness = await createAutoLoopHarness();
    await sendAutoLoopActionChunk(harness, 'request-auto-stale-camera');

    await completeAutoLoopPlayback(harness, {
      cameraFrameIndex: harness.observation.camera_frame_index as number,
    });

    const sentObservations = harness.socket.sent
      .map((message) => JSON.parse(message))
      .filter((message) => message.type === 'observation');
    expect(sentObservations).toHaveLength(1);
    expect(
      harness.postedReviews.filter((review) => review.review_status === 'COMPLETED'),
    ).toHaveLength(0);
    expect(harness.replay.autoLoop.phase).toBe('ERROR');
    expect(harness.replay.error?.code).toBe('AUTO_LOOP_STALE_CAMERA_FRAME');

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('第二步 IK 失败时立即停止并保留已完成的第一步', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-ik-error' });
    const harness = await createAutoLoopHarness();
    await sendAutoLoopActionChunk(harness, 'request-auto-first');
    await completeAutoLoopPlayback(harness);
    FakeWorker.nextResponse = {
      type: 'error',
      error: { code: 'NO_IK_SOLUTION', stage: 'ik', message: 'mock IK failure' },
    };

    await sendAutoLoopActionChunk(harness, 'request-auto-second');

    const sentObservations = harness.socket.sent
      .map((message) => JSON.parse(message))
      .filter((message) => message.type === 'observation');
    expect(sentObservations).toHaveLength(2);
    expect(
      harness.postedReviews.filter((review) => review.review_status === 'COMPLETED'),
    ).toHaveLength(1);
    expect(harness.replay.autoLoop.phase).toBe('ERROR');
    expect(harness.replay.playback).toBeNull();

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('第二步 safety verdict 不合格时不会执行或启动第三步', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-safety-error' });
    const harness = await createAutoLoopHarness();
    await sendAutoLoopActionChunk(harness, 'request-auto-first');
    await completeAutoLoopPlayback(harness);

    await sendAutoLoopActionChunk(harness, 'request-auto-second', 0);

    const sentObservations = harness.socket.sent
      .map((message) => JSON.parse(message))
      .filter((message) => message.type === 'observation');
    expect(sentObservations).toHaveLength(2);
    expect(harness.replay.playback).toBeNull();
    expect(harness.replay.reviewData).toMatchObject({
      safety_verdict: 'BLOCKED',
      review_status: 'NOT_READY_FOR_EXECUTION',
      persistence_status: 'saved',
    });
    expect(harness.replay.autoLoop.phase).toBe('ERROR');

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('执行异常立即终止自动闭环并保留未完成 review', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-execution-error' });
    const harness = await createAutoLoopHarness();
    await sendAutoLoopActionChunk(harness, 'request-auto-execution-error');
    const token = harness.replay.playback?.token as number;

    act(() => harness.replay.failPlayback(token, new Error('mock playback failure')));

    expect(harness.replay.status).toBe('error');
    expect(harness.replay.error?.code).toBe('PLAYBACK_EXECUTION_FAILED');
    expect(harness.replay.autoLoop.phase).toBe('ERROR');
    expect(harness.replay.reviewData).toMatchObject({
      review_status: 'WAITING_FOR_HUMAN_APPROVAL',
      persistence_status: 'saved',
    });
    expect(
      harness.socket.sent.filter((message) => JSON.parse(message).type === 'observation'),
    ).toHaveLength(1);

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('下一轮 observation 未使用新相机帧时拒绝发送推理请求', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-capture-error' });
    const harness = await createAutoLoopHarness();
    await sendAutoLoopActionChunk(harness, 'request-auto-capture-first');
    vi.stubGlobal(
      'fetch',
      vi.fn(async (_url: string, init: RequestInit) => {
        const review = JSON.parse(init.body as string) as Record<string, unknown>;
        const actual = review.actual_after_action as
          | {
              observation_frame_index: number;
              camera_frame_index: number;
              camera_frame_timestamp_ms: number;
              actual_tcp: Pose;
              actual_gripper_opening_m: number;
            }
          | undefined;
        if (actual) {
          harness.setObservation(
            makeAutoLoopObservation(
              actual.observation_frame_index + 1,
              actual.camera_frame_index,
              actual.camera_frame_timestamp_ms,
              actual.actual_tcp,
              actual.actual_gripper_opening_m,
            ),
          );
        }
        return {
          ok: true,
          status: 200,
          json: async () => ({ saved: true, path: '/tmp/auto-loop-step.json' }),
        };
      }),
    );
    await completeAutoLoopPlayback(harness);

    expect(
      harness.socket.sent.filter((message) => JSON.parse(message).type === 'observation'),
    ).toHaveLength(1);
    expect(harness.replay.autoLoop.phase).toBe('ERROR');
    expect(harness.replay.error?.code).toBe('AUTO_LOOP_STALE_CAMERA_FRAME');

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('执行后 review 保存失败时不启动下一次 inference', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-persist-error' });
    const harness = await createAutoLoopHarness();
    vi.stubGlobal(
      'fetch',
      vi.fn(async (_url: string, init: RequestInit) => {
        const review = JSON.parse(init.body as string) as Record<string, unknown>;
        return review.actual_after_action
          ? {
              ok: false,
              status: 500,
              json: async () => ({ detail: 'disk write failed' }),
            }
          : {
              ok: true,
              status: 200,
              json: async () => ({ saved: true, path: '/tmp/auto-loop-step.json' }),
            };
      }),
    );
    await sendAutoLoopActionChunk(harness, 'request-auto-persist-error');
    await completeAutoLoopPlayback(harness);

    expect(
      harness.socket.sent.filter((message) => JSON.parse(message).type === 'observation'),
    ).toHaveLength(1);
    expect(harness.replay.autoLoop.phase).toBe('ERROR');
    expect(harness.replay.reviewData).toMatchObject({
      review_status: 'WAITING_FOR_HUMAN_APPROVAL',
      persistence_status: 'failed',
      actual_after_action: { observation_frame_index: 8 },
    });

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('下一步必须等待上一动作 COMPLETED review 落盘', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-save-gate' });
    const harness = await createAutoLoopHarness();
    let resolveCompletedSave!: (response: {
      ok: boolean;
      status: number;
      json: () => Promise<{ saved?: boolean; path?: string }>;
    }) => void;
    vi.stubGlobal(
      'fetch',
      vi.fn(async (_url: string, init: RequestInit) => {
        const review = JSON.parse(init.body as string) as Record<string, unknown>;
        if (review.review_status === 'COMPLETED') {
          return new Promise((resolvePromise) => {
            resolveCompletedSave = resolvePromise;
          });
        }
        return {
          ok: true,
          status: 200,
          json: async () => ({ saved: true, path: '/tmp/auto-loop-step.json' }),
        };
      }),
    );
    await sendAutoLoopActionChunk(harness, 'request-auto-save-gate');
    const waitingReview = harness.replay.reviewData as Record<string, unknown>;
    const targetTcp = waitingReview.target_tcp as Pose;
    const targetJoints = waitingReview.target_joints as Record<string, number>;
    const targetGripper = waitingReview.target_gripper_opening_m as number;
    const postActionObservation = makeAutoLoopObservation(
      harness.observation.frame_index + 1,
      (harness.observation.camera_frame_index as number) + 1,
      harness.observation.camera_frame_timestamp_ms + 10,
      targetTcp,
      targetGripper,
    );
    const token = harness.replay.playback?.token as number;
    await act(async () => {
      harness.replay.completePlayback(token, {
        joint_values: targetJoints,
        tcp_pose: targetTcp,
        gripper_opening_m: targetGripper,
        observation: postActionObservation,
      });
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });

    act(() => harness.replay.step());
    expect(
      harness.socket.sent.filter((message) => JSON.parse(message).type === 'observation'),
    ).toHaveLength(1);
    expect(harness.replay.canStart).toBe(false);
    const nextObservation = makeAutoLoopObservation(
      postActionObservation.frame_index + 1,
      (postActionObservation.camera_frame_index as number) + 1,
      postActionObservation.camera_frame_timestamp_ms + 10,
      targetTcp,
      targetGripper,
    );
    act(() => harness.setObservation(nextObservation));

    await act(async () => {
      resolveCompletedSave({
        ok: true,
        status: 200,
        json: async () => ({ saved: true, path: '/tmp/auto-loop-step.json' }),
      });
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });

    const sentObservations = harness.socket.sent
      .map((message) => JSON.parse(message))
      .filter((message) => message.type === 'observation');
    expect(sentObservations).toHaveLength(2);
    expect(sentObservations[1]).toMatchObject({
      frame_index: nextObservation.frame_index,
      camera_frame_index: nextObservation.camera_frame_index,
      episode_index: 0,
    });
    expect(harness.replay.autoLoop.step).toBe(2);

    act(() => harness.root.unmount());
    harness.container.remove();
  });

  it('response frame mismatch 终止自动闭环', async () => {
    vi.stubGlobal('crypto', { randomUUID: () => 'run-auto-frame-mismatch' });
    const harness = await createAutoLoopHarness();

    await act(async () => {
      harness.socket.emitMessage({
        type: 'action_chunk',
        request_id: 'request-auto-wrong-frame',
        episode_index: 0,
        frame_index: harness.observation.frame_index + 1,
        last_chunk: false,
        actions: Array.from({ length: 16 }, () => [0.001, 0, 0, 0, 0, 0, 0.06]),
        action_pose_repr: 'relative',
        prediction: Array.from({ length: 16 }, () => [0.001, 0, 0, 1, 0, 0, 0, 1, 0, 0.06]),
        prediction_keys: ['action', 'action_pred'],
        action_pred_shape: [1, 16, 10],
        policy_tensor_shapes: { camera0_rgb: [1, 1, 3, 224, 224] },
        input_image_shape: [224, 224, 3],
      });
      await new Promise((resolvePromise) => setTimeout(resolvePromise, 0));
    });

    expect(harness.replay.autoLoop.phase).toBe('ERROR');
    expect(harness.replay.error?.code).toBe('ACTION_CONTEXT_MISMATCH');
    expect(harness.replay.playback).toBeNull();

    act(() => harness.root.unmount());
    harness.container.remove();
  });
});
