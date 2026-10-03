// 验证完整预测与执行前缀分离，并保持单步人工确认。
// @vitest-environment jsdom
import { act, createElement } from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import type {
  IKActionDebugRecord,
  IkWorkerRequest,
  IkWorkerResponse,
} from '../src/app/inferenceProtocol';
import { selectExecutionActions } from '../src/app/executionHorizon';
import { useInferenceReplay } from '../src/app/useInferenceReplay';
import type { InferenceObservation } from '../src/app/inferenceProtocol';
import { Kinematics } from '../src/core/Kinematics';
import { parseUrdf } from '../src/utils/urdfParser';
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

  onmessage: ((event: MessageEvent<IkWorkerResponse>) => void) | null = null;

  onerror: ((event: ErrorEvent) => void) | null = null;

  onmessageerror: ((event: MessageEvent) => void) | null = null;

  request: IkWorkerRequest | null = null;

  messages: IkWorkerResponse[] = [];

  constructor() {
    FakeWorker.workers.push(this);
  }

  postMessage(request: IkWorkerRequest) {
    this.request = request;
    runIkSolve(request, (message) => {
      this.messages.push(message);
      this.onmessage?.({ data: message } as MessageEvent<IkWorkerResponse>);
    });
  }

  terminate() {}
}

describe('execution horizon', () => {
  beforeEach(() => {
    FakeWebSocket.sockets = [];
    FakeWorker.workers = [];
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
      frame_index: 0,
      camera0_rgb: 'AAAA',
      robot0_eef_pos: [startPose.position, startPose.position],
      robot0_eef_rot_axis_angle: [
        [0, 0, 0],
        [0, 0, 0],
      ],
      robot0_gripper_width: [[0], [0]],
    };
    const context = {
      description,
      jointValues: seed,
      tcpPose: startPose,
      getObservation: () => observation,
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
        episode_index: 2,
        frame_index: 7,
        last_chunk: false,
        actions: predictedActions,
        action_pose_repr: 'relative',
      }),
    );

    const worker = FakeWorker.workers[0];
    expect(worker.request).toMatchObject({
      request_id: 'request-horizon',
      frame_index: 7,
      predictionCount: 16,
      configuredExecutionHorizon: 6,
      executionCount: 6,
      selectedActionIndexes: [0, 1, 2, 3, 4, 5],
      action_pose_repr: 'relative',
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
});
