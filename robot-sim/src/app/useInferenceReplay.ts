import { useEffect, useRef, useState } from 'react';
import type { JointValues, Pose, RobotDescription } from '../core/types';
import { publishDebugEvent } from './debugBus';
import { isActionChunk } from './inferenceProtocol';
import type {
  ActionChunk,
  IKActionDebugRecord,
  IkWorkerRequest,
  IkWorkerResponse,
  InferenceError,
  InferenceStatus,
  PlaybackChunk,
  InferenceObservation,
} from './inferenceProtocol';

const INFERENCE_SOCKET_URL = 'ws://localhost:8000/ws/inference';
const IK_TIMEOUT_MS = 1000;

const isIkTraceEnabled = (): boolean =>
  new URLSearchParams(window.location.search).get('ikTrace') === '1';

interface ReplayContext {
  description: RobotDescription | null;
  jointValues: JointValues;
  tcpPose: Pose | null;
  getObservation: () => InferenceObservation | null;
}

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null;

const selectJointAngles = (description: RobotDescription, jointValues: JointValues): JointValues =>
  Object.fromEntries(
    description.joints
      .filter((joint) => joint.type === 'revolute' || joint.type === 'continuous')
      .map((joint) => [joint.name, jointValues[joint.name]]),
  );

const parseServerError = (value: unknown): InferenceError => {
  const candidate = isRecord(value) ? value : {};
  return {
    ...candidate,
    code: typeof candidate.code === 'string' ? candidate.code : 'SERVER_ERROR',
    stage: typeof candidate.stage === 'string' ? candidate.stage : 'server',
    message: typeof candidate.message === 'string' ? candidate.message : '后端推理服务返回错误。',
  };
};

export const useInferenceReplay = ({
  description,
  jointValues,
  tcpPose,
  getObservation,
}: ReplayContext) => {
  const [status, setStatus] = useState<InferenceStatus>('idle');
  const [progress, setProgress] = useState('');
  const [error, setError] = useState<InferenceError | null>(null);
  const [playback, setPlayback] = useState<PlaybackChunk | null>(null);
  const statusRef = useRef(status);
  const contextRef = useRef<ReplayContext>({ description, jointValues, tcpPose, getObservation });
  const socketRef = useRef<WebSocket | null>(null);
  const workerRef = useRef<Worker | null>(null);
  const workerTimeoutRef = useRef<number | null>(null);
  const activeChunkRef = useRef<ActionChunk | null>(null);
  const activeActionIndexRef = useRef<number | null>(null);
  const activeJointAnglesRef = useRef<JointValues[]>([]);
  const playbackTokenRef = useRef(0);
  const mountedRef = useRef(true);

  contextRef.current = { description, jointValues, tcpPose, getObservation };

  const updateStatus = (nextStatus: InferenceStatus) => {
    statusRef.current = nextStatus;
    if (mountedRef.current) setStatus(nextStatus);
  };

  const clearWorker = (terminate = true) => {
    if (workerTimeoutRef.current !== null) {
      window.clearTimeout(workerTimeoutRef.current);
      workerTimeoutRef.current = null;
    }
    if (terminate) {
      workerRef.current?.terminate();
      workerRef.current = null;
    }
  };

  const closeSocket = (code: number, reason: string) => {
    const socket = socketRef.current;
    socketRef.current = null;
    if (socket && socket.readyState < WebSocket.CLOSING) socket.close(code, reason);
  };

  const sendStop = (jointAngles: JointValues[] = []) => {
    const socket = socketRef.current;
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({ type: 'stop', joint_angles: jointAngles }));
    }
  };

  const fail = (nextError: InferenceError, notifyServer: boolean) => {
    const jointValues = isRecord(nextError.joint_values)
      ? [
          selectJointAngles(
            contextRef.current.description as RobotDescription,
            nextError.joint_values as JointValues,
          ),
        ]
      : activeJointAnglesRef.current;
    clearWorker();
    activeChunkRef.current = null;
    activeJointAnglesRef.current = [];
    if (mountedRef.current) {
      setPlayback(null);
      setError(nextError);
      setProgress('');
    }
    updateStatus('error');
    publishDebugEvent('inference:error', nextError);
    if (notifyServer) {
      sendStop(jointValues);
    } else {
      closeSocket(4000, 'inference failed');
    }
  };

  const sendObservation = (socket: WebSocket) => {
    const observation = contextRef.current.getObservation();
    if (!observation) {
      fail(
        { code: 'CAMERA_NOT_READY', stage: 'observation', message: '腕部相机尚未生成有效图像。' },
        true,
      );
      return;
    }
    socket.send(JSON.stringify({ type: 'observation', ...observation }));
    if (mountedRef.current) setProgress('观测已发送，正在推理下一步动作');
    updateStatus('inferring');
    publishDebugEvent('inference:observation_sent', {
      frame_index: observation.frame_index,
      image_bytes: Math.floor((observation.camera0_rgb.length * 3) / 4),
      image_shape: [224, 224, 3],
      robot0_eef_pos_m: observation.robot0_eef_pos,
      robot0_eef_rot_axis_angle_rad: observation.robot0_eef_rot_axis_angle,
      robot0_gripper_width_m: observation.robot0_gripper_width,
    });
  };

  const handleActionChunk = (chunk: ActionChunk) => {
    const context = contextRef.current;
    if (!context.description || !context.tcpPose) {
      fail(
        {
          code: 'ROBOT_NOT_READY',
          stage: 'ik',
          message: '收到动作时机械臂模型或 TCP 位姿尚未就绪。',
          episode_index: chunk.episode_index,
          frame_index: chunk.frame_index,
        },
        true,
      );
      return;
    }
    if (activeChunkRef.current) {
      fail(
        {
          code: 'UNACKNOWLEDGED_CHUNK',
          stage: 'protocol',
          message: '后端在上一动作块确认前发送了新动作块。',
          episode_index: chunk.episode_index,
          frame_index: chunk.frame_index,
        },
        true,
      );
      return;
    }

    activeChunkRef.current = chunk;
    activeActionIndexRef.current = null;
    activeJointAnglesRef.current = [];
    if (mountedRef.current) {
      setError(null);
      setProgress('正在启动 IK Worker');
    }
    updateStatus('computing');
    publishDebugEvent('inference:action_chunk', {
      episode_index: chunk.episode_index,
      frame_index: chunk.frame_index,
      last_chunk: chunk.last_chunk,
      action_count: chunk.actions.length,
      action_layout: ['dx', 'dy', 'dz', 'rx', 'ry', 'rz', 'gripper_width'],
      action_units: ['m', 'm', 'm', 'rad', 'rad', 'rad', 'm'],
      action_pose_repr: chunk.action_pose_repr,
      actions: chunk.actions,
    });

    const worker =
      workerRef.current ??
      new Worker(new URL('../workers/umiIk.worker.ts', import.meta.url), {
        type: 'module',
      });
    workerRef.current = worker;
    workerTimeoutRef.current = window.setTimeout(() => {
      if (workerRef.current !== worker) return;
      const timeoutError: InferenceError = {
        code: 'IK_TIMEOUT',
        stage: 'ik',
        message: 'IK Worker 在 ' + IK_TIMEOUT_MS + ' ms 内未完成动作块求解。',
        episode_index: chunk.episode_index,
        frame_index: chunk.frame_index,
        ...(activeActionIndexRef.current === null
          ? {}
          : { action_index: activeActionIndexRef.current }),
      };
      fail(timeoutError, true);
    }, IK_TIMEOUT_MS);

    worker.onmessage = (event: MessageEvent<IkWorkerResponse>) => {
      if (workerRef.current !== worker || statusRef.current === 'stopped') return;
      const response = event.data;
      if (response.type === 'progress') {
        activeActionIndexRef.current = response.actionIndex;
        if (mountedRef.current) {
          setProgress('正在求解 IK：' + (response.actionIndex + 1) + '/' + response.total);
        }
        return;
      }
      if (response.type === 'ik_record') {
        const socket = socketRef.current;
        if (socket?.readyState === WebSocket.OPEN) {
          const record: IKActionDebugRecord = response.record;
          socket.send(JSON.stringify({ type: 'ik_trace', record }));
          publishDebugEvent('inference:ik_trace', {
            request_id: record.request_id,
            episode_index: record.episode_index,
            frame_index: record.frame_index,
            action_index: record.action_index,
            status: record.status,
            trace_level: record.trace_level,
          });
        }
        return;
      }
      if (response.type === 'error') {
        fail(
          {
            ...response.error,
            episode_index: chunk.episode_index,
            frame_index: chunk.frame_index,
          },
          true,
        );
        return;
      }

      clearWorker(false);
      activeJointAnglesRef.current = response.actions.map((action) =>
        selectJointAngles(context.description as RobotDescription, action.jointValues),
      );
      const nextPlayback: PlaybackChunk = {
        token: playbackTokenRef.current + 1,
        episodeIndex: chunk.episode_index,
        frameIndex: chunk.frame_index,
        lastChunk: chunk.last_chunk,
        actions: response.actions,
      };
      playbackTokenRef.current = nextPlayback.token;
      if (mountedRef.current) {
        setPlayback(nextPlayback);
        setProgress('动作块 ' + chunk.frame_index + '：' + response.actions.length + ' 个目标');
      }
      updateStatus('playing');
      publishDebugEvent('inference:ik_solved', {
        episode_index: chunk.episode_index,
        frame_index: chunk.frame_index,
        action_count: response.actions.length,
        actions: response.actions,
      });
    };

    worker.onerror = (event: ErrorEvent) => {
      fail(
        {
          code: 'IK_WORKER_ERROR',
          stage: 'ik',
          message: event.message || 'IK Worker 执行失败。',
          episode_index: chunk.episode_index,
          frame_index: chunk.frame_index,
          ...(activeActionIndexRef.current === null
            ? {}
            : { action_index: activeActionIndexRef.current }),
        },
        true,
      );
    };
    worker.onmessageerror = () => {
      fail(
        {
          code: 'IK_WORKER_MESSAGE_ERROR',
          stage: 'ik',
          message: '无法读取 IK Worker 的返回消息。',
          episode_index: chunk.episode_index,
          frame_index: chunk.frame_index,
          ...(activeActionIndexRef.current === null
            ? {}
            : { action_index: activeActionIndexRef.current }),
        },
        true,
      );
    };

    const request: IkWorkerRequest = {
      type: 'solve',
      request_id: chunk.request_id,
      episode_index: chunk.episode_index,
      frame_index: chunk.frame_index,
      description: context.description,
      startPose: context.tcpPose,
      initialJointValues: context.jointValues,
      actions: chunk.actions,
      action_pose_repr: chunk.action_pose_repr,
      traceAllIterations: isIkTraceEnabled(),
    };
    worker.postMessage(request);
  };

  const start = () => {
    if (
      statusRef.current === 'connecting' ||
      statusRef.current === 'loading' ||
      statusRef.current === 'inferring' ||
      statusRef.current === 'computing' ||
      statusRef.current === 'playing' ||
      statusRef.current === 'waiting'
    ) {
      return;
    }
    const context = contextRef.current;
    if (!context.description || !context.tcpPose) {
      fail(
        {
          code: 'ROBOT_NOT_READY',
          stage: 'connection',
          message: '机械臂模型或 TCP 位姿尚未就绪。',
        },
        false,
      );
      return;
    }

    if (mountedRef.current) {
      setError(null);
      setProgress('正在连接推理服务');
      setPlayback(null);
    }
    activeChunkRef.current = null;
    activeJointAnglesRef.current = [];
    updateStatus('connecting');
    publishDebugEvent('inference:connect', { url: INFERENCE_SOCKET_URL });

    try {
      clearWorker();
      const worker = new Worker(new URL('../workers/umiIk.worker.ts', import.meta.url), {
        type: 'module',
      });
      workerRef.current = worker;
      worker.onerror = (event: ErrorEvent) => {
        if (workerRef.current !== worker) return;
        const chunk = activeChunkRef.current;
        fail(
          {
            code: 'IK_WORKER_ERROR',
            stage: 'ik',
            message: event.message || 'IK Worker 启动失败。',
            ...(chunk
              ? { episode_index: chunk.episode_index, frame_index: chunk.frame_index }
              : {}),
          },
          socketRef.current?.readyState === WebSocket.OPEN,
        );
      };
      const socket = new WebSocket(INFERENCE_SOCKET_URL);
      socketRef.current = socket;
      socket.onopen = () => {
        publishDebugEvent('inference:connected', { url: INFERENCE_SOCKET_URL });
      };
      socket.onmessage = (event: MessageEvent<string>) => {
        if (socketRef.current !== socket || statusRef.current === 'stopped') return;
        let message: unknown;
        try {
          message = JSON.parse(event.data);
        } catch {
          fail(
            {
              code: 'INVALID_SERVER_MESSAGE',
              stage: 'protocol',
              message: '推理服务发送了无效 JSON。',
            },
            true,
          );
          return;
        }
        if (!isRecord(message) || typeof message.type !== 'string') {
          fail(
            {
              code: 'INVALID_SERVER_MESSAGE',
              stage: 'protocol',
              message: '推理服务消息缺少 type 字段。',
            },
            true,
          );
          return;
        }
        if (
          message.type === 'status' &&
          (message.status === 'loading' || message.state === 'loading')
        ) {
          updateStatus('loading');
          if (mountedRef.current) setProgress('后端正在加载官方权重');
          publishDebugEvent('inference:status', message);
          return;
        }
        if (message.type === 'status' && message.state === 'ready') {
          if (mountedRef.current) setProgress('官方权重已就绪，开始单步推理');
          sendObservation(socket);
          return;
        }
        if (message.type === 'action_chunk') {
          if (!isActionChunk(message)) {
            fail(
              {
                code: 'INVALID_ACTION_CHUNK',
                stage: 'protocol',
                message: '动作块字段无效，或动作不是 7 个有限数值。',
              },
              true,
            );
            return;
          }
          handleActionChunk(message);
          return;
        }
        if (message.type === 'complete') {
          clearWorker();
          activeChunkRef.current = null;
          activeActionIndexRef.current = null;
          const wasStopped = message.state === 'stopped';
          if (mountedRef.current) {
            setPlayback(null);
            setProgress(wasStopped ? '推理已停止' : '完整回放片段已完成');
          }
          if (statusRef.current !== 'error') {
            updateStatus(wasStopped ? 'stopped' : 'complete');
          }
          publishDebugEvent('inference:complete', message);
          closeSocket(1000, wasStopped ? 'inference stopped' : 'inference complete');
          return;
        }
        if (message.type === 'error') {
          fail(parseServerError(message), false);
          return;
        }
        fail(
          {
            code: 'UNKNOWN_SERVER_EVENT',
            stage: 'protocol',
            message: '推理服务发送了未知事件：' + message.type,
            detail: message,
          },
          true,
        );
      };
      socket.onerror = () => {
        if (socketRef.current !== socket) return;
        fail(
          {
            code: 'WEBSOCKET_ERROR',
            stage: 'connection',
            message: '无法连接推理服务，请检查后端是否运行。',
          },
          false,
        );
      };
      socket.onclose = (event: CloseEvent) => {
        if (socketRef.current !== socket) return;
        socketRef.current = null;
        if (
          statusRef.current === 'connecting' ||
          statusRef.current === 'loading' ||
          statusRef.current === 'inferring' ||
          statusRef.current === 'computing' ||
          statusRef.current === 'playing' ||
          statusRef.current === 'waiting'
        ) {
          fail(
            {
              code: 'WEBSOCKET_CLOSED',
              stage: 'connection',
              message:
                '推理连接已关闭（' + event.code + '）：' + (event.reason || '未收到结束事件'),
            },
            false,
          );
        }
      };
    } catch (caught) {
      fail(
        {
          code: 'WEBSOCKET_CREATE_FAILED',
          stage: 'connection',
          message: caught instanceof Error ? caught.message : String(caught),
        },
        false,
      );
    }
  };

  const step = () => {
    if (statusRef.current !== 'waiting') {
      start();
      return;
    }

    const socket = socketRef.current;
    if (socket?.readyState !== WebSocket.OPEN) {
      fail(
        {
          code: 'WEBSOCKET_NOT_OPEN',
          stage: 'connection',
          message: '推理连接未就绪，无法发送下一步观测。',
        },
        false,
      );
      return;
    }
    sendObservation(socket);
  };

  const stop = () => {
    if (statusRef.current === 'idle' || statusRef.current === 'complete') return;
    sendStop(activeJointAnglesRef.current);
    activeJointAnglesRef.current = [];
    clearWorker();
    activeChunkRef.current = null;
    activeActionIndexRef.current = null;
    if (mountedRef.current) {
      setPlayback(null);
      setProgress('推理已停止');
    }
    updateStatus('stopped');
    publishDebugEvent('inference:stop', { reason: 'user' });
    closeSocket(1000, 'user stopped');
  };

  const completePlayback = (token: number) => {
    const chunk = activeChunkRef.current;
    const socket = socketRef.current;
    if (!chunk || token !== playbackTokenRef.current || socket?.readyState !== WebSocket.OPEN) {
      return;
    }
    socket.send(
      JSON.stringify({
        type: 'ack',
        frame_index: chunk.frame_index,
        joint_angles: activeJointAnglesRef.current,
      }),
    );
    activeJointAnglesRef.current = [];
    activeChunkRef.current = null;
    if (mountedRef.current) {
      setPlayback(null);
      setProgress('单步动作已执行，点击“推理一步”继续');
    }
    updateStatus('waiting');
    publishDebugEvent('inference:ack', {
      episode_index: chunk.episode_index,
      frame_index: chunk.frame_index,
    });
  };

  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
      if (workerTimeoutRef.current !== null) {
        window.clearTimeout(workerTimeoutRef.current);
        workerTimeoutRef.current = null;
      }
      workerRef.current?.terminate();
      workerRef.current = null;
      const socket = socketRef.current;
      if (socket?.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ type: 'stop' }));
      }
      socketRef.current = null;
      if (socket && socket.readyState < WebSocket.CLOSING) {
        socket.close(1000, 'view unmounted');
      }
    };
  }, []);

  const isLocked =
    status === 'connecting' ||
    status === 'loading' ||
    status === 'inferring' ||
    status === 'computing' ||
    status === 'playing' ||
    status === 'waiting';

  return { status, progress, error, playback, isLocked, step, stop, completePlayback };
};
