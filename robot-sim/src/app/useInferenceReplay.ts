import { useEffect, useRef, useState } from 'react';
import type { JointValues, Pose, RobotDescription } from '../core/types';
import { findRobotConfig, type RobotConfig, type RobotModelId } from './config';
import { publishDebugEvent } from './debugBus';
import { EXECUTION_HORIZON, selectExecutionActions } from './executionHorizon';
import { isActionChunk, isPolicyInferenceOnlyEnabled } from './inferenceProtocol';
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
import { poseErrorRotation, vectorLength } from '../utils/math';

const INFERENCE_SOCKET_URL = 'ws://localhost:8000/ws/inference';
const IK_TIMEOUT_MS = 1000;

const isIkTraceEnabled = (): boolean =>
  new URLSearchParams(window.location.search).get('ikTrace') === '1';

interface ReplayContext {
  robotId: RobotModelId;
  robotConfig: RobotConfig;
  loadedRobotId: RobotModelId | null;
  loadedRobotUrdfFile: string | null;
  description: RobotDescription | null;
  jointValues: JointValues;
  tcpPose: Pose | null;
  getObservation: () => InferenceObservation | null;
}

interface PlaybackSnapshot {
  joint_values: JointValues;
  tcp_pose: Pose;
  gripper_opening_m: number | null;
  observation: InferenceObservation | null;
}

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null;

const selectJointAngles = (description: RobotDescription, jointValues: JointValues): JointValues =>
  Object.fromEntries(
    description.joints
      .filter((joint) => joint.type === 'revolute' || joint.type === 'continuous')
      .map((joint) => [joint.name, jointValues[joint.name]]),
  );

const UR5_CAD_CONFIG = findRobotConfig('UR5_CAD');
const SINGLE_ACTION_VALIDATION_ERROR = 'Single-action validation is only available for UR5_CAD.';
const ACTION_DIMENSION_SEMANTICS = [
  { index: 0, name: 'position_x', unit: 'm' },
  { index: 1, name: 'position_y', unit: 'm' },
  { index: 2, name: 'position_z', unit: 'm' },
  { index: 3, name: 'rotation_6d_r00', unit: 'dimensionless' },
  { index: 4, name: 'rotation_6d_r01', unit: 'dimensionless' },
  { index: 5, name: 'rotation_6d_r02', unit: 'dimensionless' },
  { index: 6, name: 'rotation_6d_r10', unit: 'dimensionless' },
  { index: 7, name: 'rotation_6d_r11', unit: 'dimensionless' },
  { index: 8, name: 'rotation_6d_r12', unit: 'dimensionless' },
  { index: 9, name: 'gripper_width', unit: 'm' },
] as const;

const isUr5CadSelection = (context: ReplayContext): boolean =>
  context.robotId === 'UR5_CAD' &&
  context.robotConfig.id === 'UR5_CAD' &&
  context.robotConfig.file === UR5_CAD_CONFIG.file;

const isUr5CadContext = (context: ReplayContext): boolean =>
  isUr5CadSelection(context) &&
  context.loadedRobotId === 'UR5_CAD' &&
  context.loadedRobotUrdfFile === UR5_CAD_CONFIG.file &&
  context.description !== null;

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
  robotId,
  robotConfig,
  loadedRobotId,
  loadedRobotUrdfFile,
  description,
  jointValues,
  tcpPose,
  getObservation,
  singleActionDebugEnabled,
}: ReplayContext & { singleActionDebugEnabled: boolean }) => {
  const policyInferenceOnly = isPolicyInferenceOnlyEnabled() && !singleActionDebugEnabled;
  const [status, setStatus] = useState<InferenceStatus>('idle');
  const [progress, setProgress] = useState('');
  const [error, setError] = useState<InferenceError | null>(null);
  const [playback, setPlayback] = useState<PlaybackChunk | null>(null);
  const [reviewData, setReviewData] = useState<Record<string, unknown> | null>(null);
  const reviewDataRef = useRef<Record<string, unknown> | null>(null);
  const statusRef = useRef(status);
  const contextRef = useRef<ReplayContext>({
    robotId,
    robotConfig,
    loadedRobotId,
    loadedRobotUrdfFile,
    description,
    jointValues,
    tcpPose,
    getObservation,
  });
  const socketRef = useRef<WebSocket | null>(null);
  const workerRef = useRef<Worker | null>(null);
  const workerTimeoutRef = useRef<number | null>(null);
  const activeChunkRef = useRef<ActionChunk | null>(null);
  const pendingPlaybackRef = useRef<PlaybackChunk | null>(null);
  const latestIkRecordRef = useRef<IKActionDebugRecord | null>(null);
  const lastObservationRef = useRef<InferenceObservation | null>(null);
  const activeActionIndexRef = useRef<number | null>(null);
  const activeJointAnglesRef = useRef<JointValues[]>([]);
  const playbackTokenRef = useRef(0);
  const mountedRef = useRef(true);

  contextRef.current = {
    robotId,
    robotConfig,
    loadedRobotId,
    loadedRobotUrdfFile,
    description,
    jointValues,
    tcpPose,
    getObservation,
  };

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

  const fail = (nextError: InferenceError, notifyServer: boolean, preserveReviewData = false) => {
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
    pendingPlaybackRef.current = null;
    activeJointAnglesRef.current = [];
    if (mountedRef.current) {
      setPlayback(null);
      if (!preserveReviewData) {
        reviewDataRef.current = null;
        setReviewData(null);
      }
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

  const persistReviewData = async (reviewData: Record<string, unknown>) => {
    try {
      const response = await fetch('http://localhost:8000/api/single-action-review', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(reviewData),
      });
      let result: unknown = null;
      try {
        result = await response.json();
      } catch {
        result = null;
      }
      if (
        !response.ok ||
        !isRecord(result) ||
        result.saved !== true ||
        typeof result.path !== 'string'
      ) {
        const detail = isRecord(result) && typeof result.detail === 'string' ? result.detail : '';
        throw new Error(detail || `HTTP ${response.status}`);
      }
      if (reviewDataRef.current !== reviewData) return;
      const savedReview = {
        ...reviewData,
        persistence_status: 'saved',
        persistence_path: result.path,
      };
      reviewDataRef.current = savedReview;
      if (mountedRef.current) {
        setReviewData(savedReview);
        setError(null);
        setProgress('审阅 JSON 已保存，请确认后再执行单动作');
      }
    } catch (caught) {
      if (reviewDataRef.current !== reviewData) return;
      const message = caught instanceof Error ? caught.message : String(caught);
      const failedReview = {
        ...reviewData,
        persistence_status: 'failed',
        persistence_error: message,
      };
      reviewDataRef.current = failedReview;
      if (mountedRef.current) {
        setReviewData(failedReview);
        setError({
          code: 'REVIEW_JSON_SAVE_FAILED',
          stage: 'review',
          message: `Review JSON save failed: ${message}`,
        });
        setProgress('审阅 JSON 未保存，已禁止执行动作');
      }
    }
  };

  const saveFailedIkReview = (chunk: ActionChunk, error: InferenceError): boolean => {
    const context = contextRef.current;
    if (!singleActionDebugEnabled || !isUr5CadContext(context)) return false;
    const ikRecord = latestIkRecordRef.current;
    const actionPrediction = chunk.prediction[0] ?? [];
    const robotAction = chunk.actions[0] ?? [];
    const currentTcp = ikRecord?.start_pose ?? context.tcpPose;
    const targetTcp =
      ikRecord?.target_pose ??
      (isRecord(error.target_pose) ? (error.target_pose as unknown as Pose) : null);
    const seedJoints = ikRecord?.seed_joint_values ?? context.jointValues;
    const targetJoints = ikRecord?.final_joint_values ?? error.joint_values ?? null;
    const armJoints =
      context.description?.joints.filter(
        (joint) => joint.type === 'revolute' || joint.type === 'continuous',
      ) ?? [];
    const jointDeltas = Object.fromEntries(
      armJoints.flatMap((joint) => {
        const targetValue = targetJoints?.[joint.name];
        return typeof targetValue === 'number'
          ? [[joint.name, targetValue - (seedJoints[joint.name] ?? 0)]]
          : [];
      }),
    );
    const jointLimits = Object.fromEntries(
      armJoints.map((joint) => [
        joint.name,
        { lower: joint.limit?.lower ?? null, upper: joint.limit?.upper ?? null },
      ]),
    );
    const gripperJoints =
      context.description?.joints.filter(
        (joint) =>
          joint.type === 'prismatic' && /gripper|finger/iu.test(joint.name + ' ' + joint.child),
      ) ?? [];
    const gripperJointLimits = Object.fromEntries(
      gripperJoints.map((joint) => [
        joint.name,
        { lower: joint.limit?.lower ?? 0, upper: joint.limit?.upper ?? joint.limit?.lower ?? 0 },
      ]),
    );
    const maximumGripperOpening = gripperJoints.reduce((total, joint) => {
      const lower = joint.limit?.lower ?? 0;
      const upper = joint.limit?.upper ?? lower;
      return total + Math.abs(upper - lower);
    }, 0);
    const residual = isRecord(error.residual) ? error.residual : {};
    const targetTcpNumbers = targetTcp ? [...targetTcp.position, ...targetTcp.orientation] : [];
    const currentGripperOpening =
      lastObservationRef.current?.robot0_gripper_width.at(-1)?.[0] ?? null;
    const targetGripperOpening =
      typeof robotAction[6] === 'number'
        ? Math.max(0, Math.min(maximumGripperOpening, robotAction[6]))
        : null;
    const finiteValues = [
      ...actionPrediction,
      ...robotAction,
      ...Object.values(seedJoints),
      ...Object.values(targetJoints ?? {}),
      ...Object.values(jointDeltas),
      ...targetTcpNumbers,
      ...(currentTcp ? [...currentTcp.position, ...currentTcp.orientation] : []),
      ...(currentGripperOpening === null ? [] : [currentGripperOpening]),
      ...(targetGripperOpening === null ? [] : [targetGripperOpening]),
      ...(currentGripperOpening === null || targetGripperOpening === null
        ? []
        : [targetGripperOpening - currentGripperOpening]),
      ...Object.values(residual).flatMap((value) => (typeof value === 'number' ? [value] : [])),
      ...(ikRecord?.position_residual === null || ikRecord?.position_residual === undefined
        ? []
        : [ikRecord.position_residual]),
      ...(ikRecord?.orientation_residual === null || ikRecord?.orientation_residual === undefined
        ? []
        : [ikRecord.orientation_residual]),
    ].every(Number.isFinite);
    const jointDeltaEntries = Object.entries(jointDeltas);
    const [maximumJointDeltaName, maximumJointDelta] = jointDeltaEntries.reduce(
      (maximum, current) => (Math.abs(current[1]) > Math.abs(maximum[1]) ? current : maximum),
      ['', 0],
    );
    const failedReview: Record<string, unknown> = {
      timestamp: new Date().toISOString(),
      robot_id: context.robotId,
      robot_config_id: context.robotConfig.id,
      robot_config: context.robotConfig,
      robot_config_file: context.robotConfig.file,
      urdf: {
        file: context.loadedRobotUrdfFile,
        name: context.description?.name ?? null,
        root_link: context.description?.rootLink ?? null,
        tcp_link: context.description?.tipLink ?? null,
      },
      frame_index: chunk.frame_index,
      camera_frame_index: lastObservationRef.current?.frame_index ?? null,
      camera_frame_timestamp_ms: lastObservationRef.current?.camera_frame_timestamp_ms ?? null,
      input_image_shape: chunk.input_image_shape,
      policy_input_shape: chunk.input_image_shape,
      policy_tensor_shapes: chunk.policy_tensor_shapes,
      prediction_keys: chunk.prediction_keys,
      action_pred_shape: chunk.action_pred_shape,
      prediction_horizon: chunk.prediction.length,
      action_pred_0: actionPrediction,
      action_pred_10d: actionPrediction,
      action_semantics: ACTION_DIMENSION_SEMANTICS,
      action_position_frame:
        chunk.action_pose_repr === 'relative' ? 'current TCP local frame' : 'robot base frame',
      action_orientation_frame:
        chunk.action_pose_repr === 'relative' ? 'current TCP local frame' : 'robot base frame',
      action_gripper_repr: 'absolute gripper opening width',
      rotation_6d_layout: 'R00, R01, R02, R10, R11, R12 (first two matrix rows)',
      action_pose_repr: chunk.action_pose_repr,
      action_representation: '[position xyz, rotation_6d, gripper width]',
      quaternion_order: 'x, y, z, w',
      first_executable_action_index: 0,
      normal_execution_horizon: EXECUTION_HORIZON,
      single_action_execution_horizon: 1,
      selected_action_indexes: [0],
      action_start_index: 0,
      latency_compensation_steps: 0,
      conversion_chain: [
        'checkpoint action_pred row: position xyz + rotation_6d + gripper width',
        'UMI pose10d_to_mat: rotation_6d to rotation matrix; mat_to_pose: matrix to axis-angle 7D action',
        'browser action decoder: compose the relative TCP pose with the current TCP pose',
        'target TCP pose passed to IK in URDF robot base coordinates; quaternion order x, y, z, w',
      ],
      decoded_action_7d: robotAction,
      current_joints: seedJoints,
      current_tcp: currentTcp,
      current_gripper_opening_m: currentGripperOpening,
      target_tcp: targetTcp,
      target_gripper_opening_m: targetGripperOpening,
      gripper_delta_m:
        currentGripperOpening !== null && targetGripperOpening !== null
          ? targetGripperOpening - currentGripperOpening
          : null,
      required_safety_data_available: Boolean(
        currentTcp && targetTcp && currentGripperOpening !== null && targetGripperOpening !== null,
      ),
      ik_seed_joints: seedJoints,
      ik_result: error,
      target_joints: targetJoints,
      position_residual_m: ikRecord?.position_residual ?? residual.position ?? null,
      orientation_residual_rad: ikRecord?.orientation_residual ?? residual.orientation ?? null,
      joint_deltas_rad: jointDeltas,
      joint_delta_norm_rad: Math.hypot(...Object.values(jointDeltas)),
      maximum_joint_delta_rad: Math.abs(maximumJointDelta),
      maximum_joint_delta_joint: maximumJointDeltaName,
      joint_limits_rad: jointLimits,
      gripper_joint_limits_m: gripperJointLimits,
      finite_values: finiteValues,
      non_finite_values_present: !finiteValues,
      all_arm_joints_within_limits: false,
      safety_verdict: 'BLOCKED',
      human_confirmation_required: true,
      persistence_status: 'saving',
      ik_record: ikRecord,
    };
    reviewDataRef.current = failedReview;
    if (mountedRef.current) setReviewData(failedReview);
    publishDebugEvent('inference:single_action_review', {
      episode_index: chunk.episode_index,
      frame_index: chunk.frame_index,
      ...failedReview,
    });
    void persistReviewData(failedReview);
    return true;
  };

  const sendObservation = (socket: WebSocket) => {
    const context = contextRef.current;
    const observation = context.getObservation();
    if (!observation) {
      fail(
        { code: 'CAMERA_NOT_READY', stage: 'observation', message: '腕部相机尚未生成有效图像。' },
        true,
      );
      return;
    }
    const policyObservation = singleActionDebugEnabled
      ? {
          ...observation,
          robot_id: context.robotId,
          robot_config_id: context.robotConfig.id,
          robot_config_file: context.robotConfig.file,
          urdf_name: context.description?.name,
          single_action_validation: true,
        }
      : observation;
    lastObservationRef.current = policyObservation;
    socket.send(JSON.stringify({ type: 'observation', ...policyObservation }));
    if (mountedRef.current) setProgress('观测已发送，正在推理下一步动作');
    updateStatus('inferring');
    publishDebugEvent('inference:observation_sent', {
      frame_index: observation.frame_index,
      image_bytes: Math.floor((observation.camera0_rgb.length * 3) / 4),
      image_shape: [224, 224, 3],
      policy_inference_only: observation.policy_inference_only ?? false,
      frontend_rgb: observation.camera0_rgb_stats,
      robot0_eef_pos_m: observation.robot0_eef_pos,
      robot0_eef_rot_axis_angle_rad: observation.robot0_eef_rot_axis_angle,
      robot0_gripper_width_m: observation.robot0_gripper_width,
    });
  };

  const handleActionChunk = (chunk: ActionChunk) => {
    const context = contextRef.current;
    if (singleActionDebugEnabled && !isUr5CadContext(context)) {
      fail(
        {
          code: 'SINGLE_ACTION_ROBOT_UNSUPPORTED',
          stage: 'robot_guard',
          message: SINGLE_ACTION_VALIDATION_ERROR,
          episode_index: chunk.episode_index,
          frame_index: chunk.frame_index,
        },
        true,
      );
      return;
    }
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
    const singleActionDebug = singleActionDebugEnabled;
    const selection = selectExecutionActions(chunk.actions, singleActionDebug ? 1 : undefined);
    latestIkRecordRef.current = null;
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
      input_image_shape: chunk.input_image_shape,
      policy_tensor_shapes: chunk.policy_tensor_shapes,
      prediction_keys: chunk.prediction_keys,
      action_pred_shape: chunk.action_pred_shape,
      prediction: chunk.prediction,
      actions: chunk.actions,
    });
    publishDebugEvent('inference:execution_selection', {
      request_id: chunk.request_id,
      episode_index: chunk.episode_index,
      frame_index: chunk.frame_index,
      prediction_count: selection.predictionCount,
      configured_execution_horizon: selection.configuredExecutionHorizon,
      execution_count: selection.executionCount,
      selected_action_indexes: selection.selectedActionIndexes,
      action_pose_repr: chunk.action_pose_repr,
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
      const preserveReviewData = saveFailedIkReview(chunk, timeoutError);
      fail(timeoutError, true, preserveReviewData);
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
        latestIkRecordRef.current = response.record;
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
        const ikError: InferenceError = {
          ...response.error,
          episode_index: chunk.episode_index,
          frame_index: chunk.frame_index,
        };
        const preserveReviewData = saveFailedIkReview(chunk, ikError);
        fail(ikError, true, preserveReviewData);
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
      if (singleActionDebug) pendingPlaybackRef.current = nextPlayback;
      if (mountedRef.current) {
        setPlayback(singleActionDebug ? null : nextPlayback);
        setProgress(
          singleActionDebug
            ? 'IK dry-run 已完成，请检查目标后手动执行这一条动作'
            : '动作块 ' + chunk.frame_index + '：' + response.actions.length + ' 个目标',
        );
      }
      updateStatus(singleActionDebug ? 'review' : 'playing');
      publishDebugEvent('inference:ik_solved', {
        episode_index: chunk.episode_index,
        frame_index: chunk.frame_index,
        action_count: response.actions.length,
        actions: response.actions,
      });
      if (singleActionDebug) {
        const ikRecord = latestIkRecordRef.current;
        const firstActionIndex = selection.selectedActionIndexes[0];
        const firstPredictionAction = chunk.prediction[firstActionIndex];
        const firstRobotAction = chunk.actions[firstActionIndex];
        const targetJoints = response.actions[0]?.jointValues ?? {};
        const seedJoints = ikRecord?.seed_joint_values ?? context.jointValues;
        const armJoints =
          context.description?.joints.filter(
            (joint) => joint.type === 'revolute' || joint.type === 'continuous',
          ) ?? [];
        const jointDeltas = Object.fromEntries(
          armJoints.map((joint) => [
            joint.name,
            (targetJoints[joint.name] ?? 0) - (seedJoints[joint.name] ?? 0),
          ]),
        );
        const jointDeltaNorm = Math.hypot(...Object.values(jointDeltas));
        const [maximumJointDeltaName, maximumJointDelta] = Object.entries(jointDeltas).reduce(
          (maximum, current) => (Math.abs(current[1]) > Math.abs(maximum[1]) ? current : maximum),
          ['', 0],
        );
        const jointLimits = Object.fromEntries(
          armJoints.map((joint) => [
            joint.name,
            {
              lower: joint.limit?.lower ?? null,
              upper: joint.limit?.upper ?? null,
            },
          ]),
        );
        const currentTcp = ikRecord?.start_pose ?? context.tcpPose;
        const targetTcp = ikRecord?.target_pose ?? null;
        const tcpTranslationDelta =
          currentTcp && targetTcp
            ? targetTcp.position.map((value, index) => value - currentTcp.position[index])
            : null;
        const tcpTranslationDeltaNorm = tcpTranslationDelta
          ? vectorLength(tcpTranslationDelta as [number, number, number])
          : null;
        const tcpRotationDelta =
          currentTcp && targetTcp
            ? poseErrorRotation(targetTcp.orientation, currentTcp.orientation)
            : null;
        const tcpRotationDeltaNorm = tcpRotationDelta ? vectorLength(tcpRotationDelta) : null;
        const currentGripperOpening =
          lastObservationRef.current?.robot0_gripper_width.at(-1)?.[0] ?? null;
        const gripperJoints =
          context.description?.joints.filter(
            (joint) =>
              joint.type === 'prismatic' && /gripper|finger/iu.test(joint.name + ' ' + joint.child),
          ) ?? [];
        const gripperJointLimits = Object.fromEntries(
          gripperJoints.map((joint) => [
            joint.name,
            {
              lower: joint.limit?.lower ?? 0,
              upper: joint.limit?.upper ?? joint.limit?.lower ?? 0,
            },
          ]),
        );
        const maximumGripperOpening = gripperJoints.reduce((total, joint) => {
          const lower = joint.limit?.lower ?? 0;
          const upper = joint.limit?.upper ?? lower;
          return total + Math.abs(upper - lower);
        }, 0);
        const targetGripperOpening = firstRobotAction
          ? Math.max(0, Math.min(maximumGripperOpening, firstRobotAction[6]))
          : null;
        const gripperDelta =
          currentGripperOpening !== null && targetGripperOpening !== null
            ? targetGripperOpening - currentGripperOpening
            : null;
        const allArmJointsWithinLimits = armJoints.every((joint) => {
          const value = targetJoints[joint.name];
          const lower = joint.limit?.lower;
          const upper = joint.limit?.upper;
          return (
            Number.isFinite(value) &&
            (lower === undefined || value >= lower) &&
            (upper === undefined || value <= upper)
          );
        });
        const requiredSafetyDataAvailable =
          currentTcp !== null &&
          targetTcp !== null &&
          currentGripperOpening !== null &&
          targetGripperOpening !== null &&
          typeof ikRecord?.position_residual === 'number' &&
          typeof ikRecord?.orientation_residual === 'number';
        const finiteValues = [
          ...firstPredictionAction,
          ...firstRobotAction,
          ...armJoints.map((joint) => seedJoints[joint.name]),
          ...Object.values(targetJoints),
          ...Object.values(jointDeltas),
          ...(currentTcp ? [...currentTcp.position, ...currentTcp.orientation] : []),
          ...(targetTcp ? [...targetTcp.position, ...targetTcp.orientation] : []),
          ...(tcpTranslationDelta ?? []),
          ...(tcpRotationDelta ?? []),
          jointDeltaNorm,
          maximumJointDelta,
          ...(tcpTranslationDeltaNorm === null ? [] : [tcpTranslationDeltaNorm]),
          ...(tcpRotationDeltaNorm === null ? [] : [tcpRotationDeltaNorm]),
          ...(currentGripperOpening === null ? [] : [currentGripperOpening]),
          ...(targetGripperOpening === null ? [] : [targetGripperOpening]),
          ...(gripperDelta === null ? [] : [gripperDelta]),
          ...(typeof ikRecord?.position_residual === 'number' ? [ikRecord.position_residual] : []),
          ...(typeof ikRecord?.orientation_residual === 'number'
            ? [ikRecord.orientation_residual]
            : []),
        ].every(Number.isFinite);
        const ikSucceeded = ikRecord?.status === 'success';
        const safetyVerdict =
          ikSucceeded && requiredSafetyDataAvailable && finiteValues && allArmJointsWithinLimits
            ? 'READY_FOR_HUMAN_APPROVAL'
            : 'BLOCKED';
        const nextReviewData: Record<string, unknown> = {
          timestamp: new Date().toISOString(),
          robot_id: context.robotId,
          robot_config_id: context.robotConfig.id,
          robot_config: context.robotConfig,
          robot_config_file: context.robotConfig.file,
          urdf: {
            file: context.loadedRobotUrdfFile,
            name: context.description?.name ?? null,
            root_link: context.description?.rootLink ?? null,
            tcp_link: context.description?.tipLink ?? null,
          },
          frame_index: chunk.frame_index,
          camera_frame_index: lastObservationRef.current?.frame_index ?? null,
          camera_frame_timestamp_ms: lastObservationRef.current?.camera_frame_timestamp_ms ?? null,
          input_image_shape: chunk.input_image_shape,
          policy_input_shape: chunk.input_image_shape,
          policy_tensor_shapes: chunk.policy_tensor_shapes,
          prediction_keys: chunk.prediction_keys,
          action_pred_shape: chunk.action_pred_shape,
          prediction_horizon: chunk.prediction.length,
          pose_frame: 'URDF robot base',
          quaternion_order: 'x, y, z, w',
          action_representation: '[position xyz, rotation_6d, gripper width]',
          action_semantics: ACTION_DIMENSION_SEMANTICS,
          action_position_frame:
            chunk.action_pose_repr === 'relative' ? 'current TCP local frame' : 'robot base frame',
          action_gripper_repr: 'absolute gripper opening width',
          rotation_6d_layout: 'R00, R01, R02, R10, R11, R12 (first two matrix rows)',
          conversion_chain: [
            'checkpoint action_pred row: xyz + rotation_6d + gripper width',
            'UMI pose10d_to_mat: rotation_6d to rotation matrix; mat_to_pose: matrix to axis-angle 7D action',
            'browser action decoder: compose the relative TCP pose with the current TCP pose',
            'target TCP pose passed to IK in URDF robot base coordinates; quaternion order x, y, z, w',
          ],
          first_executable_action_index: firstActionIndex,
          normal_execution_horizon: EXECUTION_HORIZON,
          single_action_execution_horizon: selection.configuredExecutionHorizon,
          selected_action_indexes: selection.selectedActionIndexes,
          action_start_index: 0,
          latency_compensation_steps: 0,
          action_pred_0: firstPredictionAction,
          action_pred_10d: firstPredictionAction,
          decoded_action_7d: firstRobotAction,
          action_pose_repr: chunk.action_pose_repr,
          current_tcp: currentTcp,
          target_tcp: targetTcp,
          tcp_translation_delta_m: tcpTranslationDelta,
          tcp_translation_delta_norm_m: tcpTranslationDeltaNorm,
          tcp_rotation_delta_axis_angle_rad: tcpRotationDelta,
          tcp_rotation_delta_norm_rad: tcpRotationDeltaNorm,
          current_joints: seedJoints,
          ik_seed_joints: seedJoints,
          target_joints: targetJoints,
          joint_deltas_rad: jointDeltas,
          joint_delta_norm_rad: jointDeltaNorm,
          maximum_joint_delta_rad: Math.abs(maximumJointDelta),
          maximum_joint_delta_joint: maximumJointDeltaName,
          joint_limits_rad: jointLimits,
          gripper_joint_limits_m: gripperJointLimits,
          current_gripper_opening_m: currentGripperOpening,
          target_gripper_opening_m: targetGripperOpening,
          gripper_delta_m: gripperDelta,
          finite_values: finiteValues,
          non_finite_values_present: !finiteValues,
          required_safety_data_available: requiredSafetyDataAvailable,
          all_arm_joints_within_limits: allArmJointsWithinLimits,
          ik_success: ikSucceeded,
          position_residual_m: ikRecord?.position_residual ?? null,
          orientation_residual_rad: ikRecord?.orientation_residual ?? null,
          ik_termination_reason: ikRecord?.termination_reason ?? null,
          safety_verdict: safetyVerdict,
          ik_result: ikRecord,
          human_confirmation_required: true,
          persistence_status: 'saving',
          ik_record: ikRecord,
        };
        reviewDataRef.current = nextReviewData;
        if (mountedRef.current) setReviewData(nextReviewData);
        publishDebugEvent('inference:single_action_review', {
          episode_index: chunk.episode_index,
          frame_index: chunk.frame_index,
          ...nextReviewData,
        });
        void persistReviewData(nextReviewData);
      }
    };

    worker.onerror = (event: ErrorEvent) => {
      const workerError: InferenceError = {
        code: 'IK_WORKER_ERROR',
        stage: 'ik',
        message: event.message || 'IK Worker 执行失败。',
        episode_index: chunk.episode_index,
        frame_index: chunk.frame_index,
        ...(activeActionIndexRef.current === null
          ? {}
          : { action_index: activeActionIndexRef.current }),
      };
      const preserveReviewData = saveFailedIkReview(chunk, workerError);
      fail(workerError, true, preserveReviewData);
    };
    worker.onmessageerror = () => {
      const workerError: InferenceError = {
        code: 'IK_WORKER_MESSAGE_ERROR',
        stage: 'ik',
        message: '无法读取 IK Worker 的返回消息。',
        episode_index: chunk.episode_index,
        frame_index: chunk.frame_index,
        ...(activeActionIndexRef.current === null
          ? {}
          : { action_index: activeActionIndexRef.current }),
      };
      const preserveReviewData = saveFailedIkReview(chunk, workerError);
      fail(workerError, true, preserveReviewData);
    };

    const request: IkWorkerRequest = {
      type: 'solve',
      request_id: chunk.request_id,
      episode_index: chunk.episode_index,
      frame_index: chunk.frame_index,
      description: context.description,
      startPose: context.tcpPose,
      initialJointValues: context.jointValues,
      actions: selection.actions,
      predictionCount: selection.predictionCount,
      configuredExecutionHorizon: selection.configuredExecutionHorizon,
      executionCount: selection.executionCount,
      selectedActionIndexes: selection.selectedActionIndexes,
      action_pose_repr: chunk.action_pose_repr,
      traceAllIterations: isIkTraceEnabled(),
    };
    worker.postMessage(request);
  };

  const handlePredictionResult = (result: Record<string, unknown>) => {
    clearWorker();
    activeChunkRef.current = null;
    if (mountedRef.current) {
      setPlayback(null);
      const actionShape = Array.isArray(result.action_shape)
        ? result.action_shape.join(' × ')
        : '未知';
      const actionDtype = typeof result.action_dtype === 'string' ? result.action_dtype : '未知';
      const nanCount = typeof result.nan_count === 'number' ? result.nan_count : '未知';
      const infCount = typeof result.inf_count === 'number' ? result.inf_count : '未知';
      setProgress(
        `policy inference succeeded · action ${actionShape} · ${actionDtype} · NaN ${nanCount} · Inf ${infCount}`,
      );
    }
    updateStatus('complete');
    publishDebugEvent('inference:prediction_result', result);
    closeSocket(1000, 'policy inference only');
  };

  const start = () => {
    if (
      statusRef.current === 'connecting' ||
      statusRef.current === 'loading' ||
      statusRef.current === 'inferring' ||
      statusRef.current === 'computing' ||
      statusRef.current === 'review' ||
      statusRef.current === 'playing' ||
      statusRef.current === 'waiting'
    ) {
      return;
    }
    const context = contextRef.current;
    if (singleActionDebugEnabled && !isUr5CadSelection(context)) {
      fail(
        {
          code: 'SINGLE_ACTION_ROBOT_UNSUPPORTED',
          stage: 'robot_guard',
          message: SINGLE_ACTION_VALIDATION_ERROR,
        },
        false,
      );
      return;
    }
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
    if (singleActionDebugEnabled && !isUr5CadContext(context)) {
      fail(
        {
          code: 'SINGLE_ACTION_ROBOT_UNSUPPORTED',
          stage: 'robot_guard',
          message: SINGLE_ACTION_VALIDATION_ERROR,
        },
        false,
      );
      return;
    }

    if (mountedRef.current) {
      setError(null);
      setProgress('正在连接推理服务');
      setPlayback(null);
      reviewDataRef.current = null;
      setReviewData(null);
    }
    activeChunkRef.current = null;
    pendingPlaybackRef.current = null;
    activeJointAnglesRef.current = [];
    updateStatus('connecting');
    publishDebugEvent('inference:connect', { url: INFERENCE_SOCKET_URL });

    try {
      clearWorker();
      if (!policyInferenceOnly) {
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
      }
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
        if (message.type === 'prediction_result' && policyInferenceOnly) {
          handlePredictionResult(message);
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
          statusRef.current === 'review' ||
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
    sendStop(pendingPlaybackRef.current ? [] : activeJointAnglesRef.current);
    pendingPlaybackRef.current = null;
    activeJointAnglesRef.current = [];
    clearWorker();
    activeChunkRef.current = null;
    activeActionIndexRef.current = null;
    if (mountedRef.current) {
      setPlayback(null);
      reviewDataRef.current = null;
      setReviewData(null);
      setProgress('推理已停止');
    }
    updateStatus('stopped');
    publishDebugEvent('inference:stop', { reason: 'user' });
    closeSocket(1000, 'user stopped');
  };

  const executePendingAction = (): boolean => {
    const pendingPlayback = pendingPlaybackRef.current;
    if (!pendingPlayback || statusRef.current !== 'review') return false;
    if (!isUr5CadContext(contextRef.current)) {
      fail(
        {
          code: 'SINGLE_ACTION_ROBOT_UNSUPPORTED',
          stage: 'robot_guard',
          message: SINGLE_ACTION_VALIDATION_ERROR,
        },
        true,
      );
      return false;
    }
    if (
      reviewDataRef.current?.safety_verdict !== 'READY_FOR_HUMAN_APPROVAL' ||
      reviewDataRef.current?.persistence_status !== 'saved'
    ) {
      return false;
    }
    pendingPlaybackRef.current = null;
    if (mountedRef.current) {
      setPlayback(pendingPlayback);
      setProgress('正在执行唯一一条已审核动作');
    }
    updateStatus('playing');
    publishDebugEvent('inference:single_action_execute', {
      episode_index: pendingPlayback.episodeIndex,
      frame_index: pendingPlayback.frameIndex,
      action_index: pendingPlayback.actions[0]?.actionIndex,
    });
    return true;
  };

  const completePlayback = (token: number, snapshot?: PlaybackSnapshot) => {
    const chunk = activeChunkRef.current;
    const socket = socketRef.current;
    if (!chunk || token !== playbackTokenRef.current || socket?.readyState !== WebSocket.OPEN) {
      return;
    }
    const currentReview = reviewDataRef.current;
    if (singleActionDebugEnabled && currentReview && snapshot) {
      const targetJoints = currentReview.target_joints as JointValues;
      const armJointNames =
        contextRef.current.description?.joints
          .filter((joint) => joint.type === 'revolute' || joint.type === 'continuous')
          .map((joint) => joint.name) ?? [];
      const armJointErrors = Object.fromEntries(
        armJointNames.map((jointName) => [
          jointName,
          (snapshot.joint_values[jointName] ?? 0) - (targetJoints[jointName] ?? 0),
        ]),
      );
      const targetTcp = currentReview.target_tcp as Pose | null;
      const tcpPositionError = targetTcp
        ? snapshot.tcp_pose.position.map((value, index) => value - targetTcp.position[index])
        : null;
      const tcpOrientationError = targetTcp
        ? poseErrorRotation(targetTcp.orientation, snapshot.tcp_pose.orientation)
        : null;
      const targetGripperOpening = currentReview.target_gripper_opening_m;
      const postObservation = snapshot.observation;
      const preActionCameraTimestamp = lastObservationRef.current?.camera_frame_timestamp_ms;
      const postActionCameraTimestamp = postObservation?.camera_frame_timestamp_ms;
      const actualAfterAction = {
        observation_frame_index: postObservation?.frame_index ?? null,
        camera_frame_timestamp_ms: postActionCameraTimestamp ?? null,
        camera_frame_advanced:
          typeof preActionCameraTimestamp === 'number' &&
          typeof postActionCameraTimestamp === 'number' &&
          postActionCameraTimestamp > preActionCameraTimestamp,
        camera_image_generated: Boolean(postObservation?.camera0_rgb),
        camera_image_shape: postObservation?.camera_image_shape ?? null,
        actual_joints: snapshot.joint_values,
        actual_tcp: snapshot.tcp_pose,
        actual_gripper_opening_m: snapshot.gripper_opening_m,
        joint_target_error_rad: armJointErrors,
        joint_target_error_norm_rad: Math.hypot(...Object.values(armJointErrors)),
        tcp_position_error_m: tcpPositionError,
        tcp_position_error_norm_m: tcpPositionError
          ? vectorLength(tcpPositionError as [number, number, number])
          : null,
        tcp_orientation_error_axis_angle_rad: tcpOrientationError,
        tcp_orientation_error_norm_rad: tcpOrientationError
          ? vectorLength(tcpOrientationError)
          : null,
        gripper_target_error_m:
          typeof targetGripperOpening === 'number' && snapshot.gripper_opening_m !== null
            ? snapshot.gripper_opening_m - targetGripperOpening
            : null,
      };
      const nextReview = {
        ...currentReview,
        actual_after_action: actualAfterAction,
        persistence_status: 'saving',
      };
      reviewDataRef.current = nextReview;
      if (mountedRef.current) setReviewData(nextReview);
      void persistReviewData(nextReview);
      publishDebugEvent('inference:single_action_complete', {
        episode_index: chunk.episode_index,
        frame_index: chunk.frame_index,
        ...actualAfterAction,
      });
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
      setProgress(
        singleActionDebugEnabled
          ? '唯一一条动作已执行，已暂停等待人工验收'
          : '单步动作已执行，点击“推理一步”继续',
      );
    }
    updateStatus(singleActionDebugEnabled ? 'complete' : 'waiting');
    publishDebugEvent('inference:ack', {
      episode_index: chunk.episode_index,
      frame_index: chunk.frame_index,
    });
    if (singleActionDebugEnabled) {
      publishDebugEvent('inference:single_action_paused', {
        episode_index: chunk.episode_index,
        frame_index: chunk.frame_index,
      });
      closeSocket(1000, 'single action completed');
    }
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
    status === 'review' ||
    status === 'playing' ||
    status === 'waiting';

  const canExecuteAction =
    singleActionDebugEnabled &&
    status === 'review' &&
    isUr5CadContext(contextRef.current) &&
    reviewData?.safety_verdict === 'READY_FOR_HUMAN_APPROVAL' &&
    reviewData?.persistence_status === 'saved';

  return {
    status,
    progress,
    error,
    playback,
    isLocked,
    canExecuteAction,
    step,
    stop,
    reviewData,
    executePendingAction,
    completePlayback,
  };
};
