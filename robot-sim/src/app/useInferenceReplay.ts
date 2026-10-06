import { useEffect, useRef, useState } from 'react';
import type { JointValues, Pose, RobotDescription } from '../core/types';
import { findRobotConfig, type RobotConfig, type RobotModelId } from './config';
import { publishDebugEvent } from './debugBus';
import { EXECUTION_HORIZON, selectExecutionActions } from './executionHorizon';
import {
  getObservationTcpPose,
  isActionChunk,
  isPolicyInferenceOnlyEnabled,
} from './inferenceProtocol';
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
import { getGripperJoints, mapGripperOpeningToJoints } from './gripperMapping';

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
const POLICY_TARGET_ZERO_EPSILON_M = 1e-6;
const POLICY_TARGET_ZERO_EPSILON_RAD = 1e-6;
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
  const episodeIndexRef = useRef(-1);
  const workerRef = useRef<Worker | null>(null);
  const workerTimeoutRef = useRef<number | null>(null);
  const activeChunkRef = useRef<ActionChunk | null>(null);
  const pendingPlaybackRef = useRef<PlaybackChunk | null>(null);
  const latestIkRecordRef = useRef<IKActionDebugRecord | null>(null);
  const lastObservationRef = useRef<InferenceObservation | null>(null);
  const activeActionIndexRef = useRef<number | null>(null);
  const activeJointAnglesRef = useRef<JointValues[]>([]);
  const reviewCompletionPendingRef = useRef(false);
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

  const persistReviewData = async (
    reviewData: Record<string, unknown>,
    persistencePayload: Record<string, unknown> = reviewData,
  ): Promise<boolean> => {
    try {
      const response = await fetch('http://localhost:8000/api/single-action-review', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(persistencePayload),
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
      if (reviewDataRef.current !== reviewData) return false;
      const savedReview = {
        ...persistencePayload,
        persistence_status: 'saved',
        persistence_path: result.path,
      };
      reviewDataRef.current = savedReview;
      if (mountedRef.current) {
        setReviewData(savedReview);
        setError(null);
        setProgress(
          persistencePayload.review_status === 'COMPLETED'
            ? '执行后状态与完成复核已保存'
            : '审阅 JSON 已保存，请确认后再执行单动作',
        );
      }
      return true;
    } catch (caught) {
      if (reviewDataRef.current !== reviewData) return false;
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
      return false;
    }
  };

  const saveFailedIkReview = (chunk: ActionChunk, error: InferenceError): boolean => {
    const context = contextRef.current;
    if (!singleActionDebugEnabled || !isUr5CadContext(context)) return false;
    const ikRecord = latestIkRecordRef.current;
    const actionPrediction = chunk.prediction[0] ?? [];
    const robotAction = chunk.actions[0] ?? [];
    const observationTcpPose = lastObservationRef.current
      ? getObservationTcpPose(lastObservationRef.current)
      : null;
    const currentTcp = ikRecord?.start_pose ?? observationTcpPose;
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
    const gripperJoints = context.description
      ? getGripperJoints(context.description, context.robotConfig.gripperJointNames)
      : [];
    const gripperJointLimits = Object.fromEntries(
      gripperJoints.map((joint) => [
        joint.name,
        { lower: joint.limit?.lower ?? 0, upper: joint.limit?.upper ?? joint.limit?.lower ?? 0 },
      ]),
    );
    const residual = isRecord(error.residual) ? error.residual : {};
    const targetTcpNumbers = targetTcp ? [...targetTcp.position, ...targetTcp.orientation] : [];
    const currentGripperOpening =
      lastObservationRef.current?.robot0_gripper_width.at(-1)?.[0] ?? null;
    const gripperMapping =
      context.description && typeof robotAction[6] === 'number'
        ? mapGripperOpeningToJoints(
            context.description,
            robotAction[6],
            context.robotConfig.gripperJointNames,
          )
        : null;
    const targetGripperOpening = gripperMapping?.clamped_gripper_opening_m ?? null;
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
      ...(gripperMapping
        ? [
            gripperMapping.requested_gripper_opening_m,
            ...Object.values(gripperMapping.joint_targets),
          ]
        : []),
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
      request_id: chunk.request_id,
      episode_index: chunk.episode_index,
      action_index: 0,
      robot_config: context.robotConfig,
      robot_config_file: context.robotConfig.file,
      urdf: {
        file: context.loadedRobotUrdfFile,
        name: context.description?.name ?? null,
        root_link: context.description?.rootLink ?? null,
        tcp_link: context.description?.tipLink ?? null,
      },
      frame_index: chunk.frame_index,
      observation_tcp_snapshot: observationTcpPose,
      action_start_pose: currentTcp,
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
      solver_options: ikRecord?.solver_options ?? null,
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
      ...(gripperMapping ?? {}),
      finite_values: finiteValues,
      non_finite_values_present: !finiteValues,
      all_arm_joints_within_limits: false,
      safety_verdict: 'BLOCKED',
      review_status: 'NOT_READY_FOR_EXECUTION',
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
    socket.send(
      JSON.stringify({
        type: 'observation',
        episode_index: episodeIndexRef.current,
        ...policyObservation,
      }),
    );
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
    const observationSnapshot = lastObservationRef.current;
    if (
      !observationSnapshot ||
      observationSnapshot.frame_index !== chunk.frame_index ||
      episodeIndexRef.current !== chunk.episode_index
    ) {
      fail(
        {
          code: 'ACTION_CONTEXT_MISMATCH',
          stage: 'protocol',
          message: '动作块与当前 inference observation 的 episode/frame 不匹配。',
          request_id: chunk.request_id,
          episode_index: chunk.episode_index,
          frame_index: chunk.frame_index,
        },
        true,
      );
      return;
    }
    const observationTcpPose = getObservationTcpPose(observationSnapshot);
    if (!observationTcpPose) {
      fail(
        {
          code: 'INVALID_OBSERVATION_TCP',
          stage: 'ik',
          message: '当前 observation 缺少有效的 EEF TCP pose。',
          request_id: chunk.request_id,
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
    const selection = selectExecutionActions(chunk.actions);
    const ikRecordsByAction = new Map<number, IKActionDebugRecord>();
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
        if (response.record.record_phase === 'result') {
          ikRecordsByAction.set(response.record.action_index, response.record);
        }
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
      const firstSelectedActionIndex = selection.selectedActionIndexes[0];
      const playbackActions = singleActionDebug
        ? response.actions.filter((action) => action.actionIndex === firstSelectedActionIndex)
        : response.actions;
      activeJointAnglesRef.current = playbackActions.map((action) =>
        selectJointAngles(context.description as RobotDescription, action.jointValues),
      );
      const nextPlayback: PlaybackChunk = {
        token: playbackTokenRef.current + 1,
        episodeIndex: chunk.episode_index,
        frameIndex: chunk.frame_index,
        lastChunk: chunk.last_chunk,
        actions: playbackActions,
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
        const firstActionIndex = firstSelectedActionIndex;
        const ikRecord = ikRecordsByAction.get(firstActionIndex) ?? null;
        const firstPredictionAction = chunk.prediction[firstActionIndex];
        const firstRobotAction = chunk.actions[firstActionIndex];
        const targetJoints =
          ikRecord?.final_joint_values ??
          response.actions.find((action) => action.actionIndex === firstActionIndex)?.jointValues ??
          null;
        const seedJoints = ikRecord?.seed_joint_values ?? context.jointValues;
        const armJoints =
          context.description?.joints.filter(
            (joint) => joint.type === 'revolute' || joint.type === 'continuous',
          ) ?? [];
        const jointDeltas = targetJoints
          ? Object.fromEntries(
              armJoints.map((joint) => [
                joint.name,
                (targetJoints[joint.name] ?? 0) - (seedJoints[joint.name] ?? 0),
              ]),
            )
          : null;
        const jointDeltaNorm = jointDeltas ? Math.hypot(...Object.values(jointDeltas)) : null;
        const maximumJointDeltaEntry = jointDeltas
          ? Object.entries(jointDeltas).reduce(
              (maximum, current) =>
                Math.abs(current[1]) > Math.abs(maximum[1]) ? current : maximum,
              ['', 0],
            )
          : null;
        const maximumJointDeltaName = maximumJointDeltaEntry?.[0] ?? null;
        const maximumJointDelta =
          maximumJointDeltaEntry === null ? null : Math.abs(maximumJointDeltaEntry[1]);
        const jointLimits = Object.fromEntries(
          armJoints.map((joint) => [
            joint.name,
            {
              lower: joint.limit?.lower ?? null,
              upper: joint.limit?.upper ?? null,
            },
          ]),
        );
        const currentTcp = ikRecord?.start_pose ?? observationTcpPose;
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
        const actionDryRunSummary = selection.selectedActionIndexes.map((actionIndex) => {
          const actionRecord = ikRecordsByAction.get(actionIndex);
          const actionStartPose = actionRecord?.start_pose ?? observationTcpPose;
          const actionTargetPose = actionRecord?.target_pose ?? null;
          const actionTranslationDelta =
            actionStartPose && actionTargetPose
              ? actionTargetPose.position.map(
                  (value, index) => value - actionStartPose.position[index],
                )
              : null;
          const actionTranslationDeltaNorm = actionTranslationDelta
            ? vectorLength(actionTranslationDelta as [number, number, number])
            : null;
          const actionRotationDelta =
            actionStartPose && actionTargetPose
              ? poseErrorRotation(actionTargetPose.orientation, actionStartPose.orientation)
              : null;
          const actionRotationDeltaNorm = actionRotationDelta
            ? vectorLength(actionRotationDelta)
            : null;
          const actionSeedJoints = actionRecord?.seed_joint_values ?? null;
          const actionTargetJoints = actionRecord?.final_joint_values ?? null;
          const actionJointDeltas =
            actionSeedJoints && actionTargetJoints
              ? Object.fromEntries(
                  armJoints.map((joint) => [
                    joint.name,
                    (actionTargetJoints[joint.name] ?? 0) - (actionSeedJoints[joint.name] ?? 0),
                  ]),
                )
              : null;
          const actionJointDeltaNorm = actionJointDeltas
            ? Math.hypot(...Object.values(actionJointDeltas))
            : null;
          const actionMaximumJointDelta = actionJointDeltas
            ? Math.max(...Object.values(actionJointDeltas).map(Math.abs))
            : null;
          const actionSucceeded = actionRecord?.status === 'success';

          return {
            action_pred_index: actionIndex,
            tcp_translation_delta_norm_m: actionTranslationDeltaNorm,
            tcp_rotation_delta_norm_rad: actionRotationDeltaNorm,
            requested_gripper_opening_m: actionRecord?.requested_gripper_opening_m ?? null,
            target_gripper_opening_m: actionRecord?.clamped_gripper_opening_m ?? null,
            gripper_target_clamped: actionRecord?.gripper_target_clamped ?? null,
            left_jaw_target: actionRecord?.left_jaw_target ?? null,
            right_jaw_target: actionRecord?.right_jaw_target ?? null,
            ik_iterations: actionRecord?.iterations ?? null,
            position_residual_m: actionRecord?.position_residual ?? null,
            orientation_residual_rad: actionRecord?.orientation_residual ?? null,
            joint_delta_norm_rad: actionJointDeltaNorm,
            max_joint_delta_rad: actionMaximumJointDelta,
            success: actionSucceeded,
            nonzero_arm_target:
              actionSucceeded && actionJointDeltaNorm !== null && actionJointDeltaNorm > 0,
            termination_reason: actionRecord?.termination_reason ?? null,
          };
        });
        const firstNonzeroArmAction = actionDryRunSummary.find(
          (action) => action.success && action.nonzero_arm_target,
        );
        const currentGripperOpening =
          lastObservationRef.current?.robot0_gripper_width.at(-1)?.[0] ?? null;
        const gripperJoints = context.description
          ? getGripperJoints(context.description, context.robotConfig.gripperJointNames)
          : [];
        const gripperJointLimits = Object.fromEntries(
          gripperJoints.map((joint) => [
            joint.name,
            {
              lower: joint.limit?.lower ?? 0,
              upper: joint.limit?.upper ?? joint.limit?.lower ?? 0,
            },
          ]),
        );
        const gripperMapping =
          context.description && firstRobotAction
            ? mapGripperOpeningToJoints(
                context.description,
                firstRobotAction[6],
                context.robotConfig.gripperJointNames,
              )
            : null;
        const targetGripperOpening = gripperMapping?.clamped_gripper_opening_m ?? null;
        const gripperDelta =
          currentGripperOpening !== null && targetGripperOpening !== null
            ? targetGripperOpening - currentGripperOpening
            : null;
        const allArmJointsWithinLimits = armJoints.every((joint) => {
          const value = targetJoints?.[joint.name];
          const lower = joint.limit?.lower;
          const upper = joint.limit?.upper;
          return (
            typeof value === 'number' &&
            Number.isFinite(value) &&
            (lower === undefined || value >= lower) &&
            (upper === undefined || value <= upper)
          );
        });
        const requiredSafetyDataAvailable =
          currentTcp !== null &&
          targetTcp !== null &&
          targetJoints !== null &&
          currentGripperOpening !== null &&
          targetGripperOpening !== null &&
          typeof ikRecord?.position_residual === 'number' &&
          typeof ikRecord?.orientation_residual === 'number';
        const finiteValues = [
          ...firstPredictionAction,
          ...firstRobotAction,
          ...armJoints.map((joint) => seedJoints[joint.name]),
          ...(targetJoints ? Object.values(targetJoints) : []),
          ...(jointDeltas ? Object.values(jointDeltas) : []),
          ...(currentTcp ? [...currentTcp.position, ...currentTcp.orientation] : []),
          ...(targetTcp ? [...targetTcp.position, ...targetTcp.orientation] : []),
          ...(tcpTranslationDelta ?? []),
          ...(tcpRotationDelta ?? []),
          ...(jointDeltaNorm === null ? [] : [jointDeltaNorm]),
          ...(maximumJointDelta === null ? [] : [maximumJointDelta]),
          ...(tcpTranslationDeltaNorm === null ? [] : [tcpTranslationDeltaNorm]),
          ...(tcpRotationDeltaNorm === null ? [] : [tcpRotationDeltaNorm]),
          ...(currentGripperOpening === null ? [] : [currentGripperOpening]),
          ...(targetGripperOpening === null ? [] : [targetGripperOpening]),
          ...(gripperMapping
            ? [
                gripperMapping.requested_gripper_opening_m,
                ...Object.values(gripperMapping.joint_targets),
              ]
            : []),
          ...(gripperDelta === null ? [] : [gripperDelta]),
          ...(typeof ikRecord?.position_residual === 'number' ? [ikRecord.position_residual] : []),
          ...(typeof ikRecord?.orientation_residual === 'number'
            ? [ikRecord.orientation_residual]
            : []),
        ].every(Number.isFinite);
        const ikSucceeded = ikRecord?.status === 'success';
        const policyTargetNearZero =
          tcpTranslationDeltaNorm !== null &&
          tcpRotationDeltaNorm !== null &&
          tcpTranslationDeltaNorm <= POLICY_TARGET_ZERO_EPSILON_M &&
          tcpRotationDeltaNorm <= POLICY_TARGET_ZERO_EPSILON_RAD;
        const armMotionClassification = !ikSucceeded
          ? 'IK_FAILED'
          : jointDeltaNorm === 0
            ? 'ZERO_WITHIN_TOLERANCE'
            : 'NONZERO_IK_TARGET';
        const safetyVerdict =
          ikSucceeded &&
          jointDeltaNorm !== null &&
          jointDeltaNorm > 0 &&
          requiredSafetyDataAvailable &&
          finiteValues &&
          allArmJointsWithinLimits
            ? 'READY_FOR_HUMAN_APPROVAL'
            : 'BLOCKED';
        const reviewStatus =
          safetyVerdict === 'READY_FOR_HUMAN_APPROVAL'
            ? 'WAITING_FOR_HUMAN_APPROVAL'
            : 'NOT_READY_FOR_EXECUTION';
        const nextReviewData: Record<string, unknown> = {
          timestamp: new Date().toISOString(),
          robot_id: context.robotId,
          robot_config_id: context.robotConfig.id,
          request_id: chunk.request_id,
          episode_index: chunk.episode_index,
          action_index: firstActionIndex,
          robot_config: context.robotConfig,
          robot_config_file: context.robotConfig.file,
          urdf: {
            file: context.loadedRobotUrdfFile,
            name: context.description?.name ?? null,
            root_link: context.description?.rootLink ?? null,
            tcp_link: context.description?.tipLink ?? null,
          },
          tcp_link: context.description?.tipLink ?? null,
          frame_index: chunk.frame_index,
          observation_tcp_snapshot: observationTcpPose,
          action_start_pose: ikRecord?.start_pose ?? observationTcpPose,
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
          single_action_execution_horizon: 1,
          selected_action_indexes: [firstActionIndex],
          ik_dry_run_action_indexes: selection.selectedActionIndexes,
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
          solver_options: ikRecord?.solver_options ?? null,
          target_joints: targetJoints,
          joint_deltas_rad: jointDeltas,
          joint_delta_norm_rad: jointDeltaNorm,
          maximum_joint_delta_rad: maximumJointDelta,
          maximum_joint_delta_joint: maximumJointDeltaName,
          joint_limits_rad: jointLimits,
          gripper_joint_limits_m: gripperJointLimits,
          ...(gripperMapping ?? {}),
          current_gripper_opening_m: currentGripperOpening,
          target_gripper_opening_m: targetGripperOpening,
          gripper_delta_m: gripperDelta,
          finite_values: finiteValues,
          non_finite_values_present: !finiteValues,
          required_safety_data_available: requiredSafetyDataAvailable,
          all_arm_joints_within_limits: allArmJointsWithinLimits,
          ik_success: ikSucceeded,
          review_status: reviewStatus,
          arm_motion_classification: armMotionClassification,
          policy_target_near_zero: policyTargetNearZero,
          policy_target_near_zero_threshold: {
            translation_m: POLICY_TARGET_ZERO_EPSILON_M,
            rotation_rad: POLICY_TARGET_ZERO_EPSILON_RAD,
          },
          ik_target_within_tolerance_without_joint_change:
            ikRecord?.iterations === 0 && jointDeltaNorm === 0,
          action_pred_0_to_5_dry_run_summary: actionDryRunSummary,
          first_nonzero_arm_action_index: firstNonzeroArmAction?.action_pred_index ?? null,
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
      gripperJointNames: context.robotConfig.gripperJointNames,
      ikOptions: context.robotConfig.ikOptions,
      startPose: observationTcpPose,
      initialJointValues: context.jointValues,
      actions: selection.actions,
      predictionCount: selection.predictionCount,
      configuredExecutionHorizon: selection.configuredExecutionHorizon,
      executionCount: selection.executionCount,
      selectedActionIndexes: selection.selectedActionIndexes,
      action_pose_repr: chunk.action_pose_repr,
      traceAllIterations: isIkTraceEnabled(),
      continueOnIkFailure: singleActionDebug,
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
    if (reviewCompletionPendingRef.current) return;
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
    episodeIndexRef.current += 1;
    updateStatus('connecting');
    publishDebugEvent('inference:connect', {
      url: INFERENCE_SOCKET_URL,
      episode_index: episodeIndexRef.current,
    });

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
    const activeSocket = socketRef.current;
    if (
      singleActionDebugEnabled &&
      statusRef.current === 'complete' &&
      activeSocket?.readyState === WebSocket.OPEN
    ) {
      if (mountedRef.current) {
        setError(null);
        reviewDataRef.current = null;
        setReviewData(null);
      }
      sendObservation(activeSocket);
      return;
    }

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
    if (
      statusRef.current === 'idle' ||
      statusRef.current === 'complete' ||
      reviewCompletionPendingRef.current
    ) {
      return;
    }
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
    let completionPersistence: Promise<boolean> | null = null;
    const actualAfterActionMissing = singleActionDebugEnabled && (!currentReview || !snapshot);
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
      const completedReview = {
        ...nextReview,
        review_status: 'COMPLETED',
      };
      reviewDataRef.current = nextReview;
      if (mountedRef.current) setReviewData(nextReview);
      reviewCompletionPendingRef.current = true;
      completionPersistence = persistReviewData(nextReview, completedReview);
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
        actualAfterActionMissing
          ? '唯一一条动作已执行，但执行后状态未采集，复核未完成'
          : singleActionDebugEnabled
            ? '唯一一条动作已执行，正在保存执行后状态'
            : '单步动作已执行，点击“推理一步”继续',
      );
    }
    updateStatus(singleActionDebugEnabled ? 'review' : 'waiting');
    publishDebugEvent('inference:ack', {
      episode_index: chunk.episode_index,
      frame_index: chunk.frame_index,
    });
    if (singleActionDebugEnabled) {
      publishDebugEvent('inference:single_action_paused', {
        episode_index: chunk.episode_index,
        frame_index: chunk.frame_index,
      });
      if (actualAfterActionMissing) {
        if (mountedRef.current) {
          setError({
            code: 'ACTUAL_AFTER_ACTION_MISSING',
            stage: 'review',
            message: '执行后状态未采集，审阅状态保持未完成。',
            episode_index: chunk.episode_index,
            frame_index: chunk.frame_index,
          });
        }
        updateStatus('error');
        closeSocket(4000, 'single action actual state missing');
      } else if (completionPersistence) {
        void completionPersistence.then((saved) => {
          reviewCompletionPendingRef.current = false;
          if (!mountedRef.current) return;
          updateStatus(saved ? 'complete' : 'error');
          if (!saved) closeSocket(4000, 'single action review persistence failed');
        });
      }
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

  const hasCompletedReview =
    reviewData?.review_status === 'COMPLETED' && reviewData.persistence_status === 'saved';
  const canStart =
    (!isLocked || status === 'waiting') &&
    !reviewCompletionPendingRef.current &&
    (!singleActionDebugEnabled ||
      (reviewData === null ? status !== 'complete' : status === 'complete' && hasCompletedReview));

  const canExecuteAction =
    singleActionDebugEnabled &&
    status === 'review' &&
    isUr5CadContext(contextRef.current) &&
    reviewData?.safety_verdict === 'READY_FOR_HUMAN_APPROVAL' &&
    reviewData?.review_status === 'WAITING_FOR_HUMAN_APPROVAL' &&
    reviewData?.persistence_status === 'saved';

  return {
    status,
    progress,
    error,
    playback,
    isLocked,
    canStart,
    canExecuteAction,
    step,
    stop,
    reviewData,
    executePendingAction,
    completePlayback,
  };
};
