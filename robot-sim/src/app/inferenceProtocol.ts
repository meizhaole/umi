import type {
  IKIterationTrace,
  IKOptions,
  IKTerminationReason,
  JointValues,
  Pose,
  RobotDescription,
} from '../core/types';

export interface ActionChunk {
  type: 'action_chunk';
  request_id: string;
  episode_index: number;
  frame_index: number;
  last_chunk: boolean;
  actions: number[][];
  action_pose_repr: string;
  prediction: number[][];
  prediction_keys: string[];
  action_pred_shape: number[];
  policy_tensor_shapes: Record<string, number[]>;
  input_image_shape: number[];
}

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null;

export const isActionChunk = (value: unknown): value is ActionChunk => {
  if (!isRecord(value) || value.type !== 'action_chunk') return false;
  if (
    typeof value.request_id !== 'string' ||
    !value.request_id ||
    !Number.isInteger(value.episode_index) ||
    !Number.isInteger(value.frame_index) ||
    typeof value.last_chunk !== 'boolean' ||
    typeof value.action_pose_repr !== 'string' ||
    !value.action_pose_repr.trim() ||
    !Array.isArray(value.actions) ||
    value.actions.length === 0 ||
    !Array.isArray(value.prediction) ||
    value.prediction.length !== value.actions.length ||
    !Array.isArray(value.prediction_keys) ||
    !value.prediction_keys.every((key) => typeof key === 'string') ||
    !Array.isArray(value.action_pred_shape) ||
    !value.action_pred_shape.every((dimension) => Number.isInteger(dimension)) ||
    typeof value.policy_tensor_shapes !== 'object' ||
    value.policy_tensor_shapes === null ||
    !Array.isArray(value.input_image_shape) ||
    !value.input_image_shape.every((dimension) => Number.isInteger(dimension))
  ) {
    return false;
  }
  const validActions = value.actions.every(
    (action) =>
      Array.isArray(action) &&
      action.length === 7 &&
      action.every((number) => typeof number === 'number' && Number.isFinite(number)),
  );
  const validPrediction = value.prediction.every(
    (action) =>
      Array.isArray(action) &&
      action.length === 10 &&
      action.every((number) => typeof number === 'number' && Number.isFinite(number)),
  );
  const validTensorShapes = Object.values(value.policy_tensor_shapes).every(
    (shape) => Array.isArray(shape) && shape.every((dimension) => Number.isInteger(dimension)),
  );
  return validActions && validPrediction && validTensorShapes;
};

export interface ResolvedAction {
  actionIndex: number;
  jointValues: JointValues;
}

export interface PlaybackChunk {
  token: number;
  episodeIndex: number;
  frameIndex: number;
  lastChunk: boolean;
  actions: ResolvedAction[];
}

export interface InferenceError {
  code: string;
  stage: string;
  message: string;
  episode_index?: number;
  frame_index?: number;
  action_index?: number;
  iterations?: number;
  residual?: unknown;
  joint_values?: JointValues;
  [key: string]: unknown;
}

export interface IkWorkerRequest {
  type: 'solve';
  request_id: string;
  episode_index: number;
  frame_index: number;
  description: RobotDescription;
  gripperJointNames?: readonly string[];
  startPose: Pose;
  initialJointValues: JointValues;
  actions: number[][];
  predictionCount: number;
  configuredExecutionHorizon: number;
  executionCount: number;
  selectedActionIndexes: number[];
  action_pose_repr: string;
  traceAllIterations: boolean;
}

export interface IKActionDebugRecord {
  record_phase: 'started' | 'result';
  request_id: string;
  episode_index: number;
  frame_index: number;
  action_index: number;
  prediction_count?: number;
  configured_execution_horizon?: number;
  execution_count?: number;
  selected_action_indexes?: number[];
  action_pose_repr: string;
  action_pose_repr_status: 'available';
  status: 'in_progress' | 'success' | 'failure';
  trace_level: 'summary' | 'full';
  raw_action: unknown;
  requested_gripper_opening_m?: number;
  clamped_gripper_opening_m?: number;
  gripper_target_clamped?: boolean;
  left_jaw_target?: number | null;
  right_jaw_target?: number | null;
  joint_targets?: JointValues;
  start_pose: Pose;
  target_pose: Pose | null;
  seed_joint_values: JointValues;
  previous_action_output_seed: JointValues | null;
  solver_options: IKOptions;
  final_joint_values: JointValues | null;
  action_output_seed: JointValues | null;
  position_residual: number | null;
  orientation_residual: number | null;
  iterations: number | null;
  blocked_joints: string[] | null;
  termination_reason: IKTerminationReason | 'invalid_action' | 'solver_exception' | null;
  iteration_trace?: IKIterationTrace[];
  error_message?: string;
}

export type IkWorkerResponse =
  | { type: 'progress'; actionIndex: number; total: number }
  | { type: 'ik_record'; record: IKActionDebugRecord }
  | { type: 'solved'; actions: ResolvedAction[] }
  | { type: 'error'; error: InferenceError };

export type InferenceStatus =
  | 'idle'
  | 'connecting'
  | 'loading'
  | 'inferring'
  | 'computing'
  | 'review'
  | 'playing'
  | 'waiting'
  | 'complete'
  | 'stopped'
  | 'error';

export interface InferenceObservation {
  frame_index: number;
  camera_frame_timestamp_ms: number;
  camera0_rgb: string;
  camera_image_shape?: number[];
  robot_id?: string;
  robot_config_id?: string;
  robot_config_file?: string;
  urdf_name?: string;
  single_action_validation?: boolean;
  robot0_eef_pos: number[][];
  robot0_eef_rot_axis_angle: number[][];
  robot0_gripper_width: number[][];
  policy_inference_only?: boolean;
  camera0_rgb_stats?: RgbImageStatistics;
}

export interface RgbImageStatistics {
  width: number;
  height: number;
  byte_length: number;
  dtype: 'uint8';
  min: number;
  max: number;
  channels: Record<'r' | 'g' | 'b', { min: number; max: number; mean: number }>;
}

export const isPolicyInferenceOnlyEnabled = (): boolean =>
  typeof window !== 'undefined' &&
  new URLSearchParams(window.location.search).get('policyInferenceOnly') === '1';

export const isSingleActionDebugEnabled = (): boolean =>
  typeof window !== 'undefined' &&
  new URLSearchParams(window.location.search).get('singleAction') === '1';
