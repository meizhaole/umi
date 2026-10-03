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
}

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
  startPose: Pose;
  initialJointValues: JointValues;
  actions: number[][];
  traceAllIterations: boolean;
}

export interface IKActionDebugRecord {
  record_phase: 'started' | 'result';
  request_id: string;
  episode_index: number;
  frame_index: number;
  action_index: number;
  action_pose_repr: null;
  action_pose_repr_status: 'unavailable';
  status: 'in_progress' | 'success' | 'failure';
  trace_level: 'summary' | 'full';
  raw_action: unknown;
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
  | 'playing'
  | 'waiting'
  | 'complete'
  | 'stopped'
  | 'error';

export interface InferenceObservation {
  frame_index: number;
  camera0_rgb: string;
  robot0_eef_pos: number[][];
  robot0_eef_rot_axis_angle: number[][];
  robot0_gripper_width: number[][];
}
