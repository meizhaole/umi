import type { JointValues, Pose, RobotDescription } from '../core/types';

export interface ActionChunk {
  type: 'action_chunk';
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
  [key: string]: unknown;
}

export interface IkWorkerRequest {
  type: 'solve';
  description: RobotDescription;
  startPose: Pose;
  initialJointValues: JointValues;
  actions: number[][];
}

export type IkWorkerResponse =
  | { type: 'progress'; actionIndex: number; total: number }
  | { type: 'solved'; actions: ResolvedAction[] }
  | { type: 'error'; error: InferenceError };

export type InferenceStatus =
  | 'idle'
  | 'connecting'
  | 'loading'
  | 'computing'
  | 'playing'
  | 'waiting'
  | 'complete'
  | 'stopped'
  | 'error';
