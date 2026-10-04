import type { JointValues, Pose } from '../core/types';
import phase2bReportUrl from '../../reports/phase2b1-umi-episode0-frames0-399.jsonl?url';

export const DATASET_REPLAY_REPORT_URL = phase2bReportUrl;
export const DATASET_REPLAY_FRAME_COUNT = 400;
export const DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC = 0.01;
export const DATASET_REPLAY_MAX_TRACKING_ERROR_RAD = 0.005;
export type DatasetReplayExecutionMode = 'KINEMATIC_REPLAY' | 'PHYSICS_MOTOR';
export const DEFAULT_DATASET_REPLAY_EXECUTION_MODE: DatasetReplayExecutionMode = 'KINEMATIC_REPLAY';

export type DatasetReplaySettlement = 'SETTLED' | 'TRACKING_ERROR' | null;

export const classifyDatasetReplaySettlement = (
  maxJointVelocityRadPerSec: number,
  maxJointTrackingErrorRad: number,
): DatasetReplaySettlement => {
  if (maxJointVelocityRadPerSec > DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC) return null;
  return maxJointTrackingErrorRad < DATASET_REPLAY_MAX_TRACKING_ERROR_RAD
    ? 'SETTLED'
    : 'TRACKING_ERROR';
};

export interface DatasetReplayFrame {
  frame_index: number;
  aligned_target_position: Pose['position'];
  aligned_target_quaternion: Pose['orientation'];
  ik_solution_joints: JointValues;
  ik_success: true;
}

export interface DatasetReplayReport {
  initial_joints: JointValues;
  frames: DatasetReplayFrame[];
}

type JsonRecord = Record<string, unknown>;

const isRecord = (value: unknown): value is JsonRecord =>
  typeof value === 'object' && value !== null && !Array.isArray(value);

const readVector = <T extends number[]>(value: unknown, length: number, field: string): T => {
  if (
    !Array.isArray(value) ||
    value.length !== length ||
    value.some((item) => typeof item !== 'number' || !Number.isFinite(item))
  ) {
    throw new Error(`Phase 2B-1 报告字段无效：${field}`);
  }
  return value as T;
};

const readJointValues = (value: unknown, field: string): JointValues => {
  if (!isRecord(value)) throw new Error(`Phase 2B-1 报告字段无效：${field}`);
  const joints = Object.fromEntries(
    Object.entries(value).map(([name, position]) => {
      if (typeof position !== 'number' || !Number.isFinite(position)) {
        throw new Error(`Phase 2B-1 报告字段无效：${field}.${name}`);
      }
      return [name, position];
    }),
  );
  if (Object.keys(joints).length !== 6) {
    throw new Error(`Phase 2B-1 报告字段无效：${field} 必须包含六个关节`);
  }
  return joints;
};

export const parseDatasetReplayReport = (contents: string): DatasetReplayReport => {
  const records = contents
    .trim()
    .split('\n')
    .map((line) => JSON.parse(line) as unknown)
    .filter(isRecord);
  const run = records.find((record) => record.record_type === 'run');
  const summary = records.find((record) => record.record_type === 'summary');
  if (!run || run.phase !== '2B-1' || run.episode_index !== 0) {
    throw new Error('Phase 2B-1 报告来源或 episode 不匹配');
  }
  if (
    !Array.isArray(run.episode_frame_range) ||
    run.episode_frame_range[0] !== 0 ||
    run.episode_frame_range[1] !== DATASET_REPLAY_FRAME_COUNT - 1
  ) {
    throw new Error('Phase 2B-1 报告帧范围不是 episode 0 的 0–399');
  }
  if (!summary || summary.status !== 'complete') {
    throw new Error('Phase 2B-1 报告没有完整的 dry-run 结果');
  }

  const initialJoints = readJointValues(run.initial_joints, 'run.initial_joints');
  const frames = records
    .filter((record) => record.record_type === 'frame')
    .map((record) => {
      if (record.ik_success !== true) {
        throw new Error(`Phase 2B-1 frame ${String(record.frame_index)} 的 IK 未成功`);
      }
      return {
        frame_index: Number(record.frame_index),
        aligned_target_position: readVector<Pose['position']>(
          record.aligned_target_position,
          3,
          'aligned_target_position',
        ),
        aligned_target_quaternion: readVector<Pose['orientation']>(
          record.aligned_target_quaternion,
          4,
          'aligned_target_quaternion',
        ),
        ik_solution_joints: readJointValues(record.ik_solution_joints, 'ik_solution_joints'),
        ik_success: true as const,
      };
    });

  if (frames.length !== DATASET_REPLAY_FRAME_COUNT) {
    throw new Error(
      `Phase 2B-1 报告帧数无效：期望 ${DATASET_REPLAY_FRAME_COUNT}，实际 ${frames.length}`,
    );
  }
  frames.forEach((frame, index) => {
    if (frame.frame_index !== index) {
      throw new Error(`Phase 2B-1 帧索引不连续：期望 ${index}，实际 ${frame.frame_index}`);
    }
  });
  if (!Object.keys(initialJoints).every((name) => name in frames[0].ik_solution_joints)) {
    throw new Error('Phase 2B-1 frame 0 关节名称与 UR5 初始配置不匹配');
  }
  if (
    Object.entries(initialJoints).some(
      ([name, value]) => Math.abs(frames[0].ik_solution_joints[name] - value) > 1e-12,
    )
  ) {
    throw new Error('Phase 2B-1 frame 0 IK 解与 run.initial_joints 不一致');
  }
  return { initial_joints: initialJoints, frames };
};
