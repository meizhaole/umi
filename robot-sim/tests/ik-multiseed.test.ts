// 验证 IK Multi-seed 样本筛选、合法 Seed 构造和重复输入标记。
// @vitest-environment jsdom
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import type { IKActionDebugRecord } from '../src/app/inferenceProtocol';
import type { IKIterationTrace } from '../src/core/types';
import { buildSeedCases, selectLatestFailureRecord } from '../src/diagnostics/ikMultiSeed';
import { parseUrdf } from '../src/utils/urdfParser';

const readRsDescription = () =>
  parseUrdf(
    readFileSync(
      resolve(
        process.cwd(),
        '../reBotArmController_ROS2/src/rebotarm_bringup/description/RS/urdf/ReBot_Arm_RS.urdf',
      ),
      'utf8',
    ),
  );

const iteration: IKIterationTrace = {
  iteration_index: 0,
  q_before: { joint1: 0, joint2: 0, joint3: 0, joint4: 0, joint5: 0, joint6: 0 },
  current_position_residual: 0.006,
  current_orientation_residual: 0.001,
  delta_q_raw: { joint1: 0, joint2: -0.01, joint3: 0, joint4: 0, joint5: 0, joint6: 0 },
  delta_q_after_active_set: {
    joint1: 0,
    joint2: null,
    joint3: 0,
    joint4: 0,
    joint5: 0,
    joint6: 0,
  },
  delta_q_after_step_limiting: {
    joint1: 0,
    joint2: 0,
    joint3: 0,
    joint4: 0,
    joint5: 0,
    joint6: 0,
  },
  q_candidate: { joint1: 0, joint2: 0, joint3: 0, joint4: 0, joint5: 0, joint6: 0 },
  q_after: { joint1: 0, joint2: 0, joint3: 0, joint4: 0, joint5: 0, joint6: 0 },
  blocked_joints: ['joint2'],
  active_joints: ['joint1', 'joint3', 'joint4', 'joint5', 'joint6'],
  clamp_information: {},
  joint_limit_margin: {},
  exit_condition: 'no_joint_motion',
};

const makeRecord = (overrides: Partial<IKActionDebugRecord> = {}): IKActionDebugRecord => ({
  record_phase: 'result',
  request_id: 'request-frame-2',
  episode_index: 0,
  frame_index: 2,
  action_index: 8,
  action_pose_repr: null,
  action_pose_repr_status: 'unavailable',
  status: 'failure',
  trace_level: 'full',
  raw_action: [0, 0, 0, 0, 0, 0, 0.08],
  start_pose: {
    position: [0.3, 0, 0.24],
    orientation: [0, 0, 0, 1],
  },
  target_pose: {
    position: [0.29, 0, 0.23],
    orientation: [0, 0, 0, 1],
  },
  seed_joint_values: {
    joint1: 0,
    joint2: 0.001,
    joint3: 0.06,
    joint4: 0,
    joint5: 0,
    joint6: 0,
    j_gripper_end: 0,
    gripper_joint1: 0.03,
    gripper_joint2: 0.05,
  },
  previous_action_output_seed: null,
  solver_options: {
    maxIterations: 100,
    damping: 0.05,
    positionTolerance: 0.005,
    orientationTolerance: 0.01,
    maxAngularStep: 0.15,
    maxLinearStep: 0.01,
  },
  final_joint_values: null,
  action_output_seed: null,
  position_residual: 0.006,
  orientation_residual: 0.001,
  iterations: 21,
  blocked_joints: ['joint2'],
  termination_reason: 'no_joint_motion',
  iteration_trace: [iteration],
  ...overrides,
});

describe('IK Multi-seed 实验输入', () => {
  it('选择 JSONL 顺序中最新的完整 no_joint_motion 失败记录', () => {
    const older = makeRecord({ request_id: 'older', frame_index: 1 });
    const ignored = makeRecord({
      request_id: 'success',
      status: 'success',
      termination_reason: 'converged',
    });
    const latest = makeRecord({ request_id: 'latest', frame_index: 3 });

    expect(selectLatestFailureRecord([older, ignored, latest])).toBe(latest);
  });

  it('构造 A 到 H Seed，标出重复项并保留夹爪值', () => {
    const original = makeRecord();
    const previousSuccessfulSeed = { ...original.seed_joint_values };
    const cases = buildSeedCases(original, previousSuccessfulSeed, readRsDescription());
    const byName = new Map(cases.map((seedCase) => [seedCase.seed_name, seedCase]));

    expect(cases.map((seedCase) => seedCase.seed_name)).toEqual([
      'Seed A — Original Failure Seed',
      'Seed B — Previous Successful Action Seed',
      'Seed C — Joint2 0.05 rad',
      'Seed D — Joint2 0.10 rad',
      'Seed E — RS Initial Reference',
      'Seed F — joint1 + 0.02 rad',
      'Seed G — joint2 + 0.02 rad',
      'Seed H — joint3 − 0.02 rad',
    ]);
    expect(byName.get('Seed B — Previous Successful Action Seed')?.duplicate_of).toBe(
      'Seed A — Original Failure Seed',
    );
    expect(byName.get('Seed C — Joint2 0.05 rad')?.seed_joint_values?.joint2).toBe(0.05);
    expect(byName.get('Seed D — Joint2 0.10 rad')?.seed_joint_values?.joint2).toBe(0.1);
    expect(byName.get('Seed E — RS Initial Reference')?.seed_joint_values).toMatchObject({
      joint1: 0,
      joint2: 0.1,
      joint3: 0.1,
      joint4: 0,
      joint5: 0,
      joint6: 0,
      gripper_joint1: original.seed_joint_values.gripper_joint1,
      gripper_joint2: original.seed_joint_values.gripper_joint2,
    });
    expect(byName.get('Seed F — joint1 + 0.02 rad')?.seed_joint_values?.joint1).toBe(0.02);
    expect(byName.get('Seed G — joint2 + 0.02 rad')?.seed_joint_values?.joint2).toBe(0.021);
    expect(byName.get('Seed H — joint3 − 0.02 rad')?.seed_joint_values?.joint3).toBeCloseTo(0.04);
    cases.forEach((seedCase) => {
      expect(seedCase.unavailable_reason).toBeUndefined();
      expect(seedCase.seed_joint_values?.gripper_joint1).toBe(
        original.seed_joint_values.gripper_joint1,
      );
      expect(seedCase.seed_joint_values?.gripper_joint2).toBe(
        original.seed_joint_values.gripper_joint2,
      );
    });
  });
});
