// 验证 scheduler waypoint 到时间化、分段安全候选计划的转换。
import { describe, expect, it } from 'vitest';
import type { IKActionDebugRecord } from '../src/app/inferenceProtocol';
import type { JointDescription, JointValues, Pose, RobotDescription } from '../src/core/types';
import type { PreContactSafetyResult } from '../src/sim/tasks/cupApproachPreContactSafety';
import {
  applyOfficialWaypointQueueInsertion,
  createUmiScheduledWaypointPlan,
  simulateOfficialWaypointReplan,
  type UmiScheduledWaypointPlanInput,
} from '../src/app/umiScheduledWaypointPlan';
import { scheduleUmiActions } from '../src/app/umiActionScheduler';
import { selectExecutionActions } from '../src/app/executionHorizon';

const joint = (
  name: string,
  type: JointDescription['type'],
  lower: number,
  upper: number,
): JointDescription => ({
  name,
  type,
  parent: 'base',
  child: `${name}_link`,
  origin: { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
  axis: [0, 0, 1],
  limit: { lower, upper },
});

const description: RobotDescription = {
  name: 'plan-test',
  rootLink: 'base',
  tipLink: 'umi_tcp_link',
  links: [],
  joints: [
    joint('shoulder', 'revolute', -1, 1),
    joint('left_jaw_joint', 'prismatic', 0, 0.055),
    joint('right_jaw_joint', 'prismatic', 0, 0.055),
  ],
};

const initialJoints: JointValues = {
  shoulder: 0,
  left_jaw_joint: 0,
  right_jaw_joint: 0,
};

const observationTcpPose: Pose = {
  position: [1, 2, 3],
  orientation: [0, 0, Math.SQRT1_2, Math.SQRT1_2],
};

const makeSchedule = () =>
  scheduleUmiActions({
    actionCount: 16,
    observationTimestampMs: 1000,
    inferenceRequestSentTimestampMs: 1000,
    inferenceResponseReceivedTimestampMs: 1550,
    schedulerCurrentTimestampMs: 1550,
    episodeStartTimestampMs: 1016.6666667,
    iterationIndex: 0,
  });

const makeActions = () =>
  Array.from({ length: 16 }, (_, actionIndex) => [actionIndex / 100, 0, 0, 0, 0, 0, 0.02]);

const makeRecord = (
  actionIndex: number,
  seed: JointValues,
  target: JointValues,
  overrides: Partial<IKActionDebugRecord> = {},
): IKActionDebugRecord => ({
  record_phase: 'result',
  request_id: 'plan-request',
  episode_index: 2,
  frame_index: 11,
  action_index: actionIndex,
  action_pose_repr: 'relative',
  action_pose_repr_status: 'available',
  status: 'success',
  trace_level: 'summary',
  raw_action: makeActions()[actionIndex],
  start_pose: observationTcpPose,
  target_pose: observationTcpPose,
  seed_joint_values: { ...seed },
  previous_action_output_seed: actionIndex === 6 ? null : { ...seed },
  solver_options: { positionTolerance: 0.0005, orientationTolerance: 0.001 },
  final_joint_values: { ...target },
  action_output_seed: { ...target },
  position_residual: 0,
  orientation_residual: 0,
  iterations: 0,
  blocked_joints: [],
  termination_reason: 'converged',
  ...overrides,
});

const makeRecords = (
  actionIndexes = makeSchedule().selectedActionIndexes,
  overrides: Partial<IKActionDebugRecord>[] = [],
) => {
  const actions = makeActions();
  const records = new Map<number, IKActionDebugRecord>();
  let seed = { ...initialJoints };
  actionIndexes.forEach((actionIndex, executionIndex) => {
    const opening = actions[actionIndex][6];
    const target: JointValues = {
      shoulder: seed.shoulder + 0.01,
      left_jaw_joint: opening / 2,
      right_jaw_joint: opening / 2,
    };
    const override = overrides[executionIndex] ?? {};
    const record = makeRecord(actionIndex, seed, target, override);
    records.set(actionIndex, record);
    seed = { ...target, ...(override.final_joint_values ?? {}) };
  });
  return records;
};

const safeSegment = (): PreContactSafetyResult =>
  ({
    verdict: 'SAFE_TO_EXECUTE',
    would_enter_unvalidated_contact: false,
  }) as PreContactSafetyResult;

const makePlanInput = (
  overrides: Partial<UmiScheduledWaypointPlanInput> = {},
): UmiScheduledWaypointPlanInput => ({
  requestId: 'plan-request',
  episodeIndex: 2,
  frameIndex: 11,
  observationTimestampMs: 1000,
  observationTcpPose,
  observationJointValues: initialJoints,
  observationGripperOpeningM: 0,
  actionPoseRepr: 'relative',
  actions: makeActions(),
  schedule: makeSchedule(),
  description,
  gripperJointNames: ['left_jaw_joint', 'right_jaw_joint'],
  ikOptions: { positionTolerance: 0.0005, orientationTolerance: 0.001 },
  ikRecords: makeRecords(),
  cupWorldAabb: null,
  evaluateSegmentSafety: safeSegment,
  ...overrides,
});

describe('UMI 时间化 waypoint plan', () => {
  it('只处理 scheduler 选中的 action 6 至 15，并保留时间戳与重规划边界', () => {
    const schedule = makeSchedule();
    const checkedSegments: Array<{ start: JointValues; end: JointValues; index: number }> = [];
    const plan = createUmiScheduledWaypointPlan(
      makePlanInput({
        ikRecords: makeRecords(schedule.selectedActionIndexes),
        evaluateSegmentSafety: (start, end, index) => {
          checkedSegments.push({ start, end, index });
          return safeSegment();
        },
      }),
    );

    expect(schedule.selectedActionIndexes).toEqual([6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
    expect(plan.selected_indexes).toEqual(schedule.selectedActionIndexes);
    expect(plan.waypoints.map(({ action_index }) => action_index)).toEqual(
      schedule.selectedActionIndexes,
    );
    expect(plan.waypoints.map(({ scheduled_timestamp_ms }) => scheduled_timestamp_ms)).toEqual(
      schedule.selectedActionIndexes.map(
        (actionIndex) => schedule.actionTimings[actionIndex].effectiveScheduledTimestampMs,
      ),
    );
    expect(plan.next_replan_timestamp_ms).toBe(schedule.nextReplanTimestampMs);
    expect(plan.execution_count).toBe(0);
    expect(checkedSegments).toHaveLength(10);
    expect(checkedSegments[0].start).toEqual(initialJoints);
    expect(checkedSegments[1].start).toEqual(plan.waypoints[0].target_joints);
    expect(checkedSegments[1].end).toEqual(plan.waypoints[1].target_joints);
    expect(plan.safe_prefix_indexes).toEqual(schedule.selectedActionIndexes);
    expect(plan.plan_verdict).toBe('ALL_SELECTED_WAYPOINTS_PASSED');
  });

  it('keeps ordinary playback horizon selection separate from timestamp selection', () => {
    const actions = makeActions();

    expect(selectExecutionActions(actions).selectedActionIndexes).toEqual([0, 1, 2, 3, 4, 5]);
    expect(createUmiScheduledWaypointPlan(makePlanInput()).selected_indexes).toEqual([
      6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    ]);
  });

  it('decodes every relative target from the same observation TCP pose', () => {
    const plan = createUmiScheduledWaypointPlan(makePlanInput());

    plan.waypoints.forEach((waypoint) => {
      expect(waypoint.absolute_tcp_target.position[0]).toBeCloseTo(1);
      expect(waypoint.absolute_tcp_target.position[1]).toBeCloseTo(2 + waypoint.action_index / 100);
      expect(waypoint.absolute_tcp_target.position[2]).toBeCloseTo(3);
    });
    expect(plan.target_pose_reference).toBe('same_observation_tcp_for_all_relative_actions');
  });

  it('propagates each successful target joint state as the next IK seed', () => {
    const plan = createUmiScheduledWaypointPlan(makePlanInput());

    expect(plan.waypoints[0].ik_seed_joints).toEqual(initialJoints);
    plan.waypoints.slice(1).forEach((waypoint, index) => {
      expect(waypoint.ik_seed_joints).toEqual(plan.waypoints[index].target_joints);
    });
  });

  it('uses the worker action output seed for the complete arm and gripper target', () => {
    const schedule = makeSchedule();
    const records = makeRecords(schedule.selectedActionIndexes);
    const first = records.get(schedule.selectedActionIndexes[0]) as IKActionDebugRecord;
    first.final_joint_values = { shoulder: 0.01, left_jaw_joint: 0, right_jaw_joint: 0 };
    first.action_output_seed = {
      shoulder: 0.01,
      left_jaw_joint: 0.01,
      right_jaw_joint: 0.01,
    };

    const plan = createUmiScheduledWaypointPlan(makePlanInput({ ikRecords: records }));

    expect(plan.waypoints[0].target_joints).toEqual(first.action_output_seed);
    expect(plan.waypoints[0].gripper_target_opening_m).toBeCloseTo(0.02);
    expect(plan.waypoints[0].block_reason).toBeUndefined();
    expect(plan.safe_prefix_indexes[0]).toBe(schedule.selectedActionIndexes[0]);
  });

  it('continues after an in-tolerance arm no-op and accepts a gripper-only waypoint', () => {
    const schedule = makeSchedule();
    const actions = makeActions();
    actions[6][6] = 0.03;
    const records = makeRecords(schedule.selectedActionIndexes);
    const first = records.get(6) as IKActionDebugRecord;
    first.final_joint_values = { ...initialJoints, left_jaw_joint: 0.015, right_jaw_joint: 0.015 };
    first.action_output_seed = { ...first.final_joint_values };
    first.position_residual = 0.0002;
    first.orientation_residual = 0.0005;
    const second = records.get(7) as IKActionDebugRecord;
    second.seed_joint_values = { ...first.final_joint_values };
    second.final_joint_values = {
      shoulder: 0.01,
      left_jaw_joint: 0.01,
      right_jaw_joint: 0.01,
    };
    second.action_output_seed = { ...second.final_joint_values };

    const plan = createUmiScheduledWaypointPlan(makePlanInput({ actions, ikRecords: records }));

    expect(plan.waypoints[0].arm_motion_classification).toBe('VALID_ARM_NO_OP_WITHIN_IK_TOLERANCE');
    expect(plan.waypoints[0].waypoint_classification).toBe('VALID_GRIPPER_ONLY');
    expect(plan.waypoints[0].gripper_target_opening_m).toBeCloseTo(0.03);
    expect(plan.waypoints[1].plan_status).toBe('SAFE_PREFIX');
    expect(plan.safe_prefix_indexes).toContain(7);
  });

  it('classifies a full no-op as valid when IK and both residual tolerances pass', () => {
    const schedule = makeSchedule();
    const records = makeRecords(schedule.selectedActionIndexes);
    const first = records.get(6) as IKActionDebugRecord;
    first.final_joint_values = { ...initialJoints };
    first.action_output_seed = { ...initialJoints };
    const actions = makeActions();
    actions[6][6] = 0;

    const plan = createUmiScheduledWaypointPlan(makePlanInput({ actions, ikRecords: records }));

    expect(plan.waypoints[0].waypoint_classification).toBe('VALID_FULL_NO_OP');
    expect(plan.waypoints[0].plan_status).toBe('SAFE_PREFIX');
  });

  it('blocks an IK failure and marks every later selected waypoint non-executable', () => {
    const schedule = makeSchedule();
    const records = makeRecords(schedule.selectedActionIndexes);
    records.set(
      8,
      makeRecord(
        8,
        records.get(7)?.final_joint_values ?? initialJoints,
        {},
        {
          status: 'failure',
          final_joint_values: null,
          position_residual: 0.02,
          error_message: 'IK did not converge',
        },
      ),
    );

    const plan = createUmiScheduledWaypointPlan(makePlanInput({ ikRecords: records }));

    expect(plan.safe_prefix_indexes).toEqual([6, 7]);
    expect(plan.blocked_action_index).toBe(8);
    expect(plan.plan_verdict).toBe('BLOCKED_AT_ACTION_8');
    expect(plan.waypoints[2].plan_status).toBe('BLOCKED');
    expect(plan.waypoints[2].gripper_target_opening_m).toBeCloseTo(0.02);
    expect(
      plan.waypoints
        .slice(3)
        .every((waypoint) => waypoint.plan_status === 'NOT_EXECUTABLE_AFTER_BLOCK'),
    ).toBe(true);
  });

  it('blocks an unsafe swept segment and does not inspect later segments', () => {
    const schedule = makeSchedule();
    const records = makeRecords(schedule.selectedActionIndexes);
    const evaluatedIndexes: number[] = [];

    const plan = createUmiScheduledWaypointPlan(
      makePlanInput({
        ikRecords: records,
        evaluateSegmentSafety: (_start, _end, segmentIndex) => {
          evaluatedIndexes.push(schedule.selectedActionIndexes[segmentIndex]);
          return segmentIndex === 2
            ? ({
                verdict: 'STOP_BEFORE_UNVALIDATED_CONTACT',
                would_enter_unvalidated_contact: true,
              } as PreContactSafetyResult)
            : safeSegment();
        },
      }),
    );

    expect(evaluatedIndexes).toEqual([6, 7, 8]);
    expect(plan.safe_prefix_indexes).toEqual([6, 7]);
    expect(plan.blocked_action_index).toBe(8);
    expect(plan.waypoints[2].waypoint_classification).toBe('BLOCKED');
    expect(
      plan.waypoints
        .slice(3)
        .every((waypoint) => waypoint.plan_status === 'NOT_EXECUTABLE_AFTER_BLOCK'),
    ).toBe(true);
  });

  it('fails closed for a target beyond a URDF joint limit', () => {
    const schedule = makeSchedule();
    const records = makeRecords(schedule.selectedActionIndexes);
    const first = records.get(6) as IKActionDebugRecord;
    first.final_joint_values = { shoulder: 2, left_jaw_joint: 0.01, right_jaw_joint: 0.01 };
    first.action_output_seed = { ...first.final_joint_values };

    const plan = createUmiScheduledWaypointPlan(makePlanInput({ ikRecords: records }));

    expect(plan.blocked_action_index).toBe(6);
    expect(plan.waypoints[0].block_reason).toContain('URDF limits');
    expect(plan.safe_prefix_indexes).toEqual([]);
  });

  it('fails closed when any selected timestamp is not strictly increasing', () => {
    const schedule = makeSchedule();
    schedule.actionTimings[7].effectiveScheduledTimestampMs = 1500;

    const plan = createUmiScheduledWaypointPlan(makePlanInput({ schedule }));

    expect(plan.blocked_action_index).toBe(7);
    expect(plan.waypoints[1].block_reason).toContain('strictly increasing');
    expect(
      plan.waypoints
        .slice(2)
        .every((waypoint) => waypoint.plan_status === 'NOT_EXECUTABLE_AFTER_BLOCK'),
    ).toBe(true);
  });

  it('blocks when the worker IK seed is not the observation or previous predicted state', () => {
    const schedule = makeSchedule();
    const records = makeRecords(schedule.selectedActionIndexes);
    const second = records.get(7) as IKActionDebugRecord;
    second.seed_joint_values = { ...initialJoints };

    const plan = createUmiScheduledWaypointPlan(makePlanInput({ ikRecords: records }));

    expect(plan.blocked_action_index).toBe(7);
    expect(plan.waypoints[1].block_reason).toContain('IK seed');
    expect(plan.safe_prefix_indexes).toEqual([6]);
  });

  it('simulates an earlier replan waypoint replacing old future targets and later ones continuing it', () => {
    const oldFuture = Array.from({ length: 10 }, (_, index) => ({
      actionIndex: index + 6,
      scheduledTimestampMs: 1600 + index * 100,
      value: `A${index + 6}`,
    }));
    const incoming = [
      { actionIndex: 6, scheduledTimestampMs: 1700, value: 'B6' },
      { actionIndex: 7, scheduledTimestampMs: 1800, value: 'B7' },
    ];
    const result = simulateOfficialWaypointReplan(oldFuture, incoming, 1550);

    expect(result.map(({ value }) => value)).toEqual(['B6', 'B7']);
    expect(result.map(({ scheduledTimestampMs }) => scheduledTimestampMs)).toEqual([1700, 1800]);
  });

  it('appends a waypoint scheduled later than the existing future trajectory', () => {
    const existing = [{ actionIndex: 9, scheduledTimestampMs: 1900, value: 'A9' }];
    const result = applyOfficialWaypointQueueInsertion(
      existing,
      { actionIndex: 10, scheduledTimestampMs: 2000, value: 'B10' },
      1550,
    );

    expect(result.map(({ value }) => value)).toEqual(['A9', 'B10']);
  });

  it('ignores an incoming waypoint at or before current queue time', () => {
    const existing = [{ actionIndex: 9, scheduledTimestampMs: 1900, value: 'A9' }];

    expect(
      applyOfficialWaypointQueueInsertion(
        existing,
        { actionIndex: 10, scheduledTimestampMs: 1550, value: 'B10' },
        1550,
      ),
    ).toEqual(existing);
  });
});
