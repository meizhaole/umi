// 验证时间戳 waypoint 队列的插值、重规划、跟踪保护与有界停止。
import { describe, expect, it, vi } from 'vitest';
import type { JointValues, Pose } from '../src/core/types';
import { selectExecutionActions } from '../src/app/executionHorizon';
import type { UmiScheduledWaypointPlan } from '../src/app/umiScheduledWaypointPlan';
import {
  MAX_INFERENCE_CYCLES,
  UmiScheduledWaypointController,
} from '../src/sim/control/UmiScheduledWaypointController';

const poseFor = (joints: JointValues): Pose => ({
  position: [joints.shoulder ?? 0, 0, 0],
  orientation: [0, 0, 0, 1],
});

const createController = (
  gripperOpeningVelocityMPerSec = 100,
  validateSegmentSafety = () => ({ verdict: 'SAFE_TO_EXECUTE' }),
) =>
  new UmiScheduledWaypointController({
    armJointNames: ['shoulder'],
    gripperJointNames: ['left_jaw_joint', 'right_jaw_joint'],
    armJointVelocityRadPerSec: { shoulder: 100 },
    gripperOpeningVelocityMPerSec,
    validateSegmentSafety,
    getTcpPose: poseFor,
    getGripperOpening: (joints) => (joints.left_jaw_joint ?? 0) + (joints.right_jaw_joint ?? 0),
    trackingTolerances: {
      jointErrorRad: 0.005,
      tcpPositionErrorM: 0.0005,
      tcpOrientationErrorRad: 0.001,
      gripperErrorM: 0.0005,
    },
  });

const makeWaypoint = (actionIndex: number, timestamp: number, shoulder: number) => ({
  request_id: `request-${actionIndex}`,
  episode_index: 4,
  frame_index: 10 + actionIndex,
  action_index: actionIndex,
  scheduled_timestamp_ms: timestamp,
  absolute_tcp_target: poseFor({ shoulder }),
  ik_seed_joints: { shoulder: 0, left_jaw_joint: 0, right_jaw_joint: 0 },
  target_joints: {
    shoulder,
    left_jaw_joint: shoulder / 10,
    right_jaw_joint: shoulder / 10,
  },
  joint_delta: null,
  position_residual_m: 0,
  orientation_residual_rad: 0,
  ik_success: true,
  gripper_current_predicted_opening_m: 0,
  gripper_target_opening_m: shoulder / 5,
  gripper_delta_m: shoulder / 5,
  gripper_target_joints: { left_jaw_joint: shoulder / 10, right_jaw_joint: shoulder / 10 },
  arm_motion_classification: 'SAFE_ARM_MOTION' as const,
  waypoint_classification: 'SAFE_ARM_MOTION' as const,
  pre_contact_verdict: 'SAFE_TO_EXECUTE',
  pre_contact_safety: null,
  segment_start_joints: { shoulder: 0, left_jaw_joint: 0, right_jaw_joint: 0 },
  segment_end_joints: {
    shoulder,
    left_jaw_joint: shoulder / 10,
    right_jaw_joint: shoulder / 10,
  },
  plan_status: 'SAFE_PREFIX' as const,
});

const makePlan = (
  requestId: string,
  waypoints: ReturnType<typeof makeWaypoint>[],
  planVerdict: UmiScheduledWaypointPlan['plan_verdict'] = 'ALL_SELECTED_WAYPOINTS_PASSED',
): UmiScheduledWaypointPlan => ({
  request_id: requestId,
  episode_index: 4,
  frame_index: 1,
  observation_timestamp_ms: 500,
  scheduler_now_ms: 600,
  next_replan_timestamp_ms: 1100,
  target_pose_reference: 'same_observation_tcp_for_all_relative_actions',
  selected_indexes: waypoints.map((waypoint) => waypoint.action_index),
  safe_prefix_indexes:
    planVerdict === 'ALL_SELECTED_WAYPOINTS_PASSED'
      ? waypoints.map((waypoint) => waypoint.action_index)
      : [],
  blocked_action_index: planVerdict === 'ALL_SELECTED_WAYPOINTS_PASSED' ? null : 8,
  plan_verdict: planVerdict,
  execution_count: 0,
  cup_world_aabb: null,
  waypoints: waypoints.map((waypoint) => ({ ...waypoint, request_id: requestId })),
});

const initialJoints: JointValues = {
  shoulder: 0,
  left_jaw_joint: 0,
  right_jaw_joint: 0,
};

const trackedMidpoint: JointValues = {
  shoulder: 0.1,
  left_jaw_joint: 0.01,
  right_jaw_joint: 0.01,
};

const trackedState = (shoulder: number): JointValues => ({
  shoulder,
  left_jaw_joint: shoulder / 10,
  right_jaw_joint: shoulder / 10,
});

describe('UMI timestamp scheduled waypoint controller', () => {
  it('accepts only finite handoff samples stable within existing tracking tolerances', () => {
    const controller = createController();
    const before: JointValues = {
      shoulder: 0.2,
      left_jaw_joint: 0.03,
      right_jaw_joint: 0.03,
    };
    const stable: JointValues = {
      ...before,
      left_jaw_joint: 0.0301,
      right_jaw_joint: 0.0301,
    };
    const movingGripper: JointValues = {
      ...before,
      left_jaw_joint: 0.0303,
      right_jaw_joint: 0.0303,
    };

    expect(controller.areExecutionStatesStable(before, stable)).toBe(true);
    expect(controller.areExecutionStatesStable(before, movingGripper)).toBe(false);
    expect(controller.isExecutionStateValid({ shoulder: 0.2 })).toBe(false);
    controller.startRun(100, before);
    expect(controller.snapshot().handoff_joint_values).toEqual(before);
  });

  it('uses remaining time to the first waypoint and interpolates arm and gripper on one clock', () => {
    const controller = createController();
    controller.startRun(600, initialJoints);
    expect(controller.beginInferenceCycle(600)).toBe(1);
    const first = makeWaypoint(7, 700, 0.2);

    const insertion = controller.completeInferenceCycle(
      makePlan('plan-A', [first]),
      601,
      initialJoints,
    );
    expect(insertion.safe).toBe(true);
    expect(insertion.available_time_to_first_waypoint_ms).toBe(99);

    const tick = controller.advance(650.5, trackedMidpoint);

    expect(tick.state).toBe('RUNNING');
    expect(tick.commanded_joint_values).toEqual(trackedMidpoint);
    expect(tick.clock_domain).toBe('window.performance.now');
    expect(controller.advance(700, first.target_joints as JointValues).events).toContainEqual(
      expect.objectContaining({ status: 'REACHED', action_index: 7 }),
    );
  });

  it('extends a 40.8 ms 0 mm to 61 mm gripper move to its velocity-feasible arrival', () => {
    const controller = createController(0.2);
    const initial: JointValues = {
      shoulder: 0,
      left_jaw_joint: 0,
      right_jaw_joint: 0,
    };
    const target = {
      ...makeWaypoint(3, 140.8, 0),
      target_joints: {
        shoulder: 0,
        left_jaw_joint: 0.0305,
        right_jaw_joint: 0.0305,
      },
      gripper_target_opening_m: 0.061,
      gripper_delta_m: 0.061,
      gripper_target_joints: {
        left_jaw_joint: 0.0305,
        right_jaw_joint: 0.0305,
      },
      segment_start_joints: {
        shoulder: 0,
        left_jaw_joint: 0,
        right_jaw_joint: 0,
      },
    };
    controller.startRun(100, initial);
    controller.beginInferenceCycle(100);
    controller.completeInferenceCycle(makePlan('plan-speed', [target]), 100, initial);

    expect(controller.snapshot().waypoints[0]).toMatchObject({
      scheduled_timestamp_ms: 140.8,
      effective_arrival_timestamp_ms: 405,
    });
  });

  it('holds instead of preparing a trajectory when a velocity limit is unavailable', () => {
    const controller = createController(0);
    controller.startRun(100, initialJoints);
    controller.beginInferenceCycle(100);

    const receipt = controller.completeInferenceCycle(
      makePlan('plan-no-gripper-speed', [makeWaypoint(3, 200, 0.1)]),
      100,
      initialJoints,
    );

    expect(receipt.safe).toBe(false);
    expect(receipt.reason).toContain('velocity constraint is missing or invalid');
    expect(controller.snapshot().state).toBe('HOLD_CURRENT_STATE');
  });

  it('uses the effective timeline for tracking and preserves monotonic waypoint arrivals', () => {
    const controller = createController(0.2);
    const initial: JointValues = {
      shoulder: 0,
      left_jaw_joint: 0,
      right_jaw_joint: 0,
    };
    const open = {
      ...makeWaypoint(3, 140.8, 0),
      target_joints: {
        shoulder: 0,
        left_jaw_joint: 0.0305,
        right_jaw_joint: 0.0305,
      },
      gripper_target_opening_m: 0.061,
      gripper_target_joints: {
        left_jaw_joint: 0.0305,
        right_jaw_joint: 0.0305,
      },
    };
    const close = {
      ...makeWaypoint(4, 181.6, 0),
      segment_start_joints: open.target_joints,
    };
    controller.startRun(100, initial);
    controller.beginInferenceCycle(100);
    controller.completeInferenceCycle(makePlan('plan-timeline', [open, close]), 100, initial);

    const nominalTimeCommand = {
      shoulder: 0,
      left_jaw_joint: (0.0305 * 40.8) / 305,
      right_jaw_joint: (0.0305 * 40.8) / 305,
    };
    const nominalTick = controller.advance(140.8, nominalTimeCommand);
    const rows = controller.snapshot().waypoints;
    const effectiveTick = controller.advance(405, open.target_joints as JointValues);

    expect(nominalTick.state).toBe('RUNNING');
    expect(nominalTick.events).toContainEqual(
      expect.objectContaining({ action_index: 3, status: 'ACTIVE' }),
    );
    expect(rows[1].effective_arrival_timestamp_ms).toBeGreaterThan(
      rows[0].effective_arrival_timestamp_ms as number,
    );
    expect(effectiveTick.events).toContainEqual(
      expect.objectContaining({ action_index: 3, status: 'REACHED' }),
    );
  });

  it('skips expired action 3 and rechecks action 4 from the current trajectory state', () => {
    const validateSegmentSafety = vi.fn(() => ({ verdict: 'SAFE_TO_EXECUTE' }));
    const controller = createController(100, validateSegmentSafety);
    const actual: JointValues = {
      shoulder: 0.2,
      left_jaw_joint: 0.01,
      right_jaw_joint: 0.01,
    };
    const expiredAction3 = makeWaypoint(3, 100, 0.8);
    const action4 = {
      ...makeWaypoint(4, 201, 0.6),
      segment_start_joints: expiredAction3.target_joints,
    };
    controller.startRun(90, actual);
    controller.beginInferenceCycle(90);

    const receipt = controller.completeInferenceCycle(
      makePlan('plan-expiry', [expiredAction3, action4]),
      101,
      actual,
    );
    const command = controller.advance(151, {
      shoulder: 0.4,
      left_jaw_joint: 0.03,
      right_jaw_joint: 0.03,
    }).commanded_joint_values;

    expect(receipt.queued_action_indexes).toEqual([4]);
    expect(controller.snapshot().waypoints[0].queue_decision).toBe('PAST_TIME_NOOP');
    expect(validateSegmentSafety).toHaveBeenCalledWith(actual, action4.target_joints, 4);
    expect(command?.shoulder).toBeCloseTo(0.4);
  });

  it('rejects an expired-action bridge that fails the existing pre-contact guard', () => {
    const validateSegmentSafety = vi.fn(() => ({
      verdict: 'STOP_BEFORE_UNVALIDATED_CONTACT',
      evaluation_error: 'test segment intersects the cup guard',
    }));
    const controller = createController(100, validateSegmentSafety);
    const actual: JointValues = {
      shoulder: 0.2,
      left_jaw_joint: 0.01,
      right_jaw_joint: 0.01,
    };
    const action3 = makeWaypoint(3, 100, 0.8);
    const action4 = {
      ...makeWaypoint(4, 201, 0.6),
      segment_start_joints: action3.target_joints,
    };
    controller.startRun(90, actual);
    controller.beginInferenceCycle(90);

    const receipt = controller.completeInferenceCycle(
      makePlan('plan-unsafe-bridge', [action3, action4]),
      101,
      actual,
    );

    expect(receipt.safe).toBe(false);
    expect(receipt.reason).toBe('test segment intersects the cup guard');
    expect(controller.snapshot().state).toBe('HOLD_CURRENT_STATE');
    expect(controller.snapshot().waypoints[1].queue_status).toBe('FAILED');
  });

  it('does not use an expired queued waypoint as the start of a replacement trajectory', () => {
    const validateSegmentSafety = vi.fn(() => ({ verdict: 'SAFE_TO_EXECUTE' }));
    const controller = createController(100, validateSegmentSafety);
    const initial: JointValues = {
      shoulder: 0,
      left_jaw_joint: 0,
      right_jaw_joint: 0,
    };
    const expiredAction3 = makeWaypoint(3, 200, 0.2);
    const futureAction4 = makeWaypoint(4, 400, 0.4);
    controller.startRun(100, initial);
    controller.beginInferenceCycle(100);
    controller.completeInferenceCycle(
      makePlan('plan-old', [expiredAction3, futureAction4]),
      100,
      initial,
    );

    const actual: JointValues = {
      shoulder: 0.12,
      left_jaw_joint: 0.012,
      right_jaw_joint: 0.012,
    };
    const replacementAction5 = makeWaypoint(5, 600, 0.6);
    controller.beginInferenceCycle(450);
    validateSegmentSafety.mockClear();
    controller.completeInferenceCycle(makePlan('plan-new', [replacementAction5]), 450, actual);

    const rows = controller.snapshot().waypoints;
    expect(
      rows
        .filter((row) => row.plan_id === 'plan-old:4:1')
        .map((row) => [row.queue_decision, row.queue_status]),
    ).toEqual([
      ['PAST_TIME_NOOP', 'CANCELLED'],
      ['PAST_TIME_NOOP', 'CANCELLED'],
    ]);
    expect(validateSegmentSafety).toHaveBeenCalledWith(actual, replacementAction5.target_joints, 5);
    const halfway = controller.advance(525, {
      shoulder: 0.36,
      left_jaw_joint: 0.036,
      right_jaw_joint: 0.036,
    });
    expect(halfway.commanded_joint_values?.shoulder).toBeCloseTo(0.36);
    expect(halfway.commanded_joint_values?.left_jaw_joint).toBeCloseTo(0.036);
  });

  it('uses a measured nonzero arm and gripper state as the first trajectory anchor', () => {
    const controller = createController();
    const actual: JointValues = {
      shoulder: 0.1,
      left_jaw_joint: 0.0185,
      right_jaw_joint: 0.0185,
    };
    const target = makeWaypoint(3, 700, 0.2);
    controller.startRun(600, actual);
    controller.beginInferenceCycle(600);
    controller.completeInferenceCycle(makePlan('plan-handoff', [target]), 601, actual);

    const midpoint = controller.advance(650.5, {
      shoulder: 0.15,
      left_jaw_joint: 0.01925,
      right_jaw_joint: 0.01925,
    });

    expect(controller.snapshot().handoff_joint_values).toEqual(actual);
    expect(midpoint.commanded_joint_values?.shoulder).toBeCloseTo(0.15);
    expect(midpoint.commanded_joint_values?.left_jaw_joint).toBeCloseTo(0.01925);
  });

  it('continues timestamp execution while inference cycle two is running', () => {
    const controller = createController();
    const first = makeWaypoint(7, 700, 0.2);
    controller.startRun(600, initialJoints);
    controller.beginInferenceCycle(600);
    controller.completeInferenceCycle(makePlan('plan-A', [first]), 601, initialJoints);
    controller.advance(650.5, trackedMidpoint);

    expect(controller.beginInferenceCycle(651)).toBe(2);
    const tick = controller.advance(675, trackedState((0.2 * 74) / 99));

    expect(controller.snapshot().inference_in_progress).toBe(true);
    expect(controller.snapshot().inference_execution_overlap_sample_count).toBe(1);
    expect(controller.snapshot().first_inference_execution_overlap_timestamp_ms).toBe(675);
    expect(tick.commanded_joint_values?.shoulder).toBeCloseTo(0.15);
    expect(tick.state).toBe('RUNNING');
  });

  it('replaces only future trajectory and preserves reached waypoint history', () => {
    const controller = createController();
    const a1 = makeWaypoint(7, 700, 0.1);
    const a2 = makeWaypoint(8, 800, 0.2);
    const a3 = makeWaypoint(9, 900, 0.3);
    controller.startRun(600, initialJoints);
    controller.beginInferenceCycle(600);
    controller.completeInferenceCycle(makePlan('plan-A', [a1, a2, a3]), 601, initialJoints);
    controller.advance(700, a1.target_joints as JointValues);
    controller.advance(750, trackedState(0.15));
    controller.beginInferenceCycle(751);
    const b1 = makeWaypoint(7, 780, 0.25);
    const b2 = makeWaypoint(8, 850, 0.3);

    const replacement = controller.completeInferenceCycle(makePlan('plan-B', [b1, b2]), 750, {
      shoulder: 0.15,
      left_jaw_joint: 0.015,
      right_jaw_joint: 0.015,
    });
    const rows = controller.snapshot().waypoints;

    expect(replacement.replaced_waypoints.map((waypoint) => waypoint.action_index)).toEqual([8, 9]);
    expect(
      rows.find((row) => row.plan_id === 'plan-A:4:1' && row.action_index === 7)?.queue_status,
    ).toBe('REACHED');
    expect(
      rows.find((row) => row.plan_id === 'plan-A:4:1' && row.action_index === 8)?.queue_status,
    ).toBe('REPLACED');
    expect(
      rows.filter((row) => row.plan_id === 'plan-B:4:1').map((row) => row.queue_status),
    ).toEqual(['PENDING', 'PENDING']);

    const tick = controller.advance(765, {
      shoulder: 0.2,
      left_jaw_joint: 0.02,
      right_jaw_joint: 0.02,
    });
    expect(tick.commanded_joint_values?.shoulder).toBeCloseTo(0.2);
  });

  it('appends later future waypoints and ignores timestamps that have already passed', () => {
    const controller = createController();
    const a1 = makeWaypoint(7, 800, 0.1);
    controller.startRun(600, initialJoints);
    controller.beginInferenceCycle(600);
    controller.completeInferenceCycle(makePlan('plan-A', [a1]), 601, initialJoints);
    controller.beginInferenceCycle(650);
    const alreadyPassed = makeWaypoint(8, 650, 0.2);
    const b1 = makeWaypoint(9, 900, 0.3);
    const appended = controller.completeInferenceCycle(
      makePlan('plan-B', [alreadyPassed, b1]),
      651,
      initialJoints,
    );

    expect(appended.replaced_waypoints).toEqual([]);
    expect(appended.prior_queue_at_insertion.future).toEqual([7]);
    expect(appended.queued_action_indexes).toEqual([9]);
    expect(appended.available_time_to_first_waypoint_ms).toBe(249);
    expect(
      controller.snapshot().waypoints.find((row) => row.action_index === 8)?.queue_decision,
    ).toBe('PAST_TIME_NOOP');
    expect(
      controller.snapshot().waypoints.filter((row) => row.queue_status === 'PENDING'),
    ).toHaveLength(2);
  });

  it('holds current state and cancels future queue when prediction B is unsafe', () => {
    const controller = createController();
    const a1 = makeWaypoint(7, 700, 0.2);
    const a2 = makeWaypoint(8, 800, 0.3);
    controller.startRun(600, initialJoints);
    controller.beginInferenceCycle(600);
    controller.completeInferenceCycle(makePlan('plan-A', [a1, a2]), 601, initialJoints);
    controller.advance(650.5, trackedMidpoint);
    controller.beginInferenceCycle(651);

    const receipt = controller.completeInferenceCycle(
      makePlan('plan-B', [makeWaypoint(8, 850, 0.4)], 'BLOCKED_AT_ACTION_8'),
      660,
      trackedMidpoint,
    );
    const tick = controller.advance(661, trackedMidpoint);

    expect(receipt.safe).toBe(false);
    expect(tick.state).toBe('HOLD_CURRENT_STATE');
    expect(tick.commanded_joint_values).toEqual(trackedMidpoint);
    expect(
      controller.snapshot().waypoints.filter((row) => row.plan_id === 'plan-A:4:1'),
    ).toMatchObject([{ queue_status: 'CANCELLED' }, { queue_status: 'CANCELLED' }]);
  });

  it('limits the run to two inference cycles and cancels future waypoints on stop', () => {
    const controller = createController();
    const target = makeWaypoint(7, 800, 0.2);
    controller.startRun(600, initialJoints);
    expect(controller.beginInferenceCycle(600)).toBe(1);
    controller.completeInferenceCycle(makePlan('plan-A', [target]), 601, initialJoints);
    expect(controller.beginInferenceCycle(700)).toBe(2);
    controller.completeInferenceCycle(
      makePlan('plan-B', [makeWaypoint(7, 900, 0.3)]),
      701,
      initialJoints,
    );

    expect(MAX_INFERENCE_CYCLES).toBe(2);
    expect(controller.beginInferenceCycle(800)).toBeNull();
    const stop = controller.stop(810, trackedMidpoint, 'TEST_STOP');

    expect(stop.final_joint_values).toEqual(trackedMidpoint);
    expect(controller.snapshot().waypoints.every((row) => row.queue_status === 'CANCELLED')).toBe(
      true,
    );
    expect(controller.snapshot().state).toBe('HOLD_CURRENT_STATE');
    expect(controller.advance(811, trackedMidpoint).commanded_joint_values).toEqual(
      trackedMidpoint,
    );
    expect(controller.advance(812, trackedMidpoint).commanded_joint_values).toBeNull();
  });

  it('fails closed on joint tracking error above the existing 0.005 rad threshold', () => {
    const controller = createController();
    const first = makeWaypoint(7, 700, 0.2);
    controller.startRun(600, initialJoints);
    controller.beginInferenceCycle(600);
    controller.completeInferenceCycle(makePlan('plan-A', [first]), 601, initialJoints);

    const tick = controller.advance(650, initialJoints);

    expect(tick.state).toBe('HOLD_CURRENT_STATE');
    expect(controller.snapshot().waypoints[0].queue_status).toBe('FAILED');
    expect(controller.snapshot().failure?.reason).toBe('TRACKING_ERROR');
  });

  it('keeps the existing non timestamp action selection for ordinary and action0 flows', () => {
    const actions = Array.from({ length: 16 }, (_, index) => [index]);

    expect(selectExecutionActions(actions).selectedActionIndexes).toEqual([0, 1, 2, 3, 4, 5]);
    expect(selectExecutionActions(actions, 1).selectedActionIndexes).toEqual([0]);
  });
});
