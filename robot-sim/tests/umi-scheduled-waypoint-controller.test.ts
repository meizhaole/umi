// 验证时间戳 waypoint 队列的插值、重规划、跟踪保护与有界停止。
import { describe, expect, it } from 'vitest';
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

const createController = () =>
  new UmiScheduledWaypointController({
    armJointNames: ['shoulder'],
    gripperJointNames: ['left_jaw_joint', 'right_jaw_joint'],
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
  it('uses remaining time to the first waypoint and interpolates arm and gripper on one clock', () => {
    const controller = createController();
    controller.startRun(600);
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

  it('continues timestamp execution while inference cycle two is running', () => {
    const controller = createController();
    const first = makeWaypoint(7, 700, 0.2);
    controller.startRun(600);
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
    controller.startRun(600);
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
    controller.startRun(600);
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
    controller.startRun(600);
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
    controller.startRun(600);
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
  });

  it('fails closed on joint tracking error above the existing 0.005 rad threshold', () => {
    const controller = createController();
    const first = makeWaypoint(7, 700, 0.2);
    controller.startRun(600);
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
