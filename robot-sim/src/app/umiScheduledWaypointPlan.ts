import type {
  IKOptions,
  JointDescription,
  JointValues,
  Pose,
  RobotDescription,
} from '../core/types';
import { composePoses, quaternionFromAxisAngle } from '../utils/math';
import type { IKActionDebugRecord } from './inferenceProtocol';
import { getGripperOpening, mapGripperOpeningToJoints } from './gripperMapping';
import type { UmiActionSchedule } from './umiActionScheduler';
import type { PreContactSafetyResult, WorldAabb } from '../sim/tasks/cupApproachPreContactSafety';

export type WaypointClassification =
  | 'SAFE_ARM_MOTION'
  | 'VALID_ARM_NO_OP_WITHIN_IK_TOLERANCE'
  | 'VALID_GRIPPER_ONLY'
  | 'VALID_FULL_NO_OP'
  | 'BLOCKED';

export type ArmMotionClassification =
  'SAFE_ARM_MOTION' | 'VALID_ARM_NO_OP_WITHIN_IK_TOLERANCE' | 'BLOCKED';

export interface UmiScheduledWaypoint {
  request_id: string;
  episode_index: number;
  frame_index: number;
  action_index: number;
  scheduled_timestamp_ms: number | null;
  absolute_tcp_target: Pose;
  ik_seed_joints: JointValues | null;
  target_joints: JointValues | null;
  joint_delta: JointValues | null;
  position_residual_m: number | null;
  orientation_residual_rad: number | null;
  ik_success: boolean;
  gripper_current_predicted_opening_m: number | null;
  gripper_target_opening_m: number | null;
  gripper_delta_m: number | null;
  gripper_target_joints: JointValues | null;
  arm_motion_classification: ArmMotionClassification;
  waypoint_classification: WaypointClassification;
  pre_contact_verdict: string;
  pre_contact_safety: PreContactSafetyResult | null;
  segment_start_joints: JointValues | null;
  segment_end_joints: JointValues | null;
  plan_status: 'SAFE_PREFIX' | 'BLOCKED' | 'NOT_EXECUTABLE_AFTER_BLOCK';
  gripper_mapping_error?: string;
  block_reason?: string;
}

export interface UmiScheduledWaypointPlan {
  request_id: string;
  episode_index: number;
  frame_index: number;
  observation_timestamp_ms: number;
  scheduler_now_ms: number;
  next_replan_timestamp_ms: number;
  target_pose_reference: 'same_observation_tcp_for_all_relative_actions';
  selected_indexes: number[];
  safe_prefix_indexes: number[];
  blocked_action_index: number | null;
  plan_verdict: 'ALL_SELECTED_WAYPOINTS_PASSED' | `BLOCKED_AT_ACTION_${number}`;
  execution_count: 0;
  cup_world_aabb: WorldAabb | null;
  waypoints: UmiScheduledWaypoint[];
}

export interface UmiScheduledWaypointPlanInput {
  requestId: string;
  episodeIndex: number;
  frameIndex: number;
  observationTimestampMs: number;
  observationTcpPose: Pose;
  observationJointValues: JointValues;
  observationGripperOpeningM: number;
  actionPoseRepr: string;
  actions: readonly number[][];
  schedule: UmiActionSchedule;
  description: RobotDescription;
  gripperJointNames?: readonly string[];
  ikOptions?: IKOptions;
  ikRecords: ReadonlyMap<number, IKActionDebugRecord>;
  cupWorldAabb: WorldAabb | null;
  evaluateSegmentSafety: (
    segmentStartJoints: JointValues,
    segmentEndJoints: JointValues,
    segmentIndex: number,
    cupWorldAabb: WorldAabb | null,
  ) => PreContactSafetyResult;
  workerFailure?: {
    actionIndex: number | null;
    message: string;
  };
}

type MovableJoint = JointDescription & {
  type: 'revolute' | 'continuous' | 'prismatic';
};

const MOVABLE_JOINT_TYPES = new Set(['revolute', 'continuous', 'prismatic']);
const ARM_JOINT_TYPES = new Set(['revolute', 'continuous']);
const ZERO_JOINT_DELTA = 1e-12;

const getActionTargetPose = (observationTcpPose: Pose, action: readonly number[]): Pose => {
  const rotationVector = action.slice(3, 6) as [number, number, number];
  const angle = Math.hypot(...rotationVector);
  const deltaPose: Pose = {
    position: action.slice(0, 3) as [number, number, number],
    orientation:
      angle > 0
        ? quaternionFromAxisAngle(
            rotationVector.map((value) => value / angle) as [number, number, number],
            angle,
          )
        : [0, 0, 0, 1],
  };
  return composePoses(observationTcpPose, deltaPose);
};

const getJointDelta = (
  joints: readonly MovableJoint[],
  seed: JointValues,
  target: JointValues,
): JointValues =>
  Object.fromEntries(joints.map((joint) => [joint.name, target[joint.name] - seed[joint.name]]));

const hasFiniteJointValues = (joints: readonly MovableJoint[], values: JointValues): boolean =>
  joints.every((joint) => Number.isFinite(values[joint.name]));

const hasJointValuesWithinLimits = (
  joints: readonly MovableJoint[],
  values: JointValues,
): boolean =>
  joints.every((joint) => {
    const value = values[joint.name];
    return (
      Number.isFinite(value) &&
      (joint.limit?.lower === undefined || value >= joint.limit.lower) &&
      (joint.limit?.upper === undefined || value <= joint.limit.upper)
    );
  });

const jointValuesMatch = (
  joints: readonly MovableJoint[],
  left: JointValues,
  right: JointValues,
): boolean =>
  joints.every(
    (joint) =>
      Number.isFinite(left[joint.name]) &&
      Number.isFinite(right[joint.name]) &&
      Math.abs(left[joint.name] - right[joint.name]) <= 1e-9,
  );

const createSkippedWaypoint = (
  input: UmiScheduledWaypointPlanInput,
  actionIndex: number,
  previousBlockedIndex: number,
): UmiScheduledWaypoint => {
  const timing = input.schedule.actionTimings.find((item) => item.actionIndex === actionIndex);
  return {
    request_id: input.requestId,
    episode_index: input.episodeIndex,
    frame_index: input.frameIndex,
    action_index: actionIndex,
    scheduled_timestamp_ms: timing?.effectiveScheduledTimestampMs ?? null,
    absolute_tcp_target: getActionTargetPose(input.observationTcpPose, input.actions[actionIndex]),
    ik_seed_joints: null,
    target_joints: null,
    joint_delta: null,
    position_residual_m: null,
    orientation_residual_rad: null,
    ik_success: false,
    gripper_current_predicted_opening_m: null,
    gripper_target_opening_m: null,
    gripper_delta_m: null,
    gripper_target_joints: null,
    arm_motion_classification: 'BLOCKED',
    waypoint_classification: 'BLOCKED',
    pre_contact_verdict: 'NOT_EVALUATED_AFTER_BLOCK',
    pre_contact_safety: null,
    segment_start_joints: null,
    segment_end_joints: null,
    plan_status: 'NOT_EXECUTABLE_AFTER_BLOCK',
    block_reason: `action ${previousBlockedIndex} blocked; later waypoint not evaluated`,
  };
};

export const createUmiScheduledWaypointPlan = (
  input: UmiScheduledWaypointPlanInput,
): UmiScheduledWaypointPlan => {
  if (input.actionPoseRepr !== 'relative') {
    throw new Error(`不支持时间化路点规划的 action_pose_repr：${input.actionPoseRepr}`);
  }
  if (input.schedule.selectedActionIndexes.length === 0) {
    throw new Error('时间戳调度没有选中任何 action。');
  }

  const selectedIndexes = [...input.schedule.selectedActionIndexes];
  const selectedTargetPoses = new Map(
    selectedIndexes.map((actionIndex) => [
      actionIndex,
      getActionTargetPose(input.observationTcpPose, input.actions[actionIndex]),
    ]),
  );
  const movableJoints = input.description.joints.filter((joint) =>
    MOVABLE_JOINT_TYPES.has(joint.type),
  ) as MovableJoint[];
  const armJoints = movableJoints.filter((joint) => ARM_JOINT_TYPES.has(joint.type));
  const positionTolerance = input.ikOptions?.positionTolerance ?? 0.005;
  const orientationTolerance = input.ikOptions?.orientationTolerance ?? 0.01;
  const waypoints: UmiScheduledWaypoint[] = [];
  const safePrefixIndexes: number[] = [];
  let segmentStartJoints = { ...input.observationJointValues };
  let predictedGripperOpening = input.observationGripperOpeningM;
  let previousTimestampMs = Number.NEGATIVE_INFINITY;
  let blockedActionIndex: number | null = null;
  let workerFailureConsumed = false;

  for (const [segmentIndex, actionIndex] of selectedIndexes.entries()) {
    if (blockedActionIndex !== null) {
      waypoints.push(createSkippedWaypoint(input, actionIndex, blockedActionIndex));
      continue;
    }

    const action = input.actions[actionIndex];
    const timing = input.schedule.actionTimings.find((item) => item.actionIndex === actionIndex);
    const absoluteTcpTarget = selectedTargetPoses.get(actionIndex) as Pose;
    const ikRecord = input.ikRecords.get(actionIndex);
    const ikSeedJoints = ikRecord?.seed_joint_values ?? null;
    const targetJoints = ikRecord?.action_output_seed ?? null;
    const positionResidual = ikRecord?.position_residual ?? null;
    const orientationResidual = ikRecord?.orientation_residual ?? null;
    const jointDelta =
      ikSeedJoints && targetJoints
        ? getJointDelta(movableJoints, ikSeedJoints, targetJoints)
        : null;
    const armJointDeltaNorm =
      ikSeedJoints && targetJoints
        ? Math.hypot(
            ...armJoints.map((joint) => targetJoints[joint.name] - ikSeedJoints[joint.name]),
          )
        : null;
    const ikSucceeded = ikRecord?.status === 'success';
    const residualWithinTolerance =
      typeof positionResidual === 'number' &&
      typeof orientationResidual === 'number' &&
      Number.isFinite(positionResidual) &&
      Number.isFinite(orientationResidual) &&
      positionResidual <= positionTolerance &&
      orientationResidual <= orientationTolerance;
    const gripperCurrentOpening = predictedGripperOpening;
    let gripperTargetOpening: number | null = null;
    let gripperDelta: number | null = null;
    let gripperTargetJoints: JointValues | null = null;
    let gripperMappingError: string | undefined;
    let blockReason: string | undefined;
    let preContactSafety: PreContactSafetyResult | null = null;
    let preContactVerdict = 'NOT_EVALUATED';
    let waypointClassification: WaypointClassification = 'BLOCKED';
    let armMotionClassification: ArmMotionClassification = 'BLOCKED';

    try {
      const gripperMapping = mapGripperOpeningToJoints(
        input.description,
        action[6],
        input.gripperJointNames,
      );
      gripperTargetOpening = gripperMapping.clamped_gripper_opening_m;
      gripperTargetJoints = gripperMapping.joint_targets;
      gripperDelta = Number.isFinite(gripperCurrentOpening)
        ? gripperTargetOpening - gripperCurrentOpening
        : null;
    } catch (error) {
      gripperMappingError = error instanceof Error ? error.message : String(error);
    }

    if (
      timing?.effectiveScheduledTimestampMs === null ||
      timing?.effectiveScheduledTimestampMs === undefined ||
      !Number.isFinite(timing.effectiveScheduledTimestampMs)
    ) {
      blockReason = 'selected action has no finite scheduled timestamp';
    } else if (timing.effectiveScheduledTimestampMs <= previousTimestampMs) {
      blockReason = 'selected waypoint timestamps are not strictly increasing';
    } else if (!ikRecord) {
      const workerFailureApplies =
        input.workerFailure &&
        (input.workerFailure.actionIndex === actionIndex ||
          (input.workerFailure.actionIndex === null && !workerFailureConsumed));
      if (workerFailureApplies) {
        blockReason = input.workerFailure?.message;
        workerFailureConsumed = true;
      } else {
        blockReason = 'IK worker did not return a result for this selected action';
      }
    } else if (ikRecord.status !== 'success') {
      blockReason = ikRecord.error_message ?? ikRecord.termination_reason ?? 'IK failed';
    } else if (ikRecord.gripper_mapping_error || gripperMappingError) {
      blockReason = ikRecord.gripper_mapping_error ?? gripperMappingError;
    } else if (!residualWithinTolerance) {
      blockReason = 'IK residual exceeds the configured tolerance or is non-finite';
    } else if (
      !ikSeedJoints ||
      !jointValuesMatch(movableJoints, segmentStartJoints, ikSeedJoints)
    ) {
      blockReason = 'IK seed does not match the observation or previous predicted joints';
    } else if (!targetJoints || !hasFiniteJointValues(movableJoints, targetJoints)) {
      blockReason = 'IK target joints are missing or non-finite';
    } else if (!hasJointValuesWithinLimits(movableJoints, targetJoints)) {
      blockReason = 'IK target joints exceed URDF limits';
    } else if (armJointDeltaNorm === null || !Number.isFinite(armJointDeltaNorm)) {
      blockReason = 'arm joint delta is missing or non-finite';
    } else if (!Number.isFinite(gripperCurrentOpening)) {
      blockReason = 'observation gripper opening is missing or non-finite';
    }

    if (!blockReason && gripperTargetOpening !== null && targetJoints) {
      const predictedTargetOpening = getGripperOpening(
        input.description,
        targetJoints,
        input.gripperJointNames,
      );
      if (Math.abs(predictedTargetOpening - gripperTargetOpening) > 1e-9) {
        blockReason = 'IK target jaw joints do not match the mapped gripper opening';
      }
    }

    if (!blockReason) {
      const armIsNoOp = (armJointDeltaNorm as number) <= ZERO_JOINT_DELTA;
      const gripperIsNoOp = Math.abs(gripperDelta as number) <= ZERO_JOINT_DELTA;
      armMotionClassification = armIsNoOp
        ? 'VALID_ARM_NO_OP_WITHIN_IK_TOLERANCE'
        : 'SAFE_ARM_MOTION';
      waypointClassification = armIsNoOp
        ? gripperIsNoOp
          ? 'VALID_FULL_NO_OP'
          : 'VALID_GRIPPER_ONLY'
        : 'SAFE_ARM_MOTION';

      try {
        preContactSafety = input.evaluateSegmentSafety(
          segmentStartJoints,
          targetJoints as JointValues,
          segmentIndex,
          input.cupWorldAabb,
        );
        preContactVerdict = preContactSafety.verdict;
        if (preContactSafety.verdict !== 'SAFE_TO_EXECUTE') {
          blockReason =
            preContactSafety.evaluation_error ?? 'segment entered unvalidated pre-contact geometry';
        }
      } catch (error) {
        blockReason = error instanceof Error ? error.message : String(error);
        preContactVerdict = 'STOP_BEFORE_UNVALIDATED_CONTACT';
      }
    }

    if (blockReason) {
      blockedActionIndex = actionIndex;
      waypointClassification = 'BLOCKED';
      armMotionClassification = 'BLOCKED';
    } else {
      safePrefixIndexes.push(actionIndex);
      segmentStartJoints = { ...(targetJoints as JointValues) };
      predictedGripperOpening = gripperTargetOpening as number;
      previousTimestampMs = timing?.effectiveScheduledTimestampMs as number;
    }

    waypoints.push({
      request_id: input.requestId,
      episode_index: input.episodeIndex,
      frame_index: input.frameIndex,
      action_index: actionIndex,
      scheduled_timestamp_ms: timing?.effectiveScheduledTimestampMs ?? null,
      absolute_tcp_target: absoluteTcpTarget,
      ik_seed_joints: ikSeedJoints,
      target_joints: targetJoints,
      joint_delta: jointDelta,
      position_residual_m: positionResidual,
      orientation_residual_rad: orientationResidual,
      ik_success: ikSucceeded && residualWithinTolerance,
      gripper_current_predicted_opening_m: gripperCurrentOpening,
      gripper_target_opening_m: gripperTargetOpening,
      gripper_delta_m: gripperDelta,
      gripper_target_joints: gripperTargetJoints,
      arm_motion_classification: armMotionClassification,
      waypoint_classification: waypointClassification,
      pre_contact_verdict: preContactVerdict,
      pre_contact_safety: preContactSafety,
      segment_start_joints: ikSeedJoints,
      segment_end_joints: targetJoints,
      plan_status: blockReason ? 'BLOCKED' : 'SAFE_PREFIX',
      ...(gripperMappingError ? { gripper_mapping_error: gripperMappingError } : {}),
      ...(blockReason ? { block_reason: blockReason } : {}),
    });
  }

  return {
    request_id: input.requestId,
    episode_index: input.episodeIndex,
    frame_index: input.frameIndex,
    observation_timestamp_ms: input.observationTimestampMs,
    scheduler_now_ms: input.schedule.timingDiagnostics.schedulerCurrentTimestampMs,
    next_replan_timestamp_ms: input.schedule.nextReplanTimestampMs,
    target_pose_reference: 'same_observation_tcp_for_all_relative_actions',
    selected_indexes: selectedIndexes,
    safe_prefix_indexes: safePrefixIndexes,
    blocked_action_index: blockedActionIndex,
    plan_verdict:
      blockedActionIndex === null
        ? 'ALL_SELECTED_WAYPOINTS_PASSED'
        : (`BLOCKED_AT_ACTION_${blockedActionIndex}` as const),
    execution_count: 0,
    cup_world_aabb: input.cupWorldAabb,
    waypoints,
  };
};

export interface ScheduledWaypoint<T> {
  actionIndex: number;
  scheduledTimestampMs: number;
  value: T;
}

export const applyOfficialWaypointQueueInsertion = <T>(
  queuedWaypoints: readonly ScheduledWaypoint<T>[],
  incomingWaypoint: ScheduledWaypoint<T>,
  currentTimestampMs: number,
): ScheduledWaypoint<T>[] => {
  if (incomingWaypoint.scheduledTimestampMs <= currentTimestampMs) {
    return [...queuedWaypoints];
  }

  const lastWaypoint = queuedWaypoints.at(-1);
  const retainedWaypoints =
    lastWaypoint && incomingWaypoint.scheduledTimestampMs <= lastWaypoint.scheduledTimestampMs
      ? queuedWaypoints.filter((waypoint) => waypoint.scheduledTimestampMs <= currentTimestampMs)
      : [...queuedWaypoints];
  return [...retainedWaypoints, incomingWaypoint];
};

export const simulateOfficialWaypointReplan = <T>(
  queuedWaypoints: readonly ScheduledWaypoint<T>[],
  incomingWaypoints: readonly ScheduledWaypoint<T>[],
  currentTimestampMs: number,
): ScheduledWaypoint<T>[] =>
  incomingWaypoints.reduce(
    (queue, waypoint) => applyOfficialWaypointQueueInsertion(queue, waypoint, currentTimestampMs),
    [...queuedWaypoints],
  );
