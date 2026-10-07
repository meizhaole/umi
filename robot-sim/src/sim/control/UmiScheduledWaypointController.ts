import type { JointValues, Pose } from '../../core/types';
import { poseErrorRotation, vectorLength } from '../../utils/math';
import type { UmiScheduledWaypointPlan } from '../../app/umiScheduledWaypointPlan';
import {
  applyOfficialWaypointQueueInsertion,
  type ScheduledWaypoint,
} from '../../app/umiScheduledWaypointPlan';

export const MAX_INFERENCE_CYCLES = 2;

export type UmiWaypointQueueStatus =
  'PENDING' | 'ACTIVE' | 'REACHED' | 'REPLACED' | 'CANCELLED' | 'FAILED';

export type UmiScheduledControllerState = 'IDLE' | 'RUNNING' | 'HOLD_CURRENT_STATE';

export interface UmiScheduledWaypointControllerOptions {
  armJointNames: readonly string[];
  gripperJointNames: readonly string[];
  getTcpPose: (jointValues: JointValues) => Pose;
  getGripperOpening: (jointValues: JointValues) => number;
  trackingTolerances: {
    jointErrorRad: number;
    tcpPositionErrorM: number;
    tcpOrientationErrorRad: number;
    gripperErrorM: number;
  };
}

export interface UmiScheduledWaypointQueueRecord {
  plan_id: string;
  request_id: string;
  episode_index: number;
  frame_index: number;
  action_index: number;
  scheduled_timestamp_ms: number;
  queue_insert_timestamp_ms: number;
  active_timestamp_ms: number | null;
  source_observation_timestamp_ms: number;
  target_arm_joints: JointValues;
  target_joint_values: JointValues;
  target_gripper: number;
  target_tcp: Pose;
  pre_contact_verdict: string;
  queue_status: UmiWaypointQueueStatus;
  queue_decision: 'INSERTED' | 'PAST_TIME_NOOP';
  actual_joints_at_scheduled_time: JointValues | null;
  actual_gripper_at_scheduled_time: number | null;
  joint_tracking_error_rad: number | null;
  tcp_tracking_error_m: number | null;
  tcp_orientation_error_rad: number | null;
  gripper_tracking_error_m: number | null;
  actual_timestamp_ms: number | null;
  timing_error_ms: number | null;
  failure_reason?: string;
}

export interface UmiScheduledWaypointEvent {
  timestamp_ms: number;
  plan_id: string;
  action_index: number;
  status: UmiWaypointQueueStatus;
  reason?: string;
}

export interface UmiScheduledWaypointTick {
  state: UmiScheduledControllerState;
  clock_domain: 'window.performance.now';
  commanded_joint_values: JointValues | null;
  events: UmiScheduledWaypointEvent[];
}

export interface UmiScheduledWaypointSnapshot {
  state: UmiScheduledControllerState;
  cycles_started: number;
  inference_in_progress: boolean;
  inference_execution_overlap_sample_count: number;
  first_inference_execution_overlap_timestamp_ms: number | null;
  last_inference_execution_overlap_timestamp_ms: number | null;
  waypoints: UmiScheduledWaypointQueueRecord[];
  events: UmiScheduledWaypointEvent[];
  failure: { reason: string; timestamp_ms: number } | null;
  final_joint_values: JointValues | null;
}

export interface UmiScheduledPlanReceipt {
  safe: boolean;
  plan_id: string;
  cycle_index: number;
  available_time_to_first_waypoint_ms: number | null;
  queued_action_indexes: number[];
  replaced_waypoints: Array<{ plan_id: string; action_index: number }>;
  prior_queue_at_insertion: {
    reached: number[];
    active: number[];
    future: number[];
  };
  events: UmiScheduledWaypointEvent[];
  reason?: string;
}

type QueueEntry = ScheduledWaypoint<UmiScheduledWaypointQueueRecord>;

interface JointAnchor {
  timestamp_ms: number;
  joint_values: JointValues;
}

const cloneJointValues = (jointValues: JointValues): JointValues => ({ ...jointValues });

const clonePose = (pose: Pose): Pose => ({
  position: [...pose.position] as Pose['position'],
  orientation: [...pose.orientation] as Pose['orientation'],
});

const interpolateJointValues = (
  start: JointValues,
  end: JointValues,
  progress: number,
): JointValues =>
  Object.fromEntries(
    Object.entries(end).map(([name, endValue]) => {
      const startValue = start[name];
      return [name, startValue + (endValue - startValue) * progress];
    }),
  );

const maximumAbsoluteJointError = (
  jointNames: readonly string[],
  actual: JointValues,
  target: JointValues,
): number => Math.max(...jointNames.map((name) => Math.abs(actual[name] - target[name])));

const tcpPositionError = (actual: Pose, target: Pose): number =>
  Math.hypot(...actual.position.map((value, index) => value - target.position[index]));

export class UmiScheduledWaypointController {
  private state: UmiScheduledControllerState = 'IDLE';

  private cyclesStarted = 0;

  private inferenceInProgress = false;

  private queue: QueueEntry[] = [];

  private rows: UmiScheduledWaypointQueueRecord[] = [];

  private events: UmiScheduledWaypointEvent[] = [];

  private pendingEvents: UmiScheduledWaypointEvent[] = [];

  private anchor: JointAnchor | null = null;

  private heldJointValues: JointValues | null = null;

  private holdCommandPending = false;

  private failure: { reason: string; timestamp_ms: number } | null = null;

  private inferenceExecutionOverlapSampleCount = 0;

  private firstInferenceExecutionOverlapTimestampMs: number | null = null;

  private lastInferenceExecutionOverlapTimestampMs: number | null = null;

  private readonly options: UmiScheduledWaypointControllerOptions;

  constructor(options: UmiScheduledWaypointControllerOptions) {
    this.options = options;
  }

  startRun(timestampMs: number): void {
    if (!Number.isFinite(timestampMs))
      throw new Error('运行起始时间必须属于 performance.now 时钟域。');
    this.state = 'RUNNING';
    this.cyclesStarted = 0;
    this.inferenceInProgress = false;
    this.queue = [];
    this.rows = [];
    this.events = [];
    this.pendingEvents = [];
    this.anchor = null;
    this.heldJointValues = null;
    this.holdCommandPending = false;
    this.failure = null;
    this.inferenceExecutionOverlapSampleCount = 0;
    this.firstInferenceExecutionOverlapTimestampMs = null;
    this.lastInferenceExecutionOverlapTimestampMs = null;
  }

  beginInferenceCycle(timestampMs: number): number | null {
    if (
      this.state !== 'RUNNING' ||
      this.inferenceInProgress ||
      this.cyclesStarted >= MAX_INFERENCE_CYCLES ||
      !Number.isFinite(timestampMs)
    ) {
      return null;
    }
    this.cyclesStarted += 1;
    this.inferenceInProgress = true;
    return this.cyclesStarted;
  }

  completeInferenceCycle(
    plan: UmiScheduledWaypointPlan,
    insertionTimestampMs: number,
    actualJointValues: JointValues,
  ): UmiScheduledPlanReceipt {
    const planId = `${plan.request_id}:${plan.episode_index}:${plan.frame_index}`;
    const cycleIndex = this.cyclesStarted;
    const eventStartIndex = this.events.length;
    const inferenceWasInProgress = this.inferenceInProgress;
    this.inferenceInProgress = false;
    const priorQueueAtInsertion = this.getQueueClassification();
    const firstTimestamp = plan.waypoints.find(
      (waypoint) =>
        Number.isFinite(waypoint.scheduled_timestamp_ms) &&
        (waypoint.scheduled_timestamp_ms as number) > insertionTimestampMs,
    )?.scheduled_timestamp_ms;
    const availableTime =
      typeof firstTimestamp === 'number' ? firstTimestamp - insertionTimestampMs : null;

    if (
      this.state !== 'RUNNING' ||
      !inferenceWasInProgress ||
      cycleIndex > MAX_INFERENCE_CYCLES ||
      plan.plan_verdict !== 'ALL_SELECTED_WAYPOINTS_PASSED' ||
      plan.waypoints.length === 0 ||
      plan.waypoints.length !== plan.selected_indexes.length ||
      plan.waypoints.some(
        (waypoint, index) => waypoint.action_index !== plan.selected_indexes[index],
      ) ||
      plan.waypoints.some(
        (waypoint) =>
          waypoint.plan_status !== 'SAFE_PREFIX' ||
          waypoint.pre_contact_verdict !== 'SAFE_TO_EXECUTE' ||
          !Number.isFinite(waypoint.scheduled_timestamp_ms) ||
          !waypoint.target_joints ||
          !Object.values(waypoint.target_joints ?? {}).every(Number.isFinite) ||
          !Number.isFinite(waypoint.gripper_target_opening_m),
      )
    ) {
      this.stop(insertionTimestampMs, actualJointValues, 'UNSAFE_PLAN');
      return {
        safe: false,
        plan_id: planId,
        cycle_index: cycleIndex,
        available_time_to_first_waypoint_ms: availableTime,
        queued_action_indexes: [],
        replaced_waypoints: [],
        prior_queue_at_insertion: priorQueueAtInsertion,
        events: this.events.slice(eventStartIndex),
        reason: 'plan verdict or a selected waypoint is not executable',
      };
    }

    const inputRows: UmiScheduledWaypointQueueRecord[] = plan.waypoints.map((waypoint) => {
      const scheduledTimestamp = waypoint.scheduled_timestamp_ms as number;
      const targetJointValues = cloneJointValues(waypoint.target_joints as JointValues);
      return {
        plan_id: planId,
        request_id: waypoint.request_id,
        episode_index: waypoint.episode_index,
        frame_index: waypoint.frame_index,
        action_index: waypoint.action_index,
        scheduled_timestamp_ms: scheduledTimestamp,
        queue_insert_timestamp_ms: insertionTimestampMs,
        active_timestamp_ms: null,
        source_observation_timestamp_ms: plan.observation_timestamp_ms,
        target_arm_joints: Object.fromEntries(
          this.options.armJointNames.map((name) => [name, targetJointValues[name]]),
        ),
        target_joint_values: targetJointValues,
        target_gripper: waypoint.gripper_target_opening_m as number,
        target_tcp: clonePose(waypoint.absolute_tcp_target),
        pre_contact_verdict: waypoint.pre_contact_verdict,
        queue_status: 'PENDING',
        queue_decision: 'INSERTED',
        actual_joints_at_scheduled_time: null,
        actual_gripper_at_scheduled_time: null,
        joint_tracking_error_rad: null,
        tcp_tracking_error_m: null,
        tcp_orientation_error_rad: null,
        gripper_tracking_error_m: null,
        actual_timestamp_ms: null,
        timing_error_ms: null,
      };
    });
    this.rows.push(...inputRows);

    const replacedWaypoints: Array<{ plan_id: string; action_index: number }> = [];
    const queuedActionIndexes: number[] = [];
    const queueWasEmpty = this.queue.length === 0;
    let trajectoryReplaced = false;

    inputRows.forEach((row) => {
      const entry: QueueEntry = {
        actionIndex: row.action_index,
        scheduledTimestampMs: row.scheduled_timestamp_ms,
        value: row,
      };
      if (entry.scheduledTimestampMs <= insertionTimestampMs) {
        row.queue_decision = 'PAST_TIME_NOOP';
        this.setWaypointStatus(row, 'CANCELLED', insertionTimestampMs, 'PAST_TIME_NOOP');
        return;
      }

      const existingQueue = this.queue;
      const nextQueue = applyOfficialWaypointQueueInsertion(
        existingQueue,
        entry,
        insertionTimestampMs,
      );
      const retained = new Set(nextQueue.map(({ value }) => value));
      existingQueue.forEach(({ value }) => {
        if (retained.has(value)) return;
        replacedWaypoints.push({ plan_id: value.plan_id, action_index: value.action_index });
        this.setWaypointStatus(value, 'REPLACED', insertionTimestampMs, 'OFFICIAL_FUTURE_REPLACE');
        trajectoryReplaced = true;
      });
      this.queue = nextQueue;
      if (nextQueue.includes(entry)) queuedActionIndexes.push(row.action_index);
    });

    if (queuedActionIndexes.length > 0 && (queueWasEmpty || trajectoryReplaced)) {
      this.anchor = {
        timestamp_ms: insertionTimestampMs,
        joint_values: cloneJointValues(actualJointValues),
      };
    }

    return {
      safe: true,
      plan_id: planId,
      cycle_index: cycleIndex,
      available_time_to_first_waypoint_ms: availableTime,
      queued_action_indexes: queuedActionIndexes,
      replaced_waypoints: replacedWaypoints,
      prior_queue_at_insertion: priorQueueAtInsertion,
      events: this.events.slice(eventStartIndex),
    };
  }

  advance(timestampMs: number, actualJointValues: JointValues): UmiScheduledWaypointTick {
    if (!Number.isFinite(timestampMs)) throw new Error('播放时间必须属于 performance.now 时钟域。');
    if (this.state !== 'RUNNING') {
      const commandedJointValues = this.holdCommandPending
        ? cloneJointValues(this.heldJointValues ?? actualJointValues)
        : null;
      this.holdCommandPending = false;
      return {
        state: this.state,
        clock_domain: 'window.performance.now',
        commanded_joint_values: commandedJointValues,
        events: this.drainPendingEvents(),
      };
    }

    if (this.inferenceInProgress && this.queue.length > 0) {
      this.inferenceExecutionOverlapSampleCount += 1;
      this.firstInferenceExecutionOverlapTimestampMs ??= timestampMs;
      this.lastInferenceExecutionOverlapTimestampMs = timestampMs;
    }

    while (this.queue.length > 0) {
      const entry = this.queue[0];
      const row = entry.value;
      const anchor = this.anchor ?? {
        timestamp_ms: row.queue_insert_timestamp_ms,
        joint_values: cloneJointValues(actualJointValues),
      };
      const intervalMs = row.scheduled_timestamp_ms - anchor.timestamp_ms;
      if (intervalMs <= 0) {
        this.queue.shift();
        row.queue_decision = 'PAST_TIME_NOOP';
        this.setWaypointStatus(row, 'CANCELLED', timestampMs, 'PAST_TIME_NOOP');
        this.anchor = {
          timestamp_ms: timestampMs,
          joint_values: cloneJointValues(actualJointValues),
        };
        continue;
      }

      if (timestampMs < row.scheduled_timestamp_ms) {
        if (row.queue_status === 'PENDING') {
          row.active_timestamp_ms = timestampMs;
          this.setWaypointStatus(row, 'ACTIVE', timestampMs);
        }
        const progress = Math.min(Math.max((timestampMs - anchor.timestamp_ms) / intervalMs, 0), 1);
        const command = interpolateJointValues(
          anchor.joint_values,
          row.target_joint_values,
          progress,
        );
        const tracking = this.measureTracking(actualJointValues, command, null, null);
        if (this.isTrackingUnsafe(tracking)) {
          this.failActiveWaypoint(row, timestampMs, actualJointValues, tracking);
          return {
            state: this.state,
            clock_domain: 'window.performance.now',
            commanded_joint_values: cloneJointValues(this.heldJointValues as JointValues),
            events: this.drainPendingEvents(),
          };
        }
        return {
          state: this.state,
          clock_domain: 'window.performance.now',
          commanded_joint_values: command,
          events: this.drainPendingEvents(),
        };
      }

      const tracking = this.measureTracking(
        actualJointValues,
        row.target_joint_values,
        row.target_tcp,
        row.target_gripper,
      );
      this.recordScheduledActual(row, timestampMs, actualJointValues, tracking);
      if (this.isTrackingUnsafe(tracking)) {
        this.failActiveWaypoint(row, timestampMs, actualJointValues, tracking);
        return {
          state: this.state,
          clock_domain: 'window.performance.now',
          commanded_joint_values: cloneJointValues(this.heldJointValues as JointValues),
          events: this.drainPendingEvents(),
        };
      }

      this.queue.shift();
      this.setWaypointStatus(row, 'REACHED', timestampMs);
      this.anchor = {
        timestamp_ms: row.scheduled_timestamp_ms,
        joint_values: cloneJointValues(row.target_joint_values),
      };
    }

    return {
      state: this.state,
      clock_domain: 'window.performance.now',
      commanded_joint_values: null,
      events: this.drainPendingEvents(),
    };
  }

  stop(
    timestampMs: number,
    actualJointValues: JointValues,
    reason = 'TEST_STOP',
  ): UmiScheduledWaypointSnapshot {
    if (!Number.isFinite(timestampMs)) throw new Error('停止时间必须属于 performance.now 时钟域。');
    this.cancelFuture(timestampMs, reason);
    this.inferenceInProgress = false;
    this.state = 'HOLD_CURRENT_STATE';
    this.heldJointValues = cloneJointValues(actualJointValues);
    this.holdCommandPending = true;
    if (reason === 'TRACKING_ERROR' || reason === 'UNSAFE_PLAN') {
      this.failure = { reason, timestamp_ms: timestampMs };
    }
    return this.snapshot();
  }

  snapshot(): UmiScheduledWaypointSnapshot {
    return {
      state: this.state,
      cycles_started: this.cyclesStarted,
      inference_in_progress: this.inferenceInProgress,
      inference_execution_overlap_sample_count: this.inferenceExecutionOverlapSampleCount,
      first_inference_execution_overlap_timestamp_ms:
        this.firstInferenceExecutionOverlapTimestampMs,
      last_inference_execution_overlap_timestamp_ms: this.lastInferenceExecutionOverlapTimestampMs,
      waypoints: this.rows.map((row) => ({
        ...row,
        target_arm_joints: cloneJointValues(row.target_arm_joints),
        target_joint_values: cloneJointValues(row.target_joint_values),
        target_tcp: clonePose(row.target_tcp),
        actual_joints_at_scheduled_time: row.actual_joints_at_scheduled_time
          ? cloneJointValues(row.actual_joints_at_scheduled_time)
          : null,
      })),
      events: this.events.map((event) => ({ ...event })),
      failure: this.failure ? { ...this.failure } : null,
      final_joint_values: this.heldJointValues ? cloneJointValues(this.heldJointValues) : null,
    };
  }

  private getQueueClassification(): UmiScheduledPlanReceipt['prior_queue_at_insertion'] {
    return {
      reached: this.rows
        .filter((row) => row.queue_status === 'REACHED')
        .map((row) => row.action_index),
      active: this.rows
        .filter((row) => row.queue_status === 'ACTIVE')
        .map((row) => row.action_index),
      future: this.queue.map(({ actionIndex }) => actionIndex),
    };
  }

  private measureTracking(
    actualJointValues: JointValues,
    targetJointValues: JointValues,
    targetTcp: Pose | null,
    targetGripper: number | null,
  ) {
    const actualTcp = this.options.getTcpPose(actualJointValues);
    const expectedTcp = targetTcp ?? this.options.getTcpPose(targetJointValues);
    const actualGripper = this.options.getGripperOpening(actualJointValues);
    const expectedGripper = targetGripper ?? this.options.getGripperOpening(targetJointValues);
    const jawJointErrorM = maximumAbsoluteJointError(
      this.options.gripperJointNames,
      actualJointValues,
      targetJointValues,
    );
    return {
      jointErrorRad: maximumAbsoluteJointError(
        this.options.armJointNames,
        actualJointValues,
        targetJointValues,
      ),
      tcpPositionErrorM: tcpPositionError(actualTcp, expectedTcp),
      tcpOrientationErrorRad: vectorLength(
        poseErrorRotation(expectedTcp.orientation, actualTcp.orientation),
      ),
      gripperErrorM: Math.max(Math.abs(actualGripper - expectedGripper), jawJointErrorM),
    };
  }

  private isTrackingUnsafe(
    tracking: ReturnType<UmiScheduledWaypointController['measureTracking']>,
  ) {
    return (
      !Object.values(tracking).every(Number.isFinite) ||
      !Object.values(this.options.trackingTolerances).every(
        (threshold) => Number.isFinite(threshold) && threshold >= 0,
      ) ||
      tracking.jointErrorRad > this.options.trackingTolerances.jointErrorRad ||
      tracking.tcpPositionErrorM > this.options.trackingTolerances.tcpPositionErrorM ||
      tracking.tcpOrientationErrorRad > this.options.trackingTolerances.tcpOrientationErrorRad ||
      tracking.gripperErrorM > this.options.trackingTolerances.gripperErrorM
    );
  }

  private recordScheduledActual(
    row: UmiScheduledWaypointQueueRecord,
    timestampMs: number,
    actualJointValues: JointValues,
    tracking: ReturnType<UmiScheduledWaypointController['measureTracking']>,
  ): void {
    row.actual_timestamp_ms = timestampMs;
    row.timing_error_ms = timestampMs - row.scheduled_timestamp_ms;
    row.actual_joints_at_scheduled_time = cloneJointValues(actualJointValues);
    row.actual_gripper_at_scheduled_time = this.options.getGripperOpening(actualJointValues);
    row.joint_tracking_error_rad = tracking.jointErrorRad;
    row.tcp_tracking_error_m = tracking.tcpPositionErrorM;
    row.tcp_orientation_error_rad = tracking.tcpOrientationErrorRad;
    row.gripper_tracking_error_m = tracking.gripperErrorM;
  }

  private failActiveWaypoint(
    row: UmiScheduledWaypointQueueRecord,
    timestampMs: number,
    actualJointValues: JointValues,
    tracking: ReturnType<UmiScheduledWaypointController['measureTracking']>,
  ): void {
    row.failure_reason = 'tracking error exceeded an existing UR5_CAD validation tolerance';
    row.joint_tracking_error_rad = tracking.jointErrorRad;
    row.tcp_tracking_error_m = tracking.tcpPositionErrorM;
    row.tcp_orientation_error_rad = tracking.tcpOrientationErrorRad;
    row.gripper_tracking_error_m = tracking.gripperErrorM;
    this.setWaypointStatus(row, 'FAILED', timestampMs, 'TRACKING_ERROR');
    this.failure = { reason: 'TRACKING_ERROR', timestamp_ms: timestampMs };
    this.cancelFuture(timestampMs, 'TRACKING_ERROR');
    this.state = 'HOLD_CURRENT_STATE';
    this.inferenceInProgress = false;
    this.heldJointValues = cloneJointValues(actualJointValues);
    this.holdCommandPending = false;
  }

  private cancelFuture(timestampMs: number, reason: string): void {
    this.queue.forEach(({ value }) => {
      if (value.queue_status === 'PENDING' || value.queue_status === 'ACTIVE') {
        this.setWaypointStatus(value, 'CANCELLED', timestampMs, reason);
      }
    });
    this.queue = [];
  }

  private setWaypointStatus(
    row: UmiScheduledWaypointQueueRecord,
    status: UmiWaypointQueueStatus,
    timestampMs: number,
    reason?: string,
  ): void {
    row.queue_status = status;
    const event = {
      timestamp_ms: timestampMs,
      plan_id: row.plan_id,
      action_index: row.action_index,
      status,
      ...(reason ? { reason } : {}),
    };
    this.events.push(event);
    this.pendingEvents.push(event);
  }

  private drainPendingEvents(): UmiScheduledWaypointEvent[] {
    const pending = this.pendingEvents;
    this.pendingEvents = [];
    return pending;
  }
}
