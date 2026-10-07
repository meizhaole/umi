import { Kinematics } from '../core/Kinematics';
import { RobotModel } from '../core/RobotModel';
import type { IKIterationTrace, IKOptions, IKResult, JointValues, Pose } from '../core/types';
import type {
  IKActionDebugRecord,
  IkWorkerRequest,
  IkWorkerResponse,
  InferenceError,
} from '../app/inferenceProtocol';
import { composePoses, quaternionFromAxisAngle } from '../utils/math';
import { mapGripperOpeningToJoints } from '../app/gripperMapping';

interface WorkerScope {
  onmessage: ((event: MessageEvent<IkWorkerRequest>) => void) | null;
  postMessage: (message: IkWorkerResponse) => void;
}

const workerScope = self as unknown as WorkerScope;
const IK_OPTIONS: IKOptions = {
  maxIterations: 100,
  damping: 0.05,
  positionTolerance: 0.005,
  orientationTolerance: 0.01,
  maxAngularStep: 0.15,
  maxLinearStep: 0.01,
};

const getIkOptions = (request: IkWorkerRequest): IKOptions => ({
  ...IK_OPTIONS,
  ...(request.ikOptions ?? {}),
});

type ActionPoseDecoder = (startPose: Pose, actionPose: Pose) => Pose;

const getActionPoseDecoder = (actionPoseRepr: string): ActionPoseDecoder | null => {
  switch (actionPoseRepr) {
    case 'relative':
      return composePoses;
    default:
      return null;
  }
};

export const decodeActionPose = (
  actionPoseRepr: string,
  startPose: Pose,
  actionPose: Pose,
): Pose => {
  const decoder = getActionPoseDecoder(actionPoseRepr);
  if (!decoder) throw new Error(`不支持 action_pose_repr：${actionPoseRepr}`);
  return decoder(startPose, actionPose);
};

const createError = (
  request: IkWorkerRequest,
  actionIndex: number,
  error: unknown,
): InferenceError => ({
  code: 'IK_SOLVE_FAILED',
  stage: 'ik',
  message: error instanceof Error ? error.message : String(error),
  request_id: request.request_id,
  action_index: actionIndex,
  episode_index: request.episode_index,
  frame_index: request.frame_index,
});

const createDebugRecord = (
  request: IkWorkerRequest,
  actionIndex: number,
  action: unknown,
  seed: JointValues,
  previousActionOutputSeed: JointValues | null,
  targetPose: Pose | null,
  result: IKResult | null,
  actionOutputSeed: JointValues | null,
  iterationTrace: IKIterationTrace[],
  fallbackReason: IKActionDebugRecord['termination_reason'],
  recordPhase: IKActionDebugRecord['record_phase'] = 'result',
  errorMessage?: string,
): IKActionDebugRecord => {
  const converged = result?.converged === true;
  const includeTrace = recordPhase === 'result' && (!converged || request.traceAllIterations);
  let gripperMapping: ReturnType<typeof mapGripperOpeningToJoints> | null = null;
  let gripperMappingError: string | undefined;
  if (Array.isArray(action) && action.length === 7 && Number.isFinite(action[6])) {
    try {
      gripperMapping = mapGripperOpeningToJoints(
        request.description,
        action[6],
        request.gripperJointNames,
      );
    } catch (error) {
      gripperMappingError = error instanceof Error ? error.message : String(error);
    }
  }

  return {
    record_phase: recordPhase,
    request_id: request.request_id,
    episode_index: request.episode_index,
    frame_index: request.frame_index,
    action_index: actionIndex,
    prediction_count: request.predictionCount,
    configured_execution_horizon: request.configuredExecutionHorizon,
    execution_count: request.executionCount,
    selected_action_indexes: [...request.selectedActionIndexes],
    action_pose_repr: request.action_pose_repr,
    action_pose_repr_status: 'available',
    status: recordPhase === 'started' ? 'in_progress' : converged ? 'success' : 'failure',
    trace_level: includeTrace ? 'full' : 'summary',
    raw_action: Array.isArray(action) ? [...action] : action,
    ...(gripperMapping ?? {}),
    ...(gripperMappingError ? { gripper_mapping_error: gripperMappingError } : {}),
    start_pose: request.startPose,
    target_pose: targetPose,
    seed_joint_values: { ...seed },
    previous_action_output_seed: previousActionOutputSeed ? { ...previousActionOutputSeed } : null,
    solver_options: getIkOptions(request),
    final_joint_values: result ? { ...result.jointValues } : null,
    action_output_seed: actionOutputSeed ? { ...actionOutputSeed } : null,
    position_residual: result?.residual.position ?? null,
    orientation_residual: result?.residual.orientation ?? null,
    iterations: result?.iterations ?? null,
    blocked_joints: result ? [...result.blockedJoints] : null,
    termination_reason:
      recordPhase === 'started' ? null : (result?.terminationReason ?? fallbackReason),
    ...(includeTrace ? { iteration_trace: iterationTrace } : {}),
    ...(errorMessage ? { error_message: errorMessage } : {}),
  };
};

export const runIkSolve = (
  request: IkWorkerRequest,
  postMessage: (message: IkWorkerResponse) => void,
): void => {
  if (request.type !== 'solve') return;
  const ikOptions = getIkOptions(request);

  const actionPoseDecoder = getActionPoseDecoder(request.action_pose_repr);
  if (!actionPoseDecoder) {
    postMessage({
      type: 'error',
      error: {
        code: 'UNSUPPORTED_ACTION_POSE_REPR',
        stage: 'ik',
        message: `Browser IK 暂不支持 action_pose_repr：${request.action_pose_repr}`,
        request_id: request.request_id,
        episode_index: request.episode_index,
        frame_index: request.frame_index,
        action_pose_repr: request.action_pose_repr,
      },
    });
    return;
  }

  const robot = new RobotModel(request.description);
  const kinematics = new Kinematics(request.description);
  const resolvedActions = [];
  let seed = request.initialJointValues;
  let previousActionOutputSeed: JointValues | null = null;

  robot.setJointValues(seed);
  seed = robot.getJointValues();

  for (let executionIndex = 0; executionIndex < request.actions.length; executionIndex += 1) {
    const actionIndex = request.selectedActionIndexes[executionIndex];
    const action = request.actions[executionIndex];
    if (!Array.isArray(action) || action.length !== 7 || !action.every(Number.isFinite)) {
      const message = '动作必须包含 7 个有限数值。';
      const record = createDebugRecord(
        request,
        actionIndex,
        action,
        seed,
        previousActionOutputSeed,
        null,
        null,
        null,
        [],
        'invalid_action',
        'result',
        message,
      );
      postMessage({ type: 'ik_record', record });
      postMessage({
        type: 'error',
        error: {
          code: 'INVALID_ACTION',
          stage: 'ik',
          message,
          request_id: request.request_id,
          action_index: actionIndex,
          episode_index: request.episode_index,
          frame_index: request.frame_index,
        },
      });
      return;
    }

    postMessage({
      type: 'progress',
      actionIndex,
      total: request.executionCount,
    });

    const actionSeed = { ...seed };
    let targetPose: Pose | null = null;
    let result: IKResult | null = null;
    const iterationTrace: IKIterationTrace[] = [];
    postMessage({
      type: 'ik_record',
      record: createDebugRecord(
        request,
        actionIndex,
        action,
        actionSeed,
        previousActionOutputSeed,
        null,
        null,
        null,
        [],
        null,
        'started',
      ),
    });

    try {
      const rotationVector = action.slice(3, 6) as [number, number, number];
      const angle = Math.hypot(...rotationVector);
      const deltaPose = {
        position: action.slice(0, 3) as [number, number, number],
        orientation:
          angle > 0
            ? quaternionFromAxisAngle(
                rotationVector.map((value) => value / angle) as [number, number, number],
                angle,
              )
            : ([0, 0, 0, 1] as [number, number, number, number]),
      };
      targetPose = actionPoseDecoder(request.startPose, deltaPose);
      result = kinematics.solveIK(targetPose, actionSeed, ikOptions, (iteration) => {
        iterationTrace.push(iteration);
      });

      if (!result.converged) {
        postMessage({
          type: 'ik_record',
          record: createDebugRecord(
            request,
            actionIndex,
            action,
            actionSeed,
            previousActionOutputSeed,
            targetPose,
            result,
            null,
            iterationTrace,
            result.terminationReason,
          ),
        });
        if (request.continueOnIkFailure) continue;
        postMessage({
          type: 'error',
          error: {
            code: 'IK_NOT_CONVERGED',
            stage: 'ik',
            message: '动作目标未能收敛到机械臂可达范围。',
            request_id: request.request_id,
            action_index: actionIndex,
            action,
            start_pose: request.startPose,
            target_pose: targetPose,
            seed_joint_values: { ...actionSeed },
            iterations: result.iterations,
            residual: result.residual,
            joint_values: result.jointValues,
            ik_options: {
              max_iterations: ikOptions.maxIterations,
              damping: ikOptions.damping,
              position_tolerance: ikOptions.positionTolerance,
              orientation_tolerance: ikOptions.orientationTolerance,
              max_angular_step: ikOptions.maxAngularStep,
              max_linear_step: ikOptions.maxLinearStep,
            },
            termination_reason: result.terminationReason,
            blocked_joints: result.blockedJoints,
          },
        });
        return;
      }

      robot.setJointValues(result.jointValues);
      const gripperMapping = mapGripperOpeningToJoints(
        request.description,
        action[6],
        request.gripperJointNames,
      );
      Object.entries(gripperMapping.joint_targets).forEach(([jointName, target]) => {
        robot.setJointValue(jointName, target);
      });

      seed = robot.getJointValues();
      resolvedActions.push({ actionIndex, jointValues: seed });
      postMessage({
        type: 'ik_record',
        record: createDebugRecord(
          request,
          actionIndex,
          action,
          actionSeed,
          previousActionOutputSeed,
          targetPose,
          result,
          seed,
          iterationTrace,
          result.terminationReason,
        ),
      });
      previousActionOutputSeed = { ...seed };
    } catch (error) {
      const errorRecord = createDebugRecord(
        request,
        actionIndex,
        action,
        actionSeed,
        previousActionOutputSeed,
        targetPose,
        result,
        null,
        iterationTrace,
        'solver_exception',
        'result',
        error instanceof Error ? error.message : String(error),
      );
      errorRecord.status = 'failure';
      postMessage({
        type: 'ik_record',
        record: errorRecord,
      });
      if (request.continueOnIkFailure) continue;
      postMessage({
        type: 'error',
        error: createError(request, actionIndex, error),
      });
      return;
    }
  }

  postMessage({ type: 'solved', actions: resolvedActions });
};

workerScope.onmessage = (event) => {
  runIkSolve(event.data, (message) => workerScope.postMessage(message));
};
