import { Kinematics } from '../core/Kinematics';
import { RobotModel } from '../core/RobotModel';
import type { IkWorkerRequest, IkWorkerResponse, InferenceError } from '../app/inferenceProtocol';
import { composePoses, quaternionFromAxisAngle } from '../utils/math';

interface WorkerScope {
  onmessage: ((event: MessageEvent<IkWorkerRequest>) => void) | null;
  postMessage: (message: IkWorkerResponse) => void;
}

const workerScope = self as unknown as WorkerScope;

const getGripperJoints = (description: IkWorkerRequest['description']) =>
  description.joints.filter(
    (joint) =>
      joint.type === 'prismatic' &&
      /gripper|finger/iu.test(joint.name + ' ' + joint.child),
  );

const createError = (actionIndex: number, error: unknown): InferenceError => ({
  code: 'IK_SOLVE_FAILED',
  stage: 'ik',
  message: error instanceof Error ? error.message : String(error),
  action_index: actionIndex,
  episode_index: undefined,
  frame_index: undefined,
});

workerScope.onmessage = (event) => {
  const request = event.data;
  if (request.type !== 'solve') return;

  const robot = new RobotModel(request.description);
  const kinematics = new Kinematics(request.description);
  const gripperJoints = getGripperJoints(request.description);
  const maximumOpening = gripperJoints.reduce((total, joint) => {
    const lower = joint.limit?.lower ?? 0;
    const upper = joint.limit?.upper ?? lower;
    return total + Math.abs(upper - lower);
  }, 0);
  const resolvedActions = [];
  let seed = request.initialJointValues;

  robot.setJointValues(seed);
  seed = robot.getJointValues();

  for (let actionIndex = 0; actionIndex < request.actions.length; actionIndex += 1) {
    const action = request.actions[actionIndex];
    if (!Array.isArray(action) || action.length !== 7 || !action.every(Number.isFinite)) {
      workerScope.postMessage({
        type: 'error',
        error: {
          code: 'INVALID_ACTION',
          stage: 'ik',
          message: '动作必须包含 7 个有限数值。',
          action_index: actionIndex,
        },
      });
      return;
    }

    workerScope.postMessage({
      type: 'progress',
      actionIndex,
      total: request.actions.length,
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
      const targetPose = composePoses(request.startPose, deltaPose);
      const result = kinematics.solveIK(targetPose, seed, { positionTolerance: 0.003 });
      if (!result.converged) {
        workerScope.postMessage({
          type: 'error',
          error: {
            code: 'IK_NOT_CONVERGED',
            stage: 'ik',
            message: '动作目标未能收敛到机械臂可达范围。',
            action_index: actionIndex,
            iterations: result.iterations,
            residual: result.residual,
          },
        });
        return;
      }

      robot.setJointValues(result.jointValues);
      const requestedOpening = Math.max(0, Math.min(maximumOpening, action[6]));
      const openingFraction = maximumOpening > 0 ? requestedOpening / maximumOpening : 0;
      gripperJoints.forEach((joint) => {
        if (joint.mimic) return;
        const lower = joint.limit?.lower ?? 0;
        const upper = joint.limit?.upper ?? lower;
        robot.setJointValue(joint.name, lower + (upper - lower) * openingFraction);
      });

      seed = robot.getJointValues();
      resolvedActions.push({ actionIndex, jointValues: seed });
    } catch (error) {
      workerScope.postMessage({
        type: 'error',
        error: createError(actionIndex, error),
      });
      return;
    }
  }

  workerScope.postMessage({ type: 'solved', actions: resolvedActions });
};
