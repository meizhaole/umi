import type {
  IKIterationExitCondition,
  IKIterationObserver,
  IKOptions,
  IKResidual,
  IKResult,
  IKTerminationReason,
  JointDescription,
  JointValues,
  Pose,
  RobotDescription,
  Vec3,
} from './types';
import {
  composePoses,
  crossVectors,
  dotVectors,
  IDENTITY_POSE,
  normalizeVector,
  poseErrorRotation,
  quaternionFromAxisAngle,
  rotateVector,
  scaleVector,
  subtractVectors,
} from '../utils/math';

interface JointFrame {
  position: Vec3;
  axis: Vec3;
}

interface ForwardResult {
  linkPoses: Record<string, Pose>;
  jointFrames: Record<string, JointFrame>;
  jointValues: JointValues;
}

const isKinematicJoint = (joint: JointDescription): boolean =>
  joint.type === 'revolute' || joint.type === 'continuous' || joint.type === 'prismatic';

const clampJointValue = (joint: JointDescription, value: number): number =>
  Math.max(
    joint.limit?.lower ?? Number.NEGATIVE_INFINITY,
    Math.min(joint.limit?.upper ?? Number.POSITIVE_INFINITY, value),
  );

const getJointLimitMargin = (joint: JointDescription, value: number): number | null => {
  const margins = [
    joint.limit?.lower === undefined ? null : value - joint.limit.lower,
    joint.limit?.upper === undefined ? null : joint.limit.upper - value,
  ].filter((margin): margin is number => margin !== null);
  return margins.length > 0 ? Math.min(...margins) : null;
};

const isOutwardStepAtLimit = (joint: JointDescription, value: number, step: number): boolean =>
  (joint.limit?.lower !== undefined && value <= joint.limit.lower + 1e-8 && step < 0) ||
  (joint.limit?.upper !== undefined && value >= joint.limit.upper - 1e-8 && step > 0);

const vectorNorm = (values: number[]): number => Math.hypot(...values);

const solveLinearSystem = (matrix: number[][], vector: number[]): number[] | undefined => {
  const size = vector.length;
  const augmented = matrix.map((row, index) => [...row, vector[index]]);

  for (let column = 0; column < size; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < size; row += 1) {
      if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) pivot = row;
    }
    if (Math.abs(augmented[pivot][column]) < 1e-12) return undefined;
    [augmented[column], augmented[pivot]] = [augmented[pivot], augmented[column]];

    const divisor = augmented[column][column];
    for (let entry = column; entry <= size; entry += 1) {
      augmented[column][entry] /= divisor;
    }
    for (let row = 0; row < size; row += 1) {
      if (row === column) continue;
      const factor = augmented[row][column];
      for (let entry = column; entry <= size; entry += 1) {
        augmented[row][entry] -= factor * augmented[column][entry];
      }
    }
  }

  return augmented.map((row) => row[size]);
};

export class Kinematics {
  private readonly description: RobotDescription;

  private readonly jointsByParent: Map<string, JointDescription[]>;

  private readonly jointsByChild: Map<string, JointDescription>;

  private readonly jointsByName: Map<string, JointDescription>;

  constructor(description: RobotDescription) {
    this.description = description;
    this.jointsByParent = new Map();
    this.jointsByChild = new Map();
    this.jointsByName = new Map(description.joints.map((joint) => [joint.name, joint]));
    description.joints.forEach((joint) => {
      const siblings = this.jointsByParent.get(joint.parent) ?? [];
      siblings.push(joint);
      this.jointsByParent.set(joint.parent, siblings);
      this.jointsByChild.set(joint.child, joint);
    });

    const linkNames = new Set(description.links.map((link) => link.name));
    if (!linkNames.has(description.rootLink) || !linkNames.has(description.tipLink)) {
      throw new Error('RobotDescription 的 rootLink 或 tipLink 不存在');
    }
  }

  forwardKinematics(jointValues: JointValues = {}, linkName = this.description.tipLink): Pose {
    const pose = this.forward(jointValues).linkPoses[linkName];
    if (!pose) throw new Error(`未知或不可达的 link：${linkName}`);
    return { position: [...pose.position], orientation: [...pose.orientation] };
  }

  forwardKinematicsAll(jointValues: JointValues = {}): Record<string, Pose> {
    const { linkPoses } = this.forward(jointValues);
    return Object.fromEntries(
      Object.entries(linkPoses).map(([name, pose]) => [
        name,
        {
          position: [...pose.position],
          orientation: [...pose.orientation],
        },
      ]),
    );
  }

  solveIK(
    targetPose: Pose,
    initialJointValues: JointValues = {},
    options: IKOptions = {},
    onIteration?: IKIterationObserver,
  ): IKResult {
    const settings = {
      maxIterations: options.maxIterations ?? 100,
      damping: options.damping ?? 0.05,
      positionTolerance: options.positionTolerance ?? 0.001,
      orientationTolerance: options.orientationTolerance ?? 0.01,
      maxAngularStep: options.maxAngularStep ?? 0.15,
      maxLinearStep: options.maxLinearStep ?? 0.01,
    };
    const path = this.getJointPath(this.description.tipLink);
    if (path.some((joint) => joint.type === 'floating' || joint.type === 'planar')) {
      throw new Error('IK 暂不支持 floating 或 planar joint');
    }
    const variables = path.filter((joint) => isKinematicJoint(joint) && !joint.mimic);
    let jointValues = this.resolveJointValues(initialJointValues);
    let current = this.forward(jointValues);
    let residual = this.measureResidual(targetPose, current.linkPoses[this.description.tipLink]);
    let iterations = 0;
    let iterationIndex = 0;
    let terminationReason: IKTerminationReason = 'max_iterations';
    const blockedJointNames = new Set<string>();

    while (iterations < settings.maxIterations && !this.isConverged(residual, settings)) {
      const currentIterationIndex = iterationIndex;
      iterationIndex += 1;
      const qBefore = { ...jointValues };
      const currentResidual = { ...residual };
      const tipPose = current.linkPoses[this.description.tipLink];
      const error = [
        ...subtractVectors(targetPose.position, tipPose.position),
        ...poseErrorRotation(targetPose.orientation, tipPose.orientation),
      ];
      let activeVariables = variables;
      let jointSteps: number[] = [];
      let rawJointSteps: number[] | null = null;
      let solved: number[] | undefined;
      const blockedJointsThisIteration = new Set<string>();

      while (activeVariables.length > 0) {
        const jacobian = activeVariables.map((joint) =>
          this.jacobianColumn(joint, current.jointFrames[joint.name], tipPose.position),
        );
        const normalMatrix = Array.from({ length: 6 }, (_, row) =>
          Array.from(
            { length: 6 },
            (_, column) =>
              jacobian.reduce((sum, axis) => sum + axis[row] * axis[column], 0) +
              (row === column ? settings.damping ** 2 : 0),
          ),
        );
        const solution = solveLinearSystem(normalMatrix, error);
        solved = solution;
        if (!solution) break;

        jointSteps = activeVariables.map(
          (_, index) =>
            dotVectors(jacobian[index].slice(0, 3) as Vec3, solution.slice(0, 3) as Vec3) +
            dotVectors(jacobian[index].slice(3, 6) as Vec3, solution.slice(3, 6) as Vec3),
        );
        if (rawJointSteps === null) rawJointSteps = [...jointSteps];
        const blockedJoints = new Set(
          activeVariables
            .filter((joint, index) =>
              isOutwardStepAtLimit(joint, jointValues[joint.name], jointSteps[index]),
            )
            .map((joint) => joint.name),
        );
        if (blockedJoints.size === 0) break;
        blockedJoints.forEach((jointName) => {
          blockedJointNames.add(jointName);
          blockedJointsThisIteration.add(jointName);
        });

        // 触限关节若被要求继续越界，就从本轮雅可比中移除并重算其他关节的步长。
        activeVariables = activeVariables.filter((joint) => !blockedJoints.has(joint.name));
      }

      const stepInformation = new Map<
        string,
        {
          step: number;
          stepLimited: boolean;
          jointLimitClamped: boolean;
          requestedJointValue: number;
          clampedJointValue: number;
        }
      >();
      const emitIteration = (
        exitCondition: IKIterationExitCondition,
        qCandidate: JointValues = qBefore,
        qAfter: JointValues = qBefore,
      ) => {
        if (!onIteration) return;
        const activeStepByName = new Map(
          activeVariables.map((joint, index) => [joint.name, jointSteps[index]]),
        );
        const rawStepByName = rawJointSteps
          ? new Map(variables.map((joint, index) => [joint.name, rawJointSteps?.[index] ?? 0]))
          : null;
        const deltaAfterStepLimiting = Object.fromEntries(
          variables.map((joint) => [
            joint.name,
            blockedJointsThisIteration.has(joint.name)
              ? 0
              : (stepInformation.get(joint.name)?.step ?? null),
          ]),
        );
        const clampInformation = Object.fromEntries(
          variables.map((joint) => {
            const information = stepInformation.get(joint.name);
            return [
              joint.name,
              {
                blocked_by_active_set: blockedJointsThisIteration.has(joint.name),
                step_limited: information?.stepLimited ?? false,
                joint_limit_clamped: information?.jointLimitClamped ?? false,
                requested_joint_value: information?.requestedJointValue ?? null,
                clamped_joint_value: information?.clampedJointValue ?? qBefore[joint.name],
              },
            ];
          }),
        );
        const trace = {
          iteration_index: currentIterationIndex,
          q_before: qBefore,
          current_position_residual: currentResidual.position,
          current_orientation_residual: currentResidual.orientation,
          delta_q_raw: rawStepByName ? Object.fromEntries(rawStepByName) : null,
          delta_q_after_active_set: Object.fromEntries(
            variables.map((joint) => [
              joint.name,
              blockedJointsThisIteration.has(joint.name)
                ? null
                : (activeStepByName.get(joint.name) ?? null),
            ]),
          ),
          delta_q_after_step_limiting: deltaAfterStepLimiting,
          q_candidate: { ...qCandidate },
          q_after: { ...qAfter },
          blocked_joints: Array.from(blockedJointsThisIteration),
          active_joints: activeVariables.map((joint) => joint.name),
          clamp_information: clampInformation,
          joint_limit_margin: Object.fromEntries(
            variables.map((joint) => [
              joint.name,
              {
                before: getJointLimitMargin(joint, qBefore[joint.name]),
                candidate: getJointLimitMargin(joint, qCandidate[joint.name]),
              },
            ]),
          ),
          exit_condition: exitCondition,
        };
        try {
          onIteration(trace);
        } catch {
          // 调试回调异常不能改变 IK 求解结果。
        }
      };

      if (!solved || activeVariables.length === 0) {
        terminationReason = solved ? 'joint_limits_blocked' : 'linear_solve_failed';
        emitIteration(terminationReason);
        break;
      }

      const nextValues = { ...jointValues };
      activeVariables.forEach((joint, index) => {
        const maximumStep =
          joint.type === 'prismatic' ? settings.maxLinearStep : settings.maxAngularStep;
        const rawStep = jointSteps[index];
        const step = Math.max(-maximumStep, Math.min(maximumStep, rawStep));
        const requestedJointValue = jointValues[joint.name] + step;
        const clampedJointValue = clampJointValue(joint, requestedJointValue);
        nextValues[joint.name] = clampedJointValue;
        stepInformation.set(joint.name, {
          step,
          stepLimited: step !== rawStep,
          jointLimitClamped: clampedJointValue !== requestedJointValue,
          requestedJointValue,
          clampedJointValue,
        });
      });

      if (
        variables.every(
          (joint) => Math.abs(nextValues[joint.name] - jointValues[joint.name]) < 1e-10,
        )
      ) {
        terminationReason = 'no_joint_motion';
        emitIteration(terminationReason, nextValues);
        break;
      }

      jointValues = this.resolveJointValues(nextValues);
      current = this.forward(jointValues);
      residual = this.measureResidual(targetPose, current.linkPoses[this.description.tipLink]);
      iterations += 1;
      emitIteration(
        this.isConverged(residual, settings)
          ? 'converged'
          : iterations >= settings.maxIterations
            ? 'max_iterations'
            : 'continue',
        nextValues,
        jointValues,
      );
    }

    const converged = this.isConverged(residual, settings);
    if (converged) terminationReason = 'converged';
    else if (iterations >= settings.maxIterations) terminationReason = 'max_iterations';

    return {
      jointValues,
      converged,
      iterations,
      residual,
      terminationReason,
      blockedJoints: Array.from(blockedJointNames),
    };
  }

  private forward(jointValues: JointValues): ForwardResult {
    const values = this.resolveJointValues(jointValues);
    const linkPoses: Record<string, Pose> = {
      [this.description.rootLink]: {
        position: [...IDENTITY_POSE.position],
        orientation: [...IDENTITY_POSE.orientation],
      },
    };
    const jointFrames: Record<string, JointFrame> = {};
    const queue = [this.description.rootLink];

    while (queue.length > 0) {
      const parentLink = queue.shift() as string;
      const parentPose = linkPoses[parentLink];
      (this.jointsByParent.get(parentLink) ?? []).forEach((joint) => {
        if (linkPoses[joint.child]) throw new Error(`URDF 关节树存在循环：${joint.child}`);
        const jointPose = composePoses(parentPose, joint.origin);
        const axis = normalizeVector(joint.axis);
        jointFrames[joint.name] = {
          position: [...jointPose.position],
          axis: rotateVector(jointPose.orientation, axis),
        };

        const motion: Pose = {
          position: [0, 0, 0],
          orientation: [0, 0, 0, 1],
        };
        const value = values[joint.name] ?? 0;
        if (joint.type === 'revolute' || joint.type === 'continuous') {
          motion.orientation = quaternionFromAxisAngle(axis, value);
        } else if (joint.type === 'prismatic') {
          motion.position = scaleVector(axis, value);
        }
        linkPoses[joint.child] = composePoses(jointPose, motion);
        queue.push(joint.child);
      });
    }

    return { linkPoses, jointFrames, jointValues: values };
  }

  private resolveJointValues(values: JointValues): JointValues {
    const resolved = Object.fromEntries(
      this.description.joints.map((joint) => [
        joint.name,
        joint.type === 'fixed' ? 0 : clampJointValue(joint, values[joint.name] ?? 0),
      ]),
    );
    const resolving = new Set<string>();
    const resolve = (joint: JointDescription): number => {
      if (!joint.mimic) return resolved[joint.name];
      if (resolving.has(joint.name)) throw new Error(`mimic 关节形成循环：${joint.name}`);
      resolving.add(joint.name);
      const source = this.jointsByName.get(joint.mimic.joint);
      if (!source) throw new Error(`mimic 引用了未知关节：${joint.mimic.joint}`);
      const value = clampJointValue(
        joint,
        resolve(source) * joint.mimic.multiplier + joint.mimic.offset,
      );
      resolving.delete(joint.name);
      resolved[joint.name] = value;
      return value;
    };

    this.description.joints.filter((joint) => joint.mimic).forEach(resolve);
    return resolved;
  }

  private getJointPath(tipLink: string): JointDescription[] {
    const path: JointDescription[] = [];
    let current = tipLink;
    while (current !== this.description.rootLink) {
      const joint = this.jointsByChild.get(current);
      if (!joint) throw new Error(`tipLink ${tipLink} 不在 rootLink 的关节树中`);
      path.unshift(joint);
      current = joint.parent;
    }
    return path;
  }

  private jacobianColumn(joint: JointDescription, frame: JointFrame, tipPosition: Vec3): number[] {
    if (joint.type === 'prismatic') return [...frame.axis, 0, 0, 0];
    const linear = crossVectors(frame.axis, subtractVectors(tipPosition, frame.position));
    return [...linear, ...frame.axis];
  }

  private measureResidual(target: Pose, current: Pose): IKResidual {
    const position = vectorNorm(subtractVectors(target.position, current.position));
    const orientation = vectorNorm(poseErrorRotation(target.orientation, current.orientation));
    return { position, orientation, total: Math.hypot(position, orientation) };
  }

  private isConverged(
    residual: IKResidual,
    settings: {
      positionTolerance: number;
      orientationTolerance: number;
    },
  ): boolean {
    return (
      residual.position <= settings.positionTolerance &&
      residual.orientation <= settings.orientationTolerance
    );
  }
}
