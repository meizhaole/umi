// 组织 IK Multi-seed 诊断样本、限位校验和逐轮结果摘要。
import type { IKActionDebugRecord } from '../app/inferenceProtocol';
import { Kinematics } from '../core/Kinematics';
import type {
  IKIterationTrace,
  IKOptions,
  IKResult,
  JointDescription,
  JointValues,
  Pose,
  RobotDescription,
} from '../core/types';

export interface IKSeedCase {
  seed_name: string;
  seed_joint_values: JointValues | null;
  duplicate_of?: string;
  unavailable_reason?: string;
}

export interface IKSeedResult {
  seed_name: string;
  seed_joint_values: JointValues | null;
  status: 'success' | 'failure' | 'unavailable';
  executed: boolean;
  duplicate_of?: string;
  unavailable_reason?: string;
  termination_reason: string | null;
  iterations: number | null;
  final_joint_values: JointValues | null;
  final_position_residual: number | null;
  final_orientation_residual: number | null;
  blocked_joints: string[];
  minimum_joint_limit_margin: number | null;
  position_tolerance_satisfied: boolean | null;
  orientation_tolerance_satisfied: boolean | null;
  joint2_initial: number | null;
  joint2_final: number | null;
  joint2_min: number | null;
  joint2_max: number | null;
  joint2_was_blocked: boolean;
  first_iteration_joint2_blocked: number | null;
  joint2_raw_delta_when_first_blocked: number | null;
  residual_history: Array<{
    iteration_index: number;
    position_residual: number;
    orientation_residual: number;
    exit_condition: string;
  }>;
  joint2_history: Array<{
    iteration_index: number;
    q_before: number | null;
    raw_delta: number | null;
    delta_after_active_set: number | null;
    q_after: number | null;
    blocked: boolean;
    active: boolean;
  }>;
}

const KINEMATIC_JOINT_TYPES = new Set(['revolute', 'continuous', 'prismatic']);
const SMALL_PERTURBATION = 0.02;
const SEED_EQUALITY_TOLERANCE = 1e-12;

const getKinematicPath = (description: RobotDescription): JointDescription[] => {
  const jointByChild = new Map(description.joints.map((joint) => [joint.child, joint]));
  const path: JointDescription[] = [];
  const visited = new Set<string>();
  let currentLink = description.tipLink;

  while (currentLink !== description.rootLink) {
    if (visited.has(currentLink)) throw new Error('RobotDescription 的关节路径形成循环');
    visited.add(currentLink);
    const joint = jointByChild.get(currentLink);
    if (!joint) throw new Error(`RobotDescription 无法从 tipLink 到达 rootLink：${currentLink}`);
    path.unshift(joint);
    currentLink = joint.parent;
  }

  return path.filter((joint) => KINEMATIC_JOINT_TYPES.has(joint.type) && !joint.mimic);
};

const validateSeed = (seed: JointValues, description: RobotDescription): string | undefined => {
  const jointsByName = new Map(description.joints.map((joint) => [joint.name, joint]));
  const pathJoints = getKinematicPath(description);

  for (const joint of pathJoints) {
    if (!Number.isFinite(seed[joint.name])) return `Seed 缺少有限关节值：${joint.name}`;
  }

  for (const [jointName, value] of Object.entries(seed)) {
    const joint = jointsByName.get(jointName);
    if (!joint) return `Seed 包含 URDF 未定义的关节：${jointName}`;
    if (!Number.isFinite(value)) return `Seed 关节值不是有限数：${jointName}`;
    if (joint.limit?.lower !== undefined && value < joint.limit.lower - SEED_EQUALITY_TOLERANCE) {
      return `${jointName} 低于 URDF 下限 ${joint.limit.lower}`;
    }
    if (joint.limit?.upper !== undefined && value > joint.limit.upper + SEED_EQUALITY_TOLERANCE) {
      return `${jointName} 高于 URDF 上限 ${joint.limit.upper}`;
    }
  }

  return undefined;
};

const seedsEqual = (left: JointValues, right: JointValues): boolean => {
  const names = new Set([...Object.keys(left), ...Object.keys(right)]);
  return Array.from(names).every((name) => {
    const leftValue = left[name];
    const rightValue = right[name];
    return (
      leftValue !== undefined &&
      rightValue !== undefined &&
      Math.abs(leftValue - rightValue) <= SEED_EQUALITY_TOLERANCE
    );
  });
};

const createReferenceSeed = (
  original: JointValues,
  description: RobotDescription,
): JointValues | null => {
  const hasExpectedArm = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'].every(
    (name) => description.joints.some((joint) => joint.name === name),
  );
  if (!description.name.includes('_RS') || !hasExpectedArm) return null;

  const reference = { ...original };
  getKinematicPath(description).forEach((joint) => {
    reference[joint.name] = 0;
  });
  reference.joint2 = 0.1;
  reference.joint3 = 0.1;
  return reference;
};

const changedSeed = (original: JointValues, jointName: string, value: number): JointValues => ({
  ...original,
  [jointName]: value,
});

const addPlan = (
  plans: IKSeedCase[],
  seedName: string,
  seed: JointValues | null,
  description: RobotDescription,
): void => {
  if (!seed) {
    plans.push({
      seed_name: seedName,
      seed_joint_values: null,
      unavailable_reason: '所需 Seed 或当前模型参考姿态不可用',
    });
    return;
  }

  const unavailableReason = validateSeed(seed, description);
  if (unavailableReason) {
    plans.push({
      seed_name: seedName,
      seed_joint_values: seed,
      unavailable_reason: unavailableReason,
    });
    return;
  }

  const duplicate = plans.find(
    (plan) =>
      plan.seed_joint_values &&
      !plan.unavailable_reason &&
      seedsEqual(plan.seed_joint_values, seed),
  );
  plans.push({
    seed_name: seedName,
    seed_joint_values: seed,
    ...(duplicate ? { duplicate_of: duplicate.seed_name } : {}),
  });
};

const perturbSeed = (
  original: JointValues,
  jointName: string,
  preferredDelta: number,
  description: RobotDescription,
): { seed: JointValues | null; delta: number | null } => {
  const initial = original[jointName];
  if (!Number.isFinite(initial)) return { seed: null, delta: null };

  for (const delta of [preferredDelta, -preferredDelta]) {
    const candidate = changedSeed(original, jointName, initial + delta);
    if (!validateSeed(candidate, description)) return { seed: candidate, delta };
  }

  return { seed: null, delta: null };
};

export const buildSeedCases = (
  failureRecord: IKActionDebugRecord,
  previousSuccessfulSeed: JointValues | null,
  description: RobotDescription,
): IKSeedCase[] => {
  const original = failureRecord.seed_joint_values;
  const plans: IKSeedCase[] = [];
  addPlan(plans, 'Seed A — Original Failure Seed', original, description);
  addPlan(plans, 'Seed B — Previous Successful Action Seed', previousSuccessfulSeed, description);

  [0.05, 0.1].forEach((joint2Value, index) => {
    const label = index === 0 ? 'Seed C — Joint2 0.05 rad' : 'Seed D — Joint2 0.10 rad';
    addPlan(plans, label, changedSeed(original, 'joint2', joint2Value), description);
  });

  addPlan(
    plans,
    'Seed E — RS Initial Reference',
    createReferenceSeed(original, description),
    description,
  );

  const perturbations = [
    { jointName: 'joint1', preferredDelta: SMALL_PERTURBATION, seedPrefix: 'Seed F' },
    { jointName: 'joint2', preferredDelta: SMALL_PERTURBATION, seedPrefix: 'Seed G' },
    { jointName: 'joint3', preferredDelta: -SMALL_PERTURBATION, seedPrefix: 'Seed H' },
  ];
  perturbations.forEach(({ jointName, preferredDelta, seedPrefix }) => {
    const { seed, delta } = perturbSeed(original, jointName, preferredDelta, description);
    const direction = delta === null ? '' : delta > 0 ? '+' : '−';
    const magnitude = delta === null ? '' : ` ${Math.abs(delta).toFixed(2)} rad`;
    addPlan(plans, `${seedPrefix} — ${jointName} ${direction}${magnitude}`, seed, description);
  });

  return plans;
};

const marginAt = (joint: JointDescription, value: number): number | null => {
  const margins = [
    joint.limit?.lower === undefined ? null : value - joint.limit.lower,
    joint.limit?.upper === undefined ? null : joint.limit.upper - value,
  ].filter((margin): margin is number => margin !== null);
  return margins.length > 0 ? Math.min(...margins) : null;
};

const getMinimumJointLimitMargin = (
  description: RobotDescription,
  seed: JointValues,
  trace: IKIterationTrace[],
): number | null => {
  const pathJoints = getKinematicPath(description);
  const margins = pathJoints.flatMap((joint) => {
    const startMargin = marginAt(joint, seed[joint.name]);
    const traceMargins = trace.flatMap((iteration) => {
      const margin = iteration.joint_limit_margin[joint.name];
      return [margin?.before ?? null, margin?.candidate ?? null];
    });
    return [startMargin, ...traceMargins].filter((margin): margin is number => margin !== null);
  });
  return margins.length > 0 ? Math.min(...margins) : null;
};

const makeUnavailableResult = (seedCase: IKSeedCase): IKSeedResult => ({
  seed_name: seedCase.seed_name,
  seed_joint_values: seedCase.seed_joint_values,
  status: 'unavailable',
  executed: false,
  ...(seedCase.duplicate_of ? { duplicate_of: seedCase.duplicate_of } : {}),
  ...(seedCase.unavailable_reason ? { unavailable_reason: seedCase.unavailable_reason } : {}),
  termination_reason: null,
  iterations: null,
  final_joint_values: null,
  final_position_residual: null,
  final_orientation_residual: null,
  blocked_joints: [],
  minimum_joint_limit_margin: null,
  position_tolerance_satisfied: null,
  orientation_tolerance_satisfied: null,
  joint2_initial: seedCase.seed_joint_values?.joint2 ?? null,
  joint2_final: null,
  joint2_min: seedCase.seed_joint_values?.joint2 ?? null,
  joint2_max: seedCase.seed_joint_values?.joint2 ?? null,
  joint2_was_blocked: false,
  first_iteration_joint2_blocked: null,
  joint2_raw_delta_when_first_blocked: null,
  residual_history: [],
  joint2_history: [],
});

const makeResult = (
  seedCase: IKSeedCase,
  result: IKResult,
  trace: IKIterationTrace[],
  description: RobotDescription,
  options: IKOptions,
): IKSeedResult => {
  const positionTolerance = options.positionTolerance ?? 0.001;
  const orientationTolerance = options.orientationTolerance ?? 0.01;
  const positionToleranceSatisfied = result.residual.position <= positionTolerance;
  const orientationToleranceSatisfied = result.residual.orientation <= orientationTolerance;
  const joint2Values = [
    seedCase.seed_joint_values?.joint2,
    ...trace.flatMap((iteration) => [iteration.q_before.joint2, iteration.q_after.joint2]),
    result.jointValues.joint2,
  ].filter((value): value is number => value !== undefined && Number.isFinite(value));
  const firstJoint2Block = trace.find((iteration) => iteration.blocked_joints.includes('joint2'));

  return {
    seed_name: seedCase.seed_name,
    seed_joint_values: seedCase.seed_joint_values,
    status:
      result.converged && positionToleranceSatisfied && orientationToleranceSatisfied
        ? 'success'
        : 'failure',
    executed: true,
    ...(seedCase.duplicate_of ? { duplicate_of: seedCase.duplicate_of } : {}),
    termination_reason: result.terminationReason,
    iterations: result.iterations,
    final_joint_values: result.jointValues,
    final_position_residual: result.residual.position,
    final_orientation_residual: result.residual.orientation,
    blocked_joints: result.blockedJoints,
    minimum_joint_limit_margin: getMinimumJointLimitMargin(
      description,
      seedCase.seed_joint_values ?? {},
      trace,
    ),
    position_tolerance_satisfied: positionToleranceSatisfied,
    orientation_tolerance_satisfied: orientationToleranceSatisfied,
    joint2_initial: seedCase.seed_joint_values?.joint2 ?? null,
    joint2_final: result.jointValues.joint2 ?? null,
    joint2_min: joint2Values.length > 0 ? Math.min(...joint2Values) : null,
    joint2_max: joint2Values.length > 0 ? Math.max(...joint2Values) : null,
    joint2_was_blocked: trace.some((iteration) => iteration.blocked_joints.includes('joint2')),
    first_iteration_joint2_blocked: firstJoint2Block?.iteration_index ?? null,
    joint2_raw_delta_when_first_blocked: firstJoint2Block?.delta_q_raw?.joint2 ?? null,
    residual_history: trace.map((iteration) => ({
      iteration_index: iteration.iteration_index,
      position_residual: iteration.current_position_residual,
      orientation_residual: iteration.current_orientation_residual,
      exit_condition: iteration.exit_condition,
    })),
    joint2_history: trace.map((iteration) => ({
      iteration_index: iteration.iteration_index,
      q_before: iteration.q_before.joint2 ?? null,
      raw_delta: iteration.delta_q_raw?.joint2 ?? null,
      delta_after_active_set: iteration.delta_q_after_active_set.joint2 ?? null,
      q_after: iteration.q_after.joint2 ?? null,
      blocked: iteration.blocked_joints.includes('joint2'),
      active: iteration.active_joints.includes('joint2'),
    })),
  };
};

export const runSeedCases = (
  description: RobotDescription,
  targetPose: Pose,
  options: IKOptions,
  seedCases: IKSeedCase[],
  previousResults: IKSeedResult[] = [],
): IKSeedResult[] => {
  const kinematics = new Kinematics(description);
  const resultsByName = new Map(previousResults.map((result) => [result.seed_name, result]));

  return seedCases.map((seedCase) => {
    if (seedCase.unavailable_reason || !seedCase.seed_joint_values) {
      const unavailable = makeUnavailableResult(seedCase);
      resultsByName.set(seedCase.seed_name, unavailable);
      return unavailable;
    }

    if (seedCase.duplicate_of) {
      const duplicateResult = resultsByName.get(seedCase.duplicate_of);
      if (!duplicateResult || duplicateResult.status === 'unavailable') {
        const unavailable = makeUnavailableResult({
          ...seedCase,
          unavailable_reason: `重复 Seed 的结果不可用：${seedCase.duplicate_of}`,
        });
        resultsByName.set(seedCase.seed_name, unavailable);
        return unavailable;
      }

      const duplicate = {
        ...duplicateResult,
        seed_name: seedCase.seed_name,
        seed_joint_values: seedCase.seed_joint_values,
        executed: false,
        duplicate_of: seedCase.duplicate_of,
      };
      resultsByName.set(seedCase.seed_name, duplicate);
      return duplicate;
    }

    const trace: IKIterationTrace[] = [];
    const result = kinematics.solveIK(
      targetPose,
      seedCase.seed_joint_values,
      options,
      (iteration) => {
        trace.push(iteration);
      },
    );
    const summary = makeResult(seedCase, result, trace, description, options);
    resultsByName.set(seedCase.seed_name, summary);
    return summary;
  });
};

export const compareOriginalReplay = (
  result: IKSeedResult,
  failureRecord: IKActionDebugRecord,
  absoluteTolerance = 1e-10,
): { matches: boolean; differences: string[] } => {
  const differences: string[] = [];
  const close = (left: number | null, right: number | null) =>
    left !== null && right !== null && Math.abs(left - right) <= absoluteTolerance;

  if (result.status !== 'failure') differences.push('status');
  if (result.termination_reason !== failureRecord.termination_reason) {
    differences.push('termination_reason');
  }
  if (result.iterations !== failureRecord.iterations) differences.push('iterations');
  if (!close(result.final_position_residual, failureRecord.position_residual)) {
    differences.push('position_residual');
  }
  if (!close(result.final_orientation_residual, failureRecord.orientation_residual)) {
    differences.push('orientation_residual');
  }
  const actualBlocked = [...result.blocked_joints].sort();
  const expectedBlocked = [...(failureRecord.blocked_joints ?? [])].sort();
  if (JSON.stringify(actualBlocked) !== JSON.stringify(expectedBlocked)) {
    differences.push('blocked_joints');
  }

  const expectedJointValues = failureRecord.final_joint_values;
  if (!result.final_joint_values || !expectedJointValues) {
    differences.push('final_joint_values');
  } else {
    const jointNames = new Set([
      ...Object.keys(result.final_joint_values),
      ...Object.keys(expectedJointValues),
    ]);
    if (
      !Array.from(jointNames).every((jointName) =>
        close(
          result.final_joint_values?.[jointName] ?? null,
          expectedJointValues[jointName] ?? null,
        ),
      )
    ) {
      differences.push('final_joint_values');
    }
  }

  return { matches: differences.length === 0, differences };
};

export const selectLatestFailureRecord = (records: IKActionDebugRecord[]): IKActionDebugRecord => {
  const latest = records
    .filter(
      (record) =>
        record.record_phase === 'result' &&
        record.status === 'failure' &&
        record.termination_reason === 'no_joint_motion' &&
        record.trace_level === 'full' &&
        Array.isArray(record.iteration_trace) &&
        record.iteration_trace.length > 0,
    )
    .at(-1);
  if (!latest) throw new Error('ik-debug.jsonl 中没有完整 trace 的 no_joint_motion 失败记录');
  return latest;
};
