import type { JointDescription, JointValues, RobotDescription } from '../core/types';

export interface GripperOpeningMapping {
  requested_gripper_opening_m: number;
  clamped_gripper_opening_m: number;
  gripper_target_clamped: boolean;
  left_jaw_target: number | null;
  right_jaw_target: number | null;
  joint_targets: JointValues;
}

export const getGripperJoints = (
  description: RobotDescription,
  configuredJointNames?: readonly string[],
): JointDescription[] => {
  if (configuredJointNames && configuredJointNames.length > 0) {
    return configuredJointNames.map((name) => {
      const joint = description.joints.find((candidate) => candidate.name === name);
      if (!joint || joint.type !== 'prismatic') {
        throw new Error(`配置的夹爪关节 ${name} 不存在或不是 prismatic joint。`);
      }
      return joint;
    });
  }

  return description.joints.filter(
    (joint) =>
      joint.type === 'prismatic' && /gripper|finger/iu.test(joint.name + ' ' + joint.child),
  );
};

const getJointBaseline = (joint: JointDescription, joints: JointDescription[]): number => {
  if (!joint.mimic) return joint.limit?.lower ?? 0;
  const source = joints.find((candidate) => candidate.name === joint.mimic?.joint);
  return (source?.limit?.lower ?? 0) * joint.mimic.multiplier + joint.mimic.offset;
};

const getJointTravel = (joint: JointDescription): number => {
  const lower = joint.limit?.lower ?? 0;
  const upper = joint.limit?.upper ?? lower;
  return Math.abs(upper - lower);
};

export const getGripperOpening = (
  description: RobotDescription,
  jointValues: JointValues,
  configuredJointNames?: readonly string[],
): number => {
  const gripperJoints = getGripperJoints(description, configuredJointNames);
  return gripperJoints.reduce((opening, joint) => {
    const baseline = getJointBaseline(joint, gripperJoints);
    const position = jointValues[joint.name] ?? baseline;
    return opening + Math.abs(position - baseline);
  }, 0);
};

export const mapGripperOpeningToJoints = (
  description: RobotDescription,
  requestedOpening: number,
  configuredJointNames?: readonly string[],
): GripperOpeningMapping => {
  const gripperJoints = getGripperJoints(description, configuredJointNames);
  const maximumOpening = gripperJoints.reduce((total, joint) => total + getJointTravel(joint), 0);
  const clampedOpening = Math.max(0, Math.min(maximumOpening, requestedOpening));
  const openingFraction = maximumOpening > 0 ? clampedOpening / maximumOpening : 0;
  const jointTargets = Object.fromEntries(
    gripperJoints.flatMap((joint) => {
      if (joint.mimic) return [];
      const lower = joint.limit?.lower ?? 0;
      const upper = joint.limit?.upper ?? lower;
      return [[joint.name, lower + (upper - lower) * openingFraction]];
    }),
  );

  return {
    requested_gripper_opening_m: requestedOpening,
    clamped_gripper_opening_m: clampedOpening,
    gripper_target_clamped: clampedOpening !== requestedOpening,
    left_jaw_target: jointTargets.left_jaw_joint ?? null,
    right_jaw_target: jointTargets.right_jaw_joint ?? null,
    joint_targets: jointTargets,
  };
};
