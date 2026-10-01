import type { JointDescription, JointValues, Pose, RobotDescription } from './types';
import { Kinematics } from './Kinematics';

const isMovable = (joint: JointDescription): boolean => joint.type !== 'fixed';

const clampJointValue = (joint: JointDescription, value: number): number => {
  const lower = joint.limit?.lower;
  const upper = joint.limit?.upper;
  return Math.max(
    lower ?? Number.NEGATIVE_INFINITY,
    Math.min(upper ?? Number.POSITIVE_INFINITY, value),
  );
};

export class RobotModel {
  readonly description: RobotDescription;

  private readonly kinematics: Kinematics;

  private jointValues: JointValues;

  constructor(description: RobotDescription) {
    this.description = description;
    this.kinematics = new Kinematics(description);
    this.jointValues = Object.fromEntries(description.joints.map((joint) => [joint.name, 0]));
    this.jointValues = this.resolveMimicJoints(this.jointValues);
  }

  getControllableJoints(): JointDescription[] {
    return this.description.joints.filter((joint) => isMovable(joint) && !joint.mimic);
  }

  getJointValues(): JointValues {
    return { ...this.jointValues };
  }

  getJointValue(jointName: string): number {
    const value = this.jointValues[jointName];
    if (value === undefined) throw new Error(`未知关节：${jointName}`);
    return value;
  }

  setJointValue(jointName: string, value: number): void {
    const joint = this.description.joints.find((item) => item.name === jointName);
    if (!joint) throw new Error(`未知关节：${jointName}`);
    if (!isMovable(joint) || joint.mimic) throw new Error(`关节 ${jointName} 不可直接控制`);
    if (!Number.isFinite(value)) throw new Error(`关节 ${jointName} 的值必须是有限数`);
    this.jointValues[jointName] = clampJointValue(joint, value);
    this.jointValues = this.resolveMimicJoints(this.jointValues);
  }

  setJointValues(values: JointValues): void {
    const nextValues = { ...this.jointValues };
    this.getControllableJoints().forEach((joint) => {
      const value = values[joint.name];
      if (value !== undefined) {
        if (!Number.isFinite(value)) throw new Error(`关节 ${joint.name} 的值必须是有限数`);
        nextValues[joint.name] = clampJointValue(joint, value);
      }
    });
    this.jointValues = this.resolveMimicJoints(nextValues);
  }

  getLinkPose(linkName = this.description.tipLink): Pose {
    return this.kinematics.forwardKinematics(this.jointValues, linkName);
  }

  private resolveMimicJoints(values: JointValues): JointValues {
    const resolved = { ...values };
    const resolving = new Set<string>();
    const resolve = (jointName: string): number => {
      const joint = this.description.joints.find((item) => item.name === jointName);
      if (!joint) throw new Error(`mimic 引用了未知关节：${jointName}`);
      if (!joint.mimic) return resolved[jointName] ?? 0;
      if (resolving.has(jointName)) throw new Error(`mimic 关节形成循环：${jointName}`);
      resolving.add(jointName);
      const source = resolve(joint.mimic.joint);
      resolving.delete(jointName);
      const value = clampJointValue(joint, source * joint.mimic.multiplier + joint.mimic.offset);
      resolved[jointName] = value;
      return value;
    };

    this.description.joints
      .filter((joint) => joint.mimic)
      .forEach((joint) => {
        resolve(joint.name);
      });
    return resolved;
  }
}
