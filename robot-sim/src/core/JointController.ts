import type { ControlMode, JointCommand, JointDescription } from './types';
import { RobotModel } from './RobotModel';

const clamp = (value: number, lower: number, upper: number): number =>
  Math.max(lower, Math.min(upper, value));

export class JointController {
  private readonly robotModel: RobotModel;

  private mode: ControlMode = 'position';

  private readonly commands = new Map<string, JointCommand>();

  constructor(robotModel: RobotModel) {
    this.robotModel = robotModel;
  }

  setMode(mode: ControlMode): void {
    this.mode = mode;
    this.commands.clear();
  }

  getMode(): ControlMode {
    return this.mode;
  }

  setCommand(jointName: string, value: number): JointCommand {
    if (!Number.isFinite(value)) throw new Error(`关节 ${jointName} 的命令必须是有限数`);
    const joint = this.getControllableJoint(jointName);
    const command = { mode: this.mode, value: this.limitCommand(joint, value) };
    this.commands.set(jointName, command);
    return { ...command };
  }

  getCommand(jointName: string): JointCommand | undefined {
    const command = this.commands.get(jointName);
    return command ? { ...command } : undefined;
  }

  getCommands(): Record<string, JointCommand> {
    return Object.fromEntries(
      Array.from(this.commands, ([name, command]) => [name, { ...command }]),
    );
  }

  private getControllableJoint(jointName: string): JointDescription {
    const joint = this.robotModel.getControllableJoints().find((item) => item.name === jointName);
    if (!joint) throw new Error(`关节 ${jointName} 不可控制`);
    return joint;
  }

  private limitCommand(joint: JointDescription, value: number): number {
    if (this.mode === 'position') {
      return clamp(
        value,
        joint.limit?.lower ?? Number.NEGATIVE_INFINITY,
        joint.limit?.upper ?? Number.POSITIVE_INFINITY,
      );
    }
    if (this.mode === 'velocity') {
      const limit = joint.limit?.velocity ?? Number.POSITIVE_INFINITY;
      return clamp(value, -limit, limit);
    }
    const limit = joint.limit?.effort ?? Number.POSITIVE_INFINITY;
    return clamp(value, -limit, limit);
  }
}
