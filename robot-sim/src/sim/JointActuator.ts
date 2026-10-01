import type { ControlMode, JointCommand, JointDescription } from '../core/types';

export interface JointActuatorAdapter {
  position: (target: number) => void;
  velocity: (target: number) => void;
  effort: (value: number, kind: 'torque' | 'force') => void;
}

export const applyJointCommand = (
  joint: JointDescription,
  mode: ControlMode,
  command: JointCommand | undefined,
  currentPosition: number,
  actuator: JointActuatorAdapter,
): void => {
  if (mode === 'position') {
    actuator.position(command?.mode === 'position' ? command.value : currentPosition);
  } else if (mode === 'velocity') {
    actuator.velocity(command?.mode === 'velocity' ? command.value : 0);
  } else {
    actuator.effort(
      command?.mode === 'effort' ? command.value : 0,
      joint.type === 'prismatic' ? 'force' : 'torque',
    );
  }
};
