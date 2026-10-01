import type { JointState, Pose } from '../core/types';

export interface RobotObservation {
  timestamp: number;
  joints: Readonly<Record<string, JointState>>;
  tcp: Pose;
}
