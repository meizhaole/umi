import type { JointCommand } from '../core/types';
import type { RobotObservation } from './Observation';

export abstract class BaseAgent {
  abstract step(observation: RobotObservation): Record<string, JointCommand>;
}
