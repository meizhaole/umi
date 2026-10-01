import type { JointCommand } from '../core/types';
import type { RobotObservation } from './Observation';
import type { BaseAgent } from './BaseAgent';

export class PolicyRunner {
  constructor(private readonly agent: BaseAgent) {}

  step(observation: RobotObservation): Record<string, JointCommand> {
    return this.agent.step(observation);
  }
}
