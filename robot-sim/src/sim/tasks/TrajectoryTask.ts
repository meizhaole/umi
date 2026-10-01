import type { JointValues, Pose } from '../../core/types';

export interface TrajectoryWaypoint {
  time: number;
  pose?: Pose;
  joints?: JointValues;
}

export interface TrajectoryTaskConfig {
  waypoints: TrajectoryWaypoint[];
}
