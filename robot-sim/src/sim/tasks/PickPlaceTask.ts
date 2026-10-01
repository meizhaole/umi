import type { Pose } from '../../core/types';

export interface PickPlaceTaskConfig {
  pickPose: Pose;
  placePose: Pose;
  gripperOpen: number;
  gripperClosed: number;
}
