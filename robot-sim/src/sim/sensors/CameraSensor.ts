import type { Pose } from '../../core/types';

export interface CameraFrame {
  timestamp: number;
  pose: Pose;
  width: number;
  height: number;
  data: Uint8Array;
}

export interface CameraSensor {
  capture(): CameraFrame | undefined;
}
