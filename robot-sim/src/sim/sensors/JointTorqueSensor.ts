export interface JointTorqueSample {
  timestamp: number;
  jointName: string;
  torque: number;
}

export interface JointTorqueSensor {
  read(jointName: string): JointTorqueSample | undefined;
}
