export type Vec3 = [number, number, number];

export type Quaternion = [number, number, number, number];

export interface Pose {
  position: Vec3;
  orientation: Quaternion;
}

export type JointType = 'fixed' | 'revolute' | 'continuous' | 'prismatic' | 'floating' | 'planar';

export type JointLimit = {
  lower?: number;
  upper?: number;
  effort?: number;
  velocity?: number;
};

export interface JointMimic {
  joint: string;
  multiplier: number;
  offset: number;
}

export interface InertiaTensor {
  ixx: number;
  ixy: number;
  ixz: number;
  iyy: number;
  iyz: number;
  izz: number;
}

export interface InertialDescription {
  origin: Pose;
  mass: number;
  inertia: InertiaTensor;
}

export type GeometryDescription =
  | { type: 'mesh'; filename: string; scale: Vec3 }
  | { type: 'box'; size: Vec3 }
  | { type: 'cylinder'; radius: number; length: number }
  | { type: 'sphere'; radius: number };

export interface VisualDescription {
  name?: string;
  origin: Pose;
  geometry: GeometryDescription;
  materialName?: string;
  color?: [number, number, number, number];
}

export interface CollisionDescription {
  name?: string;
  origin: Pose;
  geometry: GeometryDescription;
}

export interface LinkDescription {
  name: string;
  inertial?: InertialDescription;
  visuals: VisualDescription[];
  collisions: CollisionDescription[];
}

export interface JointDescription {
  name: string;
  type: JointType;
  parent: string;
  child: string;
  origin: Pose;
  axis: Vec3;
  limit?: JointLimit;
  mimic?: JointMimic;
}

export interface RobotDescription {
  name: string;
  rootLink: string;
  tipLink: string;
  links: LinkDescription[];
  joints: JointDescription[];
}

export type JointValues = Record<string, number>;

export interface JointState {
  position: number;
  velocity: number;
  effort: number;
}

export type ControlMode = 'position' | 'velocity' | 'effort';

export interface JointCommand {
  mode: ControlMode;
  value: number;
}

export interface IKOptions {
  maxIterations?: number;
  damping?: number;
  positionTolerance?: number;
  orientationTolerance?: number;
  maxAngularStep?: number;
  maxLinearStep?: number;
}

export interface IKResidual {
  position: number;
  orientation: number;
  total: number;
}

export interface IKResult {
  jointValues: JointValues;
  converged: boolean;
  iterations: number;
  residual: IKResidual;
}
