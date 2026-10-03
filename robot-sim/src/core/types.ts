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

export type IKTerminationReason =
  | 'converged'
  | 'max_iterations'
  | 'linear_solve_failed'
  | 'joint_limits_blocked'
  | 'no_joint_motion';

export interface IKResult {
  jointValues: JointValues;
  converged: boolean;
  iterations: number;
  residual: IKResidual;
  terminationReason: IKTerminationReason;
  blockedJoints: string[];
}

export type IKIterationExitCondition =
  | 'continue'
  | 'converged'
  | 'max_iterations'
  | 'linear_solve_failed'
  | 'joint_limits_blocked'
  | 'no_joint_motion';

export interface IKIterationTrace {
  iteration_index: number;
  q_before: JointValues;
  current_position_residual: number;
  current_orientation_residual: number;
  delta_q_raw: Record<string, number> | null;
  delta_q_after_active_set: Record<string, number | null>;
  delta_q_after_step_limiting: Record<string, number | null>;
  q_candidate: JointValues;
  q_after: JointValues;
  blocked_joints: string[];
  active_joints: string[];
  clamp_information: Record<
    string,
    {
      blocked_by_active_set: boolean;
      step_limited: boolean;
      joint_limit_clamped: boolean;
      requested_joint_value: number | null;
      clamped_joint_value: number;
    }
  >;
  joint_limit_margin: Record<string, { before: number | null; candidate: number | null }>;
  exit_condition: IKIterationExitCondition;
}

export type IKIterationObserver = (iteration: IKIterationTrace) => void;
