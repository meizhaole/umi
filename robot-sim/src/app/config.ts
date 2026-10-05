import type { JointValues } from '../core/types';

export interface RobotConfig {
  id: 'RS' | 'DM' | 'UR5' | 'UR5_UMI';
  type: 'rs' | 'dm' | 'ur5';
  label: string;
  positionExecution: 'joint_motors' | 'kinematic_fk';
  file: string;
  packageMappings: Readonly<Record<string, string>>;
  initialJoints: JointValues;
  tipLink?: string;
  tcpParentLink?: string;
  tcpOffset?: number;
}

export const ROBOT_MODELS = [
  {
    id: 'RS',
    type: 'rs',
    label: 'RS Series',
    positionExecution: 'joint_motors',
    file: 'description/RS/urdf/ReBot_Arm_RS.urdf',
    packageMappings: { rebotarm_bringup: '/robot' },
    initialJoints: { joint2: 0.1, joint3: 0.1 },
  },
  {
    id: 'DM',
    type: 'dm',
    label: 'DM Series',
    positionExecution: 'joint_motors',
    file: 'description/DM/urdf/ReBot_Arm_DM.urdf',
    packageMappings: { rebotarm_bringup: '/robot' },
    initialJoints: {},
  },
  {
    id: 'UR5',
    type: 'ur5',
    label: 'Universal Robots UR5',
    positionExecution: 'kinematic_fk',
    file: 'ur_description/urdf/ur5.urdf',
    packageMappings: { ur_description: '/robot/ur_description' },
    initialJoints: {
      shoulder_pan_joint: 0,
      shoulder_lift_joint: -Math.PI / 2,
      elbow_joint: -Math.PI / 2,
      wrist_1_joint: -Math.PI / 2,
      wrist_2_joint: Math.PI / 2,
      wrist_3_joint: 0,
    },
    tipLink: 'umi_tcp',
    tcpParentLink: 'tool0',
    tcpOffset: 0.0,
  },
  {
    id: 'UR5_UMI',
    type: 'ur5',
    label: 'UR5 + UMI Gripper',
    positionExecution: 'kinematic_fk',
    file: 'ur_umi/ur5_umi.urdf',
    packageMappings: {
      ur_description: '/robot/ur_description',
      umi_gripper: '/robot/umi-gripper',
    },
    initialJoints: {
      shoulder_pan_joint: 0,
      shoulder_lift_joint: -Math.PI / 2,
      elbow_joint: -Math.PI / 2,
      wrist_1_joint: -Math.PI / 2,
      wrist_2_joint: Math.PI / 2,
      wrist_3_joint: 0,
      left_finger_joint: 0,
      right_finger_joint: 0,
    },
    tipLink: 'tool0',
  },
] as const satisfies readonly RobotConfig[];

export type RobotModelId = (typeof ROBOT_MODELS)[number]['id'];
export type RobotType = (typeof ROBOT_MODELS)[number]['type'];

export const DEFAULT_ROBOT_MODEL_ID: RobotModelId = 'RS';

export const SIMULATION_CONFIG = {
  fixedTimeStep: 1 / 60,
  gravity: [0, -9.81, 0] as [number, number, number],
  background: '#10151d',
  robotAssetRoot: '/robot',
};

export const findRobotConfig = (id: RobotModelId): RobotConfig =>
  ROBOT_MODELS.find((model) => model.id === id) ?? ROBOT_MODELS[0];
