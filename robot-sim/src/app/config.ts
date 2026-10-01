export const ROBOT_MODELS = [
  { id: 'RS', label: 'RS Series', file: 'ReBot_Arm_RS.urdf' },
  { id: 'DM', label: 'DM Series', file: 'ReBot_Arm_DM.urdf' },
] as const;

export type RobotModelId = (typeof ROBOT_MODELS)[number]['id'];

export const SIMULATION_CONFIG = {
  fixedTimeStep: 1 / 60,
  gravity: [0, -9.81, 0] as [number, number, number],
  background: '#10151d',
  robotAssetRoot: '/robot/description',
};

export const findRobotConfig = (id: RobotModelId) =>
  ROBOT_MODELS.find((model) => model.id === id) ?? ROBOT_MODELS[0];
