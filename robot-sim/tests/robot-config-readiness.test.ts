import { describe, expect, it } from 'vitest';
import { doesLoadedRobotMatchConfig, findRobotConfig } from '../src/app/config';

describe('loaded robot readiness', () => {
  it('rejects a stale URDF identity when the selected model changes', () => {
    const ur5CadConfig = findRobotConfig('UR5_CAD');

    expect(
      doesLoadedRobotMatchConfig(
        { robotId: 'RS', urdfFile: 'description/RS/urdf/ReBot_Arm_RS.urdf' },
        ur5CadConfig,
      ),
    ).toBe(false);
    expect(
      doesLoadedRobotMatchConfig(
        { robotId: 'UR5_CAD', urdfFile: 'ur_description/urdf/ur5.urdf' },
        ur5CadConfig,
      ),
    ).toBe(false);
  });

  it('accepts only the selected model and URDF file pair', () => {
    const ur5CadConfig = findRobotConfig('UR5_CAD');

    expect(
      doesLoadedRobotMatchConfig({ robotId: 'UR5_CAD', urdfFile: ur5CadConfig.file }, ur5CadConfig),
    ).toBe(true);
    expect(doesLoadedRobotMatchConfig(null, ur5CadConfig)).toBe(false);
  });
});
