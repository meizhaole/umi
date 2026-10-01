// @vitest-environment jsdom
import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import type { Pose } from '../src/core/types';
import { JointController } from '../src/core/JointController';
import { Kinematics } from '../src/core/Kinematics';
import { RobotModel } from '../src/core/RobotModel';
import { applyJointCommand } from '../src/sim/JointActuator';
import { parseUrdf } from '../src/utils/urdfParser';

const planarUrdf = (upperLimit = 2): string => `
  <robot name="planar">
    <link name="base" />
    <link name="arm" />
    <link name="tool" />
    <joint name="shoulder" type="revolute">
      <parent link="base" />
      <child link="arm" />
      <axis xyz="0 0 1" />
      <limit lower="-2" upper="${upperLimit}" effort="10" velocity="2" />
    </joint>
    <joint name="elbow" type="revolute">
      <origin xyz="1 0 0" />
      <parent link="arm" />
      <child link="tool" />
      <axis xyz="0 0 1" />
      <limit lower="-2" upper="2" effort="10" velocity="2" />
    </joint>
  </robot>
`;

const readDescription = (relativePath: string) =>
  parseUrdf(readFileSync(new URL(relativePath, import.meta.url), 'utf8'));

describe('URDF 解析', () => {
  it('读取 RS 的关节、惯性和碰撞网格', () => {
    const robot = readDescription(
      '../../reBotArmController_ROS2/src/rebotarm_bringup/description/RS/urdf/ReBot_Arm_RS.urdf',
    );
    const base = robot.links.find((link) => link.name === 'base_link');

    expect(robot.joints.filter((joint) => joint.type === 'revolute')).toHaveLength(6);
    expect(robot.tipLink).toBe('gripper_end');
    expect(base?.inertial?.mass).toBeCloseTo(1.1774);
    expect(base?.collisions[0].geometry).toMatchObject({
      type: 'mesh',
      filename: 'package://rebotarm_bringup/description/RS/meshes/shared/base_link.STL',
    });
  });

  it('读取 DM 的 mimic 关系、限位和碰撞网格', () => {
    const robot = readDescription(
      '../../reBotArmController_ROS2/src/rebotarm_bringup/description/DM/urdf/ReBot_Arm_DM.urdf',
    );
    const fingerRight = robot.joints.find((joint) => joint.name === 'finger_right');
    const fingerLink = robot.links.find((link) => link.name === 'finger_right_link');

    expect(robot.joints.filter((joint) => joint.type === 'revolute')).toHaveLength(6);
    expect(robot.tipLink).toBe('end_link');
    expect(fingerRight?.mimic).toEqual({
      joint: 'finger_left',
      multiplier: -1,
      offset: 0,
    });
    expect(fingerLink?.collisions[0].geometry.type).toBe('mesh');

    const model = new RobotModel(robot);
    model.setJointValue('finger_left', 0.02);
    expect(model.getJointValue('finger_right')).toBeCloseTo(-0.02);
  });
});

describe('FK 与 DLS IK', () => {
  it('根据 URDF 关节原点计算正运动学', () => {
    const robot = parseUrdf(planarUrdf());
    const pose = new Kinematics(robot).forwardKinematics({ shoulder: Math.PI / 2, elbow: 0 });

    expect(pose.position[0]).toBeCloseTo(0);
    expect(pose.position[1]).toBeCloseTo(1);
    expect(pose.orientation[2]).toBeCloseTo(Math.SQRT1_2);
    expect(pose.orientation[3]).toBeCloseTo(Math.SQRT1_2);
  });

  it('收敛到可达的完整位姿目标', () => {
    const robot = parseUrdf(planarUrdf());
    const kinematics = new Kinematics(robot);
    const target = kinematics.forwardKinematics({ shoulder: 0.4, elbow: 0.2 });
    const result = kinematics.solveIK(target, { shoulder: 0, elbow: 0 });

    expect(result.converged).toBe(true);
    expect(result.residual.position).toBeLessThan(0.001);
    expect(result.residual.orientation).toBeLessThan(0.01);
  });

  it('报告不可达目标且保留关节限位', () => {
    const robot = parseUrdf(planarUrdf(0.25));
    const kinematics = new Kinematics(robot);
    const target: Pose = {
      position: [5, 5, 5],
      orientation: [1, 0, 0, 0],
    };
    const result = kinematics.solveIK(target, { shoulder: 0, elbow: 0 }, { maxIterations: 80 });

    expect(result.converged).toBe(false);
    expect(result.jointValues.shoulder).toBeLessThanOrEqual(0.25);
    expect(result.jointValues.shoulder).toBeGreaterThanOrEqual(-2);
    expect(result.iterations).toBeLessThanOrEqual(80);
  });

  it('模型写入与位置、速度、力矩命令都遵守 URDF 限位', () => {
    const model = new RobotModel(parseUrdf(planarUrdf(0.25)));
    const controller = new JointController(model);
    model.setJointValue('shoulder', 1);

    expect(model.getJointValue('shoulder')).toBe(0.25);
    expect(controller.setCommand('shoulder', 1).value).toBe(0.25);

    controller.setMode('velocity');
    expect(controller.setCommand('shoulder', 10).value).toBe(2);

    controller.setMode('effort');
    expect(controller.setCommand('shoulder', 100).value).toBe(10);
  });
});

describe('RS / DM Rapier 关节命令适配', () => {
  const models = [
    [
      'RS',
      '../../reBotArmController_ROS2/src/rebotarm_bringup/description/RS/urdf/ReBot_Arm_RS.urdf',
    ],
    [
      'DM',
      '../../reBotArmController_ROS2/src/rebotarm_bringup/description/DM/urdf/ReBot_Arm_DM.urdf',
    ],
  ] as const;

  it.each(models)('%s 位置、速度和 effort 命令映射到物理关节', (_modelId, relativePath) => {
    const model = new RobotModel(readDescription(relativePath));
    const controller = new JointController(model);
    const joints = model.getControllableJoints();
    const calls: Array<[string, number, string?]> = [];
    const actuator = {
      position: (value: number) => calls.push(['position', value]),
      velocity: (value: number) => calls.push(['velocity', value]),
      effort: (value: number, kind: 'torque' | 'force') => calls.push(['effort', value, kind]),
    };

    controller.setMode('position');
    joints.forEach((joint) => {
      const value =
        joint.limit?.lower !== undefined && joint.limit.upper !== undefined
          ? joint.limit.lower + (joint.limit.upper - joint.limit.lower) * 0.25
          : 0.2;
      const command = controller.setCommand(joint.name, value);
      applyJointCommand(joint, 'position', command, 0, actuator);
      expect(calls.at(-1)).toEqual(['position', command.value]);
    });

    controller.setMode('velocity');
    const velocityJoint = joints[0];
    const velocity = controller.setCommand(velocityJoint.name, 0.3);
    applyJointCommand(velocityJoint, 'velocity', velocity, 0, actuator);
    expect(calls.at(-1)).toEqual(['velocity', velocity.value]);

    controller.setMode('effort');
    joints.forEach((joint) => {
      const effort = controller.setCommand(joint.name, 1);
      applyJointCommand(joint, 'effort', effort, 0, actuator);
      expect(calls.at(-1)).toEqual([
        'effort',
        effort.value,
        joint.type === 'prismatic' ? 'force' : 'torque',
      ]);
    });
  });
});
