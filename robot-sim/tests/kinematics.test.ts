// @vitest-environment jsdom
import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import type { Pose } from '../src/core/types';
import { JointController } from '../src/core/JointController';
import { Kinematics } from '../src/core/Kinematics';
import { RobotModel } from '../src/core/RobotModel';
import { applyJointCommand } from '../src/sim/JointActuator';
import { DEFAULT_ROBOT_MODEL_ID, findRobotConfig, ROBOT_MODELS } from '../src/app/config';
import { configureRobotUrdf } from '../src/utils/configureRobotUrdf';
import { parseUrdf } from '../src/utils/urdfParser';
import { quaternionFromEuler } from '../src/utils/math';

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

const readUrdf = (relativePath: string) =>
  readFileSync(new URL(relativePath, import.meta.url), 'utf8');

const readDescription = (relativePath: string) => parseUrdf(readUrdf(relativePath));

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

describe('UR5 官方描述、TCP、FK 与 IK', () => {
  const ur5Config = findRobotConfig('UR5');
  const officialUrdf = readUrdf('../assets/ur_description/urdf/ur5.urdf');
  const makeUr5Description = (tcpOffset = ur5Config.tcpOffset ?? 0) => {
    const config = { ...ur5Config, tcpOffset };
    const xml = configureRobotUrdf(officialUrdf, config);
    return parseUrdf(xml, { tipLink: config.tipLink });
  };
  const startJoints = ur5Config.initialJoints;
  const expectedJointNames = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
  ];

  it('保留 RS、DM 并以 RS 为默认型号', () => {
    expect(ROBOT_MODELS.map((robot) => robot.id)).toEqual(['RS', 'DM', 'UR5', 'UR5_CAD']);
    expect(DEFAULT_ROBOT_MODEL_ID).toBe('RS');
    expect(findRobotConfig('RS').positionExecution).toBe('joint_motors');
    expect(findRobotConfig('DM').positionExecution).toBe('joint_motors');
    expect(findRobotConfig('UR5').positionExecution).toBe('kinematic_fk');
    expect(findRobotConfig('UR5_CAD').file).toBe('ur_umi_real/urdf/ur5_umi_real.urdf');
  });

  it('UR5 CAD 型号只在 tool0 下挂载真实 mount，并保留六轴结构', () => {
    const description = readDescription('../public/robot/ur_umi_real/urdf/ur5_umi_real.urdf');
    const revoluteJoints = description.joints
      .filter((joint) => joint.type === 'revolute')
      .map((joint) => joint.name);
    const mountJoint = description.joints.find((joint) => joint.name === 'ur5_to_mount');
    const mountLink = description.links.find((link) => link.name === 'mount_link');
    const mountVisualMeshes = mountLink?.visuals.map((visual) => visual.geometry) ?? [];

    expect(revoluteJoints).toEqual(expectedJointNames);
    expect(description.links.some((link) => link.name === 'umi_base_link')).toBe(false);
    expect(description.links.some((link) => link.name.includes('finger'))).toBe(false);
    expect(mountJoint).toMatchObject({
      type: 'fixed',
      parent: 'tool0',
      child: 'mount_link',
      origin: {
        position: [0, 0, 0],
        orientation: quaternionFromEuler(-Math.PI / 2, 0, Math.PI),
      },
    });
    expect(mountLink?.visuals.map((visual) => visual.name)).toEqual([
      'Part 1',
      'Part 2',
      'Part 3',
      'Part 4',
      'Part 5',
      'Part 6',
      'finger_holder_right',
      'finger_holder_left',
      'gripper_mount',
    ]);
    expect(mountLink?.visuals.map((visual) => visual.materialName)).toEqual([
      'cad_part_1',
      'cad_parts_2_to_6',
      'cad_parts_2_to_6',
      'cad_parts_2_to_6',
      'cad_parts_2_to_6',
      'cad_parts_2_to_6',
      'cad_finger_holders',
      'cad_finger_holders',
      'cad_gripper_mount',
    ]);
    expect(mountVisualMeshes).toEqual([
      'part_1.stl',
      'part_2.stl',
      'part_3.stl',
      'part_4.stl',
      'part_5.stl',
      'part_6.stl',
      'finger_holder_right.stl',
      'finger_holder_left.stl',
      'mount.stl',
    ].map((mesh) => ({
      type: 'mesh',
      filename: `package://ur_umi_real/meshes/${mesh}`,
      scale: [1, 1, 1],
    })));
    expect(mountLink?.visuals.map((visual) => visual.origin)).toEqual(
      Array.from({ length: 9 }, () => ({
        position: [0, 0, 0],
        orientation: [0, 0, 0, 1],
      })),
    );
    expect(mountLink?.collisions.map((collision) => collision.geometry)).toEqual([
      {
        type: 'mesh',
        filename: 'package://ur_umi_real/meshes/mount.stl',
        scale: [1, 1, 1],
      },
    ]);
  });

  it('解析官方六关节、关键连杆和显式 umi_tcp', () => {
    const description = makeUr5Description();
    const revoluteJoints = description.joints
      .filter((joint) => joint.type === 'revolute')
      .map((joint) => joint.name);

    expect(revoluteJoints).toEqual(expectedJointNames);
    expect(description.links.map((link) => link.name)).toEqual(
      expect.arrayContaining(['base_link', 'wrist_3_link', 'flange', 'tool0', 'umi_tcp']),
    );
    expect(description.tipLink).toBe('umi_tcp');
    expect(description.joints.find((joint) => joint.child === 'umi_tcp')?.type).toBe('fixed');
    expectedJointNames.forEach((jointName) => {
      const joint = description.joints.find((item) => item.name === jointName);
      expect(joint?.axis).toEqual([0, 0, 1]);
      expect(joint?.limit?.lower).toBeDefined();
      expect(joint?.limit?.upper).toBeDefined();
    });
  });

  it('初始 FK 的 tool0 与 umi_tcp 在 tcpOffset 为零时重合', () => {
    const description = makeUr5Description();
    const kinematics = new Kinematics(description);
    const tool0Pose = kinematics.forwardKinematics(startJoints, 'tool0');
    const umiTcpPose = kinematics.forwardKinematics(startJoints, 'umi_tcp');

    console.info('UR5 initial FK', JSON.stringify({ tool0Pose, umiTcpPose }));
    tool0Pose.position.forEach((value, index) => {
      expect(umiTcpPose.position[index]).toBeCloseTo(value, 12);
    });
    tool0Pose.orientation.forEach((value, index) => {
      expect(umiTcpPose.orientation[index]).toBeCloseTo(value, 12);
    });
  });

  it('tcpOffset 配置沿 tool0 局部 Z 轴建立固定变换', () => {
    const description = makeUr5Description(0.035);
    const kinematics = new Kinematics(description);
    const tool0 = kinematics.forwardKinematics(startJoints, 'tool0');
    const umiTcp = kinematics.forwardKinematics(startJoints, 'umi_tcp');
    const offset = Math.hypot(
      umiTcp.position[0] - tool0.position[0],
      umiTcp.position[1] - tool0.position[1],
      umiTcp.position[2] - tool0.position[2],
    );

    expect(offset).toBeCloseTo(0.035, 10);
  });

  it('六个关节分别改变自己的子 link，且不移动父 link', () => {
    const description = makeUr5Description();
    const kinematics = new Kinematics(description);
    const initialPoses = kinematics.forwardKinematicsAll(startJoints);

    expectedJointNames.forEach((jointName) => {
      const joint = description.joints.find((item) => item.name === jointName);
      if (!joint) throw new Error(`缺少 UR5 joint：${jointName}`);
      const movedValues = { ...startJoints, [jointName]: startJoints[jointName] + 0.1 };
      const movedPoses = kinematics.forwardKinematicsAll(movedValues);

      expect(movedPoses[joint.parent].position).toEqual(initialPoses[joint.parent].position);
      expect(movedPoses[joint.child].orientation).not.toEqual(
        initialPoses[joint.child].orientation,
      );
      console.info(
        'UR5 joint FK',
        JSON.stringify({ jointName, parent: joint.parent, child: joint.child }),
      );
    });
  });

  it.each([
    { axis: 'x', index: 0, delta: 0.02 },
    { axis: 'x', index: 0, delta: -0.02 },
    { axis: 'y', index: 1, delta: 0.02 },
    { axis: 'y', index: 1, delta: -0.02 },
    { axis: 'z', index: 2, delta: 0.02 },
    { axis: 'z', index: 2, delta: -0.02 },
  ])('IK 目标沿 $axis 轴移动 $delta m', ({ axis, index, delta }) => {
    const description = makeUr5Description();
    const kinematics = new Kinematics(description);
    const startPose = kinematics.forwardKinematics(startJoints, 'umi_tcp');
    const targetPose: Pose = {
      position: [...startPose.position],
      orientation: [...startPose.orientation],
    };
    targetPose.position[index] += delta;

    const result = kinematics.solveIK(targetPose, startJoints);
    const jointDeltaNorm = Math.hypot(
      ...expectedJointNames.map(
        (jointName) => (result.jointValues[jointName] ?? 0) - (startJoints[jointName] ?? 0),
      ),
    );
    console.info(
      `UR5 IK ${axis} ${delta > 0 ? '+' : ''}${delta}`,
      JSON.stringify({
        targetPose,
        startJoints,
        solutionJoints: Object.fromEntries(
          expectedJointNames.map((jointName) => [jointName, result.jointValues[jointName]]),
        ),
        success: result.converged,
        positionResidual: result.residual.position,
        orientationResidual: result.residual.orientation,
        jointDeltaNorm,
      }),
    );

    expect(result.converged).toBe(true);
    expect(result.residual.position).toBeLessThanOrEqual(0.001);
    expect(result.residual.orientation).toBeLessThanOrEqual(0.01);
    expect(jointDeltaNorm).toBeGreaterThan(0);
  });

  it('通用 parser 保留 xyz、rpy、任意关节轴和 fixed 变换', () => {
    const xml = `
      <robot name="generic_transform">
        <link name="base" />
        <link name="arm" />
        <link name="tip" />
        <joint name="axis_y" type="revolute">
          <origin xyz="0.1 0.2 0.3" rpy="0.3 -0.2 0.5" />
          <parent link="base" />
          <child link="arm" />
          <axis xyz="0 1 0" />
          <limit lower="-1" upper="1" effort="10" velocity="2" />
        </joint>
        <joint name="fixed_tip" type="fixed">
          <origin xyz="0 0 0.05" />
          <parent link="arm" />
          <child link="tip" />
        </joint>
      </robot>
    `;
    const description = parseUrdf(xml, { tipLink: 'tip' });
    const pose = new Kinematics(description).forwardKinematics({}, 'arm');
    const joint = description.joints.find((item) => item.name === 'axis_y');

    expect(description.tipLink).toBe('tip');
    expect(joint?.origin.position).toEqual([0.1, 0.2, 0.3]);
    expect(joint?.axis).toEqual([0, 1, 0]);
    expect(pose.orientation).toEqual(quaternionFromEuler(0.3, -0.2, 0.5));
  });
});
