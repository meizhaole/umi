// @vitest-environment jsdom
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { Quaternion, Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import { findRobotConfig } from '../src/app/config';
import { Kinematics } from '../src/core/Kinematics';
import { RobotModel } from '../src/core/RobotModel';
import { parseUrdf } from '../src/utils/urdfParser';

const ur5CadConfig = findRobotConfig('UR5_CAD');
const ur5CadUrdf = readFileSync(
  resolve('public/robot/ur_umi_real/urdf/ur5_umi_real.urdf'),
  'utf8',
);
const ur5CadDescription = parseUrdf(ur5CadUrdf, { tipLink: ur5CadConfig.tipLink });
const startJoints = ur5CadConfig.initialJoints;
const armJointNames = [
  'shoulder_pan_joint',
  'shoulder_lift_joint',
  'elbow_joint',
  'wrist_1_joint',
  'wrist_2_joint',
  'wrist_3_joint',
];

describe('UR5_CAD umi_tcp_link', () => {
  it('uses the CAD grasp center as UR5_CAD tip and preserves other model tips', () => {
    const fixedJoint = ur5CadDescription.joints.find(
      (joint) => joint.child === 'umi_tcp_link',
    );
    const tcpLink = ur5CadDescription.links.find((link) => link.name === 'umi_tcp_link');

    expect(ur5CadConfig.tipLink).toBe('umi_tcp_link');
    expect(findRobotConfig('UR5').tipLink).toBe('umi_tcp');
    expect(findRobotConfig('RS').tipLink).toBeUndefined();
    expect(findRobotConfig('DM').tipLink).toBeUndefined();
    expect(tcpLink?.visuals).toEqual([]);
    expect(tcpLink?.collisions).toEqual([]);
    expect(tcpLink?.inertial).toBeUndefined();
    expect(fixedJoint).toMatchObject({
      type: 'fixed',
      parent: 'tool0',
      child: 'umi_tcp_link',
      origin: {
        position: [0, -0.0036, 0.224785782],
        orientation: [0, 0, 0, 1],
      },
    });
  });

  it('reports tool0, TCP, and observation EEF from the configured tip link', () => {
    const kinematics = new Kinematics(ur5CadDescription);
    const model = new RobotModel(ur5CadDescription);
    model.setJointValues(startJoints);
    const tool0Pose = kinematics.forwardKinematics(startJoints, 'tool0');
    const tcpPose = kinematics.forwardKinematics(startJoints);
    const observationEefPose = model.getLinkPose();
    const distance = new Vector3(...tool0Pose.position).distanceTo(
      new Vector3(...tcpPose.position),
    );
    const composedTcpPosition = new Vector3(...tool0Pose.position).add(
      new Vector3(0, -0.0036, 0.224785782).applyQuaternion(
        new Quaternion(...tool0Pose.orientation),
      ),
    );
    const expectedDistance = Math.hypot(0.0036, 0.224785782);

    console.info(
      'UR5_CAD initial FK',
      JSON.stringify({
        tipLink: ur5CadDescription.tipLink,
        tool0Pose,
        umiTcpPose: tcpPose,
        observationEefPose,
        distance,
      }),
    );

    expect(distance).toBeCloseTo(expectedDistance, 10);
    tcpPose.position.forEach((value, index) => {
      expect(value).toBeCloseTo(composedTcpPosition.toArray()[index], 12);
    });
    tcpPose.position.forEach((value, index) => {
      expect(value).toBeCloseTo(observationEefPose.position[index], 12);
    });
    tcpPose.orientation.forEach((value, index) => {
      expect(value).toBeCloseTo(observationEefPose.orientation[index], 12);
    });
  });

  it.each([
    { name: 'A current umi_tcp pose', localDelta: [0, 0, 0] as const },
    { name: 'B local +X 5 mm', localDelta: [0.005, 0, 0] as const },
    { name: 'C local +Z 5 mm', localDelta: [0, 0, 0.005] as const },
  ])('solves IK dry-run for $name without changing the start joints', ({ localDelta }) => {
    const kinematics = new Kinematics(ur5CadDescription);
    const startPose = kinematics.forwardKinematics(startJoints);
    const worldDelta = new Vector3(...localDelta).applyQuaternion(
      new Quaternion(...startPose.orientation),
    );
    const targetPose = {
      position: new Vector3(...startPose.position).add(worldDelta).toArray() as [
        number,
        number,
        number,
      ],
      orientation: [...startPose.orientation] as [number, number, number, number],
    };
    const result = kinematics.solveIK(targetPose, startJoints);
    const jointDeltaNorm = Math.hypot(
      ...armJointNames.map(
        (jointName) => (result.jointValues[jointName] ?? 0) - (startJoints[jointName] ?? 0),
      ),
    );
    const jointDeltas = Object.fromEntries(
      armJointNames.map((jointName) => [
        jointName,
        (result.jointValues[jointName] ?? 0) - (startJoints[jointName] ?? 0),
      ]),
    );

    console.info(
      `UR5_CAD IK dry-run ${localDelta.join(',')}`,
      JSON.stringify({
        success: result.converged,
        iterations: result.iterations,
        positionResidual: result.residual.position,
        orientationResidual: result.residual.orientation,
        targetJoints: Object.fromEntries(
          armJointNames.map((jointName) => [jointName, result.jointValues[jointName]]),
        ),
        jointDeltas,
        jointDeltaNorm,
      }),
    );

    expect(result.converged).toBe(true);
    expect(result.residual.position).toBeLessThan(0.001);
    expect(result.residual.orientation).toBeLessThan(0.01);
    expect(jointDeltaNorm).toBeLessThan(0.2);
    if (localDelta.every((value) => value === 0)) {
      expect(jointDeltaNorm).toBeCloseTo(0, 12);
    } else {
      expect(jointDeltaNorm).toBeGreaterThan(0);
    }
  });
});
