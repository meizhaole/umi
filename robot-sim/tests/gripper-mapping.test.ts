// @vitest-environment jsdom
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import type { JointDescription, RobotDescription } from '../src/core/types';
import { findRobotConfig } from '../src/app/config';
import {
  getGripperJoints,
  getGripperOpening,
  mapGripperOpeningToJoints,
} from '../src/app/gripperMapping';
import { parseUrdf } from '../src/utils/urdfParser';

const UR5_CAD_GRIPPER_JOINT_NAMES = ['left_jaw_joint', 'right_jaw_joint'] as const;

const joint = (
  name: string,
  lower: number,
  upper: number,
  mimic?: JointDescription['mimic'],
  axis: [number, number, number] = [1, 0, 0],
): JointDescription => ({
  name,
  type: 'prismatic',
  parent: 'parent',
  child: name.replace('_joint', '_link'),
  origin: { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
  axis,
  limit: { lower, upper },
  ...(mimic ? { mimic } : {}),
});

const description = (...joints: JointDescription[]): RobotDescription =>
  ({ joints }) as RobotDescription;

const UR5_CAD = description(
  joint('left_jaw_joint', 0, 0.055, undefined, [-1, 0, 0]),
  joint('right_jaw_joint', 0, 0.055, undefined, [1, 0, 0]),
);

describe('UR5_CAD gripper width mapping', () => {
  it('matches the configured jaw joints and total opening in the shipped URDF', () => {
    const config = findRobotConfig('UR5_CAD');
    const description = parseUrdf(
      readFileSync(resolve('public/robot/ur_umi_real/urdf/ur5_umi_real.urdf'), 'utf8'),
      { tipLink: config.tipLink },
    );
    const gripperJoints = getGripperJoints(description, config.gripperJointNames);

    expect(gripperJoints.map(({ name, type, limit }) => ({ name, type, limit }))).toEqual([
      {
        name: 'left_jaw_joint',
        type: 'prismatic',
        limit: expect.objectContaining({ lower: 0, upper: 0.055 }),
      },
      {
        name: 'right_jaw_joint',
        type: 'prismatic',
        limit: expect.objectContaining({ lower: 0, upper: 0.055 }),
      },
    ]);
    expect(
      getGripperOpening(
        description,
        { left_jaw_joint: 0.055, right_jaw_joint: 0.055 },
        config.gripperJointNames,
      ),
    ).toBeCloseTo(0.11, 12);
  });

  it('reports zero total opening at both jaw zero positions', () => {
    expect(
      getGripperOpening(
        UR5_CAD,
        { left_jaw_joint: 0, right_jaw_joint: 0 },
        UR5_CAD_GRIPPER_JOINT_NAMES,
      ),
    ).toBe(0);
  });

  it('reports the sum of both jaw travels at an intermediate opening', () => {
    expect(
      getGripperOpening(
        UR5_CAD,
        { left_jaw_joint: 0.0275, right_jaw_joint: 0.0275 },
        UR5_CAD_GRIPPER_JOINT_NAMES,
      ),
    ).toBeCloseTo(0.055, 12);
  });

  it('reports 110 mm at both URDF maximum jaw positions', () => {
    expect(
      getGripperOpening(
        UR5_CAD,
        { left_jaw_joint: 0.055, right_jaw_joint: 0.055 },
        UR5_CAD_GRIPPER_JOINT_NAMES,
      ),
    ).toBeCloseTo(0.11, 12);
  });

  it('maps a legal total opening evenly across both jaws', () => {
    const mapping = mapGripperOpeningToJoints(UR5_CAD, 0.06, UR5_CAD_GRIPPER_JOINT_NAMES);

    expect(mapping).toMatchObject({
      requested_gripper_opening_m: 0.06,
      clamped_gripper_opening_m: 0.06,
      gripper_target_clamped: false,
      left_jaw_target: 0.03,
      right_jaw_target: 0.03,
    });
  });

  it('clamps an over-limit total opening to the summed URDF travel', () => {
    const mapping = mapGripperOpeningToJoints(UR5_CAD, 0.12, UR5_CAD_GRIPPER_JOINT_NAMES);

    expect(mapping).toMatchObject({
      requested_gripper_opening_m: 0.12,
      clamped_gripper_opening_m: 0.11,
      gripper_target_clamped: true,
      left_jaw_target: 0.055,
      right_jaw_target: 0.055,
    });
  });

  it('does not turn the recorded 60.0188 mm policy width into zero', () => {
    const mapping = mapGripperOpeningToJoints(
      UR5_CAD,
      0.06001878157258034,
      UR5_CAD_GRIPPER_JOINT_NAMES,
    );

    expect(mapping.clamped_gripper_opening_m).toBe(0.06001878157258034);
    expect(mapping.gripper_target_clamped).toBe(false);
    expect(mapping.left_jaw_target).toBeCloseTo(0.03000939078629017, 12);
    expect(mapping.right_jaw_target).toBeCloseTo(0.03000939078629017, 12);
  });
});

describe('existing robot gripper mappings', () => {
  it('preserves RS two-joint opening and proportional command behavior', () => {
    const rs = description(joint('gripper_joint1', 0, 0.05), joint('gripper_joint2', 0, 0.0715));

    expect(getGripperOpening(rs, { gripper_joint1: 0.02, gripper_joint2: 0.03 })).toBeCloseTo(
      0.05,
      12,
    );
    const mapping = mapGripperOpeningToJoints(rs, 0.06);
    expect(mapping.gripper_target_clamped).toBe(false);
    expect(mapping.joint_targets.gripper_joint1).toBeCloseTo(0.06 * (0.05 / 0.1215), 12);
    expect(mapping.joint_targets.gripper_joint2).toBeCloseTo(0.06 * (0.0715 / 0.1215), 12);
  });

  it('preserves DM mimic-joint width and command behavior', () => {
    const dm = description(
      joint('finger_left_joint', 0, 0.05),
      joint('finger_right_joint', -0.05, 0, {
        joint: 'finger_left_joint',
        multiplier: -1,
        offset: 0,
      }),
    );

    expect(
      getGripperOpening(dm, { finger_left_joint: 0.02, finger_right_joint: -0.02 }),
    ).toBeCloseTo(0.04, 12);
    const mapping = mapGripperOpeningToJoints(dm, 0.06);
    expect(mapping.joint_targets.finger_left_joint).toBeCloseTo(0.03, 12);
    expect(mapping.joint_targets).not.toHaveProperty('finger_right_joint');
  });

  it('preserves no-gripper behavior for the legacy UR5', () => {
    const legacyUr5 = description(joint('shoulder_pan_joint', 0, 0));
    const mapping = mapGripperOpeningToJoints(legacyUr5, 0.06);

    expect(getGripperOpening(legacyUr5, { shoulder_pan_joint: 0 })).toBe(0);
    expect(mapping).toMatchObject({
      clamped_gripper_opening_m: 0,
      gripper_target_clamped: true,
      left_jaw_target: null,
      right_jaw_target: null,
      joint_targets: {},
    });
  });
});
