// 验证杯子接近测试的真实几何预测与接触前停止判据。
// @vitest-environment jsdom
import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';
import URDFLoader from 'urdf-loader';
import type { RobotDescription } from '../src/core/types';
import {
  cloneRobotAtJointValues,
  distanceAabbToAabb,
  evaluateCupApproachPreContactSafety,
  evaluateSweptAabbSafety,
  GUARDED_END_EFFECTOR_GEOMETRIES,
  type PreContactAabbSample,
  type WorldAabb,
} from '../src/sim/tasks/cupApproachPreContactSafety';

const aabb = (min: [number, number, number], max: [number, number, number]): WorldAabb => ({
  min,
  max,
  center: min.map((value, index) => (value + max[index]) / 2) as [number, number, number],
  size: min.map((value, index) => max[index] - value) as [number, number, number],
});

const separatedBox = (x: number): WorldAabb => aabb([x, 0, 0], [x + 0.1, 0.1, 0.1]);

const makeDescriptionJoint = (
  name: string,
  type: 'fixed' | 'prismatic',
  parent: string,
  child: string,
  axis: [number, number, number] = [0, 0, 1],
  limit?: { lower: number; upper: number },
): RobotDescription['joints'][number] => ({
  name,
  type,
  parent,
  child,
  origin: { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
  axis,
  ...(limit ? { limit: { ...limit, effort: 1, velocity: 1 } } : {}),
});

const resultFor = (
  samples: PreContactAabbSample[],
  names: string[] = ['left_soft_finger'],
  safetyMarginM = 0,
) => {
  const current = Object.fromEntries(names.map((name) => [name, separatedBox(0)]));
  const target = Object.fromEntries(names.map((name) => [name, separatedBox(2)]));
  return evaluateSweptAabbSafety({
    cupWorldAabb: separatedBox(1),
    guardedGeometries: names,
    currentGeometryAabbs: current,
    targetGeometryAabbs: target,
    samples,
    safetyMarginM,
    safetyMarginBasis: 'test margin',
  });
};

describe('distanceAabbToAabb', () => {
  it('分离时返回最近欧氏距离', () => {
    expect(
      distanceAabbToAabb(aabb([0, 0, 0], [1, 1, 1]), aabb([1.3, 1.4, 1], [2, 2, 2])),
    ).toBeCloseTo(0.5);
  });

  it('接触或重叠时返回零', () => {
    const box = aabb([0, 0, 0], [1, 1, 1]);
    expect(distanceAabbToAabb(box, aabb([1, 0.2, 0.2], [2, 0.8, 0.8]))).toBe(0);
    expect(distanceAabbToAabb(box, aabb([0.5, 0.2, 0.2], [1.5, 0.8, 0.8]))).toBe(0);
  });
});

describe('evaluateSweptAabbSafety', () => {
  it('当前安全但目标 AABB 重叠时停止', () => {
    const cup = separatedBox(1);
    const current = separatedBox(0);
    const overlapping = aabb([1.05, 0, 0], [1.15, 0.1, 0.1]);
    const result = evaluateSweptAabbSafety({
      cupWorldAabb: cup,
      guardedGeometries: ['left_soft_finger'],
      currentGeometryAabbs: { left_soft_finger: current },
      targetGeometryAabbs: { left_soft_finger: overlapping },
      samples: [
        { sampleIndex: 0, geometryAabbs: { left_soft_finger: current } },
        { sampleIndex: 1, geometryAabbs: { left_soft_finger: overlapping } },
      ],
      safetyMarginM: 0,
      safetyMarginBasis: 'test margin',
    });

    expect(result.current_min_distance_m).toBeGreaterThan(0);
    expect(result.target_min_distance_m).toBe(0);
    expect(result.verdict).toBe('STOP_BEFORE_UNVALIDATED_CONTACT');
  });

  it('两端安全但中间插值样本重叠时停止', () => {
    const cup = separatedBox(1);
    const start = separatedBox(0);
    const middle = aabb([1.02, 0, 0], [1.12, 0.1, 0.1]);
    const end = separatedBox(2);
    const result = evaluateSweptAabbSafety({
      cupWorldAabb: cup,
      guardedGeometries: ['right_soft_finger'],
      currentGeometryAabbs: { right_soft_finger: start },
      targetGeometryAabbs: { right_soft_finger: end },
      samples: [
        { sampleIndex: 0, geometryAabbs: { right_soft_finger: start } },
        { sampleIndex: 1, geometryAabbs: { right_soft_finger: middle } },
        { sampleIndex: 2, geometryAabbs: { right_soft_finger: end } },
      ],
      safetyMarginM: 0,
      safetyMarginBasis: 'test margin',
    });

    expect(result.target_min_distance_m).toBeGreaterThan(0);
    expect(result.swept_closest_geometry).toBe('right_soft_finger');
    expect(result.swept_closest_sample).toBe(1);
    expect(result.verdict).toBe('STOP_BEFORE_UNVALIDATED_CONTACT');
  });

  it.each(GUARDED_END_EFFECTOR_GEOMETRIES.map(({ name }) => name))('%s 触发时停止', (geometry) => {
    const overlap = aabb([1, 0, 0], [1.1, 0.1, 0.1]);
    const result = resultFor(
      [{ sampleIndex: 4, geometryAabbs: { [geometry]: overlap } }],
      [geometry],
    );

    expect(result.verdict).toBe('STOP_BEFORE_UNVALIDATED_CONTACT');
    expect(result.swept_closest_geometry).toBe(geometry);
  });

  it('完整安全路径允许执行原动作', () => {
    const result = resultFor([
      { sampleIndex: 0, geometryAabbs: { left_soft_finger: separatedBox(0) } },
      { sampleIndex: 1, geometryAabbs: { left_soft_finger: separatedBox(2) } },
    ]);

    expect(result.verdict).toBe('SAFE_TO_EXECUTE');
    expect(result.would_enter_unvalidated_contact).toBe(false);
  });

  it('按调用时提供的 cup AABB 计算当前距离', () => {
    const robot = separatedBox(0);
    const first = resultFor([{ sampleIndex: 0, geometryAabbs: { left_soft_finger: robot } }]);
    const movedCup = aabb([4, 0, 0], [4.1, 0.1, 0.1]);
    const moved = evaluateSweptAabbSafety({
      cupWorldAabb: movedCup,
      guardedGeometries: ['left_soft_finger'],
      currentGeometryAabbs: { left_soft_finger: robot },
      targetGeometryAabbs: { left_soft_finger: robot },
      samples: [{ sampleIndex: 0, geometryAabbs: { left_soft_finger: robot } }],
      safetyMarginM: 0,
      safetyMarginBasis: 'test margin',
    });

    expect(moved.cup_world_aabb).toEqual(movedCup);
    expect(moved.current_min_distance_m).toBeGreaterThan(first.current_min_distance_m as number);
  });
});

describe('cloneRobotAtJointValues', () => {
  it('使用 target joints 预测视觉几何，且不移动 live robot', () => {
    const robot = new URDFLoader().parse(`
      <robot name="clone-test">
        <link name="base" />
        <link name="jaw">
          <visual name="left_finger">
            <geometry><box size="0.1 0.1 0.1" /></geometry>
          </visual>
        </link>
        <joint name="slide" type="prismatic">
          <parent link="base" />
          <child link="jaw" />
          <axis xyz="1 0 0" />
          <limit lower="0" upper="1" effort="1" velocity="1" />
        </joint>
      </robot>
    `);
    robot.setJointValues({ slide: 0 });
    robot.updateMatrixWorld(true);

    const predicted = cloneRobotAtJointValues(robot, { slide: 0.4 });

    expect(predicted).not.toBe(robot);
    expect(predicted.joints.slide.jointValue[0]).toBeCloseTo(0.4);
    expect(robot.joints.slide.jointValue[0]).toBe(0);
    expect(predicted.visual.left_finger.getWorldPosition(new Vector3()).x).toBeCloseTo(0.4);
  });

  it('目标完整 mesh AABB 来自 target joints，并读取调用时的 cup AABB', () => {
    const robot = new URDFLoader().parse(`
      <robot name="guard-test">
        <link name="base" />
        <link name="wrist_2_link"><visual><geometry><box size="0.04 0.04 0.04" /></geometry></visual></link>
        <link name="wrist_3_link"><visual><geometry><box size="0.04 0.04 0.04" /></geometry></visual></link>
        <link name="tool0" />
        <link name="umi_tcp_link" />
        <link name="mount_link">
          <visual name="gripper_mount"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
          <visual name="gopro"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
        </link>
        <link name="fixed_mount_link">
          <visual name="Part 1"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
          <visual name="Part 2"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
          <visual name="Part 3"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
          <visual name="Part 4"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
          <visual name="Part 5"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
          <visual name="Part 6"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
        </link>
        <link name="left_jaw_link">
          <visual name="finger_holder_left"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
          <visual name="left_finger"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
        </link>
        <link name="right_jaw_link">
          <visual name="finger_holder_right"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
          <visual name="right_finger"><geometry><box size="0.04 0.04 0.04" /></geometry></visual>
        </link>
        <joint name="slide" type="prismatic">
          <parent link="base" /><child link="wrist_2_link" />
          <axis xyz="1 0 0" /><limit lower="0" upper="1" effort="1" velocity="1" />
        </joint>
        <joint name="wrist3_fixed" type="fixed">
          <parent link="wrist_2_link" /><child link="wrist_3_link" />
        </joint>
        <joint name="tool_fixed" type="fixed">
          <parent link="wrist_3_link" /><child link="tool0" />
        </joint>
        <joint name="tcp_fixed" type="fixed">
          <parent link="tool0" /><child link="umi_tcp_link" />
        </joint>
        <joint name="mount_fixed" type="fixed">
          <parent link="tool0" /><child link="mount_link" />
        </joint>
        <joint name="fixed_mount_joint" type="fixed">
          <parent link="mount_link" /><child link="fixed_mount_link" />
        </joint>
        <joint name="left_jaw_joint" type="prismatic">
          <parent link="fixed_mount_link" /><child link="left_jaw_link" />
          <axis xyz="-1 0 0" /><limit lower="0" upper="0.055" effort="1" velocity="1" />
        </joint>
        <joint name="right_jaw_joint" type="prismatic">
          <parent link="fixed_mount_link" /><child link="right_jaw_link" />
          <axis xyz="1 0 0" /><limit lower="0" upper="0.055" effort="1" velocity="1" />
        </joint>
      </robot>
    `);
    robot.rotation.set(-Math.PI / 2, 0, 0);
    robot.setJointValues({ slide: 0, left_jaw_joint: 0, right_jaw_joint: 0 });
    robot.updateMatrixWorld(true);
    const description: RobotDescription = {
      name: 'guard-test',
      rootLink: 'base',
      tipLink: 'umi_tcp_link',
      links: [
        'base',
        'wrist_2_link',
        'wrist_3_link',
        'tool0',
        'umi_tcp_link',
        'mount_link',
        'fixed_mount_link',
        'left_jaw_link',
        'right_jaw_link',
      ].map((name) => ({ name, visuals: [], collisions: [] })),
      joints: [
        makeDescriptionJoint('slide', 'prismatic', 'base', 'wrist_2_link', [1, 0, 0], {
          lower: 0,
          upper: 1,
        }),
        makeDescriptionJoint('wrist3_fixed', 'fixed', 'wrist_2_link', 'wrist_3_link'),
        makeDescriptionJoint('tool_fixed', 'fixed', 'wrist_3_link', 'tool0'),
        makeDescriptionJoint('tcp_fixed', 'fixed', 'tool0', 'umi_tcp_link'),
        makeDescriptionJoint('mount_fixed', 'fixed', 'tool0', 'mount_link'),
        makeDescriptionJoint('fixed_mount_joint', 'fixed', 'mount_link', 'fixed_mount_link'),
        makeDescriptionJoint(
          'left_jaw_joint',
          'prismatic',
          'fixed_mount_link',
          'left_jaw_link',
          [-1, 0, 0],
          { lower: 0, upper: 0.055 },
        ),
        makeDescriptionJoint(
          'right_jaw_joint',
          'prismatic',
          'fixed_mount_link',
          'right_jaw_link',
          [1, 0, 0],
          { lower: 0, upper: 0.055 },
        ),
      ],
    };
    const currentJoints = { slide: 0, left_jaw_joint: 0, right_jaw_joint: 0 };
    const targetJoints = { slide: 0.4, left_jaw_joint: 0.02, right_jaw_joint: 0.02 };
    const farCup = aabb([4, 4, 4], [4.1, 4.1, 4.1]);
    const currentFinger = robot.visual.left_finger.getWorldPosition(new Vector3()).x;
    const first = evaluateCupApproachPreContactSafety({
      robot,
      description,
      currentJointValues: currentJoints,
      targetJointValues: targetJoints,
      cupWorldAabb: farCup,
    });

    expect(first.verdict).toBe('SAFE_TO_EXECUTE');
    expect(first.current_geometry_aabbs.left_soft_finger.center[0]).toBeCloseTo(currentFinger);
    expect(first.target_geometry_aabbs.left_soft_finger.center[0]).not.toBeCloseTo(currentFinger);
    expect(first.target_joint_values).toMatchObject(targetJoints);
    expect(first.interpolation_intervals).toBeGreaterThan(1);
    expect(robot.visual.left_finger.getWorldPosition(new Vector3()).x).toBeCloseTo(currentFinger);

    const movedCup = first.target_geometry_aabbs.left_soft_finger;
    const second = evaluateCupApproachPreContactSafety({
      robot,
      description,
      currentJointValues: currentJoints,
      targetJointValues: targetJoints,
      cupWorldAabb: movedCup,
    });

    expect(second.cup_world_aabb).toEqual(movedCup);
    expect(second.verdict).toBe('STOP_BEFORE_UNVALIDATED_CONTACT');
  });
});
