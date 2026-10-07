// 验证杯子几何与 TCP 接近指标只依赖当前 world transform。
import { BoxGeometry, Group, Mesh } from 'three';
import { describe, expect, it } from 'vitest';
import {
  evaluateCupApproachSample,
  getObjectWorldAabb,
  robotPositionToSceneWorld,
  summarizeCupApproachDistances,
  type WorldAabb,
} from '../src/sim/tasks/cupApproachEvaluation';

const cupAabb: WorldAabb = {
  min: [-1, -2, -3],
  max: [1, 2, 3],
  center: [0, 0, 0],
  size: [2, 4, 6],
};

describe('cup approach evaluation geometry', () => {
  it('计算点到 AABB 表面代理的距离', () => {
    const sample = evaluateCupApproachSample([2, 0, 0], cupAabb);

    expect(sample.tcpToCupSurfaceProxyDistanceM).toBe(1);
    expect(sample.tcpToCupCenterDistanceM).toBe(2);
  });

  it('计算 AABB 外不同方向的最近距离', () => {
    expect(evaluateCupApproachSample([2, 3, 3], cupAabb).tcpToCupSurfaceProxyDistanceM).toBe(
      Math.sqrt(2),
    );
    expect(evaluateCupApproachSample([0, -3, 0], cupAabb).tcpToCupSurfaceProxyDistanceM).toBe(1);
    expect(evaluateCupApproachSample([0, 0, 5], cupAabb).tcpToCupSurfaceProxyDistanceM).toBe(2);
  });

  it('TCP 位于 AABB 内时表面代理距离为零', () => {
    expect(evaluateCupApproachSample([0, 0, 0], cupAabb).tcpToCupSurfaceProxyDistanceM).toBe(0);
  });

  it('每次读取都按网格当前 world transform 重新计算 AABB', () => {
    const object = new Group();
    const mesh = new Mesh(new BoxGeometry(2, 4, 6));
    object.add(mesh);
    object.position.set(5, 0, 0);
    object.rotation.y = Math.PI / 2;

    const firstBounds = getObjectWorldAabb(object);
    const firstSample = firstBounds ? evaluateCupApproachSample([0, 0, 0], firstBounds) : null;
    object.position.x = 6;
    const movedBounds = getObjectWorldAabb(object);
    const movedSample = movedBounds ? evaluateCupApproachSample([0, 0, 0], movedBounds) : null;

    firstBounds?.min.forEach((value, axis) => {
      expect(value).toBeCloseTo([2, -2, -1][axis]);
    });
    firstBounds?.max.forEach((value, axis) => {
      expect(value).toBeCloseTo([8, 2, 1][axis]);
    });
    movedBounds?.min.forEach((value, axis) => {
      expect(value).toBeCloseTo([3, -2, -1][axis]);
    });
    movedBounds?.max.forEach((value, axis) => {
      expect(value).toBeCloseTo([9, 2, 1][axis]);
    });
    expect(firstSample?.tcpToCupSurfaceProxyDistanceM).toBeCloseTo(2);
    expect(movedSample?.tcpToCupSurfaceProxyDistanceM).toBeCloseTo(3);
    mesh.geometry.dispose();
  });

  it('将 umi_tcp_link robot-base 坐标转换到 Three scene world 坐标', () => {
    expect(robotPositionToSceneWorld([1, 2, 3])).toEqual([1, 3, -2]);
  });

  it('从逐步距离生成 run-level 变化与总体趋势', () => {
    const summary = summarizeCupApproachDistances([0.3, 0.29, 0.31, 0.27]);

    expect(summary).not.toBeNull();
    expect(summary?.initialDistanceM).toBe(0.3);
    expect(summary?.finalDistanceM).toBe(0.27);
    expect(summary?.minimumDistanceM).toBe(0.27);
    expect(summary?.maximumDistanceM).toBe(0.31);
    expect(summary?.netDistanceChangeM).toBeCloseTo(-0.03);
    expect(summary?.approachPercent).toBeCloseTo(10);
    expect(summary?.trend).toBe('APPROACHING');
    expect(summarizeCupApproachDistances([3, 2, 3, 2, 3])?.trend).toBe('OSCILLATING');
    expect(summarizeCupApproachDistances([1, 1])?.trend).toBe('STATIONARY');
    expect(summarizeCupApproachDistances([1, 2])?.trend).toBe('MOVING_AWAY');
  });
});
