// 只从当前杯子网格与 TCP 位姿计算接近评估指标。
import { Box3, Object3D } from 'three';
import type { Vec3 } from '../../core/types';

export interface WorldAabb {
  min: Vec3;
  max: Vec3;
  center: Vec3;
  size: Vec3;
}

export interface CupApproachSample {
  cupWorldAabb: WorldAabb;
  cupCenterWorld: Vec3;
  tcpWorldPosition: Vec3;
  tcpToCupCenterDistanceM: number;
  tcpToCupSurfaceProxyDistanceM: number;
}

export interface CupApproachSummary {
  initialDistanceM: number;
  finalDistanceM: number;
  minimumDistanceM: number;
  maximumDistanceM: number;
  netDistanceChangeM: number;
  approachPercent: number | null;
  trend: 'APPROACHING' | 'STATIONARY' | 'MOVING_AWAY' | 'OSCILLATING';
}

export type CupEvaluationGeometryRef = { current: Object3D | null };

export const getObjectWorldAabb = (object: Object3D | null): WorldAabb | null => {
  if (!object) return null;
  object.updateWorldMatrix(true, true);
  const bounds = new Box3().setFromObject(object, true);
  if (bounds.isEmpty()) return null;

  const min = bounds.min.toArray() as Vec3;
  const max = bounds.max.toArray() as Vec3;
  const size = bounds.getSize(bounds.min.clone()).toArray() as Vec3;
  const center = bounds.getCenter(bounds.min.clone()).toArray() as Vec3;
  if (![...min, ...max, ...size, ...center].every(Number.isFinite)) return null;

  return { min, max, center, size };
};

export const robotPositionToSceneWorld = ([x, y, z]: Vec3): Vec3 => [x, z, -y];

export const evaluateCupApproachSample = (
  tcpWorldPosition: Vec3,
  cupWorldAabb: WorldAabb,
): CupApproachSample => {
  const axisDistances = tcpWorldPosition.map((value, axis) =>
    Math.max(cupWorldAabb.min[axis] - value, 0, value - cupWorldAabb.max[axis]),
  );

  return {
    cupWorldAabb,
    cupCenterWorld: cupWorldAabb.center,
    tcpWorldPosition,
    tcpToCupCenterDistanceM: Math.hypot(
      tcpWorldPosition[0] - cupWorldAabb.center[0],
      tcpWorldPosition[1] - cupWorldAabb.center[1],
      tcpWorldPosition[2] - cupWorldAabb.center[2],
    ),
    tcpToCupSurfaceProxyDistanceM: Math.hypot(...axisDistances),
  };
};

export const summarizeCupApproachDistances = (distancesM: number[]): CupApproachSummary | null => {
  if (distancesM.length < 2 || !distancesM.every(Number.isFinite)) return null;

  const initialDistanceM = distancesM[0];
  const finalDistanceM = distancesM[distancesM.length - 1];
  const netDistanceChangeM = finalDistanceM - initialDistanceM;
  const directions = distancesM
    .slice(1)
    .map((distance, index) => Math.sign(distance - distancesM[index]))
    .filter((direction) => direction !== 0);
  const reversals = directions
    .slice(1)
    .filter((direction, index) => direction !== directions[index]);
  const trend =
    netDistanceChangeM < 0
      ? 'APPROACHING'
      : netDistanceChangeM > 0
        ? 'MOVING_AWAY'
        : reversals.length >= 1
          ? 'OSCILLATING'
          : 'STATIONARY';

  return {
    initialDistanceM,
    finalDistanceM,
    minimumDistanceM: Math.min(...distancesM),
    maximumDistanceM: Math.max(...distancesM),
    netDistanceChangeM,
    approachPercent:
      initialDistanceM === 0
        ? null
        : ((initialDistanceM - finalDistanceM) / initialDistanceM) * 100,
    trend,
  };
};
