// 用真实 URDF visual mesh 的 world AABB 阻止未验证接触。
import { Box3, Vector3 } from 'three';
import type { Object3D } from 'three';
import type { URDFRobot } from 'urdf-loader';
import type { JointDescription, JointValues, RobotDescription } from '../../core/types';
import type { WorldAabb } from './cupApproachEvaluation';
export type { WorldAabb } from './cupApproachEvaluation';

export type PreContactVerdict = 'SAFE_TO_EXECUTE' | 'STOP_BEFORE_UNVALIDATED_CONTACT';

export interface PreContactAabbSample {
  sampleIndex: number;
  geometryAabbs: Record<string, WorldAabb>;
}

export interface PreContactSafetyResult {
  guarded_geometries: string[];
  cup_world_aabb: WorldAabb | null;
  current_geometry_aabbs: Record<string, WorldAabb>;
  current_min_distance_m: number | null;
  target_geometry_aabbs: Record<string, WorldAabb>;
  target_min_distance_m: number | null;
  swept_sample_count: number;
  swept_min_distance_m: number | null;
  swept_closest_geometry: string | null;
  swept_closest_sample: number | null;
  safety_margin_m: number | null;
  safety_margin_basis: string;
  would_enter_unvalidated_contact: boolean;
  verdict: PreContactVerdict;
  evaluation_error?: string;
  current_joint_values?: JointValues;
  target_joint_values?: JointValues;
  interpolation_intervals?: number;
  maximum_geometry_path_bound_m?: number;
  critical_geometry_dimension_m?: number;
}

export interface SweptAabbSafetyInput {
  cupWorldAabb: WorldAabb | null;
  guardedGeometries: string[];
  currentGeometryAabbs: Record<string, WorldAabb>;
  targetGeometryAabbs: Record<string, WorldAabb>;
  samples: PreContactAabbSample[];
  safetyMarginM: number | null;
  safetyMarginBasis: string;
  evaluationError?: string;
}

interface GuardedGeometry {
  name: string;
  objectName: string;
  linkName: string;
  source: 'visual' | 'link';
}

const ROBOT_MOVABLE_JOINT_TYPES = new Set(['revolute', 'continuous', 'prismatic']);
const MAX_RESOLUTION_RATIO = 10;
const MAX_INTERPOLATION_INTERVALS = 50_000;
const AABB_DISTANCE_MARGIN_FACTOR = Math.sqrt(3);

export const GUARDED_END_EFFECTOR_GEOMETRIES = [
  {
    name: 'left_soft_finger',
    objectName: 'left_finger',
    linkName: 'left_jaw_link',
    source: 'visual',
  },
  {
    name: 'right_soft_finger',
    objectName: 'right_finger',
    linkName: 'right_jaw_link',
    source: 'visual',
  },
  {
    name: 'left_holder',
    objectName: 'finger_holder_left',
    linkName: 'left_jaw_link',
    source: 'visual',
  },
  {
    name: 'right_holder',
    objectName: 'finger_holder_right',
    linkName: 'right_jaw_link',
    source: 'visual',
  },
  { name: 'mount', objectName: 'gripper_mount', linkName: 'mount_link', source: 'visual' },
  { name: 'gopro', objectName: 'gopro', linkName: 'mount_link', source: 'visual' },
  {
    name: 'fixed_mount_part_1',
    objectName: 'Part 1',
    linkName: 'fixed_mount_link',
    source: 'visual',
  },
  {
    name: 'fixed_mount_part_2',
    objectName: 'Part 2',
    linkName: 'fixed_mount_link',
    source: 'visual',
  },
  {
    name: 'fixed_mount_part_3',
    objectName: 'Part 3',
    linkName: 'fixed_mount_link',
    source: 'visual',
  },
  {
    name: 'fixed_mount_part_4',
    objectName: 'Part 4',
    linkName: 'fixed_mount_link',
    source: 'visual',
  },
  {
    name: 'fixed_mount_part_5',
    objectName: 'Part 5',
    linkName: 'fixed_mount_link',
    source: 'visual',
  },
  {
    name: 'fixed_mount_part_6',
    objectName: 'Part 6',
    linkName: 'fixed_mount_link',
    source: 'visual',
  },
  { name: 'wrist_3_link', objectName: 'wrist_3_link', linkName: 'wrist_3_link', source: 'link' },
  { name: 'wrist_2_link', objectName: 'wrist_2_link', linkName: 'wrist_2_link', source: 'link' },
] satisfies GuardedGeometry[];

const getGeometryObject = (robot: URDFRobot, geometry: GuardedGeometry): Object3D | null =>
  geometry.source === 'visual'
    ? (robot.visual[geometry.objectName] ?? null)
    : (robot.links[geometry.objectName] ?? null);

const getWorldAabb = (object: Object3D | null): WorldAabb | null => {
  if (!object) return null;
  object.updateWorldMatrix(true, true);
  const bounds = new Box3().setFromObject(object);
  if (bounds.isEmpty()) return null;

  const min = bounds.min.toArray() as WorldAabb['min'];
  const max = bounds.max.toArray() as WorldAabb['max'];
  const size = bounds.getSize(new Vector3()).toArray() as WorldAabb['size'];
  const center = bounds.getCenter(new Vector3()).toArray() as WorldAabb['center'];
  if (![...min, ...max, ...size, ...center].every(Number.isFinite)) return null;
  return { min, max, center, size };
};

export const distanceAabbToAabb = (a: WorldAabb, b: WorldAabb): number => {
  const axisDistances = [0, 1, 2].map((axis) =>
    Math.max(a.min[axis] - b.max[axis], b.min[axis] - a.max[axis], 0),
  );
  return Math.hypot(...axisDistances);
};

export const evaluateSweptAabbSafety = ({
  cupWorldAabb,
  guardedGeometries,
  currentGeometryAabbs,
  targetGeometryAabbs,
  samples,
  safetyMarginM,
  safetyMarginBasis,
  evaluationError,
}: SweptAabbSafetyInput): PreContactSafetyResult => {
  const currentDistances = guardedGeometries.flatMap((name) =>
    currentGeometryAabbs[name] && cupWorldAabb
      ? [distanceAabbToAabb(currentGeometryAabbs[name], cupWorldAabb)]
      : [],
  );
  const targetDistances = guardedGeometries.flatMap((name) =>
    targetGeometryAabbs[name] && cupWorldAabb
      ? [distanceAabbToAabb(targetGeometryAabbs[name], cupWorldAabb)]
      : [],
  );
  let sweptMinDistanceM = Number.POSITIVE_INFINITY;
  let closestGeometry: string | null = null;
  let closestSample: number | null = null;
  let hasCompleteSamples = samples.length > 0;

  samples.forEach((sample) => {
    guardedGeometries.forEach((name) => {
      const geometryAabb = sample.geometryAabbs[name];
      if (!geometryAabb || !cupWorldAabb) {
        hasCompleteSamples = false;
        return;
      }
      const distance = distanceAabbToAabb(geometryAabb, cupWorldAabb);
      if (distance < sweptMinDistanceM) {
        sweptMinDistanceM = distance;
        closestGeometry = name;
        closestSample = sample.sampleIndex;
      }
    });
  });

  const currentMinDistanceM = currentDistances.length ? Math.min(...currentDistances) : null;
  const targetMinDistanceM = targetDistances.length ? Math.min(...targetDistances) : null;
  const sweptDistance = Number.isFinite(sweptMinDistanceM) ? sweptMinDistanceM : null;
  const missingGeometry =
    guardedGeometries.some((name) => !currentGeometryAabbs[name] || !targetGeometryAabbs[name]) ||
    !hasCompleteSamples;
  const wouldEnterUnvalidatedContact =
    Boolean(evaluationError) ||
    missingGeometry ||
    !cupWorldAabb ||
    safetyMarginM === null ||
    sweptDistance === null ||
    sweptDistance <= (safetyMarginM ?? 0);

  return {
    guarded_geometries: [...guardedGeometries],
    cup_world_aabb: cupWorldAabb,
    current_geometry_aabbs: currentGeometryAabbs,
    current_min_distance_m: currentMinDistanceM,
    target_geometry_aabbs: targetGeometryAabbs,
    target_min_distance_m: targetMinDistanceM,
    swept_sample_count: samples.length,
    swept_min_distance_m: sweptDistance,
    swept_closest_geometry: closestGeometry,
    swept_closest_sample: closestSample,
    safety_margin_m: safetyMarginM,
    safety_margin_basis: safetyMarginBasis,
    would_enter_unvalidated_contact: wouldEnterUnvalidatedContact,
    verdict: wouldEnterUnvalidatedContact ? 'STOP_BEFORE_UNVALIDATED_CONTACT' : 'SAFE_TO_EXECUTE',
    ...(evaluationError ? { evaluation_error: evaluationError } : {}),
  };
};

export const cloneRobotAtJointValues = (robot: URDFRobot, jointValues: JointValues): URDFRobot => {
  const clone = robot.clone(true) as URDFRobot;
  clone.setJointValues(jointValues);
  clone.updateMatrixWorld(true);
  return clone;
};

const getAabbs = (robot: URDFRobot): Record<string, WorldAabb> => {
  const entries = GUARDED_END_EFFECTOR_GEOMETRIES.flatMap((geometry) => {
    const bounds = getWorldAabb(getGeometryObject(robot, geometry));
    return bounds ? [[geometry.name, bounds] as const] : [];
  });
  return Object.fromEntries(entries);
};

const getObjectRadiusInLinkFrame = (object: Object3D, link: Object3D): number | null => {
  const worldBounds = new Box3().setFromObject(object);
  if (worldBounds.isEmpty()) return null;
  link.updateWorldMatrix(true, false);
  const worldToLink = link.matrixWorld.clone().invert();
  const corners = [
    new Vector3(worldBounds.min.x, worldBounds.min.y, worldBounds.min.z),
    new Vector3(worldBounds.min.x, worldBounds.min.y, worldBounds.max.z),
    new Vector3(worldBounds.min.x, worldBounds.max.y, worldBounds.min.z),
    new Vector3(worldBounds.min.x, worldBounds.max.y, worldBounds.max.z),
    new Vector3(worldBounds.max.x, worldBounds.min.y, worldBounds.min.z),
    new Vector3(worldBounds.max.x, worldBounds.min.y, worldBounds.max.z),
    new Vector3(worldBounds.max.x, worldBounds.max.y, worldBounds.min.z),
    new Vector3(worldBounds.max.x, worldBounds.max.y, worldBounds.max.z),
  ];
  return Math.max(...corners.map((corner) => corner.applyMatrix4(worldToLink).length()));
};

const getSmallestMeshExtent = (objects: Object3D[]): number | null => {
  const dimensions: number[] = [];
  objects.forEach((object) => {
    object.traverse((child) => {
      const mesh = child as Object3D & {
        geometry?: { boundingBox?: Box3 | null; computeBoundingBox?: () => void };
      };
      if (!mesh.geometry) return;
      if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox?.();
      const bounds = mesh.geometry.boundingBox;
      if (!bounds) return;
      const scale = mesh.getWorldScale(new Vector3());
      const size = bounds.getSize(new Vector3()).multiply(scale);
      dimensions.push(Math.min(Math.abs(size.x), Math.abs(size.y), Math.abs(size.z)));
    });
  });
  const positiveDimensions = dimensions.filter(
    (dimension) => Number.isFinite(dimension) && dimension > 0,
  );
  return positiveDimensions.length ? Math.min(...positiveDimensions) : null;
};

const getJointPath = (
  description: RobotDescription,
  linkName: string,
): JointDescription[] | null => {
  const jointsByChild = new Map(description.joints.map((joint) => [joint.child, joint]));
  const path: JointDescription[] = [];
  let currentLink = linkName;
  while (currentLink !== description.rootLink) {
    const joint = jointsByChild.get(currentLink);
    if (!joint) return null;
    path.unshift(joint);
    currentLink = joint.parent;
  }
  return path;
};

const getPathMovementBound = (
  path: JointDescription[],
  current: JointValues,
  target: JointValues,
  geometryRadius: number,
): number =>
  path.reduce((bound, joint, index) => {
    if (!ROBOT_MOVABLE_JOINT_TYPES.has(joint.type)) return bound;
    const delta = Math.abs(target[joint.name] - current[joint.name]);
    if (joint.type === 'prismatic') return bound + delta;

    const downstreamJoints = path.slice(index + 1);
    const downstreamOffsets = downstreamJoints.reduce(
      (sum, downstream) => sum + Math.hypot(...downstream.origin.position),
      0,
    );
    const downstreamPrismaticTravel = downstreamJoints.reduce((sum, downstream) => {
      if (downstream.type !== 'prismatic') return sum;
      return sum + Math.max(Math.abs(current[downstream.name]), Math.abs(target[downstream.name]));
    }, 0);
    return bound + delta * (geometryRadius + downstreamOffsets + downstreamPrismaticTravel);
  }, 0);

export const createPreContactSafetyFailure = (
  cupWorldAabb: WorldAabb | null,
  evaluationError: string,
  currentGeometryAabbs: Record<string, WorldAabb> = {},
  targetGeometryAabbs: Record<string, WorldAabb> = {},
): PreContactSafetyResult =>
  evaluateSweptAabbSafety({
    cupWorldAabb,
    guardedGeometries: GUARDED_END_EFFECTOR_GEOMETRIES.map(({ name }) => name),
    currentGeometryAabbs,
    targetGeometryAabbs,
    samples: [],
    safetyMarginM: null,
    safetyMarginBasis: '当前真实 visual geometry 或 cup AABB 无法完整读取，采用 fail-closed 保护。',
    evaluationError,
  });

export const evaluateCupApproachPreContactSafety = ({
  robot,
  description,
  currentJointValues,
  targetJointValues,
  cupWorldAabb,
}: {
  robot: URDFRobot | null;
  description: RobotDescription | null;
  currentJointValues: JointValues;
  targetJointValues: JointValues;
  cupWorldAabb: WorldAabb | null;
}): PreContactSafetyResult => {
  if (!robot) return createPreContactSafetyFailure(cupWorldAabb, 'URDF visual robot 未就绪。');
  if (!description) {
    return createPreContactSafetyFailure(cupWorldAabb, 'URDF description 未就绪。');
  }
  if (!cupWorldAabb) return createPreContactSafetyFailure(null, '当前 cup world AABB 不可用。');

  const movableJoints = description.joints.filter((joint) =>
    ROBOT_MOVABLE_JOINT_TYPES.has(joint.type),
  );
  const currentComplete: JointValues = {};
  const targetComplete: JointValues = {};
  for (const joint of movableJoints) {
    const current = currentJointValues[joint.name];
    const target = targetJointValues[joint.name] ?? current;
    if (!Number.isFinite(current) || !Number.isFinite(target)) {
      return createPreContactSafetyFailure(
        cupWorldAabb,
        `关节 ${joint.name} 的当前或目标值不可用。`,
      );
    }
    if (
      (joint.limit?.lower !== undefined && target < joint.limit.lower) ||
      (joint.limit?.upper !== undefined && target > joint.limit.upper)
    ) {
      return createPreContactSafetyFailure(
        cupWorldAabb,
        `关节 ${joint.name} 的目标值超出 URDF 限位。`,
      );
    }
    currentComplete[joint.name] = current;
    targetComplete[joint.name] = target;
  }

  const visualJointMismatch = movableJoints.find((joint) => {
    const visualJointValue = robot.joints[joint.name]?.jointValue[0];
    return (
      !Number.isFinite(visualJointValue) ||
      Math.abs(visualJointValue - currentComplete[joint.name]) > 1e-6
    );
  });
  if (visualJointMismatch) {
    return createPreContactSafetyFailure(
      cupWorldAabb,
      `live URDF visual joint ${visualJointMismatch.name} 与当前 joint snapshot 不一致。`,
    );
  }

  robot.updateMatrixWorld(true);
  const currentGeometryAabbs = getAabbs(robot);
  const currentObjects = GUARDED_END_EFFECTOR_GEOMETRIES.map((geometry) =>
    getGeometryObject(robot, geometry),
  );
  if (
    currentObjects.some((object) => !object) ||
    GUARDED_END_EFFECTOR_GEOMETRIES.some(({ linkName }) => !robot.links[linkName])
  ) {
    return createPreContactSafetyFailure(
      cupWorldAabb,
      '一个或多个受保护 URDF visual geometry 不存在或没有网格。',
      currentGeometryAabbs,
    );
  }

  let targetRobot: URDFRobot;
  try {
    targetRobot = cloneRobotAtJointValues(robot, targetComplete);
  } catch (error) {
    return createPreContactSafetyFailure(
      cupWorldAabb,
      error instanceof Error ? error.message : String(error),
      currentGeometryAabbs,
    );
  }
  const targetJointMismatch = movableJoints.find((joint) => {
    const visualJointValue = targetRobot.joints[joint.name]?.jointValue[0];
    return (
      !Number.isFinite(visualJointValue) ||
      Math.abs(visualJointValue - targetComplete[joint.name]) > 1e-6
    );
  });
  if (targetJointMismatch) {
    return createPreContactSafetyFailure(
      cupWorldAabb,
      `target URDF visual joint ${targetJointMismatch.name} 未应用 target joints。`,
      currentGeometryAabbs,
    );
  }
  const targetGeometryAabbs = getAabbs(targetRobot);
  if (GUARDED_END_EFFECTOR_GEOMETRIES.some(({ name }) => !targetGeometryAabbs[name])) {
    return createPreContactSafetyFailure(
      cupWorldAabb,
      '一个或多个目标 visual geometry 无法计算 AABB。',
      currentGeometryAabbs,
      targetGeometryAabbs,
    );
  }

  const radii = GUARDED_END_EFFECTOR_GEOMETRIES.map((geometry) => {
    const object = getGeometryObject(robot, geometry);
    const link = robot.links[geometry.linkName];
    return object && link ? getObjectRadiusInLinkFrame(object, link) : null;
  });
  const smallestMeshExtent = getSmallestMeshExtent(
    currentObjects.filter((object): object is Object3D => object !== null),
  );
  if (
    radii.some((radius) => radius === null || !Number.isFinite(radius)) ||
    smallestMeshExtent === null
  ) {
    return createPreContactSafetyFailure(
      cupWorldAabb,
      '无法从真实 visual mesh 计算有限的尺寸或运动界。',
      currentGeometryAabbs,
      targetGeometryAabbs,
    );
  }

  const geometryMovementBounds = GUARDED_END_EFFECTOR_GEOMETRIES.map((geometry, index) => {
    const path = getJointPath(description, geometry.linkName);
    if (!path) return null;
    return getPathMovementBound(path, currentComplete, targetComplete, radii[index] as number);
  });
  if (geometryMovementBounds.some((bound) => bound === null || !Number.isFinite(bound))) {
    return createPreContactSafetyFailure(
      cupWorldAabb,
      '无法从 URDF joint path 计算有限的运动界。',
      currentGeometryAabbs,
      targetGeometryAabbs,
    );
  }

  const maximumGeometryPathBoundM = Math.max(...(geometryMovementBounds as number[]));
  const criticalGeometryDimensionM = Math.min(
    smallestMeshExtent,
    ...Object.values(currentGeometryAabbs).flatMap(({ size }) => size),
    ...Object.values(targetGeometryAabbs).flatMap(({ size }) => size),
  );
  if (!(criticalGeometryDimensionM > 0)) {
    return createPreContactSafetyFailure(
      cupWorldAabb,
      '受保护 geometry 的有效尺寸为零。',
      currentGeometryAabbs,
      targetGeometryAabbs,
    );
  }

  const maximumSampleMovementM = criticalGeometryDimensionM / MAX_RESOLUTION_RATIO;
  const interpolationIntervals = Math.max(
    1,
    Math.ceil(maximumGeometryPathBoundM / maximumSampleMovementM),
  );
  if (
    !Number.isSafeInteger(interpolationIntervals) ||
    interpolationIntervals > MAX_INTERPOLATION_INTERVALS
  ) {
    return {
      ...createPreContactSafetyFailure(
        cupWorldAabb,
        `保守安全分辨率需要 ${interpolationIntervals} 个插值区间，超过计算上限 ${MAX_INTERPOLATION_INTERVALS}。`,
        currentGeometryAabbs,
        targetGeometryAabbs,
      ),
      current_joint_values: currentComplete,
      target_joint_values: targetComplete,
      interpolation_intervals: interpolationIntervals,
      maximum_geometry_path_bound_m: maximumGeometryPathBoundM,
      critical_geometry_dimension_m: criticalGeometryDimensionM,
    };
  }
  const sampleRobot = cloneRobotAtJointValues(robot, currentComplete);
  const samples: PreContactAabbSample[] = [];
  for (let sampleIndex = 0; sampleIndex <= interpolationIntervals; sampleIndex += 1) {
    const alpha = sampleIndex / interpolationIntervals;
    const joints = Object.fromEntries(
      movableJoints.map((joint) => [
        joint.name,
        currentComplete[joint.name] +
          alpha * (targetComplete[joint.name] - currentComplete[joint.name]),
      ]),
    );
    sampleRobot.setJointValues(joints);
    const sampleJointMismatch = movableJoints.find((joint) => {
      const visualJointValue = sampleRobot.joints[joint.name]?.jointValue[0];
      return (
        !Number.isFinite(visualJointValue) || Math.abs(visualJointValue - joints[joint.name]) > 1e-6
      );
    });
    if (sampleJointMismatch) {
      return createPreContactSafetyFailure(
        cupWorldAabb,
        `插值样本 ${sampleIndex} 未应用关节 ${sampleJointMismatch.name}。`,
        currentGeometryAabbs,
        targetGeometryAabbs,
      );
    }
    sampleRobot.updateMatrixWorld(true);
    const geometryAabbs = getAabbs(sampleRobot);
    if (GUARDED_END_EFFECTOR_GEOMETRIES.some(({ name }) => !geometryAabbs[name])) {
      return createPreContactSafetyFailure(
        cupWorldAabb,
        `插值样本 ${sampleIndex} 的 visual geometry AABB 不完整。`,
        currentGeometryAabbs,
        targetGeometryAabbs,
      );
    }
    samples.push({ sampleIndex, geometryAabbs });
  }

  const safetyMarginM =
    (AABB_DISTANCE_MARGIN_FACTOR * maximumGeometryPathBoundM) / interpolationIntervals;
  const safetyMarginBasis =
    'AABB 取自真实 URDF visual mesh；插值间隔按 URDF joint-origin 长度、下游 prismatic 行程和 visual mesh 半径形成的路径位移上界，使单间隔位移不超过最小几何尺寸的 1/10；margin 为 sqrt(3) 倍单间隔位移上界。';
  return {
    ...evaluateSweptAabbSafety({
      cupWorldAabb,
      guardedGeometries: GUARDED_END_EFFECTOR_GEOMETRIES.map(({ name }) => name),
      currentGeometryAabbs,
      targetGeometryAabbs,
      samples,
      safetyMarginM,
      safetyMarginBasis,
    }),
    current_joint_values: currentComplete,
    target_joint_values: targetComplete,
    interpolation_intervals: interpolationIntervals,
    maximum_geometry_path_bound_m: maximumGeometryPathBoundM,
    critical_geometry_dimension_m: criticalGeometryDimensionM,
  };
};
