import type { Pose, Quaternion, Vec3 } from '../core/types';

export const IDENTITY_POSE: Pose = {
  position: [0, 0, 0],
  orientation: [0, 0, 0, 1],
};

export const addVectors = (left: Vec3, right: Vec3): Vec3 => [
  left[0] + right[0],
  left[1] + right[1],
  left[2] + right[2],
];

export const subtractVectors = (left: Vec3, right: Vec3): Vec3 => [
  left[0] - right[0],
  left[1] - right[1],
  left[2] - right[2],
];

export const scaleVector = (vector: Vec3, scale: number): Vec3 => [
  vector[0] * scale,
  vector[1] * scale,
  vector[2] * scale,
];

export const dotVectors = (left: Vec3, right: Vec3): number =>
  left[0] * right[0] + left[1] * right[1] + left[2] * right[2];

export const crossVectors = (left: Vec3, right: Vec3): Vec3 => [
  left[1] * right[2] - left[2] * right[1],
  left[2] * right[0] - left[0] * right[2],
  left[0] * right[1] - left[1] * right[0],
];

export const vectorLength = (vector: Vec3): number => Math.sqrt(dotVectors(vector, vector));

export const normalizeVector = (vector: Vec3): Vec3 => {
  const length = vectorLength(vector);
  return length > 0 ? scaleVector(vector, 1 / length) : [0, 0, 0];
};

export const normalizeQuaternion = (quaternion: Quaternion): Quaternion => {
  const length = Math.hypot(...quaternion);
  return length > 0 ? (quaternion.map((value) => value / length) as Quaternion) : [0, 0, 0, 1];
};

export const multiplyQuaternions = (left: Quaternion, right: Quaternion): Quaternion => {
  const [x1, y1, z1, w1] = left;
  const [x2, y2, z2, w2] = right;
  return [
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
  ];
};

export const inverseQuaternion = (quaternion: Quaternion): Quaternion => {
  const normalized = normalizeQuaternion(quaternion);
  return [-normalized[0], -normalized[1], -normalized[2], normalized[3]];
};

export const rotateVector = (quaternion: Quaternion, vector: Vec3): Vec3 => {
  const [x, y, z, w] = normalizeQuaternion(quaternion);
  const [vx, vy, vz] = vector;
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + y * tz - z * ty,
    vy + w * ty + z * tx - x * tz,
    vz + w * tz + x * ty - y * tx,
  ];
};

export const quaternionFromEuler = (roll: number, pitch: number, yaw: number): Quaternion => {
  const halfRoll = roll / 2;
  const halfPitch = pitch / 2;
  const halfYaw = yaw / 2;
  const rollQuaternion: Quaternion = [Math.sin(halfRoll), 0, 0, Math.cos(halfRoll)];
  const pitchQuaternion: Quaternion = [0, Math.sin(halfPitch), 0, Math.cos(halfPitch)];
  const yawQuaternion: Quaternion = [0, 0, Math.sin(halfYaw), Math.cos(halfYaw)];
  return normalizeQuaternion(
    multiplyQuaternions(multiplyQuaternions(yawQuaternion, pitchQuaternion), rollQuaternion),
  );
};

export const quaternionFromAxisAngle = (axis: Vec3, angle: number): Quaternion => {
  const normalizedAxis = normalizeVector(axis);
  const halfAngle = angle / 2;
  const sine = Math.sin(halfAngle);
  return normalizeQuaternion([
    normalizedAxis[0] * sine,
    normalizedAxis[1] * sine,
    normalizedAxis[2] * sine,
    Math.cos(halfAngle),
  ]);
};

export const quaternionToRotationVector = (quaternion: Quaternion): Vec3 => {
  const normalized = normalizeQuaternion(quaternion);
  const shortest =
    normalized[3] < 0 ? (normalized.map((value) => -value) as Quaternion) : normalized;
  const vector: Vec3 = [shortest[0], shortest[1], shortest[2]];
  const sineHalfAngle = vectorLength(vector);
  if (sineHalfAngle < 1e-9) return scaleVector(vector, 2);
  const angle = 2 * Math.atan2(sineHalfAngle, Math.max(-1, Math.min(1, shortest[3])));
  return scaleVector(vector, angle / sineHalfAngle);
};

export const composePoses = (parent: Pose, child: Pose): Pose => ({
  position: addVectors(parent.position, rotateVector(parent.orientation, child.position)),
  orientation: normalizeQuaternion(multiplyQuaternions(parent.orientation, child.orientation)),
});

export const poseErrorRotation = (target: Quaternion, current: Quaternion): Vec3 =>
  quaternionToRotationVector(multiplyQuaternions(target, inverseQuaternion(current)));

export const clonePose = (pose: Pose): Pose => ({
  position: [...pose.position],
  orientation: [...pose.orientation],
});
