import { useCallback, useEffect, useMemo, useRef } from 'react';
import { useAfterPhysicsStep, useBeforePhysicsStep, useRapier } from '@react-three/rapier';
import { Quaternion, Vector3 } from 'three';
import { Kinematics } from '../core/Kinematics';
import type {
  CollisionDescription,
  ControlMode,
  JointCommand,
  JointDescription,
  JointValues,
  Pose,
  RobotDescription,
} from '../core/types';
import { composePoses } from '../utils/math';
import { resolvePackageUri } from '../utils/resolvePackageUri.mjs';
import { getRapierMassProperties } from '../utils/urdfInertialProperties';
import { applyJointCommand } from './JointActuator';
import { publishDebugEvent } from '../app/debugBus';

interface RobotBodyProps {
  description: RobotDescription;
  packageMappings: Readonly<Record<string, string>>;
  diagnostics?: {
    gravity: [number, number, number];
    solverIterations?: number;
    disableCollisions: boolean;
    lockUpstream: boolean;
    traceMotorWrites?: boolean;
  };
  positionExecution: 'joint_motors' | 'kinematic_fk';
  jointValues: JointValues;
  commands: Record<string, JointCommand>;
  mode: ControlMode;
  onJointState: (values: JointValues) => void;
  onWrist3DebugState: (state: Wrist3PhysicsDebugState | null) => void;
  onReady: (ready: boolean) => void;
}

type RapierContext = ReturnType<typeof useRapier>;
type RapierWorld = RapierContext['world'];
type RapierModule = RapierContext['rapier'];
type RapierBody = ReturnType<RapierWorld['createRigidBody']>;
type RapierJoint = ReturnType<RapierWorld['createImpulseJoint']>;
type JointMotor = RapierJoint & {
  configureMotorPosition?: (position: number, stiffness: number, damping: number) => void;
  configureMotorVelocity?: (velocity: number, damping: number) => void;
  setLimits?: (minimum: number, maximum: number) => void;
};
type VectorObject = { x: number; y: number; z: number };
type RotationObject = { x: number; y: number; z: number; w: number };

interface SimulatedJoint {
  description: JointDescription;
  joint: JointMotor;
  parent: RapierBody;
  child: RapierBody;
  axisLocal: Vector3;
  childAnchor: Vector3;
  parentAnchor: Vector3;
  initialRelativeRotation: Quaternion;
  initialJointFrameRelativeRotation: Quaternion | null;
  initialPosition: number;
}

export interface Wrist3PhysicsDebugState {
  readback: number | null;
  bodyAngle: number | null;
  childAngularVelocity: [number, number, number];
  swingErrorRad: number | null;
  anchorErrorM: number | null;
  jointHandle: number;
  parentBodyHandle: number;
  childBodyHandle: number;
}

interface BodyFrame {
  body: RapierBody;
  robotToBodyRotation: Quaternion;
  isDynamicLink: boolean;
}

interface Phase2B23Sample {
  time_ms: number;
  joint_values: Record<string, number>;
  joints: Record<string, Record<string, unknown>>;
  links: Record<string, Record<string, unknown>>;
}

interface Phase2B24MotorWrite {
  physics_step: number | null;
  writer_id: string;
  joint_name: string;
  kind: 'position' | 'velocity';
  absolute_command: number | null;
  configure_motor_position_target: number | null;
  configure_motor_velocity_target: number | null;
  stiffness: number | null;
  damping: number;
  write_index_in_step: number;
  last_write: boolean;
}

interface Phase2B24Sample {
  checkpoint_s: number;
  elapsed_s: number;
  physics_step: number;
  writer_id: string | null;
  q_command: number | null;
  q_configureMotorPosition: number | null;
  q_readback: number;
  q_body: number | null;
  relative_joint_frame_quaternion: number[] | null;
  relative_angular_residual_rad: number | null;
  parent_body_world_quaternion: number[];
  child_body_world_quaternion: number[];
  frameX1: number[] | null;
  frameX2: number[] | null;
  writes_this_step: Phase2B24MotorWrite[];
}

interface Phase2B24Trace {
  setup: Record<string, unknown>;
  physics_step: number;
  wrist3_initial_position: number;
  experiment_start_step: number | null;
  experiment_start_time_ms: number | null;
  checkpoints_s: number[];
  next_checkpoint_index: number;
  motor_writes: Phase2B24MotorWrite[];
  samples: Phase2B24Sample[];
}

declare global {
  interface Window {
    __ROBOT_SIM_PHASE2B23__?: {
      setup: Record<string, unknown>;
      samples: Phase2B23Sample[];
    };
    __ROBOT_SIM_PHASE2B24__?: Phase2B24Trace;
  }
}

const ROS_TO_SCENE = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), -Math.PI / 2);
const DEFAULT_JOINT_STIFFNESS = 100;
const DEFAULT_JOINT_DAMPING = 18;
const MAX_HULL_POINTS = 512;
const MAX_PHASE2B23_SAMPLES = 4096;
const PHASE2B24_CHECKPOINTS_S = [0, 0.25, 0.5, 1, 2, 3, 4, 6, 10, 20];
const meshPointCache = new Map<string, Promise<Float32Array>>();

const vectorObject = (vector: Vector3): VectorObject => ({ x: vector.x, y: vector.y, z: vector.z });
const vectorArray = (vector?: VectorObject): number[] | null =>
  vector ? [vector.x, vector.y, vector.z] : null;
const rotationArray = (rotation?: RotationObject): number[] | null =>
  rotation ? [rotation.x, rotation.y, rotation.z, rotation.w] : null;

const recordPhase2B24MotorWrite = (
  write: Omit<Phase2B24MotorWrite, 'write_index_in_step' | 'last_write'>,
  initialPosition: number,
): void => {
  if (typeof window === 'undefined' || write.joint_name !== 'wrist_3_joint') return;
  const trace = window.__ROBOT_SIM_PHASE2B24__;
  if (!trace) return;

  let writeIndex = 1;
  for (let index = trace.motor_writes.length - 1; index >= 0; index -= 1) {
    const previous = trace.motor_writes[index];
    if (previous.physics_step !== write.physics_step) break;
    previous.last_write = false;
    writeIndex += 1;
  }
  trace.motor_writes.push({ ...write, write_index_in_step: writeIndex, last_write: true });

  if (
    write.kind === 'position' &&
    write.physics_step !== null &&
    trace.experiment_start_time_ms === null &&
    write.absolute_command !== null &&
    Math.abs(write.absolute_command - initialPosition) >= 0.05
  ) {
    trace.experiment_start_step = write.physics_step;
    trace.experiment_start_time_ms = performance.now();
  }
};

const writePositionMotor = (
  joint: JointMotor,
  jointName: string,
  absoluteCommand: number,
  initialPosition: number,
  stiffness: number,
  damping: number,
  writerId: string,
  physicsStep: number | null | undefined,
  onBeforeSet?: (relativeTarget: number) => void,
): void => {
  if (!joint.configureMotorPosition) return;
  const relativeTarget = absoluteCommand - initialPosition;
  const currentStep =
    physicsStep === null
      ? null
      : (physicsStep ??
        (typeof window !== 'undefined'
          ? (window.__ROBOT_SIM_PHASE2B24__?.physics_step ?? null)
          : null));
  recordPhase2B24MotorWrite(
    {
      physics_step: currentStep,
      writer_id: writerId,
      joint_name: jointName,
      kind: 'position',
      absolute_command: absoluteCommand,
      configure_motor_position_target: relativeTarget,
      configure_motor_velocity_target: null,
      stiffness,
      damping,
    },
    initialPosition,
  );
  onBeforeSet?.(relativeTarget);
  joint.configureMotorPosition(relativeTarget, stiffness, damping);
};

const writeVelocityMotor = (
  joint: JointMotor,
  jointName: string,
  velocity: number,
  damping: number,
  writerId: string,
): void => {
  if (!joint.configureMotorVelocity) return;
  const physicsStep =
    typeof window !== 'undefined' ? (window.__ROBOT_SIM_PHASE2B24__?.physics_step ?? null) : null;
  recordPhase2B24MotorWrite(
    {
      physics_step: physicsStep,
      writer_id: writerId,
      joint_name: jointName,
      kind: 'velocity',
      absolute_command: null,
      configure_motor_position_target: null,
      configure_motor_velocity_target: velocity,
      stiffness: null,
      damping,
    },
    0,
  );
  joint.configureMotorVelocity(velocity, damping);
};

const rotationObject = (rotation: Quaternion): RotationObject => ({
  x: rotation.x,
  y: rotation.y,
  z: rotation.z,
  w: rotation.w,
});

const worldPose = (pose: Pose): { position: Vector3; rotation: Quaternion } => ({
  position: new Vector3(...pose.position).applyQuaternion(ROS_TO_SCENE),
  rotation: ROS_TO_SCENE.clone().multiply(new Quaternion(...pose.orientation)),
});

const loadMeshPoints = async (
  uri: string,
  packageMappings: Readonly<Record<string, string>>,
): Promise<Float32Array> => {
  const url = resolvePackageUri(uri, packageMappings);
  const cached = meshPointCache.get(url);
  if (cached) return cached;

  const loading = (async () => {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`碰撞网格加载失败：${url} (${response.status})`);
    const { STLLoader } = await import('three/examples/jsm/loaders/STLLoader.js');
    const geometry = new STLLoader().parse(await response.arrayBuffer());
    const points = new Float32Array(geometry.attributes.position.array);
    geometry.dispose();
    return points;
  })();
  meshPointCache.set(url, loading);
  return loading;
};

const sampleHullPoints = (source: Float32Array, scale: Vector3): Float32Array => {
  const vertexCount = source.length / 3;
  const sampledIndices = new Set<number>();
  const sampleCount = Math.min(vertexCount, MAX_HULL_POINTS);
  const minimumIndices = [0, 0, 0];
  const maximumIndices = [0, 0, 0];
  const minimumValues = [
    Number.POSITIVE_INFINITY,
    Number.POSITIVE_INFINITY,
    Number.POSITIVE_INFINITY,
  ];
  const maximumValues = [
    Number.NEGATIVE_INFINITY,
    Number.NEGATIVE_INFINITY,
    Number.NEGATIVE_INFINITY,
  ];

  for (let vertex = 0; vertex < vertexCount; vertex += 1) {
    const offset = vertex * 3;
    for (let axis = 0; axis < 3; axis += 1) {
      const value = source[offset + axis] * scale.getComponent(axis);
      if (value < minimumValues[axis]) {
        minimumValues[axis] = value;
        minimumIndices[axis] = offset;
      }
      if (value > maximumValues[axis]) {
        maximumValues[axis] = value;
        maximumIndices[axis] = offset;
      }
    }
  }

  for (let sample = 0; sample < sampleCount; sample += 1) {
    sampledIndices.add(Math.floor((sample * vertexCount) / sampleCount) * 3);
  }
  minimumIndices.forEach((index) => sampledIndices.add(index));
  maximumIndices.forEach((index) => sampledIndices.add(index));

  const points: number[] = [];
  const uniquePoints = new Set<string>();
  sampledIndices.forEach((index) => {
    const x = source[index] * scale.x;
    const y = source[index + 1] * scale.y;
    const z = source[index + 2] * scale.z;
    const key = `${Math.round(x * 10000)}:${Math.round(y * 10000)}:${Math.round(z * 10000)}`;
    if (uniquePoints.has(key)) return;
    uniquePoints.add(key);
    points.push(x, y, z);
  });

  return new Float32Array(points);
};

const geometryDescriptor = async (
  rapier: RapierModule,
  collision: CollisionDescription,
  linkToBody: Quaternion,
  packageMappings: Readonly<Record<string, string>>,
) => {
  const bodyFromLink = linkToBody.clone().invert();
  const originRotation = new Quaternion(...collision.origin.orientation);
  const bodyPosition = new Vector3(...collision.origin.position).applyQuaternion(bodyFromLink);
  const bodyRotation = bodyFromLink.clone().multiply(originRotation);
  const geometry = collision.geometry;
  let descriptor: InstanceType<typeof rapier.ColliderDesc> | null = null;

  if (geometry.type === 'mesh') {
    const source = await loadMeshPoints(geometry.filename, packageMappings);
    const vertices = sampleHullPoints(source, new Vector3(...geometry.scale));
    descriptor = rapier.ColliderDesc.convexHull(vertices);
  } else if (geometry.type === 'box') {
    descriptor = rapier.ColliderDesc.cuboid(
      ...(geometry.size.map((size) => size / 2) as [number, number, number]),
    );
  } else if (geometry.type === 'cylinder') {
    const axisRotation = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 2);
    bodyRotation.multiply(axisRotation);
    descriptor = rapier.ColliderDesc.cylinder(geometry.length / 2, geometry.radius);
  } else if (geometry.type === 'sphere') {
    descriptor = rapier.ColliderDesc.ball(geometry.radius);
  }

  if (!descriptor) return null;
  descriptor
    .setTranslation(bodyPosition.x, bodyPosition.y, bodyPosition.z)
    .setRotation(rotationObject(bodyRotation))
    .setFriction(0.7)
    .setRestitution(0)
    .setDensity(0);
  return descriptor;
};

const createJoint = (
  rapier: RapierModule,
  world: RapierWorld,
  joint: JointDescription,
  parentBody: RapierBody,
  childBody: RapierBody,
  parentLinkPose: Pose,
  childLinkPose: Pose,
  parentBodyRotation: Quaternion,
  childBodyRotation: Quaternion,
  initialPosition: number,
): SimulatedJoint | null => {
  if (joint.type === 'floating' || joint.type === 'planar') return null;

  const jointPose = composePoses(parentLinkPose, joint.origin);
  const worldAnchor = worldPose({
    position: jointPose.position,
    orientation: jointPose.orientation,
  }).position;
  const parentPosition = worldPose(parentLinkPose).position;
  const childPosition = worldPose(childLinkPose).position;
  const parentAnchor = worldAnchor
    .clone()
    .sub(parentPosition)
    .applyQuaternion(parentBodyRotation.clone().invert());
  const childAnchor = worldAnchor
    .clone()
    .sub(childPosition)
    .applyQuaternion(childBodyRotation.clone().invert());
  const worldJointRotation = worldPose({
    position: [0, 0, 0],
    orientation: jointPose.orientation,
  }).rotation;
  const frame1 = parentBodyRotation.clone().invert().multiply(worldJointRotation);
  const frame2 = childBodyRotation.clone().invert().multiply(worldJointRotation);

  if (joint.type === 'fixed') {
    const handle = world.createImpulseJoint(
      rapier.JointData.fixed(
        vectorObject(parentAnchor),
        rotationObject(frame1),
        vectorObject(childAnchor),
        rotationObject(frame2),
      ),
      parentBody,
      childBody,
      true,
    );
    handle.setContactsEnabled(false);
    return null;
  }

  const jointAxis = new Vector3(...joint.axis).normalize();
  const axisWorld = jointAxis.clone().applyQuaternion(worldJointRotation).normalize();
  const axisLocal = axisWorld
    .clone()
    .applyQuaternion(parentBodyRotation.clone().invert())
    .normalize();
  const data =
    joint.type === 'prismatic'
      ? rapier.JointData.prismatic(
          vectorObject(parentAnchor),
          vectorObject(childAnchor),
          vectorObject(axisLocal),
        )
      : rapier.JointData.revolute(
          vectorObject(parentAnchor),
          vectorObject(childAnchor),
          vectorObject(axisLocal),
        );
  const handle = world.createImpulseJoint(data, parentBody, childBody, true) as JointMotor;
  handle.setContactsEnabled(false);
  if (joint.limit?.lower !== undefined && joint.limit.upper !== undefined) {
    handle.setLimits?.(joint.limit.lower, joint.limit.upper);
  }
  writePositionMotor(
    handle,
    joint.name,
    initialPosition,
    initialPosition,
    DEFAULT_JOINT_STIFFNESS,
    DEFAULT_JOINT_DAMPING,
    'RobotBody.createJoint.initialPosition',
    null,
  );

  const initialRelativeRotation = parentBodyRotation.clone().invert().multiply(childBodyRotation);
  const jointFrame = handle as JointMotor & {
    frameX1?: () => RotationObject;
    frameX2?: () => RotationObject;
  };
  const frameX1 = jointFrame.frameX1?.();
  const frameX2 = jointFrame.frameX2?.();
  const initialJointFrameRelativeRotation =
    frameX1 && frameX2
      ? parentBodyRotation
          .clone()
          .multiply(new Quaternion(frameX1.x, frameX1.y, frameX1.z, frameX1.w))
          .invert()
          .multiply(
            childBodyRotation
              .clone()
              .multiply(new Quaternion(frameX2.x, frameX2.y, frameX2.z, frameX2.w)),
          )
          .normalize()
      : null;
  return {
    description: joint,
    joint: handle,
    parent: parentBody,
    child: childBody,
    axisLocal,
    childAnchor,
    parentAnchor,
    initialRelativeRotation,
    initialJointFrameRelativeRotation,
    initialPosition,
  };
};

const setKinematicBodies = (
  description: RobotDescription,
  values: JointValues,
  frames: Map<string, BodyFrame>,
): void => {
  const poses = new Kinematics(description).forwardKinematicsAll(values);
  const jointsByChild = new Map(description.joints.map((joint) => [joint.child, joint]));

  description.links.forEach((link) => {
    const frame = frames.get(link.name);
    if (!frame || !frame.body.isKinematic()) return;
    const pose = poses[link.name];
    if (!pose) return;
    const converted = worldPose(pose);
    converted.rotation.multiply(frame.robotToBodyRotation);
    frame.body.setNextKinematicTranslation(vectorObject(converted.position));
    frame.body.setNextKinematicRotation(rotationObject(converted.rotation));
    const joint = jointsByChild.get(link.name);
    if (joint?.mimic) return;
  });
};

const setRobotPositionControl = (
  rapier: RapierModule,
  frames: Map<string, BodyFrame>,
  enabled: boolean,
  lockedLinkNames: ReadonlySet<string>,
): void => {
  frames.forEach((frame, linkName) => {
    if (!frame.isDynamicLink) return;
    if (lockedLinkNames.has(linkName)) {
      if (!frame.body.isKinematic()) {
        frame.body.setBodyType(rapier.RigidBodyType.KinematicPositionBased, true);
      }
      return;
    }
    if (enabled && frame.body.isDynamic()) {
      frame.body.setBodyType(rapier.RigidBodyType.KinematicPositionBased, true);
    } else if (!enabled && !frame.body.isDynamic()) {
      frame.body.setBodyType(rapier.RigidBodyType.Dynamic, true);
    }
  });
};

const readJointState = (joints: SimulatedJoint[]): JointValues => {
  const values: JointValues = {};
  joints.forEach((entry) => {
    const parentRotation = entry.parent.rotation();
    const childRotation = entry.child.rotation();
    if (entry.description.type === 'prismatic') {
      const parentPosition = entry.parent.translation();
      const childPosition = entry.child.translation();
      const parentAnchor = new Vector3(parentPosition.x, parentPosition.y, parentPosition.z).add(
        entry.parentAnchor
          .clone()
          .applyQuaternion(
            new Quaternion(parentRotation.x, parentRotation.y, parentRotation.z, parentRotation.w),
          ),
      );
      const childAnchor = new Vector3(childPosition.x, childPosition.y, childPosition.z).add(
        entry.childAnchor
          .clone()
          .applyQuaternion(
            new Quaternion(childRotation.x, childRotation.y, childRotation.z, childRotation.w),
          ),
      );
      const axisWorld = entry.axisLocal
        .clone()
        .applyQuaternion(
          new Quaternion(parentRotation.x, parentRotation.y, parentRotation.z, parentRotation.w),
        );
      values[entry.description.name] =
        entry.initialPosition + childAnchor.sub(parentAnchor).dot(axisWorld);
      return;
    }

    const relative = new Quaternion(
      parentRotation.x,
      parentRotation.y,
      parentRotation.z,
      parentRotation.w,
    )
      .invert()
      .multiply(new Quaternion(childRotation.x, childRotation.y, childRotation.z, childRotation.w));
    const delta = entry.initialRelativeRotation.clone().invert().multiply(relative).normalize();
    const projected =
      delta.x * entry.axisLocal.x + delta.y * entry.axisLocal.y + delta.z * entry.axisLocal.z;
    values[entry.description.name] = entry.initialPosition + 2 * Math.atan2(projected, delta.w);
  });
  return values;
};

const axisInWorld = (joint: SimulatedJoint): Vector3 => {
  const rotation = joint.parent.rotation();
  return joint.axisLocal
    .clone()
    .applyQuaternion(new Quaternion(rotation.x, rotation.y, rotation.z, rotation.w))
    .normalize();
};

const getRapierJointFrameMotion = (entry: SimulatedJoint) => {
  const joint = entry.joint as JointMotor & {
    frameX1?: () => RotationObject;
    frameX2?: () => RotationObject;
    rawAxis?: () => number;
  };
  const frameX1 = joint.frameX1?.();
  const frameX2 = joint.frameX2?.();
  if (!frameX1 || !frameX2 || !entry.initialJointFrameRelativeRotation) return null;

  const parentBodyRotationValue = entry.parent.rotation();
  const childBodyRotationValue = entry.child.rotation();
  const parentBodyRotation = new Quaternion(
    parentBodyRotationValue.x,
    parentBodyRotationValue.y,
    parentBodyRotationValue.z,
    parentBodyRotationValue.w,
  );
  const childBodyRotation = new Quaternion(
    childBodyRotationValue.x,
    childBodyRotationValue.y,
    childBodyRotationValue.z,
    childBodyRotationValue.w,
  );
  const parentJointFrameWorld = parentBodyRotation.multiply(
    new Quaternion(frameX1.x, frameX1.y, frameX1.z, frameX1.w),
  );
  const childJointFrameWorld = childBodyRotation.multiply(
    new Quaternion(frameX2.x, frameX2.y, frameX2.z, frameX2.w),
  );
  const relativeJointFrame = parentJointFrameWorld
    .clone()
    .invert()
    .multiply(childJointFrameWorld)
    .normalize();
  const motion = entry.initialJointFrameRelativeRotation
    .clone()
    .invert()
    .multiply(relativeJointFrame)
    .normalize();
  const offAxisVectorMagnitude = Math.hypot(motion.y, motion.z);

  return {
    q_body: entry.initialPosition + 2 * Math.atan2(motion.x, motion.w),
    relative_joint_frame_quaternion: [
      relativeJointFrame.x,
      relativeJointFrame.y,
      relativeJointFrame.z,
      relativeJointFrame.w,
    ],
    relative_angular_residual_rad:
      2 * Math.atan2(offAxisVectorMagnitude, Math.hypot(motion.x, motion.w)),
    parent_body_world_quaternion: [
      parentBodyRotation.x,
      parentBodyRotation.y,
      parentBodyRotation.z,
      parentBodyRotation.w,
    ],
    child_body_world_quaternion: [
      childBodyRotation.x,
      childBodyRotation.y,
      childBodyRotation.z,
      childBodyRotation.w,
    ],
    frameX1: [frameX1.x, frameX1.y, frameX1.z, frameX1.w],
    frameX2: [frameX2.x, frameX2.y, frameX2.z, frameX2.w],
    rawAxis: joint.rawAxis?.() ?? null,
    delta_joint_frame_quaternion: [motion.x, motion.y, motion.z, motion.w],
  };
};

const getWrist3PhysicsDebugState = (
  entry: SimulatedJoint,
  measured: JointValues,
): Wrist3PhysicsDebugState => {
  const bodyMotion = getRapierJointFrameMotion(entry);
  const joint = entry.joint as JointMotor & {
    anchor1?: () => VectorObject;
    anchor2?: () => VectorObject;
  };
  const parentAnchor = joint.anchor1?.();
  const childAnchor = joint.anchor2?.();
  let swingErrorRad: number | null = null;
  let anchorErrorM: number | null = null;

  if (bodyMotion) {
    const parentBodyRotation = new Quaternion().fromArray(
      bodyMotion.parent_body_world_quaternion,
    );
    const childBodyRotation = new Quaternion().fromArray(
      bodyMotion.child_body_world_quaternion,
    );
    const parentJointFrame = parentBodyRotation
      .clone()
      .multiply(new Quaternion().fromArray(bodyMotion.frameX1));
    const childJointFrame = childBodyRotation
      .clone()
      .multiply(new Quaternion().fromArray(bodyMotion.frameX2));
    const parentWorldAxis = new Vector3(1, 0, 0)
      .applyQuaternion(parentJointFrame)
      .normalize();
    const childWorldAxis = new Vector3(1, 0, 0)
      .applyQuaternion(childJointFrame)
      .normalize();
    swingErrorRad = Math.acos(
      Math.max(-1, Math.min(1, parentWorldAxis.dot(childWorldAxis))),
    );

    if (parentAnchor && childAnchor) {
      const parentPosition = entry.parent.translation();
      const childPosition = entry.child.translation();
      const parentAnchorWorld = new Vector3(parentPosition.x, parentPosition.y, parentPosition.z)
        .add(
          new Vector3(parentAnchor.x, parentAnchor.y, parentAnchor.z).applyQuaternion(
            parentBodyRotation,
          ),
        );
      const childAnchorWorld = new Vector3(childPosition.x, childPosition.y, childPosition.z).add(
        new Vector3(childAnchor.x, childAnchor.y, childAnchor.z).applyQuaternion(childBodyRotation),
      );
      anchorErrorM = parentAnchorWorld.distanceTo(childAnchorWorld);
    }
  }

  const childAngularVelocity = entry.child.angvel();
  return {
    readback: measured.wrist_3_joint ?? null,
    bodyAngle: bodyMotion?.q_body ?? null,
    childAngularVelocity: [
      childAngularVelocity.x,
      childAngularVelocity.y,
      childAngularVelocity.z,
    ],
    swingErrorRad,
    anchorErrorM,
    jointHandle: entry.joint.handle,
    parentBodyHandle: entry.parent.handle,
    childBodyHandle: entry.child.handle,
  };
};

const appendPhase2B24Checkpoint = (
  trace: Phase2B24Trace,
  joints: SimulatedJoint[],
  checkpointS: number,
  elapsedS: number,
): void => {
  const measured = readJointState(joints);
  const wrist3 = joints.find((entry) => entry.description.name === 'wrist_3_joint');
  const bodyMotion = wrist3 ? getRapierJointFrameMotion(wrist3) : null;
  const writesThisStep = trace.motor_writes.filter(
    (write) => write.physics_step === trace.physics_step && write.joint_name === 'wrist_3_joint',
  );
  const lastWrite = writesThisStep[writesThisStep.length - 1];
  trace.samples.push({
    checkpoint_s: checkpointS,
    elapsed_s: elapsedS,
    physics_step: trace.physics_step,
    writer_id: lastWrite?.writer_id ?? null,
    q_command: lastWrite?.kind === 'position' ? lastWrite.absolute_command : null,
    q_configureMotorPosition:
      lastWrite?.kind === 'position' ? lastWrite.configure_motor_position_target : null,
    q_readback: measured.wrist_3_joint ?? Number.NaN,
    q_body: bodyMotion?.q_body ?? null,
    relative_joint_frame_quaternion: bodyMotion?.relative_joint_frame_quaternion ?? null,
    relative_angular_residual_rad: bodyMotion?.relative_angular_residual_rad ?? null,
    parent_body_world_quaternion: bodyMotion?.parent_body_world_quaternion ?? [],
    child_body_world_quaternion: bodyMotion?.child_body_world_quaternion ?? [],
    frameX1: bodyMotion?.frameX1 ?? null,
    frameX2: bodyMotion?.frameX2 ?? null,
    writes_this_step: writesThisStep.map((write) => ({ ...write })),
  });
  trace.next_checkpoint_index += 1;
};

export const RobotBody = ({
  description,
  packageMappings,
  diagnostics,
  positionExecution,
  jointValues,
  commands,
  mode,
  onJointState,
  onWrist3DebugState,
  onReady,
}: RobotBodyProps) => {
  const { world, rapier } = useRapier();
  const lockedLinkNames = useMemo(
    () =>
      new Set(
        diagnostics?.lockUpstream
          ? description.joints
              .filter((joint) => joint.type !== 'fixed' && joint.name !== 'wrist_3_joint')
              .map((joint) => joint.child)
          : [],
      ),
    [description, diagnostics?.lockUpstream],
  );
  const bodies = useRef(new Map<string, BodyFrame>());
  const joints = useRef<SimulatedJoint[]>([]);
  const latestValues = useRef(jointValues);
  const latestCommands = useRef(commands);
  const latestMode = useRef(mode);
  const latestStateCallback = useRef(onJointState);
  const heldGripperPositions = useRef(new Map<string, number>());
  const heldPositionMode = useRef(mode);
  const lastStatePublish = useRef(0);
  const lastPhysicsSignature = useRef('');
  const lastCommandSignature = useRef('');

  latestValues.current = jointValues;
  latestCommands.current = commands;
  latestMode.current = mode;
  latestStateCallback.current = onJointState;

  useEffect(() => {
    let cancelled = false;
    const createdBodies: RapierBody[] = [];
    const bodyFrames = new Map<string, BodyFrame>();
    const createdJoints: SimulatedJoint[] = [];
    heldGripperPositions.current.clear();
    heldPositionMode.current = latestMode.current;
    lastPhysicsSignature.current = '';
    let colliderCount = 0;
    const kinematics = new Kinematics(description);
    const initialValues = { ...latestValues.current };
    const currentPoses = kinematics.forwardKinematicsAll(initialValues);
    if (diagnostics?.traceMotorWrites && typeof window !== 'undefined') {
      window.__ROBOT_SIM_PHASE2B24__ = {
        setup: {
          model: description.name,
          initial_joint_values: initialValues,
          gravity: diagnostics.gravity,
          collision_disabled: diagnostics.disableCollisions,
          position_execution: positionExecution,
          stiffness: DEFAULT_JOINT_STIFFNESS,
          damping: DEFAULT_JOINT_DAMPING,
        },
        physics_step: 0,
        wrist3_initial_position: initialValues.wrist_3_joint ?? 0,
        experiment_start_step: null,
        experiment_start_time_ms: null,
        checkpoints_s: PHASE2B24_CHECKPOINTS_S,
        next_checkpoint_index: 0,
        motor_writes: [],
        samples: [],
      };
    }

    const initialize = async () => {
      try {
        const rootLink = description.links.find((link) => link.name === description.rootLink);
        if (!rootLink) throw new Error(`URDF 根连杆不存在：${description.rootLink}`);
        const rootPose = currentPoses[description.rootLink];
        const rootTransform = worldPose(rootPose);
        publishDebugEvent('physics:progress', { stage: 'root-body' });
        const rootBody = world.createRigidBody(
          rapier.RigidBodyDesc.fixed()
            .setTranslation(
              rootTransform.position.x,
              rootTransform.position.y,
              rootTransform.position.z,
            )
            .setRotation(rotationObject(rootTransform.rotation)),
        );
        createdBodies.push(rootBody);
        bodyFrames.set(description.rootLink, {
          body: rootBody,
          robotToBodyRotation: new Quaternion(),
          isDynamicLink: false,
        });
        if (!diagnostics?.disableCollisions) {
          await attachCollisions(
            rapier,
            world,
            rootBody,
            rootLink,
            new Quaternion(),
            packageMappings,
          );
        }
        publishDebugEvent('physics:progress', { stage: 'root-colliders' });

        const jointsByParent = new Map<string, JointDescription[]>();
        description.joints.forEach((joint) => {
          const siblings = jointsByParent.get(joint.parent) ?? [];
          siblings.push(joint);
          jointsByParent.set(joint.parent, siblings);
        });
        const queue = [description.rootLink];

        while (queue.length && !cancelled) {
          const parentName = queue.shift() as string;
          const parentFrame = bodyFrames.get(parentName);
          const parentPose = currentPoses[parentName];
          if (!parentFrame || !parentPose) throw new Error(`URDF 连杆不可达：${parentName}`);

          for (const joint of jointsByParent.get(parentName) ?? []) {
            if (cancelled) break;
            publishDebugEvent('physics:progress', { stage: 'child-body', joint: joint.name });
            const link = description.links.find((item) => item.name === joint.child);
            const childPose = currentPoses[joint.child];
            if (!link || !childPose) throw new Error(`URDF 子连杆不存在：${joint.child}`);
            const parentBodyRotation = new Quaternion(
              parentFrame.body.rotation().x,
              parentFrame.body.rotation().y,
              parentFrame.body.rotation().z,
              parentFrame.body.rotation().w,
            );
            const childBodyRotation = parentBodyRotation.clone();
            const childTransform = worldPose(childPose);
            const childLinkToBodyRotation = childTransform.rotation
              .clone()
              .invert()
              .multiply(childBodyRotation);
            const massProperties = link.inertial
              ? getRapierMassProperties(link.inertial, childLinkToBodyRotation)
              : null;
            const rigidBodyDescription = massProperties
              ? rapier.RigidBodyDesc.dynamic()
                  .setAdditionalMassProperties(
                    massProperties.mass,
                    vectorObject(massProperties.centerOfMass),
                    vectorObject(massProperties.principalAngularInertia),
                    rotationObject(massProperties.angularInertiaLocalFrame),
                  )
                  .setLinearDamping(0.6)
                  .setAngularDamping(1.2)
                  .setGravityScale(0)
              : rapier.RigidBodyDesc.kinematicPositionBased();
            const body = world.createRigidBody(
              rigidBodyDescription
                .setTranslation(
                  childTransform.position.x,
                  childTransform.position.y,
                  childTransform.position.z,
                )
                .setRotation(rotationObject(childBodyRotation)),
            );
            createdBodies.push(body);
            const frame: BodyFrame = {
              body,
              robotToBodyRotation: childLinkToBodyRotation,
              isDynamicLink: Boolean(link.inertial),
            };
            bodyFrames.set(link.name, frame);
            if (!diagnostics?.disableCollisions) {
              await attachCollisions(
                rapier,
                world,
                body,
                link,
                childLinkToBodyRotation,
                packageMappings,
              );
            }
            if (cancelled) break;
            publishDebugEvent('physics:progress', { stage: 'child-colliders', joint: joint.name });

            const simulatedJoint = createJoint(
              rapier,
              world,
              joint,
              parentFrame.body,
              body,
              parentPose,
              childPose,
              parentBodyRotation,
              childBodyRotation,
              initialValues[joint.name] ?? 0,
            );
            if (simulatedJoint) createdJoints.push(simulatedJoint);
            publishDebugEvent('physics:progress', { stage: 'joint-created', joint: joint.name });
            queue.push(link.name);
          }
        }

        if (cancelled) return;
        lockedLinkNames.forEach((linkName) => {
          const frame = bodyFrames.get(linkName);
          if (frame?.isDynamicLink) {
            frame.body.setBodyType(rapier.RigidBodyType.KinematicPositionBased, true);
          }
        });
        createdBodies.forEach((body) => {
          if (body.isDynamic()) body.setGravityScale(1, true);
        });
        bodies.current = bodyFrames;
        joints.current = createdJoints;
        if (diagnostics && typeof window !== 'undefined') {
          const rootFrame = bodyFrames.get(description.rootLink);
          const jointFrames = Object.fromEntries(
            createdJoints.map((entry) => {
              const joint = entry.joint as JointMotor & {
                anchor1?: () => VectorObject;
                anchor2?: () => VectorObject;
                frameX1?: () => RotationObject;
                frameX2?: () => RotationObject;
                rawAxis?: () => number;
                configureMotorMaxForce?: (force: number) => void;
                limitsEnabled?: () => boolean;
                limitsMin?: () => number;
                limitsMax?: () => number;
              };
              return [
                entry.description.name,
                {
                  parent: entry.description.parent,
                  child: entry.description.child,
                  origin_position: entry.description.origin.position,
                  origin_orientation: entry.description.origin.orientation,
                  urdf_axis: entry.description.axis,
                  axis_local: entry.axisLocal.toArray(),
                  anchor1: vectorArray(joint.anchor1?.()),
                  anchor2: vectorArray(joint.anchor2?.()),
                  frameX1: rotationArray(joint.frameX1?.()),
                  frameX2: rotationArray(joint.frameX2?.()),
                  rawAxis: joint.rawAxis?.(),
                  motor_api_available: typeof joint.configureMotorPosition === 'function',
                  motor_max_force_api_available: typeof joint.configureMotorMaxForce === 'function',
                  limits: {
                    enabled: joint.limitsEnabled?.(),
                    minimum: joint.limitsMin?.(),
                    maximum: joint.limitsMax?.(),
                  },
                  initial_position: entry.initialPosition,
                  initial_relative_rotation: [
                    entry.initialRelativeRotation.x,
                    entry.initialRelativeRotation.y,
                    entry.initialRelativeRotation.z,
                    entry.initialRelativeRotation.w,
                  ],
                },
              ];
            }),
          );
          const bodyProperties = Object.fromEntries(
            Array.from(bodyFrames, ([name, frame]) => {
              return [
                name,
                {
                  type: frame.body.isFixed()
                    ? 'fixed'
                    : frame.body.isDynamic()
                      ? 'dynamic'
                      : 'kinematic',
                  has_urdf_inertial: frame.isDynamicLink,
                  gravity_scale: frame.body.gravityScale(),
                  linear_damping: frame.body.linearDamping(),
                  angular_damping: frame.body.angularDamping(),
                },
              ];
            }),
          );
          window.__ROBOT_SIM_PHASE2B23__ = {
            setup: {
              model: description.name,
              root_link: description.rootLink,
              tip_link: description.tipLink,
              gravity: diagnostics.gravity,
              collision_disabled: diagnostics.disableCollisions,
              locked_upstream: diagnostics.lockUpstream,
              position_execution: positionExecution,
              motor_model_explicitly_configured: false,
              default_stiffness: DEFAULT_JOINT_STIFFNESS,
              default_damping: DEFAULT_JOINT_DAMPING,
              solver_iterations: diagnostics.solverIterations ?? 4,
              internal_pgs_iterations: 1,
              base_body_type: rootFrame?.body.isFixed() ? 'fixed' : 'other',
              joints: jointFrames,
              bodies: bodyProperties,
            },
            samples: [],
          };
        }
        if (diagnostics?.traceMotorWrites && typeof window !== 'undefined') {
          const trace = window.__ROBOT_SIM_PHASE2B24__;
          const wrist3 = createdJoints.find((entry) => entry.description.name === 'wrist_3_joint');
          if (trace && wrist3) {
            const joint = wrist3.joint as JointMotor & {
              frameX1?: () => RotationObject;
              frameX2?: () => RotationObject;
              rawAxis?: () => number;
            };
            trace.setup = {
              ...trace.setup,
              wrist3_joint: {
                parent: wrist3.description.parent,
                child: wrist3.description.child,
                origin_position: wrist3.description.origin.position,
                origin_orientation: wrist3.description.origin.orientation,
                urdf_axis: wrist3.description.axis,
                axis_local: wrist3.axisLocal.toArray(),
                anchor1: vectorArray(joint.anchor1?.()),
                anchor2: vectorArray(joint.anchor2?.()),
                frameX1: rotationArray(joint.frameX1?.()),
                frameX2: rotationArray(joint.frameX2?.()),
                rawAxis: joint.rawAxis?.(),
                initial_joint_frame_relative_rotation:
                  wrist3.initialJointFrameRelativeRotation?.toArray() ?? null,
              },
            };
          }
        }
        meshPointCache.clear();
        publishDebugEvent('physics:ready', {
          model: description.name,
          links: bodyFrames.size,
          colliders: colliderCount,
          joints: createdJoints.length,
        });
        onReady(true);
      } catch (error) {
        if (!cancelled) {
          const message = error instanceof Error ? error.message : String(error);
          console.error(`Rapier 机器人初始化失败：${message}`, error);
          publishDebugEvent('physics:error', { model: description.name, message });
          onReady(false);
        }
        meshPointCache.clear();
      }
    };

    const attachCollisions = async (
      module: RapierModule,
      physicsWorld: RapierWorld,
      body: RapierBody,
      link: RobotDescription['links'][number],
      linkToBodyRotation: Quaternion,
      mappings: Readonly<Record<string, string>>,
    ) => {
      for (const collision of link.collisions) {
        const collider = await geometryDescriptor(module, collision, linkToBodyRotation, mappings);
        if (cancelled) return;
        if (collider) {
          physicsWorld.createCollider(collider, body);
          colliderCount += 1;
        }
      }
    };

    void initialize();
    return () => {
      cancelled = true;
      bodies.current = new Map();
      joints.current = [];
      lastPhysicsSignature.current = '';
      meshPointCache.clear();
      onReady(false);
    };
  }, [
    description,
    diagnostics,
    lockedLinkNames,
    onReady,
    packageMappings,
    positionExecution,
    rapier,
    world,
  ]);

  const applyCommands = useCallback(() => {
    if (diagnostics?.traceMotorWrites && typeof window !== 'undefined') {
      const trace = window.__ROBOT_SIM_PHASE2B24__;
      if (trace) trace.physics_step += 1;
    }
    const values = latestValues.current;
    const activeMode = latestMode.current;
    const activeCommands = latestCommands.current;
    const positionControl = activeMode === 'position' && positionExecution === 'kinematic_fk';
    setRobotPositionControl(rapier, bodies.current, positionControl, lockedLinkNames);
    const commandSignature = JSON.stringify({ mode: activeMode, commands: activeCommands });
    if (commandSignature !== lastCommandSignature.current) {
      lastCommandSignature.current = commandSignature;
      publishDebugEvent('robot:physics_command', {
        mode: activeMode,
        commands: activeCommands,
        motorAvailability: Object.fromEntries(
          joints.current.map((entry) => [
            entry.description.name,
            typeof entry.joint.configureMotorPosition === 'function',
          ]),
        ),
      });
    }
    if (activeMode !== heldPositionMode.current) {
      heldGripperPositions.current.clear();
      heldPositionMode.current = activeMode;
    }

    joints.current.forEach((entry) => {
      const { description: joint, child, parent } = entry;
      if (child.isKinematic()) return;

      let currentPosition = values[joint.name] ?? 0;
      const isGripperJoint = /gripper|finger/iu.test(joint.name + ' ' + joint.child);
      if (
        activeMode === 'position' &&
        isGripperJoint &&
        activeCommands[joint.name]?.mode !== 'position'
      ) {
        const heldPosition = heldGripperPositions.current.get(joint.name);
        if (heldPosition === undefined)
          heldGripperPositions.current.set(joint.name, currentPosition);
        currentPosition = heldGripperPositions.current.get(joint.name) ?? currentPosition;
      }

      applyJointCommand(joint, activeMode, activeCommands[joint.name], currentPosition, {
        position: (target) =>
          writePositionMotor(
            entry.joint,
            joint.name,
            target,
            entry.initialPosition,
            DEFAULT_JOINT_STIFFNESS,
            DEFAULT_JOINT_DAMPING,
            'RobotBody.useBeforePhysicsStep.applyCommands.position',
            undefined,
            () => {
              if (typeof window === 'undefined') return;
              const trace = window.__ROBOT_SIM_PHASE2B24__;
              if (
                trace &&
                trace.experiment_start_step === trace.physics_step &&
                trace.next_checkpoint_index === 0
              ) {
                appendPhase2B24Checkpoint(trace, joints.current, 0, 0);
              }
            },
          ),
        velocity: (target) =>
          writeVelocityMotor(
            entry.joint,
            joint.name,
            target,
            DEFAULT_JOINT_DAMPING,
            'RobotBody.useBeforePhysicsStep.applyCommands.velocity',
          ),
        effort: (value, kind) => {
          writePositionMotor(
            entry.joint,
            joint.name,
            entry.initialPosition,
            entry.initialPosition,
            0,
            0,
            'RobotBody.useBeforePhysicsStep.applyCommands.effortDisarm',
            undefined,
          );
          const force = vectorObject(axisInWorld(entry).multiplyScalar(value));
          if (kind === 'force') {
            child.addForce(force, true);
            parent.addForce({ x: -force.x, y: -force.y, z: -force.z }, true);
          } else {
            child.addTorque(force, true);
            parent.addTorque({ x: -force.x, y: -force.y, z: -force.z }, true);
          }
        },
      });
    });

    setKinematicBodies(description, values, bodies.current);
  }, [description, diagnostics?.traceMotorWrites, lockedLinkNames, positionExecution, rapier]);

  useBeforePhysicsStep(applyCommands);
  useAfterPhysicsStep(() => {
    const now = performance.now();
    const phase2b24Trace =
      diagnostics?.traceMotorWrites && typeof window !== 'undefined'
        ? window.__ROBOT_SIM_PHASE2B24__
        : undefined;
    if (
      phase2b24Trace &&
      phase2b24Trace.experiment_start_time_ms !== null &&
      phase2b24Trace.next_checkpoint_index < phase2b24Trace.checkpoints_s.length
    ) {
      const checkpointS = phase2b24Trace.checkpoints_s[phase2b24Trace.next_checkpoint_index];
      const elapsedS = (now - phase2b24Trace.experiment_start_time_ms) / 1000;
      if (elapsedS >= checkpointS) {
        appendPhase2B24Checkpoint(phase2b24Trace, joints.current, checkpointS, elapsedS);
      }
    }
    if (now - lastStatePublish.current < 100) return;
    lastStatePublish.current = now;
    const measured = readJointState(joints.current);
    const wrist3 = joints.current.find((entry) => entry.description.name === 'wrist_3_joint');
    onWrist3DebugState(wrist3 ? getWrist3PhysicsDebugState(wrist3, measured) : null);
    if (diagnostics && typeof window !== 'undefined') {
      const fkPoses = new Kinematics(description).forwardKinematicsAll(measured);
      const jointSamples = Object.fromEntries(
        joints.current.map((entry) => {
          const command = latestCommands.current[entry.description.name];
          const currentPosition =
            latestValues.current[entry.description.name] ?? entry.initialPosition;
          const commandedPosition =
            latestMode.current === 'position' && command?.mode === 'position'
              ? command.value
              : currentPosition;
          const parentVelocity = entry.parent.angvel();
          const childVelocity = entry.child.angvel();
          const relativeAngularVelocity = new Vector3(
            childVelocity.x - parentVelocity.x,
            childVelocity.y - parentVelocity.y,
            childVelocity.z - parentVelocity.z,
          );
          const worldAxis = axisInWorld(entry);
          const joint = entry.joint as JointMotor & {
            anchor1?: () => VectorObject;
            anchor2?: () => VectorObject;
            frameX1?: () => RotationObject;
            frameX2?: () => RotationObject;
            rawAxis?: () => number;
            configureMotorMaxForce?: (force: number) => void;
            limitsEnabled?: () => boolean;
            limitsMin?: () => number;
            limitsMax?: () => number;
          };
          return [
            entry.description.name,
            {
              commanded: commandedPosition,
              commanded_motor_relative_target: commandedPosition - entry.initialPosition,
              actual: measured[entry.description.name],
              tracking_error: measured[entry.description.name] - commandedPosition,
              delta_from_initial: measured[entry.description.name] - entry.initialPosition,
              relative_angular_velocity: relativeAngularVelocity.dot(worldAxis),
              world_axis: worldAxis.toArray(),
              anchor1: vectorArray(joint.anchor1?.()),
              anchor2: vectorArray(joint.anchor2?.()),
              frameX1: rotationArray(joint.frameX1?.()),
              frameX2: rotationArray(joint.frameX2?.()),
              rawAxis: joint.rawAxis?.(),
              motor_api_available: typeof joint.configureMotorPosition === 'function',
              motor_max_force_api_available: typeof joint.configureMotorMaxForce === 'function',
              limits: {
                enabled: joint.limitsEnabled?.(),
                minimum: joint.limitsMin?.(),
                maximum: joint.limitsMax?.(),
              },
              stiffness: DEFAULT_JOINT_STIFFNESS,
              damping: DEFAULT_JOINT_DAMPING,
            },
          ];
        }),
      );
      const linkSamples = Object.fromEntries(
        Array.from(bodies.current, ([name, frame]) => {
          const bodyPosition = frame.body.translation();
          const bodyRotationValue = frame.body.rotation();
          const bodyRotation = new Quaternion(
            bodyRotationValue.x,
            bodyRotationValue.y,
            bodyRotationValue.z,
            bodyRotationValue.w,
          );
          const linkRotation = bodyRotation
            .clone()
            .multiply(frame.robotToBodyRotation.clone().invert());
          const bodyOrientation = [bodyRotation.x, bodyRotation.y, bodyRotation.z, bodyRotation.w];
          const fkPose = fkPoses[name];
          const sceneFkPose = fkPose ? worldPose(fkPose) : null;
          const orientationDot = sceneFkPose
            ? Math.abs(
                linkRotation.x * sceneFkPose.rotation.x +
                  linkRotation.y * sceneFkPose.rotation.y +
                  linkRotation.z * sceneFkPose.rotation.z +
                  linkRotation.w * sceneFkPose.rotation.w,
              )
            : 0;
          const positionError = sceneFkPose
            ? new Vector3(bodyPosition.x, bodyPosition.y, bodyPosition.z).distanceTo(
                sceneFkPose.position,
              )
            : null;
          const orientationError = sceneFkPose ? 2 * Math.acos(Math.min(1, orientationDot)) : null;
          const linearVelocity = frame.body.linvel();
          const angularVelocity = frame.body.angvel();
          const localCom = frame.body.localCom();
          const principalInertia = frame.body.principalInertia();
          const principalInertiaLocalFrame = frame.body.principalInertiaLocalFrame();
          return [
            name,
            {
              body_position: [bodyPosition.x, bodyPosition.y, bodyPosition.z],
              body_orientation: bodyOrientation,
              link_position: [bodyPosition.x, bodyPosition.y, bodyPosition.z],
              link_orientation: [linkRotation.x, linkRotation.y, linkRotation.z, linkRotation.w],
              fk_position: sceneFkPose?.position.toArray() ?? null,
              fk_orientation: sceneFkPose
                ? [
                    sceneFkPose.rotation.x,
                    sceneFkPose.rotation.y,
                    sceneFkPose.rotation.z,
                    sceneFkPose.rotation.w,
                  ]
                : null,
              position_error: positionError,
              orientation_error_rad: orientationError,
              linear_velocity: [linearVelocity.x, linearVelocity.y, linearVelocity.z],
              angular_velocity: [angularVelocity.x, angularVelocity.y, angularVelocity.z],
              mass: frame.body.mass(),
              local_com: [localCom.x, localCom.y, localCom.z],
              principal_inertia: [principalInertia.x, principalInertia.y, principalInertia.z],
              principal_inertia_local_frame: [
                principalInertiaLocalFrame.x,
                principalInertiaLocalFrame.y,
                principalInertiaLocalFrame.z,
                principalInertiaLocalFrame.w,
              ],
              gravity_scale: frame.body.gravityScale(),
              linear_damping: frame.body.linearDamping(),
              angular_damping: frame.body.angularDamping(),
              body_type: frame.body.isFixed()
                ? 'fixed'
                : frame.body.isDynamic()
                  ? 'dynamic'
                  : 'kinematic',
            },
          ];
        }),
      );
      const diagnosticState = window.__ROBOT_SIM_PHASE2B23__;
      if (diagnosticState) {
        diagnosticState.samples.push({
          time_ms: now,
          joint_values: measured,
          joints: jointSamples,
          links: linkSamples,
        });
        if (diagnosticState.samples.length > MAX_PHASE2B23_SAMPLES) {
          diagnosticState.samples.shift();
        }
      }
    }
    if (Object.keys(measured).length > 0) {
      const signature = Object.entries(measured)
        .map(([name, value]) => `${name}:${value.toFixed(4)}`)
        .join('|');
      if (signature !== lastPhysicsSignature.current) {
        lastPhysicsSignature.current = signature;
        const linkPoses = Object.fromEntries(
          Array.from(bodies.current, ([name, frame]) => {
            const position = frame.body.translation();
            const orientation = frame.body.rotation();
            return [
              name,
              {
                position: [position.x, position.y, position.z],
                orientation: [orientation.x, orientation.y, orientation.z, orientation.w],
              },
            ];
          }),
        );
        publishDebugEvent('robot:physics_state', { jointValues: measured, linkPoses });
      }
      latestStateCallback.current({ ...latestValues.current, ...measured });
    }
  });

  return null;
};
