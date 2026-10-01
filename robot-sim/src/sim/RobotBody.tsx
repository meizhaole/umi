import { useCallback, useEffect, useRef } from 'react';
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
import { applyJointCommand } from './JointActuator';
import { publishDebugEvent } from '../app/debugBus';

interface RobotBodyProps {
  description: RobotDescription;
  jointValues: JointValues;
  commands: Record<string, JointCommand>;
  mode: ControlMode;
  onJointState: (values: JointValues) => void;
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
}

interface BodyFrame {
  body: RapierBody;
  robotToBodyRotation: Quaternion;
}

const ROS_TO_SCENE = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), -Math.PI / 2);
const DEFAULT_JOINT_STIFFNESS = 100;
const DEFAULT_JOINT_DAMPING = 18;
const MAX_HULL_POINTS = 512;
const meshPointCache = new Map<string, Promise<Float32Array>>();

const vectorObject = (vector: Vector3): VectorObject => ({ x: vector.x, y: vector.y, z: vector.z });

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

const meshUrl = (uri: string): string => {
  const prefix = 'package://rebotarm_bringup/';
  if (!uri.startsWith(prefix)) throw new Error(`不支持的碰撞网格 URI：${uri}`);
  return `/robot/${uri.slice(prefix.length)}`;
};

const loadMeshPoints = async (uri: string): Promise<Float32Array> => {
  const url = meshUrl(uri);
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
) => {
  const bodyFromLink = linkToBody.clone().invert();
  const originRotation = new Quaternion(...collision.origin.orientation);
  const bodyPosition = new Vector3(...collision.origin.position).applyQuaternion(bodyFromLink);
  const bodyRotation = bodyFromLink.clone().multiply(originRotation);
  const geometry = collision.geometry;
  let descriptor: InstanceType<typeof rapier.ColliderDesc> | null = null;

  if (geometry.type === 'mesh') {
    const source = await loadMeshPoints(geometry.filename);
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
  handle.configureMotorPosition?.(0, DEFAULT_JOINT_STIFFNESS, DEFAULT_JOINT_DAMPING);

  const initialRelativeRotation = parentBodyRotation.clone().invert().multiply(childBodyRotation);
  return {
    description: joint,
    joint: handle,
    parent: parentBody,
    child: childBody,
    axisLocal,
    childAnchor,
    parentAnchor,
    initialRelativeRotation,
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
      values[entry.description.name] = childAnchor.sub(parentAnchor).dot(axisWorld);
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
    values[entry.description.name] = 2 * Math.atan2(projected, delta.w);
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

export const RobotBody = ({
  description,
  jointValues,
  commands,
  mode,
  onJointState,
  onReady,
}: RobotBodyProps) => {
  const { world, rapier } = useRapier();
  const bodies = useRef(new Map<string, BodyFrame>());
  const joints = useRef<SimulatedJoint[]>([]);
  const latestValues = useRef(jointValues);
  const latestCommands = useRef(commands);
  const latestMode = useRef(mode);
  const latestStateCallback = useRef(onJointState);
  const lastStatePublish = useRef(0);

  latestValues.current = jointValues;
  latestCommands.current = commands;
  latestMode.current = mode;
  latestStateCallback.current = onJointState;

  useEffect(() => {
    let cancelled = false;
    const createdBodies: RapierBody[] = [];
    const bodyFrames = new Map<string, BodyFrame>();
    const createdJoints: SimulatedJoint[] = [];
    let colliderCount = 0;
    const kinematics = new Kinematics(description);
    const currentPoses = kinematics.forwardKinematicsAll(latestValues.current);

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
        });
        await attachCollisions(rapier, world, rootBody, rootLink, new Quaternion());
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
            const jointPose = composePoses(parentPose, joint.origin);
            const jointWorldRotation = worldPose({
              position: [0, 0, 0],
              orientation: jointPose.orientation,
            }).rotation;
            const jointWorldAxis = new Vector3(...joint.axis)
              .applyQuaternion(jointWorldRotation)
              .normalize();
            const parentBodyRotation = new Quaternion(
              parentFrame.body.rotation().x,
              parentFrame.body.rotation().y,
              parentFrame.body.rotation().z,
              parentFrame.body.rotation().w,
            );
            const parentAxis = jointWorldAxis
              .clone()
              .applyQuaternion(parentBodyRotation.clone().invert());
            const childBodyRotation =
              joint.type === 'fixed'
                ? parentBodyRotation.clone()
                : new Quaternion().setFromUnitVectors(parentAxis, jointWorldAxis);
            const childTransform = worldPose(childPose);
            const rigidBodyDescription = !link.inertial
              ? rapier.RigidBodyDesc.kinematicPositionBased()
              : rapier.RigidBodyDesc.dynamic()
                  .setAdditionalMass(Math.max(link.inertial.mass, 0.001))
                  .setLinearDamping(0.6)
                  .setAngularDamping(1.2)
                  .setGravityScale(0);
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
            const childLinkToBodyRotation = childTransform.rotation
              .clone()
              .invert()
              .multiply(childBodyRotation);
            const frame: BodyFrame = { body, robotToBodyRotation: childLinkToBodyRotation };
            bodyFrames.set(link.name, frame);
            await attachCollisions(rapier, world, body, link, childLinkToBodyRotation);
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
            );
            if (simulatedJoint) createdJoints.push(simulatedJoint);
            publishDebugEvent('physics:progress', { stage: 'joint-created', joint: joint.name });
            queue.push(link.name);
          }
        }

        if (cancelled) return;
        createdBodies.forEach((body) => {
          if (body.isDynamic()) body.setGravityScale(1, true);
        });
        bodies.current = bodyFrames;
        joints.current = createdJoints;
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
    ) => {
      for (const collision of link.collisions) {
        const collider = await geometryDescriptor(module, collision, linkToBodyRotation);
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
      meshPointCache.clear();
      onReady(false);
    };
  }, [description, onReady, rapier, world]);

  const applyCommands = useCallback(() => {
    const values = latestValues.current;
    const activeMode = latestMode.current;
    const activeCommands = latestCommands.current;

    joints.current.forEach((entry) => {
      const { description: joint, child, parent } = entry;
      if (child.isKinematic()) return;

      applyJointCommand(joint, activeMode, activeCommands[joint.name], values[joint.name] ?? 0, {
        position: (target) =>
          entry.joint.configureMotorPosition?.(
            target,
            DEFAULT_JOINT_STIFFNESS,
            DEFAULT_JOINT_DAMPING,
          ),
        velocity: (target) => entry.joint.configureMotorVelocity?.(target, DEFAULT_JOINT_DAMPING),
        effort: (value, kind) => {
          entry.joint.configureMotorPosition?.(0, 0, 0);
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
  }, [description]);

  useBeforePhysicsStep(applyCommands);
  useAfterPhysicsStep(() => {
    const now = performance.now();
    if (now - lastStatePublish.current < 100) return;
    lastStatePublish.current = now;
    const measured = readJointState(joints.current);
    if (Object.keys(measured).length > 0) {
      latestStateCallback.current({ ...latestValues.current, ...measured });
    }
  });

  return null;
};
