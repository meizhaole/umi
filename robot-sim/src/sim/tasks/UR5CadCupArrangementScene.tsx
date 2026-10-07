import { useCallback, useRef, useState } from 'react';
import {
  CapsuleCollider,
  CylinderCollider,
  RigidBody,
  useAfterPhysicsStep,
} from '@react-three/rapier';
import type { RapierRigidBody } from '@react-three/rapier';
import { CubicBezierCurve3, DoubleSide, Vector2, Vector3 } from 'three';

import { TABLE_RADIUS } from '../tableConstants';
import type { CupBodyRef } from './CupArrangementScene';
import type { CupEvaluationGeometryRef } from './cupApproachEvaluation';
import { usePlanarDrag } from './usePlanarDrag';

const SAUCER_POSITION: [number, number, number] = [-0.578, 0, 0];
const CUP_POSITION: [number, number, number] = [-0.578, 0.001, -0.24];
const CUP_HEIGHT = 0.09;
const SAUCER_RADIUS = 0.095;
const SAUCER_COLLIDER_HALF_HEIGHT = 0.004;
const CUP_HANDLE_COLLIDER_RADIUS = 0.006;
const CUP_HANDLE_COLLIDER_SEGMENT_COUNT = 6;
const CUP_EDGE_CLEARANCE = 0.11;
const MAX_CUP_CENTER_RADIUS = TABLE_RADIUS - CUP_EDGE_CLEARANCE;
const MAX_SAUCER_CENTER_RADIUS = TABLE_RADIUS - SAUCER_RADIUS;
const MAX_CUP_HEIGHT = 0.45;
const MAX_CUP_SPEED = 0.8;
const CUP_PROFILE = [
  new Vector2(0, 0),
  new Vector2(0.028, 0),
  new Vector2(0.031, 0.004),
  new Vector2(0.036, 0.017),
  new Vector2(0.041, 0.068),
  new Vector2(0.043, 0.083),
  new Vector2(0.044, 0.088),
  new Vector2(0.042, CUP_HEIGHT),
  new Vector2(0.037, CUP_HEIGHT),
  new Vector2(0.036, 0.084),
  new Vector2(0.033, 0.069),
  new Vector2(0.028, 0.019),
  new Vector2(0, 0.012),
];
const SAUCER_PROFILE = [
  new Vector2(0, 0.008),
  new Vector2(0.045, 0.008),
  new Vector2(0.065, 0.009),
  new Vector2(0.08, 0.013),
  new Vector2(0.09, 0.019),
  new Vector2(SAUCER_RADIUS, 0.023),
  new Vector2(0.093, 0.024),
  new Vector2(0.09, 0.022),
  new Vector2(0.086, 0.018),
  new Vector2(0.08, 0.014),
  new Vector2(0.06, 0.01),
  new Vector2(0, 0.01),
];
const CUP_HANDLE_CURVE = new CubicBezierCurve3(
  new Vector3(0.039, 0.077, 0),
  new Vector3(0.108, 0.09, 0),
  new Vector3(0.108, 0.016, 0),
  new Vector3(0.03, 0.018, 0),
);
const CUP_HANDLE_COLLIDER_POINTS = CUP_HANDLE_CURVE.getPoints(CUP_HANDLE_COLLIDER_SEGMENT_COUNT);
const CUP_HANDLE_COLLIDERS = CUP_HANDLE_COLLIDER_POINTS.slice(0, -1).map((start, index) => {
  const end = CUP_HANDLE_COLLIDER_POINTS[index + 1];
  const segment = end.clone().sub(start);

  return {
    halfHeight: segment.length() / 2,
    position: start.clone().add(end).multiplyScalar(0.5).toArray(),
    rotation: [0, 0, Math.atan2(segment.y, segment.x) - Math.PI / 2] as [number, number, number],
  };
});

interface UR5CadCupArrangementSceneProps {
  bodyRef: CupBodyRef;
  evaluationGeometryRef: CupEvaluationGeometryRef;
  onDragStateChange: (dragging: boolean) => void;
}

interface SaucerProps {
  onDragStateChange: (dragging: boolean) => void;
}

const Saucer = ({ onDragStateChange }: SaucerProps) => {
  const bodyRef = useRef<RapierRigidBody | null>(null);
  const dragHandlers = usePlanarDrag({
    bodyRef,
    centerY: SAUCER_POSITION[1],
    maxRadius: MAX_SAUCER_CENTER_RADIUS,
    onDragStateChange,
  });

  return (
    <RigidBody ref={bodyRef} type="kinematicPosition" colliders={false} position={SAUCER_POSITION}>
      <CylinderCollider
        args={[SAUCER_COLLIDER_HALF_HEIGHT, SAUCER_RADIUS]}
        friction={0.9}
        position={[0, SAUCER_COLLIDER_HALF_HEIGHT, 0]}
      />
      <mesh
        onPointerDown={dragHandlers.handlePointerDown}
        onPointerMove={dragHandlers.handlePointerMove}
        onPointerUp={dragHandlers.handlePointerUp}
        onPointerCancel={dragHandlers.handlePointerUp}
        receiveShadow
      >
        <latheGeometry args={[SAUCER_PROFILE, 48]} />
        <meshStandardMaterial color="#e8e2d8" roughness={0.32} side={DoubleSide} />
      </mesh>
    </RigidBody>
  );
};

interface UR5CadCupProps {
  bodyRef: CupBodyRef;
  evaluationGeometryRef: CupEvaluationGeometryRef;
  onDragStateChange: (dragging: boolean) => void;
}

const UR5CadCup = ({ bodyRef, evaluationGeometryRef, onDragStateChange }: UR5CadCupProps) => {
  const [isSelected, setIsSelected] = useState(false);
  const dragHandlers = usePlanarDrag({
    bodyRef,
    centerY: CUP_POSITION[1],
    maxRadius: MAX_CUP_CENTER_RADIUS,
    onDragStateChange,
  });

  const enforceCupBounds = useCallback(() => {
    const body = bodyRef.current;
    if (!body) return;

    const position = body.translation();
    const velocity = body.linvel();
    let x = position.x;
    let y = position.y;
    let z = position.z;
    const nextVelocity = new Vector3(velocity.x, velocity.y, velocity.z);
    const radius = Math.hypot(x, z);

    if (radius > MAX_CUP_CENTER_RADIUS) {
      x = (x / radius) * MAX_CUP_CENTER_RADIUS;
      z = (z / radius) * MAX_CUP_CENTER_RADIUS;
      const outwardSpeed =
        nextVelocity.x * (x / MAX_CUP_CENTER_RADIUS) + nextVelocity.z * (z / MAX_CUP_CENTER_RADIUS);
      if (outwardSpeed > 0) {
        nextVelocity.x -= outwardSpeed * (x / MAX_CUP_CENTER_RADIUS);
        nextVelocity.z -= outwardSpeed * (z / MAX_CUP_CENTER_RADIUS);
      }
    }

    if (y > MAX_CUP_HEIGHT) {
      y = MAX_CUP_HEIGHT;
      nextVelocity.y = Math.min(nextVelocity.y, 0);
    }

    const speed = nextVelocity.length();
    if (speed > MAX_CUP_SPEED) nextVelocity.multiplyScalar(MAX_CUP_SPEED / speed);

    if (x !== position.x || y !== position.y || z !== position.z) {
      body.setTranslation({ x, y, z }, true);
    }
    if (
      nextVelocity.x !== velocity.x ||
      nextVelocity.y !== velocity.y ||
      nextVelocity.z !== velocity.z
    ) {
      body.setLinvel({ x: nextVelocity.x, y: nextVelocity.y, z: nextVelocity.z }, true);
    }
  }, [bodyRef]);

  useAfterPhysicsStep(enforceCupBounds);

  return (
    <RigidBody
      ref={bodyRef}
      angularDamping={2.5}
      colliders={false}
      ccd
      friction={0.8}
      linearDamping={1.2}
      mass={0.12}
      position={CUP_POSITION}
      restitution={0.04}
      rotation={[0, -Math.PI / 2, 0]}
    >
      <CylinderCollider args={[0.008, 0.032]} friction={0.8} position={[0, 0.008, 0]} />
      <CylinderCollider args={[0.021, 0.039]} friction={0.8} position={[0, 0.037, 0]} />
      <CylinderCollider args={[0.016, 0.044]} friction={0.8} position={[0, 0.074, 0]} />
      {CUP_HANDLE_COLLIDERS.map((collider, index) => (
        <CapsuleCollider
          key={index}
          args={[collider.halfHeight, CUP_HANDLE_COLLIDER_RADIUS]}
          friction={0.8}
          position={collider.position}
          rotation={collider.rotation}
        />
      ))}
      <group ref={evaluationGeometryRef}>
        <mesh
          castShadow
          onPointerDown={(event) => {
            if (event.button === 0) setIsSelected(true);
            dragHandlers.handlePointerDown(event);
          }}
          onPointerMove={dragHandlers.handlePointerMove}
          onPointerUp={dragHandlers.handlePointerUp}
          onPointerCancel={dragHandlers.handlePointerUp}
          receiveShadow
        >
          <latheGeometry args={[CUP_PROFILE, 48]} />
          <meshStandardMaterial
            color={isSelected ? '#8fcbd0' : '#d5e4e5'}
            roughness={0.24}
            side={DoubleSide}
          />
        </mesh>
        <mesh
          castShadow
          onPointerDown={(event) => {
            if (event.button === 0) setIsSelected(true);
            dragHandlers.handlePointerDown(event);
          }}
          onPointerMove={dragHandlers.handlePointerMove}
          onPointerUp={dragHandlers.handlePointerUp}
          onPointerCancel={dragHandlers.handlePointerUp}
          receiveShadow
        >
          <tubeGeometry args={[CUP_HANDLE_CURVE, 32, 0.006, 12, false]} />
          <meshStandardMaterial color={isSelected ? '#8fcbd0' : '#d5e4e5'} roughness={0.24} />
        </mesh>
      </group>
    </RigidBody>
  );
};

export const UR5CadCupArrangementScene = ({
  bodyRef,
  evaluationGeometryRef,
  onDragStateChange,
}: UR5CadCupArrangementSceneProps) => (
  <>
    <Saucer onDragStateChange={onDragStateChange} />
    <UR5CadCup
      bodyRef={bodyRef}
      evaluationGeometryRef={evaluationGeometryRef}
      onDragStateChange={onDragStateChange}
    />
  </>
);
