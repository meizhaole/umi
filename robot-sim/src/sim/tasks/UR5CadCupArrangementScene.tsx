import { useCallback, useEffect, useRef, useState } from 'react';
import type { ThreeEvent } from '@react-three/fiber';
import {
  CapsuleCollider,
  CylinderCollider,
  RigidBody,
  useAfterPhysicsStep,
} from '@react-three/rapier';
import { CubicBezierCurve3, DoubleSide, Plane, Vector2, Vector3 } from 'three';

import type { CupBodyRef } from './CupArrangementScene';

const SAUCER_POSITION: [number, number, number] = [-0.578, 0, 0];
const CUP_POSITION: [number, number, number] = [-0.578, 0.001, -0.24];
const CUP_HEIGHT = 0.09;
const SAUCER_RADIUS = 0.095;
const SAUCER_COLLIDER_HALF_HEIGHT = 0.004;
const CUP_HANDLE_COLLIDER_RADIUS = 0.006;
const CUP_HANDLE_COLLIDER_SEGMENT_COUNT = 6;
const MAX_CUP_CENTER_RADIUS = 0.64;
const MAX_CUP_HEIGHT = 0.45;
const MAX_CUP_SPEED = 0.8;
const DRAG_PLANE = new Plane(new Vector3(0, 1, 0), 0);
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
  onDragStateChange: (dragging: boolean) => void;
}

const Saucer = () => (
  <RigidBody type="fixed" colliders={false} position={SAUCER_POSITION}>
    <CylinderCollider
      args={[SAUCER_COLLIDER_HALF_HEIGHT, SAUCER_RADIUS]}
      friction={0.9}
      position={[0, SAUCER_COLLIDER_HALF_HEIGHT, 0]}
    />
    <mesh receiveShadow>
      <latheGeometry args={[SAUCER_PROFILE, 48]} />
      <meshStandardMaterial color="#e8e2d8" roughness={0.32} side={DoubleSide} />
    </mesh>
  </RigidBody>
);

interface UR5CadCupProps {
  bodyRef: CupBodyRef;
  onDragStateChange: (dragging: boolean) => void;
}

const UR5CadCup = ({ bodyRef, onDragStateChange }: UR5CadCupProps) => {
  const dragOffset = useRef(new Vector2());
  const dragPoint = useRef(new Vector3());
  const isDragging = useRef(false);
  const [isSelected, setIsSelected] = useState(false);

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

  useEffect(
    () => () => {
      onDragStateChange(false);
    },
    [onDragStateChange],
  );

  const handlePointerDown = useCallback(
    (event: ThreeEvent<PointerEvent>) => {
      if (event.button !== 0) return;
      event.stopPropagation();
      event.nativeEvent.preventDefault();
      event.nativeEvent.stopImmediatePropagation();
      setIsSelected(true);
      const body = bodyRef.current;
      if (!body || !event.ray.intersectPlane(DRAG_PLANE, dragPoint.current)) return;

      const position = body.translation();
      dragOffset.current.set(position.x - dragPoint.current.x, position.z - dragPoint.current.z);
      const canvas = event.nativeEvent.currentTarget as HTMLCanvasElement | null;
      canvas?.setPointerCapture(event.pointerId);
      isDragging.current = true;
      onDragStateChange(true);
    },
    [bodyRef, onDragStateChange],
  );

  const handlePointerMove = useCallback(
    (event: ThreeEvent<PointerEvent>) => {
      if (!isDragging.current) return;
      if ((event.nativeEvent.buttons & 1) === 0) {
        isDragging.current = false;
        onDragStateChange(false);
        return;
      }

      const body = bodyRef.current;
      if (!body || !event.ray.intersectPlane(DRAG_PLANE, dragPoint.current)) return;

      let x = dragPoint.current.x + dragOffset.current.x;
      let z = dragPoint.current.z + dragOffset.current.y;
      const radius = Math.hypot(x, z);
      if (radius > MAX_CUP_CENTER_RADIUS) {
        x = (x / radius) * MAX_CUP_CENTER_RADIUS;
        z = (z / radius) * MAX_CUP_CENTER_RADIUS;
      }
      body.setTranslation({ x, y: CUP_POSITION[1], z }, true);
    },
    [bodyRef, onDragStateChange],
  );

  const handlePointerUp = useCallback(
    (event: ThreeEvent<PointerEvent>) => {
      if (!isDragging.current) return;
      event.stopPropagation();
      const canvas = event.nativeEvent.currentTarget as HTMLCanvasElement | null;
      if (canvas?.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
      isDragging.current = false;
      onDragStateChange(false);
    },
    [onDragStateChange],
  );

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
      <mesh
        castShadow
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
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
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        receiveShadow
      >
        <tubeGeometry args={[CUP_HANDLE_CURVE, 32, 0.006, 12, false]} />
        <meshStandardMaterial color={isSelected ? '#8fcbd0' : '#d5e4e5'} roughness={0.24} />
      </mesh>
    </RigidBody>
  );
};

export const UR5CadCupArrangementScene = ({
  bodyRef,
  onDragStateChange,
}: UR5CadCupArrangementSceneProps) => (
  <>
    <Saucer />
    <UR5CadCup bodyRef={bodyRef} onDragStateChange={onDragStateChange} />
  </>
);
