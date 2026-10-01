import { useCallback, useRef, useState } from 'react';
import type { ThreeEvent } from '@react-three/fiber';
import {
  CuboidCollider,
  CylinderCollider,
  RigidBody,
  useAfterPhysicsStep,
} from '@react-three/rapier';
import type { RapierRigidBody } from '@react-three/rapier';
import { DoubleSide, Plane, Vector2, Vector3 } from 'three';

const SAUCER_POSITION: [number, number, number] = [0.47, 0, 0.1];
const CUP_POSITION: [number, number, number] = [0.29, 0.001, 0.1];
const CUP_HEIGHT = 0.078;
const SAUCER_RADIUS = 0.085;
const SAUCER_HALF_HEIGHT = 0.005;
const MAX_TABLE_RADIUS = 0.68;
const MAX_CUP_HEIGHT = 0.45;
const MAX_CUP_SPEED = 0.8;
const DRAG_PLANE = new Plane(new Vector3(0, 1, 0), 0);
const CUP_PROFILE = [
  new Vector2(0, 0.008),
  new Vector2(0.027, 0.008),
  new Vector2(0.033, 0.068),
  new Vector2(0.041, 0.075),
  new Vector2(0.045, 0.075),
  new Vector2(0.037, 0.066),
  new Vector2(0.03, 0),
  new Vector2(0, 0),
];

const Saucer = () => (
  <RigidBody type="fixed" colliders={false} position={SAUCER_POSITION}>
    <CylinderCollider
      args={[SAUCER_HALF_HEIGHT, SAUCER_RADIUS]}
      friction={0.9}
      position={[0, SAUCER_HALF_HEIGHT, 0]}
    />
    <mesh position={[0, SAUCER_HALF_HEIGHT, 0]} receiveShadow>
      <cylinderGeometry args={[0.074, SAUCER_RADIUS, SAUCER_HALF_HEIGHT * 2, 48]} />
      <meshStandardMaterial color="#deddd7" roughness={0.38} />
    </mesh>
    <mesh position={[0, SAUCER_HALF_HEIGHT * 2 + 0.001, 0]}>
      <torusGeometry args={[0.073, 0.0018, 8, 48]} />
      <meshStandardMaterial color="#91aeb0" metalness={0.18} roughness={0.32} />
    </mesh>
  </RigidBody>
);

export type CupBodyRef = { current: RapierRigidBody | null };

interface EspressoCupProps {
  bodyRef: CupBodyRef;
  onDragStateChange: (dragging: boolean) => void;
}

const EspressoCup = ({ bodyRef, onDragStateChange }: EspressoCupProps) => {
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
    if (radius > MAX_TABLE_RADIUS) {
      x = (x / radius) * MAX_TABLE_RADIUS;
      z = (z / radius) * MAX_TABLE_RADIUS;
      const outwardSpeed =
        nextVelocity.x * (x / MAX_TABLE_RADIUS) + nextVelocity.z * (z / MAX_TABLE_RADIUS);
      if (outwardSpeed > 0) {
        nextVelocity.x -= outwardSpeed * (x / MAX_TABLE_RADIUS);
        nextVelocity.z -= outwardSpeed * (z / MAX_TABLE_RADIUS);
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
      if (radius > MAX_TABLE_RADIUS) {
        x = (x / radius) * MAX_TABLE_RADIUS;
        z = (z / radius) * MAX_TABLE_RADIUS;
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
      <CylinderCollider
        args={[CUP_HEIGHT / 2, 0.039]}
        friction={0.8}
        position={[0, CUP_HEIGHT / 2, 0]}
      />
      <CuboidCollider args={[0.004, 0.018, 0.004]} friction={0.8} position={[0.055, 0.04, 0.021]} />
      <CuboidCollider
        args={[0.004, 0.018, 0.004]}
        friction={0.8}
        position={[0.055, 0.04, -0.021]}
      />
      <CuboidCollider args={[0.004, 0.004, 0.018]} friction={0.8} position={[0.055, 0.061, 0]} />
      <CuboidCollider args={[0.004, 0.004, 0.018]} friction={0.8} position={[0.055, 0.019, 0]} />
      <mesh
        castShadow
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        receiveShadow
      >
        <latheGeometry args={[CUP_PROFILE, 32]} />
        <meshStandardMaterial
          color={isSelected ? '#8fcbd0' : '#75aeb4'}
          roughness={0.24}
          side={DoubleSide}
        />
      </mesh>
      <mesh
        position={[0.055, 0.04, 0]}
        rotation={[0, Math.PI / 2, 0]}
        castShadow
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
      >
        <torusGeometry args={[0.023, 0.004, 10, 32]} />
        <meshStandardMaterial color={isSelected ? '#8fcbd0' : '#75aeb4'} roughness={0.24} />
      </mesh>
    </RigidBody>
  );
};

interface OfficialCupArrangementSceneProps {
  bodyRef: CupBodyRef;
  onDragStateChange: (dragging: boolean) => void;
}

export const OfficialCupArrangementScene = ({
  bodyRef,
  onDragStateChange,
}: OfficialCupArrangementSceneProps) => (
  <>
    <Saucer />
    <EspressoCup bodyRef={bodyRef} onDragStateChange={onDragStateChange} />
  </>
);
