import { useCallback, useRef, useState } from 'react';
import {
  CuboidCollider,
  CylinderCollider,
  RigidBody,
  useAfterPhysicsStep,
} from '@react-three/rapier';
import type { RapierRigidBody } from '@react-three/rapier';
import { DoubleSide, Vector2, Vector3 } from 'three';

import { TABLE_RADIUS } from '../tableConstants';
import { usePlanarDrag } from './usePlanarDrag';

const SAUCER_POSITION: [number, number, number] = [0.47, 0, 0.1];
const CUP_POSITION: [number, number, number] = [0.29, 0.001, 0.1];
const CUP_HEIGHT = 0.078;
const SAUCER_RADIUS = 0.085;
const SAUCER_HALF_HEIGHT = 0.005;
const CUP_EDGE_CLEARANCE = 0.07;
const SAUCER_EDGE_CLEARANCE = SAUCER_RADIUS;
const MAX_CUP_CENTER_RADIUS = TABLE_RADIUS - CUP_EDGE_CLEARANCE;
const MAX_SAUCER_CENTER_RADIUS = TABLE_RADIUS - SAUCER_EDGE_CLEARANCE;
const MAX_CUP_HEIGHT = 0.45;
const MAX_CUP_SPEED = 0.8;
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
        args={[SAUCER_HALF_HEIGHT, SAUCER_RADIUS]}
        friction={0.9}
        position={[0, SAUCER_HALF_HEIGHT, 0]}
      />
      <mesh
        position={[0, SAUCER_HALF_HEIGHT, 0]}
        onPointerDown={dragHandlers.handlePointerDown}
        onPointerMove={dragHandlers.handlePointerMove}
        onPointerUp={dragHandlers.handlePointerUp}
        onPointerCancel={dragHandlers.handlePointerUp}
        receiveShadow
      >
        <cylinderGeometry args={[0.074, SAUCER_RADIUS, SAUCER_HALF_HEIGHT * 2, 48]} />
        <meshStandardMaterial color="#deddd7" roughness={0.38} />
      </mesh>
      <mesh
        position={[0, SAUCER_HALF_HEIGHT * 2 + 0.001, 0]}
        onPointerDown={dragHandlers.handlePointerDown}
        onPointerMove={dragHandlers.handlePointerMove}
        onPointerUp={dragHandlers.handlePointerUp}
        onPointerCancel={dragHandlers.handlePointerUp}
      >
        <torusGeometry args={[0.073, 0.0018, 8, 48]} />
        <meshStandardMaterial color="#91aeb0" metalness={0.18} roughness={0.32} />
      </mesh>
    </RigidBody>
  );
};

export type CupBodyRef = { current: RapierRigidBody | null };

interface EspressoCupProps {
  bodyRef: CupBodyRef;
  onDragStateChange: (dragging: boolean) => void;
}

const EspressoCup = ({ bodyRef, onDragStateChange }: EspressoCupProps) => {
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
        onPointerDown={(event) => {
          if (event.button === 0) setIsSelected(true);
          dragHandlers.handlePointerDown(event);
        }}
        onPointerMove={dragHandlers.handlePointerMove}
        onPointerUp={dragHandlers.handlePointerUp}
        onPointerCancel={dragHandlers.handlePointerUp}
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
        onPointerDown={(event) => {
          if (event.button === 0) setIsSelected(true);
          dragHandlers.handlePointerDown(event);
        }}
        onPointerMove={dragHandlers.handlePointerMove}
        onPointerUp={dragHandlers.handlePointerUp}
        onPointerCancel={dragHandlers.handlePointerUp}
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
    <Saucer onDragStateChange={onDragStateChange} />
    <EspressoCup bodyRef={bodyRef} onDragStateChange={onDragStateChange} />
  </>
);
