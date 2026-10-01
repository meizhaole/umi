import { useCallback, useRef } from 'react';
import type { ThreeEvent } from '@react-three/fiber';
import { CuboidCollider, CylinderCollider, RigidBody } from '@react-three/rapier';
import type { RapierRigidBody } from '@react-three/rapier';
import { DoubleSide, Plane, Vector2, Vector3 } from 'three';

const SAUCER_POSITION: [number, number, number] = [0.47, 0, 0.1];
const CUP_POSITION: [number, number, number] = [0.29, 0.001, 0.1];
const CUP_HEIGHT = 0.078;
const SAUCER_RADIUS = 0.085;
const SAUCER_HALF_HEIGHT = 0.005;
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

  const handlePointerDown = useCallback(
    (event: ThreeEvent<PointerEvent>) => {
      event.stopPropagation();
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
      const body = bodyRef.current;
      if (!isDragging.current || !body || !event.ray.intersectPlane(DRAG_PLANE, dragPoint.current))
        return;

      body.setTranslation(
        {
          x: dragPoint.current.x + dragOffset.current.x,
          y: CUP_POSITION[1],
          z: dragPoint.current.z + dragOffset.current.y,
        },
        true,
      );
    },
    [bodyRef],
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
      angularDamping={1.8}
      colliders={false}
      friction={0.8}
      linearDamping={0.7}
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
        <meshStandardMaterial color="#75aeb4" roughness={0.24} side={DoubleSide} />
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
        <meshStandardMaterial color="#75aeb4" roughness={0.24} />
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
