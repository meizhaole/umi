import { CuboidCollider, CylinderCollider, RigidBody } from '@react-three/rapier';
import { DoubleSide, Vector2 } from 'three';

const SAUCER_POSITION: [number, number, number] = [0.47, 0, 0.1];
const CUP_POSITION: [number, number, number] = [0.29, 0.001, 0.1];
const CUP_HEIGHT = 0.078;
const SAUCER_RADIUS = 0.085;
const SAUCER_HALF_HEIGHT = 0.005;
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

const EspressoCup = () => (
  <RigidBody
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
    <CuboidCollider args={[0.004, 0.018, 0.004]} friction={0.8} position={[0.055, 0.04, -0.021]} />
    <CuboidCollider args={[0.004, 0.004, 0.018]} friction={0.8} position={[0.055, 0.061, 0]} />
    <CuboidCollider args={[0.004, 0.004, 0.018]} friction={0.8} position={[0.055, 0.019, 0]} />
    <mesh castShadow receiveShadow>
      <latheGeometry args={[CUP_PROFILE, 32]} />
      <meshStandardMaterial color="#75aeb4" roughness={0.24} side={DoubleSide} />
    </mesh>
    <mesh position={[0.055, 0.04, 0]} rotation={[0, Math.PI / 2, 0]} castShadow>
      <torusGeometry args={[0.023, 0.004, 10, 32]} />
      <meshStandardMaterial color="#75aeb4" roughness={0.24} />
    </mesh>
  </RigidBody>
);

export const OfficialCupArrangementScene = () => (
  <>
    <Saucer />
    <EspressoCup />
  </>
);
