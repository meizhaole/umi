import { CylinderCollider, RigidBody } from '@react-three/rapier';

const TABLE_RADIUS = 0.75;
const TABLE_THICKNESS = 0.06;

export const PhysicsWorld = () => (
  <RigidBody type="fixed" colliders={false} position={[0, -TABLE_THICKNESS / 2, 0]}>
    <CylinderCollider args={[TABLE_THICKNESS / 2, TABLE_RADIUS]} friction={0.8} />
    <mesh receiveShadow>
      <cylinderGeometry args={[TABLE_RADIUS, TABLE_RADIUS, TABLE_THICKNESS, 64]} />
      <meshStandardMaterial color="#d1d0ca" roughness={0.86} />
    </mesh>
  </RigidBody>
);
