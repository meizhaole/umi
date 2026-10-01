import { RigidBody, CuboidCollider } from '@react-three/rapier';

export const PhysicsWorld = () => (
  <RigidBody type="fixed" colliders={false}>
    <CuboidCollider args={[3, 0.03, 3]} position={[0, -0.03, 0]} friction={0.8} />
  </RigidBody>
);
