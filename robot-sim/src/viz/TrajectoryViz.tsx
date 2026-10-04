import { BufferGeometry, Float32BufferAttribute } from 'three';
import { useMemo } from 'react';
import { setDebugLayer } from './sceneLayers';

export const TrajectoryViz = ({ points }: { points: [number, number, number][] }) => {
  const geometry = useMemo(() => {
    const path = new BufferGeometry();
    path.setAttribute('position', new Float32BufferAttribute(points.flat(), 3));
    return path;
  }, [points]);

  return (
    <lineSegments ref={setDebugLayer} name="trajectory-helper" geometry={geometry}>
      <lineBasicMaterial color="#62d6c7" transparent opacity={0.8} />
    </lineSegments>
  );
};
