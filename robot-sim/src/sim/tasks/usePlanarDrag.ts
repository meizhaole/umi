import { useCallback, useEffect, useRef } from 'react';
import type { ThreeEvent } from '@react-three/fiber';
import type { RapierRigidBody } from '@react-three/rapier';
import { Plane, Vector2, Vector3 } from 'three';

const DRAG_PLANE = new Plane(new Vector3(0, 1, 0), 0);

type PlanarBodyRef = { current: RapierRigidBody | null };

interface PlanarDragOptions {
  bodyRef: PlanarBodyRef;
  centerY: number;
  maxRadius: number;
  onDragStateChange: (dragging: boolean) => void;
}

interface PointerCaptureTarget {
  setPointerCapture: (pointerId: number) => void;
  releasePointerCapture: (pointerId: number) => void;
}

export const usePlanarDrag = ({
  bodyRef,
  centerY,
  maxRadius,
  onDragStateChange,
}: PlanarDragOptions) => {
  const dragOffset = useRef(new Vector2());
  const dragPoint = useRef(new Vector3());
  const isDragging = useRef(false);

  const stopDragging = useCallback(
    (event: ThreeEvent<PointerEvent>) => {
      if (!isDragging.current) return;

      (event.target as unknown as PointerCaptureTarget).releasePointerCapture(event.pointerId);
      isDragging.current = false;
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

      const body = bodyRef.current;
      if (!body || !event.ray.intersectPlane(DRAG_PLANE, dragPoint.current)) return;

      const position = body.translation();
      dragOffset.current.set(position.x - dragPoint.current.x, position.z - dragPoint.current.z);
      (event.target as unknown as PointerCaptureTarget).setPointerCapture(event.pointerId);
      isDragging.current = true;
      onDragStateChange(true);
    },
    [bodyRef, onDragStateChange],
  );

  const handlePointerMove = useCallback(
    (event: ThreeEvent<PointerEvent>) => {
      if (!isDragging.current) return;
      if ((event.nativeEvent.buttons & 1) === 0) {
        stopDragging(event);
        return;
      }

      const body = bodyRef.current;
      if (!body || !event.ray.intersectPlane(DRAG_PLANE, dragPoint.current)) return;

      let x = dragPoint.current.x + dragOffset.current.x;
      let z = dragPoint.current.z + dragOffset.current.y;
      const radius = Math.hypot(x, z);
      if (radius > maxRadius) {
        x = (x / radius) * maxRadius;
        z = (z / radius) * maxRadius;
      }

      body.setTranslation({ x, y: centerY, z }, true);
    },
    [bodyRef, centerY, maxRadius, stopDragging],
  );

  const handlePointerUp = useCallback(
    (event: ThreeEvent<PointerEvent>) => {
      if (!isDragging.current) return;
      event.stopPropagation();
      stopDragging(event);
    },
    [stopDragging],
  );

  useEffect(
    () => () => {
      isDragging.current = false;
      onDragStateChange(false);
    },
    [onDragStateChange],
  );

  return { handlePointerDown, handlePointerMove, handlePointerUp };
};
