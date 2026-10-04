import type { Object3D } from 'three';

export const SIMULATION_LAYER = 0;
export const DEBUG_LAYER = 1;

export const setDebugLayer = (object: Object3D | null): void => {
  if (!object) return;
  object.layers.set(DEBUG_LAYER);
  object.userData.robotSimLayer = 'debug';
};
