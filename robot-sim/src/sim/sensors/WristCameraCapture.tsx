import { useEffect, useMemo } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { PerspectiveCamera, SRGBColorSpace, WebGLRenderTarget } from 'three';
import type { URDFRobot } from 'urdf-loader';
import { Vector3 } from 'three';
import type { JointValues, Pose, RobotDescription } from '../../core/types';
import { publishDebugEvent } from '../../app/debugBus';
import { SIMULATION_LAYER } from '../../viz/sceneLayers';
import { CAMERA_IMAGE_SIZE } from './CameraSensor';
import type { CupBodyRef } from '../tasks/CupArrangementScene';

export interface SimCameraFrame {
  timestamp: number;
  rgb: Uint8Array;
  eefPose: Pose;
  jointValues: JointValues;
  gripperWidth: number;
}

interface WristCameraCaptureProps {
  robot: URDFRobot;
  description: RobotDescription;
  eefPose: Pose;
  jointValues: JointValues;
  enabled: boolean;
  inferenceActive: boolean;
  cupBodyRef: CupBodyRef;
  onCapture: (frame: SimCameraFrame) => void;
}

const IMAGE_SIZE = CAMERA_IMAGE_SIZE;
const CAPTURE_INTERVAL_MS = 50;
const CAMERA_MOUNT_POSITION: [number, number, number] = [0, 0, 0.055];
const CAMERA_MOUNT_ROTATION: [number, number, number] = [0, 1.355 - Math.PI / 2, 0];

const getGripperWidth = (description: RobotDescription, jointValues: JointValues) =>
  description.joints
    .filter(
      (joint) =>
        joint.type === 'prismatic' && /gripper|finger/iu.test(joint.name + ' ' + joint.child),
    )
    .reduce((width, joint) => {
      const source = joint.mimic
        ? description.joints.find((candidate) => candidate.name === joint.mimic?.joint)
        : null;
      const baseline = joint.mimic
        ? (source?.limit?.lower ?? 0) * joint.mimic.multiplier + joint.mimic.offset
        : (joint.limit?.lower ?? 0);
      return width + Math.abs((jointValues[joint.name] ?? baseline) - baseline);
    }, 0);

export const WristCameraCapture = ({
  robot,
  description,
  eefPose,
  jointValues,
  enabled,
  inferenceActive,
  cupBodyRef,
  onCapture,
}: WristCameraCaptureProps) => {
  const { gl, scene } = useThree();
  const camera = useMemo(() => {
    const sensorCamera = new PerspectiveCamera(60, 1, 0.01, 5);
    sensorCamera.position.set(...CAMERA_MOUNT_POSITION);
    sensorCamera.rotation.set(...CAMERA_MOUNT_ROTATION);
    sensorCamera.layers.set(SIMULATION_LAYER);
    sensorCamera.updateProjectionMatrix();
    return sensorCamera;
  }, []);
  const target = useMemo(() => new WebGLRenderTarget(IMAGE_SIZE, IMAGE_SIZE), []);
  target.texture.colorSpace = SRGBColorSpace;
  const pixels = useMemo(() => new Uint8Array(IMAGE_SIZE * IMAGE_SIZE * 4), []);
  const rgb = useMemo(() => new Uint8Array(IMAGE_SIZE * IMAGE_SIZE * 3), []);
  const cupTarget = useMemo(() => new Vector3(), []);
  const cadence = useMemo(() => ({ lastCapture: 0 }), []);
  const layerAudit = useMemo(() => ({ signature: '' }), []);

  useEffect(() => {
    const link = robot.links[description.tipLink] ?? robot.links.end_link;
    if (!link) throw new Error(`URDF 缺少末端 link：${description.tipLink}`);
    link.add(camera);
    return () => {
      link.remove(camera);
    };
  }, [camera, description.tipLink, robot]);

  useEffect(() => () => target.dispose(), [target]);

  useFrame(({ clock }) => {
    if (!enabled || clock.elapsedTime * 1000 - cadence.lastCapture < CAPTURE_INTERVAL_MS) {
      return;
    }
    cadence.lastCapture = clock.elapsedTime * 1000;
    scene.updateMatrixWorld(true);
    const debugObjects: { name: string; type: string; visibleToWristCamera: boolean }[] = [];
    scene.traverse((object) => {
      if (object.userData.robotSimLayer !== 'debug') return;
      debugObjects.push({
        name: object.name || '(unnamed)',
        type: object.type,
        visibleToWristCamera: camera.layers.test(object.layers),
      });
    });
    const layerAuditSignature = JSON.stringify(debugObjects);
    if (layerAudit.signature !== layerAuditSignature) {
      publishDebugEvent('camera:wrist_debug_layer_audit', {
        cameraMask: camera.layers.mask,
        simulationLayer: SIMULATION_LAYER,
        debugObjects,
      });
      layerAudit.signature = layerAuditSignature;
    }
    const cupPosition = cupBodyRef.current?.translation();
    if (inferenceActive && cupPosition) {
      cupTarget.set(cupPosition.x, cupPosition.y + 0.039, cupPosition.z);
      camera.lookAt(cupTarget);
      camera.updateMatrixWorld(true);
    } else {
      camera.rotation.set(...CAMERA_MOUNT_ROTATION);
      camera.updateMatrixWorld(true);
    }
    const previousTarget = gl.getRenderTarget();
    const previousXr = gl.xr.enabled;
    gl.xr.enabled = false;
    try {
      gl.setRenderTarget(target);
      gl.render(scene, camera);
      gl.readRenderTargetPixels(target, 0, 0, IMAGE_SIZE, IMAGE_SIZE, pixels);
    } finally {
      gl.setRenderTarget(previousTarget);
      gl.xr.enabled = previousXr;
    }

    for (let y = 0; y < IMAGE_SIZE; y += 1) {
      for (let x = 0; x < IMAGE_SIZE; x += 1) {
        const source = ((IMAGE_SIZE - y - 1) * IMAGE_SIZE + x) * 4;
        const destination = (y * IMAGE_SIZE + x) * 3;
        rgb[destination] = pixels[source];
        rgb[destination + 1] = pixels[source + 1];
        rgb[destination + 2] = pixels[source + 2];
      }
    }
    onCapture({
      timestamp: performance.now(),
      rgb: rgb.slice(),
      eefPose: { position: [...eefPose.position], orientation: [...eefPose.orientation] },
      jointValues: { ...jointValues },
      gripperWidth: getGripperWidth(description, jointValues),
    });
  }, -1);

  return null;
};
