import { useEffect, useMemo } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { Group, PerspectiveCamera, SRGBColorSpace, WebGLRenderTarget } from 'three';
import type { URDFRobot } from 'urdf-loader';
import type { JointValues, Pose, RobotDescription } from '../../core/types';
import { publishDebugEvent } from '../../app/debugBus';
import { SIMULATION_LAYER } from '../../viz/sceneLayers';
import { CAMERA_IMAGE_SIZE } from './CameraSensor';
import { WRIST_CAMERA_CONFIG } from './wristCameraConfig';

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
  onCapture: (frame: SimCameraFrame) => void;
}

const CAPTURE_INTERVAL_MS = 50;

const createCanvasPipeline = (captureWidth: number, captureHeight: number) => {
  const sourceCanvas = document.createElement('canvas');
  sourceCanvas.width = captureWidth;
  sourceCanvas.height = captureHeight;
  const sourceContext = sourceCanvas.getContext('2d', { willReadFrequently: true });
  const outputCanvas = document.createElement('canvas');
  outputCanvas.width = CAMERA_IMAGE_SIZE;
  outputCanvas.height = CAMERA_IMAGE_SIZE;
  const outputContext = outputCanvas.getContext('2d', { willReadFrequently: true });

  if (!sourceContext || !outputContext) {
    throw new Error('无法创建腕部相机裁剪画布');
  }

  return {
    sourceCanvas,
    sourceContext,
    sourceImageData: sourceContext.createImageData(captureWidth, captureHeight),
    outputContext,
    cropSize: Math.min(captureWidth, captureHeight),
    cropLeft: Math.floor((captureWidth - Math.min(captureWidth, captureHeight)) / 2),
    cropTop: Math.floor((captureHeight - Math.min(captureWidth, captureHeight)) / 2),
  };
};

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
  onCapture,
}: WristCameraCaptureProps) => {
  const { gl, scene } = useThree();
  const isGoProMount = Boolean(robot.links.mount_link);
  const captureWidth = isGoProMount ? WRIST_CAMERA_CONFIG.captureWidth : CAMERA_IMAGE_SIZE;
  const captureHeight = isGoProMount ? WRIST_CAMERA_CONFIG.captureHeight : CAMERA_IMAGE_SIZE;
  const camera = useMemo(() => {
    const sensorCamera = new PerspectiveCamera(
      WRIST_CAMERA_CONFIG.fieldOfViewDegrees,
      captureWidth / captureHeight,
      WRIST_CAMERA_CONFIG.near,
      WRIST_CAMERA_CONFIG.far,
    );
    sensorCamera.layers.set(SIMULATION_LAYER);
    sensorCamera.updateProjectionMatrix();
    return sensorCamera;
  }, []);
  const cameraAnchor = useMemo(() => new Group(), []);
  const target = useMemo(
    () => new WebGLRenderTarget(captureWidth, captureHeight),
    [captureHeight, captureWidth],
  );
  target.texture.colorSpace = SRGBColorSpace;
  const pixels = useMemo(
    () => new Uint8Array(captureWidth * captureHeight * 4),
    [captureHeight, captureWidth],
  );
  const rgb = useMemo(() => new Uint8Array(CAMERA_IMAGE_SIZE * CAMERA_IMAGE_SIZE * 3), []);
  const canvasPipeline = useMemo(
    () => (isGoProMount ? createCanvasPipeline(captureWidth, captureHeight) : null),
    [captureHeight, captureWidth, isGoProMount],
  );
  const cadence = useMemo(() => ({ lastCapture: 0 }), []);
  const layerAudit = useMemo(() => ({ signature: '' }), []);

  useEffect(() => {
    const mountLink = robot.links.mount_link;
    const parentLink = mountLink ?? robot.links[description.tipLink] ?? robot.links.end_link;
    if (!parentLink) throw new Error(`URDF 缺少末端 link：${description.tipLink}`);

    camera.aspect = captureWidth / captureHeight;
    camera.updateProjectionMatrix();
    cameraAnchor.name = mountLink ? 'gopro_camera_anchor' : 'wrist_camera_anchor';
    if (mountLink) {
      cameraAnchor.position.set(...WRIST_CAMERA_CONFIG.goproVisualOrigin);
      cameraAnchor.rotation.set(...WRIST_CAMERA_CONFIG.goproVisualRotation);
      camera.position.set(...WRIST_CAMERA_CONFIG.goproCameraPosition);
      camera.quaternion.set(...WRIST_CAMERA_CONFIG.goproCameraQuaternion);
    } else {
      cameraAnchor.position.set(...WRIST_CAMERA_CONFIG.legacyMountPosition);
      cameraAnchor.rotation.set(...WRIST_CAMERA_CONFIG.legacyMountRotation);
      camera.position.set(0, 0, 0);
      camera.quaternion.identity();
    }
    cameraAnchor.add(camera);
    parentLink.add(cameraAnchor);
    publishDebugEvent('camera:wrist_mount', {
      parentLink: mountLink ? 'mount_link' : description.tipLink,
      anchor: cameraAnchor.name,
      opticalCenterStatus: mountLink ? 'geometric_proxy_requires_calibration' : 'legacy_mount',
      localCameraPosition: camera.position.toArray(),
      localCameraQuaternion: camera.quaternion.toArray(),
      renderResolution: [captureWidth, captureHeight],
      centerCrop: canvasPipeline
        ? {
            x: canvasPipeline.cropLeft,
            y: canvasPipeline.cropTop,
            width: canvasPipeline.cropSize,
            height: canvasPipeline.cropSize,
          }
        : { x: 0, y: 0, width: CAMERA_IMAGE_SIZE, height: CAMERA_IMAGE_SIZE },
      policyResolution: [CAMERA_IMAGE_SIZE, CAMERA_IMAGE_SIZE],
      fieldOfViewDegrees: WRIST_CAMERA_CONFIG.fieldOfViewDegrees,
      near: WRIST_CAMERA_CONFIG.near,
      far: WRIST_CAMERA_CONFIG.far,
    });

    return () => {
      parentLink.remove(cameraAnchor);
    };
  }, [
    camera,
    cameraAnchor,
    canvasPipeline,
    captureHeight,
    captureWidth,
    description.tipLink,
    robot,
  ]);

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

    const previousTarget = gl.getRenderTarget();
    const previousXr = gl.xr.enabled;
    gl.xr.enabled = false;
    try {
      gl.setRenderTarget(target);
      gl.render(scene, camera);
      gl.readRenderTargetPixels(target, 0, 0, captureWidth, captureHeight, pixels);
    } finally {
      gl.setRenderTarget(previousTarget);
      gl.xr.enabled = previousXr;
    }

    if (canvasPipeline) {
      for (let y = 0; y < captureHeight; y += 1) {
        const sourceStart = (captureHeight - y - 1) * captureWidth * 4;
        const targetStart = y * captureWidth * 4;
        canvasPipeline.sourceImageData.data.set(
          pixels.subarray(sourceStart, sourceStart + captureWidth * 4),
          targetStart,
        );
      }
      canvasPipeline.sourceContext.putImageData(canvasPipeline.sourceImageData, 0, 0);
      canvasPipeline.outputContext.drawImage(
        canvasPipeline.sourceCanvas,
        canvasPipeline.cropLeft,
        canvasPipeline.cropTop,
        canvasPipeline.cropSize,
        canvasPipeline.cropSize,
        0,
        0,
        CAMERA_IMAGE_SIZE,
        CAMERA_IMAGE_SIZE,
      );
      const outputPixels = canvasPipeline.outputContext.getImageData(
        0,
        0,
        CAMERA_IMAGE_SIZE,
        CAMERA_IMAGE_SIZE,
      ).data;
      for (let source = 0, destination = 0; source < outputPixels.length; source += 4) {
        rgb[destination] = outputPixels[source];
        rgb[destination + 1] = outputPixels[source + 1];
        rgb[destination + 2] = outputPixels[source + 2];
        destination += 3;
      }
    } else {
      for (let y = 0; y < CAMERA_IMAGE_SIZE; y += 1) {
        for (let x = 0; x < CAMERA_IMAGE_SIZE; x += 1) {
          const source = ((CAMERA_IMAGE_SIZE - y - 1) * CAMERA_IMAGE_SIZE + x) * 4;
          const destination = (y * CAMERA_IMAGE_SIZE + x) * 3;
          rgb[destination] = pixels[source];
          rgb[destination + 1] = pixels[source + 1];
          rgb[destination + 2] = pixels[source + 2];
        }
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
