import { Suspense, useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import type { RapierRigidBody } from '@react-three/rapier';
import { Quaternion, Vector3 } from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Physics } from '@react-three/rapier';
import { SIMULATION_CONFIG, findRobotConfig, type RobotModelId } from '../app/config';
import type { PlaybackChunk } from '../app/inferenceProtocol';
import type { ControlMode, JointCommand, JointValues, RobotDescription } from '../core/types';
import { PhysicsWorld } from '../sim/PhysicsWorld';
import { OfficialCupArrangementScene, type CupBodyRef } from '../sim/tasks/CupArrangementScene';
import {
  RobotBody,
  type Wrist3ExperimentTrace,
  type Wrist3PhysicsDebugState,
} from '../sim/RobotBody';
import { publishDebugEvent } from '../app/debugBus';
import { URDFViewer } from './URDFViewer';
import { DEBUG_LAYER, setDebugLayer } from './sceneLayers';
import type { SimCameraFrame } from '../sim/sensors/WristCameraCapture';
import type { Pose } from '../core/types';

interface SceneManagerProps {
  modelId: RobotModelId;
  description: RobotDescription;
  urdfXml: string;
  jointValues: JointValues;
  commands: Record<string, JointCommand>;
  mode: ControlMode;
  isRunning: boolean;
  playback: PlaybackChunk | null;
  inferenceActive: boolean;
  positionExecutionOverride?: 'joint_motors' | 'kinematic_fk';
  datasetReplayActive?: boolean;
  datasetReplayTargetPose?: Pose | null;
  datasetReplayActualPose?: Pose | null;
  onJointState: (values: JointValues) => void;
  onPhysicsReady: (ready: boolean) => void;
  onPlaybackStep: (values: JointValues, isFinal: boolean) => void;
  onPlaybackComplete: (token: number) => void;
  cameraEnabled: boolean;
  tcpPose: Pose;
  onCameraFrame: (frame: SimCameraFrame) => void;
  onWrist3DebugState: (state: Wrist3PhysicsDebugState | null) => void;
  onWrist3ExperimentComplete: (trace: Wrist3ExperimentTrace) => void;
}

const PLAYBACK_ACTION_SECONDS = 0.5;
const MAX_PLAYBACK_DELTA_SECONDS = 0.05;
const ROBOT_TO_SCENE = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), -Math.PI / 2);
const ZERO_GRAVITY: [number, number, number] = [0, 0, 0];

interface DatasetPoseMarkersProps {
  targetPose: Pose | null;
  actualPose: Pose | null;
}

const DatasetPoseMarkers = ({ targetPose, actualPose }: DatasetPoseMarkersProps) => {
  const targetPosition = targetPose
    ? new Vector3(...targetPose.position).applyQuaternion(ROBOT_TO_SCENE).toArray()
    : null;
  const marker = (pose: Pose | null, name: string, color: string, radius: number) => {
    if (!pose) return null;
    const position = new Vector3(...pose.position).applyQuaternion(ROBOT_TO_SCENE);
    return (
      <mesh ref={setDebugLayer} name={name} position={position.toArray()}>
        <sphereGeometry args={[radius, 20, 16]} />
        <meshBasicMaterial color={color} depthTest={false} toneMapped={false} />
      </mesh>
    );
  };

  return (
    <group renderOrder={10}>
      {targetPosition ? (
        <mesh
          ref={setDebugLayer}
          name="dataset-target-marker-ring"
          position={targetPosition}
          renderOrder={10}
        >
          <torusGeometry args={[0.035, 0.0025, 8, 32]} />
          <meshBasicMaterial color="#63e6d0" depthTest={false} toneMapped={false} />
        </mesh>
      ) : null}
      {marker(targetPose, 'dataset-target-marker', '#63e6d0', 0.025)}
      {marker(actualPose, 'dataset-actual-marker', '#ffb45e', 0.015)}
    </group>
  );
};

interface ActionPlaybackProps {
  playback: PlaybackChunk | null;
  jointValues: JointValues;
  onStep: (values: JointValues, isFinal: boolean) => void;
  onComplete: (token: number) => void;
}

const ActionPlayback = ({ playback, jointValues, onStep, onComplete }: ActionPlaybackProps) => {
  const accumulator = useMemo(
    () => ({
      seconds: 0,
      index: 0,
      token: 0,
      startValues: {} as JointValues,
      targetValues: null as JointValues | null,
    }),
    [],
  );

  useFrame((_, delta) => {
    if (!playback) return;
    if (accumulator.token !== playback.token) {
      accumulator.seconds = 0;
      accumulator.index = 0;
      accumulator.token = playback.token;
      accumulator.startValues = { ...jointValues };
      accumulator.targetValues = null;
    }

    const action = playback.actions[accumulator.index];
    if (!action) {
      onComplete(playback.token);
      return;
    }
    if (!accumulator.targetValues) accumulator.targetValues = action.jointValues;

    accumulator.seconds += Math.min(Math.max(0, delta), MAX_PLAYBACK_DELTA_SECONDS);
    const progress = Math.min(accumulator.seconds / PLAYBACK_ACTION_SECONDS, 1);
    const easedProgress = progress * progress * (3 - 2 * progress);
    const interpolatedValues = Object.fromEntries(
      Object.entries(accumulator.targetValues).map(([jointName, targetValue]) => {
        const startValue = accumulator.startValues[jointName] ?? targetValue;
        return [jointName, startValue + (targetValue - startValue) * easedProgress];
      }),
    ) as JointValues;
    onStep(interpolatedValues, progress === 1);

    if (progress < 1) return;
    accumulator.seconds = 0;
    accumulator.index += 1;
    accumulator.startValues = { ...accumulator.targetValues };
    accumulator.targetValues = null;
    if (accumulator.index === playback.actions.length) onComplete(playback.token);
  });
  return null;
};

interface CameraControlsProps {
  cupBodyRef: CupBodyRef;
  inferenceActive: boolean;
  onControlsChange: (controls: OrbitControls | null) => void;
}

const CameraControls = ({ cupBodyRef, inferenceActive, onControlsChange }: CameraControlsProps) => {
  const { camera, gl } = useThree();
  const controls = useMemo(() => new OrbitControls(camera, gl.domElement), [camera, gl]);
  const desiredTarget = useMemo(() => new Vector3(), []);
  const panDelta = useMemo(() => new Vector3(), []);

  useEffect(() => {
    controls.enableDamping = false;
    controls.minDistance = 0.7;
    controls.maxDistance = 5.5;
    controls.target.set(0, 0.2, 0);
    onControlsChange(controls);
    return () => {
      onControlsChange(null);
      controls.dispose();
    };
  }, [controls, onControlsChange]);
  useFrame((_, delta) => {
    const cupPosition = cupBodyRef.current?.translation();
    if (inferenceActive && cupPosition) {
      desiredTarget.set(cupPosition.x, cupPosition.y + 0.039, cupPosition.z);
    } else {
      desiredTarget.set(0, 0.2, 0);
    }
    panDelta.subVectors(desiredTarget, controls.target).multiplyScalar(1 - Math.exp(-6 * delta));
    camera.position.add(panDelta);
    controls.target.add(panDelta);
    controls.update();
  });
  return null;
};

export const SceneManager = ({
  modelId,
  description,
  urdfXml,
  jointValues,
  commands,
  mode,
  isRunning,
  playback,
  inferenceActive,
  positionExecutionOverride,
  datasetReplayActive = false,
  datasetReplayTargetPose = null,
  datasetReplayActualPose = null,
  onJointState,
  onWrist3DebugState,
  onPhysicsReady,
  onPlaybackStep,
  onPlaybackComplete,
  cameraEnabled,
  tcpPose,
  onCameraFrame,
  onWrist3ExperimentComplete,
}: SceneManagerProps) => {
  const model = findRobotConfig(modelId);
  const diagnosticQuery = new URLSearchParams(
    typeof window === 'undefined' ? '' : window.location.search,
  );
  const phase2b23Diagnostics = diagnosticQuery.get('phase2b23') === '1' && model.type === 'ur5';
  const phase2b24Diagnostics = diagnosticQuery.get('phase2b24') === '1' && model.type === 'ur5';
  const phase2bDiagnostics = phase2b23Diagnostics || phase2b24Diagnostics;
  const diagnosticGravity =
    phase2bDiagnostics && diagnosticQuery.get('gravity') === 'off'
      ? ZERO_GRAVITY
      : SIMULATION_CONFIG.gravity;
  const diagnosticSolverIterationsValue = Number(diagnosticQuery.get('solverIterations'));
  const diagnosticSolverIterations =
    phase2b23Diagnostics &&
    diagnosticQuery.has('solverIterations') &&
    Number.isInteger(diagnosticSolverIterationsValue) &&
    diagnosticSolverIterationsValue > 0
      ? diagnosticSolverIterationsValue
      : undefined;
  const diagnosticCollisionOff = phase2bDiagnostics && diagnosticQuery.get('collisions') === 'off';
  const diagnosticLockUpstream =
    phase2b23Diagnostics && diagnosticQuery.get('lockUpstream') === '1';
  const robotDiagnostics = useMemo(
    () =>
      phase2bDiagnostics
        ? {
            gravity: diagnosticGravity,
            solverIterations: diagnosticSolverIterations,
            disableCollisions: diagnosticCollisionOff,
            lockUpstream: diagnosticLockUpstream,
            traceMotorWrites: phase2b24Diagnostics,
          }
        : undefined,
    [
      diagnosticCollisionOff,
      diagnosticGravity,
      diagnosticLockUpstream,
      diagnosticSolverIterations,
      phase2b24Diagnostics,
      phase2bDiagnostics,
    ],
  );
  const [readyModelId, setReadyModelId] = useState<RobotModelId | null>(null);
  const robotPhysicsReady = readyModelId === modelId;
  const controlsRef = useRef<OrbitControls | null>(null);
  const cupBodyRef = useRef<RapierRigidBody | null>(null);
  const handleControlsChange = useCallback((controls: OrbitControls | null) => {
    controlsRef.current = controls;
  }, []);
  const handleCupDragStateChange = useCallback((dragging: boolean) => {
    if (controlsRef.current) controlsRef.current.enabled = !dragging;
  }, []);
  const handleViewerError = useCallback((message: string) => {
    console.error(message);
  }, []);
  const handleViewerLoaded = useCallback(() => undefined, []);
  const handlePhysicsReady = useCallback(
    (ready: boolean) => {
      setReadyModelId((current) => {
        if (ready) return modelId;
        return current === modelId ? null : current;
      });
      onPhysicsReady(ready);
    },
    [modelId, onPhysicsReady],
  );

  return (
    <Canvas
      camera={{ position: [2.15, 1.55, 2.35], fov: 39, near: 0.01, far: 100 }}
      dpr={[1, 1.5]}
      gl={{ antialias: true, alpha: true }}
      onCreated={({ camera }) => {
        camera.layers.enable(DEBUG_LAYER);
        publishDebugEvent('scene:main_camera_layers', {
          mask: camera.layers.mask,
          simulationLayer: 0,
          debugLayer: DEBUG_LAYER,
        });
      }}
    >
      <color attach="background" args={[SIMULATION_CONFIG.background]} />
      <ambientLight intensity={0.68} />
      <directionalLight intensity={2.8} position={[2.8, 4.5, 3.6]} shadow-mapSize={[2048, 2048]} />
      <pointLight color="#59a6ff" intensity={13} position={[-2.4, 1.1, 1.4]} />
      <pointLight color="#5ce1c5" intensity={8} position={[1.5, 0.2, -2]} />
      <gridHelper
        ref={setDebugLayer}
        name="scene-debug-grid"
        args={[3.5, 35, '#315056', '#1d2c35']}
        position={[0, -0.012, 0]}
      />
      <axesHelper ref={setDebugLayer} name="scene-debug-axes" args={[0.3]} />
      <ActionPlayback
        jointValues={jointValues}
        onComplete={onPlaybackComplete}
        onStep={onPlaybackStep}
        playback={playback}
      />
      <Physics
        colliders={false}
        gravity={diagnosticGravity}
        numSolverIterations={diagnosticSolverIterations}
        paused={!isRunning || !robotPhysicsReady}
        timeStep={SIMULATION_CONFIG.fixedTimeStep}
      >
        <PhysicsWorld />
        <OfficialCupArrangementScene
          bodyRef={cupBodyRef}
          onDragStateChange={handleCupDragStateChange}
        />
        <RobotBody
          commands={commands}
          description={description}
          diagnostics={robotDiagnostics}
          packageMappings={model.packageMappings}
          positionExecution={
            phase2bDiagnostics
              ? 'joint_motors'
              : (positionExecutionOverride ?? model.positionExecution)
          }
          jointValues={jointValues}
          mode={mode}
          onJointState={onJointState}
          onWrist3DebugState={onWrist3DebugState}
          onWrist3ExperimentComplete={onWrist3ExperimentComplete}
          onReady={handlePhysicsReady}
        />
      </Physics>
      <Suspense fallback={null}>
        <URDFViewer
          jointValues={jointValues}
          onError={handleViewerError}
          onLoaded={handleViewerLoaded}
          description={description}
          cameraEnabled={cameraEnabled}
          cupBodyRef={cupBodyRef}
          inferenceActive={inferenceActive}
          tcpPose={tcpPose}
          onCameraFrame={onCameraFrame}
          packageMappings={model.packageMappings}
          urdfXml={urdfXml}
        />
      </Suspense>
      {datasetReplayActive ? (
        <DatasetPoseMarkers
          targetPose={datasetReplayTargetPose}
          actualPose={datasetReplayActualPose}
        />
      ) : null}
      <CameraControls
        cupBodyRef={cupBodyRef}
        inferenceActive={inferenceActive}
        onControlsChange={handleControlsChange}
      />
    </Canvas>
  );
};
