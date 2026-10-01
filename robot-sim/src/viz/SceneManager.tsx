import { Suspense, useCallback, useEffect, useMemo } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Physics } from '@react-three/rapier';
import { SIMULATION_CONFIG, findRobotConfig, type RobotModelId } from '../app/config';
import type { PlaybackChunk } from '../app/inferenceProtocol';
import type { ControlMode, JointCommand, JointValues, RobotDescription } from '../core/types';
import { PhysicsWorld } from '../sim/PhysicsWorld';
import { OfficialCupArrangementScene } from '../sim/tasks/CupArrangementScene';
import { RobotBody } from '../sim/RobotBody';
import { URDFViewer } from './URDFViewer';
import type { SimCameraFrame } from '../sim/sensors/WristCameraCapture';
import type { Pose } from '../core/types';

interface SceneManagerProps {
  modelId: RobotModelId;
  description: RobotDescription;
  jointValues: JointValues;
  commands: Record<string, JointCommand>;
  mode: ControlMode;
  isRunning: boolean;
  playback: PlaybackChunk | null;
  onJointState: (values: JointValues) => void;
  onPhysicsReady: (ready: boolean) => void;
  onPlaybackStep: (values: JointValues, isFinal: boolean) => void;
  onPlaybackComplete: (token: number) => void;
  cameraEnabled: boolean;
  tcpPose: Pose;
  onCameraFrame: (frame: SimCameraFrame) => void;
}

const PLAYBACK_ACTION_SECONDS = 0.5;
const MAX_PLAYBACK_DELTA_SECONDS = 0.05;

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

const CameraControls = () => {
  const { camera, gl } = useThree();
  const controls = useMemo(() => new OrbitControls(camera, gl.domElement), [camera, gl]);

  useEffect(() => {
    controls.enableDamping = true;
    controls.minDistance = 0.7;
    controls.maxDistance = 5.5;
    controls.target.set(0, 0.2, 0);
    return () => controls.dispose();
  }, [controls]);
  useFrame(() => controls.update());
  return null;
};

export const SceneManager = ({
  modelId,
  description,
  jointValues,
  commands,
  mode,
  isRunning,
  playback,
  onJointState,
  onPhysicsReady,
  onPlaybackStep,
  onPlaybackComplete,
  cameraEnabled,
  tcpPose,
  onCameraFrame,
}: SceneManagerProps) => {
  const model = findRobotConfig(modelId);
  const handleViewerError = useCallback((message: string) => {
    console.error(message);
  }, []);
  const handleViewerLoaded = useCallback(() => undefined, []);

  return (
    <Canvas
      camera={{ position: [2.15, 1.55, 2.35], fov: 39, near: 0.01, far: 100 }}
      dpr={[1, 1.5]}
      gl={{ antialias: true, alpha: true }}
    >
      <color attach="background" args={[SIMULATION_CONFIG.background]} />
      <ambientLight intensity={0.68} />
      <directionalLight intensity={2.8} position={[2.8, 4.5, 3.6]} shadow-mapSize={[2048, 2048]} />
      <pointLight color="#59a6ff" intensity={13} position={[-2.4, 1.1, 1.4]} />
      <pointLight color="#5ce1c5" intensity={8} position={[1.5, 0.2, -2]} />
      <gridHelper args={[3.5, 35, '#315056', '#1d2c35']} position={[0, -0.012, 0]} />
      <axesHelper args={[0.3]} />
      <ActionPlayback
        jointValues={jointValues}
        onComplete={onPlaybackComplete}
        onStep={onPlaybackStep}
        playback={playback}
      />
      <Physics
        colliders={false}
        gravity={SIMULATION_CONFIG.gravity}
        paused={!isRunning}
        timeStep={SIMULATION_CONFIG.fixedTimeStep}
      >
        <PhysicsWorld />
        <OfficialCupArrangementScene />
        <RobotBody
          commands={commands}
          description={description}
          jointValues={jointValues}
          mode={mode}
          onJointState={onJointState}
          onReady={onPhysicsReady}
        />
      </Physics>
      <Suspense fallback={null}>
        <URDFViewer
          jointValues={jointValues}
          modelId={modelId}
          onError={handleViewerError}
          onLoaded={handleViewerLoaded}
          description={description}
          cameraEnabled={cameraEnabled}
          tcpPose={tcpPose}
          onCameraFrame={onCameraFrame}
          urdfFile={model.file}
        />
      </Suspense>
      <CameraControls />
    </Canvas>
  );
};
