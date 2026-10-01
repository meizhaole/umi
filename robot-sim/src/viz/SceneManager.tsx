import { Suspense, useCallback, useEffect, useMemo } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Physics } from '@react-three/rapier';
import { SIMULATION_CONFIG, findRobotConfig, type RobotModelId } from '../app/config';
import type { PlaybackChunk } from '../app/inferenceProtocol';
import type { ControlMode, JointCommand, JointValues, RobotDescription } from '../core/types';
import { PhysicsWorld } from '../sim/PhysicsWorld';
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
  onPlaybackStep: (values: JointValues) => void;
  onPlaybackComplete: (token: number) => void;
  cameraEnabled: boolean;
  tcpPose: Pose;
  onCameraFrame: (frame: SimCameraFrame) => void;
}

const PLAYBACK_STEP_SECONDS = 0.05;

interface ActionPlaybackProps {
  playback: PlaybackChunk | null;
  onStep: (values: JointValues) => void;
  onComplete: (token: number) => void;
}

const ActionPlayback = ({ playback, onStep, onComplete }: ActionPlaybackProps) => {
  const accumulator = useMemo(() => ({ seconds: 0, index: 0, token: 0 }), []);

  useFrame((_, delta) => {
    if (!playback) return;
    if (accumulator.token !== playback.token) {
      accumulator.seconds = 0;
      accumulator.index = 0;
      accumulator.token = playback.token;
    }

    accumulator.seconds += Math.max(0, delta);
    while (accumulator.seconds >= PLAYBACK_STEP_SECONDS) {
      const action = playback.actions[accumulator.index];
      if (!action) break;
      accumulator.seconds -= PLAYBACK_STEP_SECONDS;
      onStep(action.jointValues);
      accumulator.index += 1;
    }

    if (accumulator.index === playback.actions.length) {
      accumulator.seconds = 0;
      onComplete(playback.token);
    }
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
      <ActionPlayback onComplete={onPlaybackComplete} onStep={onPlaybackStep} playback={playback} />
      <Physics
        colliders={false}
        gravity={SIMULATION_CONFIG.gravity}
        paused={!isRunning}
        timeStep={SIMULATION_CONFIG.fixedTimeStep}
      >
        <PhysicsWorld />
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
