import { useCallback, useEffect, useRef, useState } from 'react';
import { JointController } from '../core/JointController';
import { Kinematics } from '../core/Kinematics';
import { RobotModel } from '../core/RobotModel';
import type {
  ControlMode,
  IKResult,
  JointCommand,
  JointValues,
  Pose,
  RobotDescription,
} from '../core/types';
import { parseUrdf } from '../utils/urdfParser';
import { publishDebugEvent } from './debugBus';
import { useInferenceReplay } from './useInferenceReplay';
import { findRobotConfig, ROBOT_MODELS, SIMULATION_CONFIG, type RobotModelId } from './config';
import { DebugOverlay } from '../viz/DebugOverlay';
import { SceneManager } from '../viz/SceneManager';
import { JointPanel } from '../ui/JointPanel';
import { StatusBar } from '../ui/StatusBar';
import { TaskPanel } from '../ui/TaskPanel';
import { InferencePanel } from '../ui/InferencePanel';
import { quaternionToRotationVector } from '../utils/math';
import type { InferenceObservation } from './inferenceProtocol';
import type { SimCameraFrame } from '../sim/sensors/WristCameraCapture';
import { CAMERA_IMAGE_SIZE } from '../sim/sensors/CameraSensor';

interface LoadedRobot {
  description: RobotDescription;
  controller: JointController;
  kinematics: Kinematics;
  model: RobotModel;
}

const DEFAULT_TARGET: Pose = {
  position: [0.35, 0, 0.45],
  orientation: [0, 0, 0, 1],
};
const CAMERA_PREVIEW_INTERVAL_MS = 100;

export const App = () => {
  const [modelId, setModelId] = useState<RobotModelId>('RS');
  const [loadedRobot, setLoadedRobot] = useState<LoadedRobot | null>(null);
  const [loadError, setLoadError] = useState('');
  const [jointValues, setJointValues] = useState<JointValues>({});
  const [mode, setMode] = useState<ControlMode>('position');
  const [commands, setCommands] = useState<Record<string, JointCommand>>({});
  const [isRunning, setIsRunning] = useState(true);
  const [targetPose, setTargetPose] = useState<Pose>(DEFAULT_TARGET);
  const [ikResult, setIkResult] = useState<IKResult | null>(null);
  const [ikMessage, setIkMessage] = useState('');
  const [physicsReady, setPhysicsReady] = useState(false);
  const [cameraPreviewReady, setCameraPreviewReady] = useState(false);
  const cameraFramesRef = useRef<SimCameraFrame[]>([]);
  const cameraPreviewCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const cameraPreviewImageDataRef = useRef<ImageData | null>(null);
  const cameraPreviewLastDrawRef = useRef(0);
  const cameraPreviewReadyRef = useRef(false);
  const observationIndexRef = useRef(0);

  const currentModel = findRobotConfig(modelId);

  useEffect(() => {
    let active = true;
    setLoadedRobot(null);
    setLoadError('');
    setJointValues({});
    setCommands({});
    setPhysicsReady(false);
    setCameraPreviewReady(false);
    cameraPreviewReadyRef.current = false;
    cameraPreviewImageDataRef.current = null;
    cameraPreviewLastDrawRef.current = 0;
    cameraFramesRef.current = [];

    const loadRobot = async () => {
      try {
        const url = `${SIMULATION_CONFIG.robotAssetRoot}/${modelId}/urdf/${currentModel.file}`;
        const response = await fetch(url);
        if (!response.ok) throw new Error(`URDF 加载失败：HTTP ${response.status}`);
        const description = parseUrdf(await response.text());
        const model = new RobotModel(description);
        if (!active) return;

        setLoadedRobot({
          description,
          controller: new JointController(model),
          kinematics: new Kinematics(description),
          model,
        });
        setJointValues(model.getJointValues());
        publishDebugEvent('robot:loaded', { model: modelId, name: description.name });
      } catch (error) {
        if (!active) return;
        const message = error instanceof Error ? error.message : String(error);
        setLoadError(message);
        publishDebugEvent('robot:error', { model: modelId, message });
      }
    };

    void loadRobot();
    return () => {
      active = false;
    };
  }, [currentModel.file, modelId]);

  const tcpPose = loadedRobot?.model.getLinkPose() ?? null;
  const handleCameraFrame = useCallback((frame: SimCameraFrame) => {
    cameraFramesRef.current = [...cameraFramesRef.current.slice(-1), frame];

    const now = performance.now();
    if (now - cameraPreviewLastDrawRef.current < CAMERA_PREVIEW_INTERVAL_MS) return;

    const canvas = cameraPreviewCanvasRef.current;
    const context = canvas?.getContext('2d');
    if (!canvas || !context) return;

    const imageData =
      cameraPreviewImageDataRef.current ??
      context.createImageData(CAMERA_IMAGE_SIZE, CAMERA_IMAGE_SIZE);
    cameraPreviewImageDataRef.current = imageData;
    for (let sourceIndex = 0, targetIndex = 0; sourceIndex < frame.rgb.length; sourceIndex += 3) {
      imageData.data[targetIndex] = frame.rgb[sourceIndex];
      imageData.data[targetIndex + 1] = frame.rgb[sourceIndex + 1];
      imageData.data[targetIndex + 2] = frame.rgb[sourceIndex + 2];
      imageData.data[targetIndex + 3] = 255;
      targetIndex += 4;
    }
    context.putImageData(imageData, 0, 0);
    cameraPreviewLastDrawRef.current = now;

    if (!cameraPreviewReadyRef.current) {
      cameraPreviewReadyRef.current = true;
      setCameraPreviewReady(true);
    }
  }, []);
  const getObservation = useCallback((): InferenceObservation | null => {
    const frames = cameraFramesRef.current;
    const latest = frames[frames.length - 1];
    if (!latest || frames.length === 0) return null;
    const history = frames.length > 1 ? frames.slice(-2) : [latest, latest];
    let binary = '';
    for (let offset = 0; offset < latest.rgb.length; offset += 0x8000) {
      binary += String.fromCharCode(...latest.rgb.subarray(offset, offset + 0x8000));
    }
    return {
      frame_index: observationIndexRef.current++,
      camera0_rgb: btoa(binary),
      robot0_eef_pos: history.map((frame) => [...frame.eefPose.position]),
      robot0_eef_rot_axis_angle: history.map((frame) =>
        quaternionToRotationVector(frame.eefPose.orientation),
      ),
      robot0_gripper_width: history.map((frame) => [frame.gripperWidth]),
    };
  }, []);
  const inference = useInferenceReplay({
    description: loadedRobot?.description ?? null,
    jointValues,
    tcpPose,
    getObservation,
  });

  const setControlMode = (nextMode: ControlMode) => {
    if (inference.isLocked) return;
    setMode(nextMode);
    loadedRobot?.controller.setMode(nextMode);
    setCommands(loadedRobot?.controller.getCommands() ?? {});
    publishDebugEvent('control:mode', { mode: nextMode });
  };

  const commandJoint = (jointName: string, value: number) => {
    if (!loadedRobot || inference.isLocked) return;
    try {
      const command = loadedRobot.controller.setCommand(jointName, value);
      setCommands(loadedRobot.controller.getCommands());
      if (command.mode === 'position') {
        loadedRobot.model.setJointValue(jointName, command.value);
        const nextValues = loadedRobot.model.getJointValues();
        setJointValues(nextValues);
        publishDebugEvent('joint:command', { joint: jointName, ...command });
      } else {
        publishDebugEvent('joint:command', { joint: jointName, ...command });
      }
    } catch (error) {
      setIkMessage(error instanceof Error ? error.message : String(error));
    }
  };

  const updatePhysicsState = (values: JointValues) => {
    if (!loadedRobot) return;
    loadedRobot.model.setJointValues(values);
    setJointValues(loadedRobot.model.getJointValues());
  };

  const solveIk = () => {
    if (!loadedRobot || inference.isLocked) return;
    const result = loadedRobot.kinematics.solveIK(targetPose, jointValues);
    setIkResult(result);
    setIkMessage(
      result.converged
        ? `收敛于 ${result.iterations} 次迭代`
        : `未收敛，位置残差 ${result.residual.position.toFixed(4)} m`,
    );
    publishDebugEvent('ik:result', result);
    if (result.converged) {
      setControlMode('position');
      loadedRobot.model.setJointValues(result.jointValues);
      const nextValues = loadedRobot.model.getJointValues();
      setJointValues(nextValues);
      loadedRobot.model.getControllableJoints().forEach((joint) => {
        loadedRobot.controller.setCommand(joint.name, nextValues[joint.name]);
      });
      setCommands(loadedRobot.controller.getCommands());
    }
  };

  const applyReplayStep = (values: JointValues) => {
    if (!loadedRobot) return;
    loadedRobot.model.setJointValues(values);
    const nextValues = loadedRobot.model.getJointValues();
    loadedRobot.model.getControllableJoints().forEach((joint) => {
      loadedRobot.controller.setCommand(joint.name, nextValues[joint.name]);
    });
    setJointValues(nextValues);
    setCommands(loadedRobot.controller.getCommands());
    publishDebugEvent('inference:action_applied', { joints: nextValues });
  };

  const setTargetCoordinate = (field: 'position' | 'orientation', index: number, value: number) => {
    setTargetPose((current) => {
      const next = [...current[field]] as number[];
      next[index] = value;
      return { ...current, [field]: next } as Pose;
    });
  };

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand-lockup">
          <div className="brand-mark">
            <span />
            <span />
            <span />
          </div>
          <div>
            <p className="eyebrow">REBOT ARM LAB</p>
            <h1>
              Robot Sim <span>Studio</span>
            </h1>
          </div>
        </div>
        <div className="topbar-center">
          <span className="status-dot" />
          <span>本地仿真环境</span>
          <span className="topbar-divider" />
          <span className="muted">
            固定步长 {Math.round(SIMULATION_CONFIG.fixedTimeStep * 1000)} ms
          </span>
        </div>
        <div className="model-switch" aria-label="机械臂型号">
          {ROBOT_MODELS.map((model) => (
            <button
              className={model.id === modelId ? 'model-tab active' : 'model-tab'}
              key={model.id}
              disabled={inference.isLocked}
              onClick={() => setModelId(model.id)}
              type="button"
            >
              {model.id}
            </button>
          ))}
        </div>
      </header>

      <div className="workspace">
        <aside className="control-column">
          <section className="panel robot-summary">
            <div className="summary-heading">
              <div>
                <p className="eyebrow">ACTIVE ROBOT</p>
                <h2>{currentModel.label}</h2>
              </div>
              <span className="model-badge">{modelId}</span>
            </div>
            <div className="summary-meta">
              <span>
                <i className="mini-square" />6 DOF
              </span>
              <span>
                <i className="mini-square cyan" />
                URDF
              </span>
              <span>
                <i className="mini-square violet" />
                Rapier
              </span>
            </div>
          </section>

          <InferencePanel
            canStart={Boolean(loadedRobot && physicsReady && !loadError)}
            error={inference.error}
            isLocked={inference.isLocked}
            onStart={() => {
              if (mode !== 'position') setControlMode('position');
              setIsRunning(true);
              inference.start();
            }}
            onStop={inference.stop}
            progress={inference.progress}
            status={inference.status}
          />

          {loadError ? (
            <section className="panel error-panel">
              <p className="panel-label">模型加载失败</p>
              <p>{loadError}</p>
              <button
                className="secondary-button"
                onClick={() => setModelId(modelId)}
                type="button"
              >
                重试
              </button>
            </section>
          ) : null}

          <JointPanel
            commands={commands}
            joints={loadedRobot?.model.getControllableJoints() ?? []}
            jointValues={jointValues}
            mode={mode}
            onCommand={commandJoint}
            onModeChange={setControlMode}
            disabled={inference.isLocked}
          />

          <TaskPanel
            disabled={inference.isLocked}
            ikMessage={ikMessage}
            ikResult={ikResult}
            onSolveIk={solveIk}
            onTargetChange={setTargetCoordinate}
            onUseCurrentPose={() => {
              if (tcpPose) setTargetPose(tcpPose);
            }}
            targetPose={targetPose}
          />
        </aside>

        <section className="viewport-column">
          <div className="viewport-toolbar">
            <div className="viewport-title">
              <span className="viewport-icon">⌘</span>
              <div>
                <strong>仿真视口</strong>
                <span>交互旋转 · 缩放 · 平移</span>
              </div>
            </div>
            <div className="viewport-actions">
              <span className={physicsReady ? 'physics-pill ready' : 'physics-pill'}>
                <i />
                {physicsReady ? 'Physics ready' : 'Physics loading'}
              </span>
              <button
                className="icon-button"
                title={isRunning ? '暂停仿真' : '运行仿真'}
                disabled={inference.isLocked}
                onClick={() => setIsRunning(!isRunning)}
                type="button"
              >
                {isRunning ? 'Ⅱ' : '▶'}
              </button>
              <button
                className="icon-button"
                disabled={inference.isLocked}
                title="重置关节"
                onClick={() => {
                  if (!loadedRobot) return;
                  loadedRobot.controller.setMode('position');
                  loadedRobot.model.setJointValues({});
                  const nextValues = loadedRobot.model.getJointValues();
                  loadedRobot.model.getControllableJoints().forEach((joint) => {
                    loadedRobot.controller.setCommand(joint.name, nextValues[joint.name]);
                  });
                  setMode('position');
                  setJointValues(nextValues);
                  setCommands(loadedRobot.controller.getCommands());
                }}
                type="button"
              >
                ↺
              </button>
            </div>
          </div>

          <div className="viewport-frame">
            <div className="viewport-background-grid" />
            <section className="camera-preview" aria-label="腕部相机预览">
              <div className="camera-preview-header">
                <div>
                  <p className="eyebrow">WRIST CAMERA</p>
                  <h2>相机画面</h2>
                </div>
                <span
                  className={
                    cameraPreviewReady ? 'camera-preview-status ready' : 'camera-preview-status'
                  }
                >
                  {cameraPreviewReady ? '实时' : '等待'}
                </span>
              </div>
              <div className="camera-preview-frame">
                <canvas
                  ref={cameraPreviewCanvasRef}
                  width={CAMERA_IMAGE_SIZE}
                  height={CAMERA_IMAGE_SIZE}
                  aria-label="机械臂腕部相机实时画面"
                  role="img"
                />
                {!cameraPreviewReady ? <span>等待腕部相机图像</span> : null}
              </div>
              <div className="camera-preview-meta">224 × 224 RGB</div>
            </section>
            {loadedRobot ? (
              <SceneManager
                commands={commands}
                description={loadedRobot.description}
                isRunning={isRunning}
                mode={mode}
                jointValues={jointValues}
                modelId={modelId}
                onJointState={updatePhysicsState}
                onPhysicsReady={setPhysicsReady}
                playback={inference.playback}
                onPlaybackStep={applyReplayStep}
                onPlaybackComplete={inference.completePlayback}
                cameraEnabled={Boolean(loadedRobot)}
                tcpPose={tcpPose ?? DEFAULT_TARGET}
                onCameraFrame={handleCameraFrame}
              />
            ) : (
              <div className="loading-state">
                <div className="loading-orbit">
                  <span />
                </div>
                <strong>{loadError ? '等待模型资源' : '正在加载机械臂'}</strong>
                <span>
                  {loadError ? '请检查 ROS 描述目录和资源同步' : '解析 URDF 几何与关节树…'}
                </span>
              </div>
            )}
            <DebugOverlay jointValues={jointValues} pose={tcpPose} />
            <div className="view-axis">
              <span>X</span>
              <span>Y</span>
              <span>Z</span>
              <div />
            </div>
          </div>

          <StatusBar
            isRunning={isRunning}
            jointCount={loadedRobot?.model.getControllableJoints().length ?? 0}
            modelId={modelId}
            pose={tcpPose}
          />
        </section>
      </div>
    </main>
  );
};
