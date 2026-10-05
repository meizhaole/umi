import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Quaternion, Vector3 } from 'three';
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
import { configureRobotUrdf } from '../utils/configureRobotUrdf';
import { resolvePackageUris } from '../utils/resolvePackageUri.mjs';
import { publishDebugEvent } from './debugBus';
import { useInferenceReplay } from './useInferenceReplay';
import {
  DEFAULT_ROBOT_MODEL_ID,
  findRobotConfig,
  ROBOT_MODELS,
  SIMULATION_CONFIG,
  type RobotModelId,
} from './config';
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
import {
  DATASET_REPLAY_REPORT_URL,
  DEFAULT_DATASET_REPLAY_EXECUTION_MODE,
  classifyDatasetReplaySettlement,
  DATASET_REPLAY_MAX_TRACKING_ERROR_RAD,
  DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC,
  type DatasetReplayFrame,
  type DatasetReplayExecutionMode,
  type DatasetReplayReport,
  parseDatasetReplayReport,
} from './datasetReplay';
import { DatasetReplayPanel, type DatasetReplayState } from '../ui/DatasetReplayPanel';
import { Wrist3PhysicsDebugPanel } from '../ui/Wrist3PhysicsDebugPanel';
import type { Wrist3ExperimentTrace, Wrist3PhysicsDebugState } from '../sim/RobotBody';

interface LoadedRobot {
  description: RobotDescription;
  urdfXml: string;
  controller: JointController;
  kinematics: Kinematics;
  model: RobotModel;
}

const DEFAULT_TARGET: Pose = {
  position: [0.35, 0, 0.45],
  orientation: [0, 0, 0, 1],
};
const CAMERA_PREVIEW_INTERVAL_MS = 100;
const REPLAY_STABLE_SAMPLE_DELTA_RAD = 0.001;
const REPLAY_STABLE_SAMPLE_COUNT = 3;
const REPLAY_NO_MOTION_TIMEOUT_MS = 1500;
const REPLAY_NO_MOTION_THRESHOLD_RAD = 0.0001;
const REPLAY_STEP_TIMEOUT_MS = 15000;
const WRIST3_AB_TARGET = 0.101814692820414;
const WRIST3_AB_STEPS = Math.round(2 / SIMULATION_CONFIG.fixedTimeStep);
const DATASET_REPLAY_EXECUTION_MODE: DatasetReplayExecutionMode =
  DEFAULT_DATASET_REPLAY_EXECUTION_MODE;

interface DatasetReplayStepRecord {
  frame_index: number;
  execution_mode: DatasetReplayExecutionMode;
  replay_state: 'SETTLED' | 'TRACKING_ERROR' | 'ERROR';
  error_reason: string | null;
  target_tcp_pose: Pose;
  commanded_joint_positions: JointValues;
  actual_joint_positions: JointValues;
  q_body: Record<string, number | null>;
  joint_tracking_error: JointValues;
  max_joint_tracking_error_rad: number;
  max_joint_tracking_error_joint: string;
  max_joint_velocity_rad_per_sec: number;
  max_joint_velocity_joint: string;
  actual_tcp_pose: Pose;
  tcp_position_tracking_error_m: number;
  tcp_orientation_tracking_error_rad: number;
  expected_joint_delta_norm: number;
  actual_joint_motion_norm: number;
  max_joint_change_per_sample_rad: number;
  duration_ms: number;
  physics_ready: boolean;
  actual_joint_samples: DatasetReplayStepSample[];
}

interface DatasetReplayStepSample {
  elapsed_ms: number;
  actual_joint_positions: JointValues;
  q_body: Record<string, number | null>;
  joint_velocity_rad_per_sec: JointValues;
  max_joint_velocity_rad_per_sec: number;
  joint_tracking_error: JointValues;
  max_joint_tracking_error_rad: number;
  actual_tcp_pose: Pose;
  tcp_position_tracking_error_m: number;
  tcp_orientation_tracking_error_rad: number;
}

interface DatasetReplayDebugSnapshot {
  source: string;
  episode_index: 0;
  execution_mode: DatasetReplayExecutionMode;
  current_frame: number | null;
  replay_state: DatasetReplayState;
  precomputed_key_frames: DatasetReplayFrame[];
  records: DatasetReplayStepRecord[];
}

interface DatasetReplayMotionState {
  frame_index: number | null;
  started_at_ms: number;
  baseline_joints: JointValues;
  previous_actual_joints: JointValues;
  previous_sample_at_ms: number;
  expected_joint_delta_norm: number;
  stable_sample_count: number;
  maximum_sample_delta_rad: number;
  maximum_joint_velocity_rad_per_sec: number;
  maximum_joint_tracking_error_rad: number;
  maximum_joint_velocity_joint: string;
  maximum_joint_tracking_error_joint: string;
  samples: DatasetReplayStepSample[];
}

declare global {
  interface Window {
    __ROBOT_SIM_DATASET_REPLAY__?: DatasetReplayDebugSnapshot;
  }
}

export const App = () => {
  const [modelId, setModelId] = useState<RobotModelId>(DEFAULT_ROBOT_MODEL_ID);
  const [loadedRobot, setLoadedRobot] = useState<LoadedRobot | null>(null);
  const [loadError, setLoadError] = useState('');
  const [jointValues, setJointValues] = useState<JointValues>({});
  const [jointBodyAngles, setJointBodyAngles] = useState<Record<string, number | null>>({});
  const [mode, setMode] = useState<ControlMode>('position');
  const [commands, setCommands] = useState<Record<string, JointCommand>>({});
  const [isRunning, setIsRunning] = useState(true);
  const [targetPose, setTargetPose] = useState<Pose>(DEFAULT_TARGET);
  const [ikResult, setIkResult] = useState<IKResult | null>(null);
  const [ikTargetPose, setIkTargetPose] = useState<Pose | null>(null);
  const [jointDeltaNorm, setJointDeltaNorm] = useState<number | null>(null);
  const [ikMessage, setIkMessage] = useState('');
  const [physicsReady, setPhysicsReady] = useState(false);
  const [wrist3PhysicsDebugState, setWrist3PhysicsDebugState] =
    useState<Wrist3PhysicsDebugState | null>(null);
  const [wrist3ExperimentStatus, setWrist3ExperimentStatus] = useState('READY');
  const [wrist3ExperimentResults, setWrist3ExperimentResults] = useState<
    Partial<Record<'MANUAL' | 'REPLAY', { qBody: number | null; steps: number }>>
  >({});
  const [datasetReplayReport, setDatasetReplayReport] = useState<DatasetReplayReport | null>(null);
  const [datasetReplayEnabled, setDatasetReplayEnabled] = useState(false);
  const [datasetReplayLoading, setDatasetReplayLoading] = useState(false);
  const [datasetReplayFrameIndex, setDatasetReplayFrameIndex] = useState<number | null>(null);
  const [datasetReplayState, setDatasetReplayState] = useState<DatasetReplayState>('IDLE');
  const [datasetReplayError, setDatasetReplayError] = useState('');
  const [datasetReplayLog, setDatasetReplayLog] = useState<DatasetReplayStepRecord[]>([]);
  const datasetReplayLogRef = useRef<DatasetReplayStepRecord[]>([]);
  const datasetReplayMotionRef = useRef<DatasetReplayMotionState>({
    frame_index: null,
    started_at_ms: 0,
    baseline_joints: {},
    previous_actual_joints: {},
    previous_sample_at_ms: 0,
    expected_joint_delta_norm: 0,
    stable_sample_count: 0,
    maximum_sample_delta_rad: 0,
    maximum_joint_velocity_rad_per_sec: 0,
    maximum_joint_tracking_error_rad: 0,
    maximum_joint_velocity_joint: '',
    maximum_joint_tracking_error_joint: '',
    samples: [],
  });
  const [cameraPreviewReady, setCameraPreviewReady] = useState(false);
  const cameraFramesRef = useRef<SimCameraFrame[]>([]);
  const cameraPreviewCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const cameraPreviewImageDataRef = useRef<ImageData | null>(null);
  const cameraPreviewLastDrawRef = useRef(0);
  const cameraPreviewReadyRef = useRef(false);
  const observationIndexRef = useRef(0);

  const currentModel = findRobotConfig(modelId);
  const hasWrist3Joint =
    modelId !== 'UR5_CAD' &&
    loadedRobot?.description.joints.some((joint) => joint.name === 'wrist_3_joint');
  const wrist3CommandValue =
    mode === 'position'
      ? commands.wrist_3_joint?.mode === 'position'
        ? commands.wrist_3_joint.value
        : (jointValues.wrist_3_joint ?? null)
      : null;
  const wrist3AtZero =
    Math.abs(wrist3PhysicsDebugState?.readback ?? Number.POSITIVE_INFINITY) < 0.005 &&
    Math.abs(wrist3PhysicsDebugState?.bodyAngle ?? Number.POSITIVE_INFINITY) < 0.005 &&
    Math.hypot(
      ...(wrist3PhysicsDebugState?.childAngularVelocity ?? [Infinity, Infinity, Infinity]),
    ) < 0.01;

  useEffect(() => {
    if (wrist3ExperimentStatus === 'RESETTING' && wrist3AtZero) {
      setWrist3ExperimentStatus('READY AT ZERO');
    }
  }, [wrist3AtZero, wrist3ExperimentStatus]);

  const handleWrist3ExperimentComplete = useCallback((trace: Wrist3ExperimentTrace) => {
    const finalSample = trace.records[trace.records.length - 1];
    setWrist3ExperimentResults((current) => ({
      ...current,
      [trace.controlSource]: { qBody: finalSample?.qBody ?? null, steps: trace.records.length },
    }));
    setWrist3ExperimentStatus(`${trace.controlSource} COMPLETE`);
  }, []);

  useEffect(() => {
    let active = true;
    setLoadedRobot(null);
    setLoadError('');
    setIkResult(null);
    setIkTargetPose(null);
    setJointDeltaNorm(null);
    setIkMessage('');
    setJointValues({});
    setJointBodyAngles({});
    setCommands({});
    setPhysicsReady(false);
    setWrist3PhysicsDebugState(null);
    setDatasetReplayReport(null);
    setDatasetReplayEnabled(false);
    setDatasetReplayLoading(false);
    setDatasetReplayFrameIndex(null);
    setDatasetReplayState('IDLE');
    setDatasetReplayError('');
    datasetReplayLogRef.current = [];
    setDatasetReplayLog([]);
    datasetReplayMotionRef.current.frame_index = null;
    setCameraPreviewReady(false);
    cameraPreviewReadyRef.current = false;
    cameraPreviewImageDataRef.current = null;
    cameraPreviewLastDrawRef.current = 0;
    cameraFramesRef.current = [];

    const loadRobot = async () => {
      try {
        const url = `${SIMULATION_CONFIG.robotAssetRoot}/${currentModel.file}`;
        const response = await fetch(url);
        if (!response.ok) throw new Error(`URDF 加载失败：HTTP ${response.status}`);
        const sourceUrdf = configureRobotUrdf(await response.text(), currentModel);
        const urdfXml = resolvePackageUris(sourceUrdf, currentModel.packageMappings);
        const description = parseUrdf(sourceUrdf, { tipLink: currentModel.tipLink });
        const model = new RobotModel(description);
        model.setJointValues(currentModel.initialJoints);
        if (!active) return;

        setLoadedRobot({
          description,
          urdfXml,
          controller: new JointController(model),
          kinematics: new Kinematics(description),
          model,
        });
        setJointValues(model.getJointValues());
        publishDebugEvent('robot:loaded', {
          model: modelId,
          type: currentModel.type,
          name: description.name,
          rootLink: description.rootLink,
          tipLink: description.tipLink,
          joints: model.getControllableJoints().map((joint) => joint.name),
        });
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
  }, [currentModel, modelId]);

  const tcpPose = loadedRobot?.model.getLinkPose() ?? null;
  const tool0Pose = loadedRobot?.description.links.some((link) => link.name === 'tool0')
    ? loadedRobot.model.getLinkPose('tool0')
    : null;
  const datasetReplayFrame = useMemo(
    () =>
      datasetReplayFrameIndex === null
        ? null
        : (datasetReplayReport?.frames[datasetReplayFrameIndex] ?? null),
    [datasetReplayFrameIndex, datasetReplayReport],
  );
  const datasetReplayTargetPose = useMemo<Pose | null>(
    () =>
      datasetReplayFrame
        ? {
            position: datasetReplayFrame.aligned_target_position,
            orientation: datasetReplayFrame.aligned_target_quaternion,
          }
        : null,
    [datasetReplayFrame],
  );
  const datasetReplayActualPose = useMemo(
    () =>
      loadedRobot && datasetReplayEnabled
        ? loadedRobot.kinematics.forwardKinematics(jointValues, loadedRobot.description.tipLink)
        : null,
    [datasetReplayEnabled, jointValues, loadedRobot],
  );
  const datasetReplayJointNames = useMemo(
    () => loadedRobot?.model.getControllableJoints().map((joint) => joint.name) ?? [],
    [loadedRobot],
  );
  const datasetReplayCommandedJoints = useMemo(
    () =>
      Object.fromEntries(
        datasetReplayJointNames.map((name) => [
          name,
          commands[name]?.mode === 'position'
            ? commands[name].value
            : (datasetReplayFrame?.ik_solution_joints[name] ?? 0),
        ]),
      ),
    [commands, datasetReplayFrame, datasetReplayJointNames],
  );
  const datasetReplayJointTrackingErrors = useMemo(
    () =>
      Object.fromEntries(
        datasetReplayJointNames.map((name) => [
          name,
          (jointValues[name] ?? 0) - (datasetReplayCommandedJoints[name] ?? 0),
        ]),
      ),
    [datasetReplayCommandedJoints, datasetReplayJointNames, jointValues],
  );
  const datasetReplayMaxJointTrackingEntry = useMemo(
    () =>
      Object.entries(datasetReplayJointTrackingErrors).reduce<[string, number] | null>(
        (maximum, entry) =>
          !maximum || Math.abs(entry[1]) > Math.abs(maximum[1]) ? entry : maximum,
        null,
      ),
    [datasetReplayJointTrackingErrors],
  );
  const datasetReplayTcpPositionTrackingError = useMemo(
    () =>
      datasetReplayActualPose && datasetReplayTargetPose
        ? new Vector3(...datasetReplayActualPose.position).distanceTo(
            new Vector3(...datasetReplayTargetPose.position),
          )
        : null,
    [datasetReplayActualPose, datasetReplayTargetPose],
  );
  const datasetReplayTcpOrientationTrackingError = useMemo(
    () =>
      datasetReplayActualPose && datasetReplayTargetPose
        ? new Quaternion(...datasetReplayActualPose.orientation).angleTo(
            new Quaternion(...datasetReplayTargetPose.orientation),
          )
        : null,
    [datasetReplayActualPose, datasetReplayTargetPose],
  );
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

  const commandJoint = (jointName: string, value: number): number | null => {
    if (!loadedRobot || inference.isLocked) return null;
    try {
      const command = loadedRobot.controller.setCommand(jointName, value);
      setCommands(loadedRobot.controller.getCommands());
      const experiment = typeof window === 'undefined' ? undefined : window.__ROBOT_SIM_WRIST3_AB__;
      if (
        experiment?.active &&
        experiment.controlSource === 'MANUAL' &&
        jointName === 'wrist_3_joint'
      ) {
        experiment.controllerTarget = loadedRobot.controller.getCommand(jointName)?.value ?? null;
        experiment.controllerWrites.push('JointController.setCommand(wrist_3_joint)');
        experiment.reactStateWrites.push('App.commands');
      }
      if (command.mode === 'position') {
        loadedRobot.model.setJointValue(jointName, command.value);
        const nextValues = loadedRobot.model.getJointValues();
        setJointValues(nextValues);
        if (
          experiment?.active &&
          experiment.controlSource === 'MANUAL' &&
          jointName === 'wrist_3_joint'
        ) {
          experiment.modelWrites.push('RobotModel.setJointValue(wrist_3_joint)');
          experiment.reactStateWrites.push('App.jointValues');
        }
        publishDebugEvent('joint:command', { joint: jointName, ...command });
      } else {
        publishDebugEvent('joint:command', { joint: jointName, ...command });
      }
      return command.value;
    } catch (error) {
      setIkMessage(error instanceof Error ? error.message : String(error));
      return null;
    }
  };

  const startWrist3Experiment = (controlSource: 'MANUAL' | 'REPLAY'): Wrist3ExperimentTrace => {
    const trace: Wrist3ExperimentTrace = {
      controlSource,
      target: WRIST3_AB_TARGET,
      stepsToRecord: WRIST3_AB_STEPS,
      active: true,
      completed: false,
      controllerTarget: null,
      controllerWrites: [],
      reactStateWrites: [],
      modelWrites: [],
      refWrites: [],
      currentStep: null,
      records: [],
    };
    if (typeof window !== 'undefined') {
      window.__ROBOT_SIM_WRIST3_AB__ = trace;
      window.__ROBOT_SIM_WRIST3_AB_HISTORY__ ??= [];
      window.__ROBOT_SIM_WRIST3_AB_HISTORY__.push(trace);
    }
    setWrist3ExperimentStatus(`RECORDING ${controlSource}`);
    return trace;
  };

  const handleManualWrist3Test = () => {
    if (!loadedRobot || !canManualWrist3Test) return;
    const trace = startWrist3Experiment('MANUAL');
    const appliedTarget = commandJoint('wrist_3_joint', WRIST3_AB_TARGET);
    if (appliedTarget !== WRIST3_AB_TARGET) {
      trace.active = false;
      setWrist3ExperimentStatus('MANUAL COMMAND FAILED');
    }
  };

  const updatePhysicsState = (values: JointValues) => {
    if (!loadedRobot) return;
    loadedRobot.model.setJointValues(values);
    setJointValues(loadedRobot.model.getJointValues());
    const experiment = typeof window === 'undefined' ? undefined : window.__ROBOT_SIM_WRIST3_AB__;
    if (experiment?.active) {
      if (!experiment.modelWrites.includes('RobotModel.setJointValues (physics readback)')) {
        experiment.modelWrites.push('RobotModel.setJointValues (physics readback)');
      }
      if (!experiment.reactStateWrites.includes('App.jointValues (physics readback)')) {
        experiment.reactStateWrites.push('App.jointValues (physics readback)');
      }
    }
  };

  const writeDatasetReplaySnapshot = (
    frameIndex: number | null,
    replayState: DatasetReplayState,
    frames = datasetReplayReport?.frames ?? [],
  ) => {
    if (typeof window === 'undefined') return;
    window.__ROBOT_SIM_DATASET_REPLAY__ = {
      source: DATASET_REPLAY_REPORT_URL,
      episode_index: 0,
      execution_mode: DATASET_REPLAY_EXECUTION_MODE,
      current_frame: frameIndex,
      replay_state: replayState,
      precomputed_key_frames: [0, 65, 99, 218, 399].flatMap((index) => {
        const frame = frames[index];
        return frame ? [frame] : [];
      }),
      records: datasetReplayLogRef.current,
    };
  };

  const appendDatasetReplayRecord = (record: DatasetReplayStepRecord) => {
    const nextRecords = [...datasetReplayLogRef.current, record];
    datasetReplayLogRef.current = nextRecords;
    setDatasetReplayLog(nextRecords);
    if (typeof window !== 'undefined') {
      window.__ROBOT_SIM_DATASET_REPLAY__ = {
        source: DATASET_REPLAY_REPORT_URL,
        episode_index: 0,
        execution_mode: DATASET_REPLAY_EXECUTION_MODE,
        current_frame: record.frame_index,
        replay_state: record.replay_state,
        precomputed_key_frames: [0, 65, 99, 218, 399].flatMap((index) => {
          const frame = datasetReplayReport?.frames[index];
          return frame ? [frame] : [];
        }),
        records: nextRecords,
      };
    }
    publishDebugEvent('dataset_replay:step', record);
  };

  const commandDatasetReplayFrame = (report: DatasetReplayReport, frameIndex: number) => {
    if (!loadedRobot || modelId !== 'UR5' || !physicsReady || inference.isLocked) return;
    const frame = report.frames[frameIndex];
    if (!frame) return;
    if (loadedRobot.controller.getMode() !== 'position') {
      setDatasetReplayState('ERROR');
      setDatasetReplayError('请先将关节控制模式切换为 position，再加载 Dataset Replay。');
      return;
    }
    const jointNames = loadedRobot.model.getControllableJoints().map((joint) => joint.name);
    const commandValues = frame.ik_solution_joints;
    const baselineJoints = Object.fromEntries(
      jointNames.map((name) => [name, jointValues[name] ?? 0]),
    );

    jointNames.forEach((jointName) => {
      const target = commandValues[jointName];
      if (target === undefined) throw new Error(`Phase 2B-1 缺少 ${jointName} 的关节解`);
      const appliedValue = commandJoint(jointName, target);
      if (appliedValue === null || Math.abs(appliedValue - target) > 1e-12) {
        throw new Error(`Phase 2C-1 ${jointName} 未能应用 Phase 2B-1 IK 解`);
      }
    });
    const nextCommands = loadedRobot.controller.getCommands();
    const resolvedCommandValues = Object.fromEntries(
      jointNames.map((name) => [name, nextCommands[name]?.value ?? commandValues[name]]),
    );
    const expectedJointDeltaNorm = Math.hypot(
      ...jointNames.map((name) => resolvedCommandValues[name] - baselineJoints[name]),
    );
    const startedAtMs = performance.now();

    setCommands(nextCommands);
    setIsRunning(true);
    setDatasetReplayFrameIndex(frameIndex);
    setDatasetReplayState('MOVING');
    setDatasetReplayError('');
    datasetReplayMotionRef.current = {
      frame_index: frameIndex,
      started_at_ms: startedAtMs,
      baseline_joints: baselineJoints,
      previous_actual_joints: baselineJoints,
      previous_sample_at_ms: startedAtMs,
      expected_joint_delta_norm: expectedJointDeltaNorm,
      stable_sample_count: 0,
      maximum_sample_delta_rad: 0,
      maximum_joint_velocity_rad_per_sec: 0,
      maximum_joint_tracking_error_rad: 0,
      maximum_joint_velocity_joint: '',
      maximum_joint_tracking_error_joint: '',
      samples: [],
    };
    const wrist3Experiment =
      typeof window === 'undefined' ? undefined : window.__ROBOT_SIM_WRIST3_AB__;
    if (wrist3Experiment?.active && wrist3Experiment.controlSource === 'REPLAY') {
      wrist3Experiment.controllerTarget =
        loadedRobot.controller.getCommand('wrist_3_joint')?.value ?? null;
      wrist3Experiment.controllerWrites.push(
        ...jointNames.map((name) => `JointController.setCommand(${name})`),
      );
      wrist3Experiment.reactStateWrites.push(
        'App.commands',
        'App.isRunning',
        'App.datasetReplayFrameIndex',
        'App.datasetReplayState',
        'App.datasetReplayError',
      );
      wrist3Experiment.refWrites.push(
        'datasetReplayMotionRef.current',
        'window.__ROBOT_SIM_DATASET_REPLAY__',
      );
    }
    writeDatasetReplaySnapshot(frameIndex, 'MOVING', report.frames);
    publishDebugEvent('dataset_replay:command', {
      episode_index: 0,
      frame_index: frameIndex,
      target_tcp_pose: {
        position: frame.aligned_target_position,
        orientation: frame.aligned_target_quaternion,
      },
      commanded_joint_positions: resolvedCommandValues,
      execution_mode: DATASET_REPLAY_EXECUTION_MODE,
      physics_ready: physicsReady,
      position_execution:
        DATASET_REPLAY_EXECUTION_MODE === 'KINEMATIC_REPLAY' ? 'kinematic_fk' : 'joint_motors',
    });
  };

  const experimentRecording = wrist3ExperimentStatus.startsWith('RECORDING');
  const canManualWrist3Test = Boolean(
    loadedRobot &&
    modelId === 'UR5' &&
    physicsReady &&
    isRunning &&
    mode === 'position' &&
    !datasetReplayEnabled &&
    !inference.isLocked &&
    !experimentRecording &&
    wrist3AtZero,
  );
  const canReplayWrist3Test = Boolean(
    loadedRobot &&
    modelId === 'UR5' &&
    physicsReady &&
    isRunning &&
    datasetReplayReport &&
    (!datasetReplayEnabled || datasetReplayState === 'IDLE' || datasetReplayState === 'SETTLED') &&
    !inference.isLocked &&
    !experimentRecording &&
    wrist3AtZero,
  );
  const canResetWrist3 = Boolean(
    loadedRobot &&
    physicsReady &&
    isRunning &&
    mode === 'position' &&
    !inference.isLocked &&
    !experimentRecording &&
    (!datasetReplayEnabled || (datasetReplayState !== 'MOVING' && Boolean(datasetReplayReport))),
  );

  const handleWrist3Reset = () => {
    if (!canResetWrist3) return;
    setWrist3ExperimentStatus('RESETTING');
    if (datasetReplayEnabled) exitDatasetReplay();
    commandJoint('wrist_3_joint', 0);
  };

  const handleReplayWrist3Test = () => {
    if (!datasetReplayReport || !canReplayWrist3Test) return;
    const frames = datasetReplayReport.frames.map((frame, index) =>
      index === 0
        ? {
            ...frame,
            ik_solution_joints: {
              ...frame.ik_solution_joints,
              wrist_3_joint: WRIST3_AB_TARGET,
            },
          }
        : frame,
    );
    const testReport: DatasetReplayReport = { ...datasetReplayReport, frames };
    const trace = startWrist3Experiment('REPLAY');
    if (!datasetReplayEnabled) {
      setDatasetReplayEnabled(true);
      trace.reactStateWrites.push('App.datasetReplayEnabled');
    }
    commandDatasetReplayFrame(testReport, 0);
  };

  const loadDatasetReplay = async () => {
    if (!loadedRobot || modelId !== 'UR5' || !physicsReady || inference.isLocked) return;
    setDatasetReplayLoading(true);
    setDatasetReplayError('');
    try {
      const response = await fetch(DATASET_REPLAY_REPORT_URL);
      if (!response.ok) throw new Error(`Phase 2B-1 报告加载失败：HTTP ${response.status}`);
      const report = parseDatasetReplayReport(await response.text());
      const jointNames = loadedRobot.model.getControllableJoints().map((joint) => joint.name);
      if (jointNames.length !== 6 || jointNames.some((name) => !(name in report.initial_joints))) {
        throw new Error('当前 UR5 关节名称与 Phase 2B-1 报告不匹配');
      }
      setDatasetReplayReport(report);
      setDatasetReplayEnabled(true);
      setDatasetReplayState('IDLE');
      datasetReplayLogRef.current = [];
      setDatasetReplayLog([]);
      writeDatasetReplaySnapshot(0, 'IDLE', report.frames);
      commandDatasetReplayFrame(report, 0);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      setDatasetReplayError(message);
      setDatasetReplayState('ERROR');
      publishDebugEvent('dataset_replay:error', { message });
    } finally {
      setDatasetReplayLoading(false);
    }
  };

  const exitDatasetReplay = () => {
    if (!loadedRobot || datasetReplayState === 'MOVING') return;
    setCommands({});
    setDatasetReplayEnabled(false);
    setDatasetReplayFrameIndex(null);
    setDatasetReplayState('IDLE');
    setDatasetReplayError('');
    publishDebugEvent('dataset_replay:exit', {
      completed_steps: datasetReplayLogRef.current.length,
    });
  };

  const writeDatasetReplayStepResult = (
    resultState: 'SETTLED' | 'TRACKING_ERROR' | 'ERROR',
    errorReason: string | null,
    actualValues: JointValues,
    maxJointChangePerSample: number,
  ) => {
    if (!loadedRobot || datasetReplayFrameIndex === null || !datasetReplayTargetPose) return;
    const frame = datasetReplayFrame;
    if (!frame) return;
    const jointNames = loadedRobot.model.getControllableJoints().map((joint) => joint.name);
    const activeCommands = loadedRobot.controller.getCommands();
    const commandedJoints = Object.fromEntries(
      jointNames.map((name) => [
        name,
        activeCommands[name]?.mode === 'position'
          ? activeCommands[name].value
          : frame.ik_solution_joints[name],
      ]),
    );
    const actualJoints = Object.fromEntries(
      jointNames.map((name) => [name, actualValues[name] ?? 0]),
    );
    const qBody = Object.fromEntries(
      jointNames.map((name) => [name, jointBodyAngles[name] ?? null]),
    );
    const trackingErrors = Object.fromEntries(
      jointNames.map((name) => [name, actualJoints[name] - commandedJoints[name]]),
    );
    const actualTcpPose = loadedRobot.kinematics.forwardKinematics(
      actualValues,
      loadedRobot.description.tipLink,
    );
    const motionState = datasetReplayMotionRef.current;
    const actualJointMotionNorm = Math.hypot(
      ...jointNames.map(
        (name) => actualJoints[name] - (motionState.baseline_joints[name] ?? actualJoints[name]),
      ),
    );
    const record: DatasetReplayStepRecord = {
      frame_index: datasetReplayFrameIndex,
      execution_mode: DATASET_REPLAY_EXECUTION_MODE,
      replay_state: resultState,
      error_reason: errorReason,
      target_tcp_pose: datasetReplayTargetPose,
      commanded_joint_positions: commandedJoints,
      actual_joint_positions: actualJoints,
      q_body: qBody,
      joint_tracking_error: trackingErrors,
      max_joint_tracking_error_rad: motionState.maximum_joint_tracking_error_rad,
      max_joint_tracking_error_joint: motionState.maximum_joint_tracking_error_joint,
      max_joint_velocity_rad_per_sec: motionState.maximum_joint_velocity_rad_per_sec,
      max_joint_velocity_joint: motionState.maximum_joint_velocity_joint,
      actual_tcp_pose: actualTcpPose,
      tcp_position_tracking_error_m: new Vector3(...actualTcpPose.position).distanceTo(
        new Vector3(...datasetReplayTargetPose.position),
      ),
      tcp_orientation_tracking_error_rad: new Quaternion(...actualTcpPose.orientation).angleTo(
        new Quaternion(...datasetReplayTargetPose.orientation),
      ),
      expected_joint_delta_norm: motionState.expected_joint_delta_norm,
      actual_joint_motion_norm: actualJointMotionNorm,
      max_joint_change_per_sample_rad: maxJointChangePerSample,
      duration_ms: performance.now() - motionState.started_at_ms,
      physics_ready: physicsReady,
      actual_joint_samples: [...motionState.samples],
    };
    appendDatasetReplayRecord(record);
    setDatasetReplayState(resultState);
    if (errorReason) setDatasetReplayError(errorReason);
    writeDatasetReplaySnapshot(datasetReplayFrameIndex, resultState);
  };

  const finishDatasetReplayStepRef = useRef(writeDatasetReplayStepResult);
  finishDatasetReplayStepRef.current = writeDatasetReplayStepResult;

  useEffect(() => {
    if (
      !datasetReplayEnabled ||
      datasetReplayState !== 'MOVING' ||
      datasetReplayFrameIndex === null ||
      !datasetReplayReport ||
      !loadedRobot ||
      !physicsReady ||
      !isRunning
    ) {
      return;
    }

    const motionState = datasetReplayMotionRef.current;
    if (motionState.frame_index !== datasetReplayFrameIndex) return;
    const jointNames = loadedRobot.model.getControllableJoints().map((joint) => joint.name);
    const sampleActualState = () => {
      const now = performance.now();
      const actualJoints = Object.fromEntries(
        jointNames.map((name) => [name, jointValues[name] ?? Number.NaN]),
      );
      if (Object.values(actualJoints).some((value) => !Number.isFinite(value))) {
        finishDatasetReplayStepRef.current(
          'ERROR',
          'actual_joint_state_invalid: Rapier 未提供全部六个有限关节状态',
          actualJoints,
          Number.NaN,
        );
        return;
      }

      const sampleDurationSec = Math.max((now - motionState.previous_sample_at_ms) / 1000, 0.001);
      const jointDeltas = Object.fromEntries(
        jointNames.map((name) => [
          name,
          actualJoints[name] - (motionState.previous_actual_joints[name] ?? actualJoints[name]),
        ]),
      );
      const jointVelocities = Object.fromEntries(
        jointNames.map((name) => [name, jointDeltas[name] / sampleDurationSec]),
      );
      const maximumSampleDelta = Math.max(...Object.values(jointDeltas).map(Math.abs));
      const maximumVelocityEntry = Object.entries(jointVelocities).reduce<[string, number]>(
        (maximum, entry) =>
          Math.abs(entry[1]) > Math.abs(maximum[1]) ? [entry[0], entry[1]] : maximum,
        [jointNames[0], jointVelocities[jointNames[0]]],
      );
      const maximumVelocity = Math.abs(maximumVelocityEntry[1]);
      motionState.maximum_sample_delta_rad = Math.max(
        motionState.maximum_sample_delta_rad,
        maximumSampleDelta,
      );
      motionState.maximum_joint_velocity_rad_per_sec = Math.max(
        motionState.maximum_joint_velocity_rad_per_sec,
        maximumVelocity,
      );
      if (maximumVelocity >= motionState.maximum_joint_velocity_rad_per_sec) {
        motionState.maximum_joint_velocity_joint = maximumVelocityEntry[0];
      }
      motionState.stable_sample_count =
        maximumVelocity <= DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC
          ? motionState.stable_sample_count + 1
          : 0;

      const elapsedMs = now - motionState.started_at_ms;
      const actualMotionNorm = Math.hypot(
        ...jointNames.map(
          (name) => actualJoints[name] - (motionState.baseline_joints[name] ?? actualJoints[name]),
        ),
      );
      const frame = datasetReplayReport.frames[datasetReplayFrameIndex];
      const activeCommands = loadedRobot.controller.getCommands();
      const commandedJoints = Object.fromEntries(
        jointNames.map((name) => [
          name,
          activeCommands[name]?.mode === 'position'
            ? activeCommands[name].value
            : frame.ik_solution_joints[name],
        ]),
      );
      const trackingErrors = Object.fromEntries(
        jointNames.map((name) => [name, actualJoints[name] - commandedJoints[name]]),
      );
      const maximumTrackingEntry = Object.entries(trackingErrors).reduce<[string, number]>(
        (maximum, entry) =>
          Math.abs(entry[1]) > Math.abs(maximum[1]) ? [entry[0], entry[1]] : maximum,
        [jointNames[0], trackingErrors[jointNames[0]]],
      );
      const maximumTrackingError = Math.abs(maximumTrackingEntry[1]);
      motionState.maximum_joint_tracking_error_rad = Math.max(
        motionState.maximum_joint_tracking_error_rad,
        maximumTrackingError,
      );
      if (maximumTrackingError >= motionState.maximum_joint_tracking_error_rad) {
        motionState.maximum_joint_tracking_error_joint = maximumTrackingEntry[0];
      }
      const actualTcpPose = loadedRobot.kinematics.forwardKinematics(
        actualJoints,
        loadedRobot.description.tipLink,
      );
      const targetTcpPose: Pose = {
        position: frame.aligned_target_position,
        orientation: frame.aligned_target_quaternion,
      };
      motionState.samples.push({
        elapsed_ms: elapsedMs,
        actual_joint_positions: actualJoints,
        q_body: Object.fromEntries(jointNames.map((name) => [name, jointBodyAngles[name] ?? null])),
        joint_velocity_rad_per_sec: jointVelocities,
        max_joint_velocity_rad_per_sec: maximumVelocity,
        joint_tracking_error: trackingErrors,
        max_joint_tracking_error_rad: maximumTrackingError,
        actual_tcp_pose: actualTcpPose,
        tcp_position_tracking_error_m: new Vector3(...actualTcpPose.position).distanceTo(
          new Vector3(...targetTcpPose.position),
        ),
        tcp_orientation_tracking_error_rad: new Quaternion(...actualTcpPose.orientation).angleTo(
          new Quaternion(...targetTcpPose.orientation),
        ),
      });
      motionState.previous_actual_joints = actualJoints;
      motionState.previous_sample_at_ms = now;

      if (elapsedMs >= REPLAY_STEP_TIMEOUT_MS) {
        finishDatasetReplayStepRef.current(
          'ERROR',
          `step_timeout: ${REPLAY_STEP_TIMEOUT_MS / 1000} 秒内实际关节状态未稳定`,
          actualJoints,
          motionState.maximum_sample_delta_rad,
        );
        return;
      }

      if (
        motionState.expected_joint_delta_norm > REPLAY_STABLE_SAMPLE_DELTA_RAD &&
        elapsedMs >= REPLAY_NO_MOTION_TIMEOUT_MS &&
        actualMotionNorm < REPLAY_NO_MOTION_THRESHOLD_RAD
      ) {
        finishDatasetReplayStepRef.current(
          'ERROR',
          'no_joint_motion: commanded joints changed, but Rapier actual joints did not move',
          actualJoints,
          motionState.maximum_sample_delta_rad,
        );
        return;
      }

      if (
        motionState.stable_sample_count >= REPLAY_STABLE_SAMPLE_COUNT &&
        (motionState.expected_joint_delta_norm <= REPLAY_STABLE_SAMPLE_DELTA_RAD ||
          actualMotionNorm >= REPLAY_NO_MOTION_THRESHOLD_RAD)
      ) {
        const settledState = classifyDatasetReplaySettlement(maximumVelocity, maximumTrackingError);
        if (settledState) {
          const errorReason =
            settledState === 'TRACKING_ERROR'
              ? `tracking_error: 关节速度低于 ${DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC.toFixed(3)} rad/s，但最大跟踪误差 ${maximumTrackingError.toFixed(6)} rad 未低于 ${DATASET_REPLAY_MAX_TRACKING_ERROR_RAD.toFixed(3)} rad`
              : null;
          finishDatasetReplayStepRef.current(
            settledState,
            errorReason,
            actualJoints,
            motionState.maximum_sample_delta_rad,
          );
        }
      }
    };

    sampleActualState();
  }, [
    datasetReplayEnabled,
    datasetReplayFrameIndex,
    datasetReplayReport,
    datasetReplayState,
    isRunning,
    jointBodyAngles,
    jointValues,
    loadedRobot,
    physicsReady,
  ]);

  const solveIk = () => {
    if (!loadedRobot || inference.isLocked) return;
    const startJointValues = { ...jointValues };
    const requestedPose = {
      position: [...targetPose.position] as [number, number, number],
      orientation: [...targetPose.orientation] as [number, number, number, number],
    };
    const result = loadedRobot.kinematics.solveIK(requestedPose, startJointValues);
    const deltaNorm = Math.hypot(
      ...loadedRobot.model
        .getControllableJoints()
        .map(
          (joint) => (result.jointValues[joint.name] ?? 0) - (startJointValues[joint.name] ?? 0),
        ),
    );
    setIkResult(result);
    setIkTargetPose(requestedPose);
    setJointDeltaNorm(deltaNorm);
    setIkMessage(
      result.converged
        ? `收敛于 ${result.iterations} 次迭代`
        : `未收敛，位置残差 ${result.residual.position.toFixed(4)} m`,
    );
    publishDebugEvent('ik:result', {
      ...result,
      targetPose: requestedPose,
      startJointValues,
      jointDeltaNorm: deltaNorm,
    });
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

  const applyReplayStep = (values: JointValues, isFinal: boolean) => {
    if (!loadedRobot) return;
    loadedRobot.model.setJointValues(values);
    const nextValues = loadedRobot.model.getJointValues();
    loadedRobot.model.getControllableJoints().forEach((joint) => {
      loadedRobot.controller.setCommand(joint.name, nextValues[joint.name]);
    });
    setJointValues(nextValues);
    setCommands(loadedRobot.controller.getCommands());
    if (isFinal) publishDebugEvent('inference:action_applied', { joints: nextValues });
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
              aria-label={model.label}
              className={model.id === modelId ? 'model-tab active' : 'model-tab'}
              key={model.id}
              disabled={
                inference.isLocked || datasetReplayState === 'MOVING' || experimentRecording
              }
              onClick={() => setModelId(model.id)}
              type="button"
            >
              {model.id === 'UR5_CAD' ? 'UR5 CAD' : model.id}
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

          {modelId !== 'UR5_CAD' ? (
            <InferencePanel
              canStep={
                Boolean(loadedRobot && physicsReady && !loadError) &&
                !datasetReplayEnabled &&
                !experimentRecording &&
                (!inference.isLocked || inference.status === 'waiting')
              }
              error={inference.error}
              isLocked={inference.isLocked}
              onStep={() => {
                if (mode !== 'position' && !inference.isLocked) setControlMode('position');
                setIsRunning(true);
                inference.step();
              }}
              onStop={inference.stop}
              progress={inference.progress}
              status={inference.status}
            />
          ) : null}

          {modelId === 'UR5' ? (
            <DatasetReplayPanel
              active={datasetReplayEnabled}
              canControl={Boolean(
                loadedRobot &&
                modelId === 'UR5' &&
                physicsReady &&
                !loadError &&
                !inference.isLocked &&
                !experimentRecording,
              )}
              currentFrame={datasetReplayFrameIndex}
              executionMode={DATASET_REPLAY_EXECUTION_MODE}
              error={datasetReplayError}
              isLoading={datasetReplayLoading}
              jointNames={datasetReplayJointNames}
              logCount={datasetReplayLog.length}
              commandedJoints={datasetReplayCommandedJoints}
              actualJoints={jointValues}
              bodyJointAngles={jointBodyAngles}
              jointTrackingErrors={datasetReplayJointTrackingErrors}
              targetPose={datasetReplayTargetPose}
              actualPose={datasetReplayActualPose}
              maxJointTrackingError={datasetReplayMaxJointTrackingEntry?.[1] ?? null}
              maxJointTrackingErrorName={datasetReplayMaxJointTrackingEntry?.[0] ?? null}
              tcpPositionTrackingError={datasetReplayTcpPositionTrackingError}
              tcpOrientationTrackingError={datasetReplayTcpOrientationTrackingError}
              physicsReady={physicsReady}
              replayState={datasetReplayState}
              onLoad={loadDatasetReplay}
              onPrevious={() => {
                if (datasetReplayReport && datasetReplayFrameIndex !== null) {
                  commandDatasetReplayFrame(datasetReplayReport, datasetReplayFrameIndex - 1);
                }
              }}
              onNext={() => {
                if (datasetReplayReport && datasetReplayFrameIndex !== null) {
                  commandDatasetReplayFrame(datasetReplayReport, datasetReplayFrameIndex + 1);
                }
              }}
              onReset={() => {
                if (datasetReplayReport) commandDatasetReplayFrame(datasetReplayReport, 0);
              }}
              onExit={exitDatasetReplay}
            />
          ) : null}

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
            disabled={inference.isLocked || datasetReplayEnabled || experimentRecording}
          />

          <TaskPanel
            disabled={inference.isLocked || datasetReplayEnabled || experimentRecording}
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
                disabled={
                  inference.isLocked || datasetReplayState === 'MOVING' || experimentRecording
                }
                onClick={() => setIsRunning(!isRunning)}
                type="button"
              >
                {isRunning ? 'Ⅱ' : '▶'}
              </button>
              <button
                className="icon-button"
                disabled={inference.isLocked || datasetReplayEnabled || experimentRecording}
                title="重置关节"
                onClick={() => {
                  if (!loadedRobot) return;
                  loadedRobot.controller.setMode('position');
                  loadedRobot.model.setJointValues(currentModel.initialJoints);
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
                urdfXml={loadedRobot.urdfXml}
                isRunning={isRunning}
                mode={mode}
                jointValues={jointValues}
                modelId={modelId}
                onJointState={updatePhysicsState}
                onJointBodyAngles={setJointBodyAngles}
                onWrist3DebugState={setWrist3PhysicsDebugState}
                onWrist3ExperimentComplete={handleWrist3ExperimentComplete}
                onPhysicsReady={setPhysicsReady}
                playback={inference.playback}
                inferenceActive={inference.isLocked}
                positionExecutionOverride={
                  datasetReplayEnabled
                    ? DATASET_REPLAY_EXECUTION_MODE === 'KINEMATIC_REPLAY'
                      ? 'kinematic_fk'
                      : 'joint_motors'
                    : undefined
                }
                datasetReplayActive={datasetReplayEnabled}
                datasetReplayTargetPose={datasetReplayTargetPose}
                datasetReplayActualPose={datasetReplayActualPose}
                onPlaybackStep={applyReplayStep}
                onPlaybackComplete={inference.completePlayback}
                cameraEnabled={modelId !== 'UR5_CAD' && Boolean(loadedRobot)}
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
            <DebugOverlay
              controllableJointNames={
                loadedRobot?.model.getControllableJoints().map((joint) => joint.name) ?? []
              }
              ikResult={ikResult}
              ikTargetPose={ikTargetPose}
              jointDeltaNorm={jointDeltaNorm}
              jointValues={jointValues}
              pose={tcpPose}
              robotType={currentModel.type}
              targetPose={targetPose}
              tool0Pose={tool0Pose}
            />
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
      {hasWrist3Joint ? (
        <Wrist3PhysicsDebugPanel
          commandValue={wrist3CommandValue}
          state={wrist3PhysicsDebugState}
          canReset={canResetWrist3}
          canManualTest={canManualWrist3Test}
          canReplayTest={canReplayWrist3Test}
          experimentStatus={wrist3ExperimentStatus}
          experimentResults={wrist3ExperimentResults}
          onReset={handleWrist3Reset}
          onManualTest={handleManualWrist3Test}
          onReplayTest={handleReplayWrist3Test}
        />
      ) : null}
    </main>
  );
};
