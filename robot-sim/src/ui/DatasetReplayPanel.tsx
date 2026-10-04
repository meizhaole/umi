import type { JointValues, Pose } from '../core/types';

export type DatasetReplayState = 'IDLE' | 'MOVING' | 'SETTLED' | 'TRACKING_ERROR' | 'ERROR';

interface DatasetReplayPanelProps {
  active: boolean;
  canControl: boolean;
  currentFrame: number | null;
  error: string;
  isLoading: boolean;
  jointNames: string[];
  logCount: number;
  commandedJoints: JointValues;
  actualJoints: JointValues;
  jointTrackingErrors: JointValues;
  targetPose: Pose | null;
  actualPose: Pose | null;
  maxJointTrackingError: number | null;
  maxJointTrackingErrorName: string | null;
  tcpPositionTrackingError: number | null;
  tcpOrientationTrackingError: number | null;
  physicsReady: boolean;
  replayState: DatasetReplayState;
  onLoad: () => void;
  onPrevious: () => void;
  onNext: () => void;
  onReset: () => void;
  onExit: () => void;
}

const formatVector = (values: number[], digits = 4): string =>
  `[${values.map((value) => value.toFixed(digits)).join(', ')}]`;

const formatPose = (pose: Pose | null): string =>
  pose
    ? `p ${formatVector(pose.position)} m · q ${formatVector(pose.orientation)}`
    : '等待实际状态';

const formatJoints = (jointNames: string[], values: JointValues): string =>
  jointNames.map((name) => `${name}: ${(values[name] ?? 0).toFixed(4)}`).join(' · ');

export const DatasetReplayPanel = ({
  active,
  canControl,
  currentFrame,
  error,
  isLoading,
  jointNames,
  logCount,
  commandedJoints,
  actualJoints,
  jointTrackingErrors,
  targetPose,
  actualPose,
  maxJointTrackingError,
  maxJointTrackingErrorName,
  tcpPositionTrackingError,
  tcpOrientationTrackingError,
  physicsReady,
  replayState,
  onLoad,
  onPrevious,
  onNext,
  onReset,
  onExit,
}: DatasetReplayPanelProps) => (
  <section className="panel dataset-replay-panel" aria-label="UMI dataset 单帧回放">
    <div className="panel-heading">
      <div>
        <p className="eyebrow">KINEMATIC VALIDATION</p>
        <h2>UMI dataset 回放</h2>
      </div>
      <span className={`dataset-replay-state state-${replayState.toLowerCase()}`}>
        {replayState}
      </span>
    </div>

    <div className="dataset-replay-meta">
      <span>Episode: 0</span>
      <span>Frame: {currentFrame === null ? '—' : `${currentFrame} / 399`}</span>
      <span>Physics: {physicsReady ? 'READY' : 'NOT READY'}</span>
      <span>记录: {logCount} 步</span>
    </div>

    {!active ? (
      <button
        className="primary-button dataset-load-button"
        disabled={!canControl || isLoading}
        onClick={onLoad}
        type="button"
      >
        {isLoading ? '正在加载 400 帧…' : 'Load episode 0'}
      </button>
    ) : (
      <>
        <div className="dataset-replay-actions">
          <button
            className="secondary-button"
            disabled={!canControl || replayState === 'MOVING' || currentFrame === 0}
            onClick={onPrevious}
            type="button"
          >
            Previous
          </button>
          <button
            className="primary-button"
            disabled={!canControl || replayState === 'MOVING' || currentFrame === 399}
            onClick={onNext}
            type="button"
          >
            Next
          </button>
          <button
            className="secondary-button"
            disabled={!canControl || replayState === 'MOVING'}
            onClick={onReset}
            type="button"
          >
            Reset
          </button>
          <button
            className="secondary-button"
            disabled={replayState === 'MOVING'}
            onClick={onExit}
            type="button"
          >
            Exit
          </button>
        </div>

        <div className="dataset-replay-readouts">
          <div>
            <b>Target TCP</b>
            <span>{formatPose(targetPose)}</span>
          </div>
          <div>
            <b>Actual TCP</b>
            <span>{formatPose(actualPose)}</span>
          </div>
          <div>
            <b>Commanded joints (rad)</b>
            <span>{formatJoints(jointNames, commandedJoints)}</span>
          </div>
          <div>
            <b>Actual joints (rad)</b>
            <span>{formatJoints(jointNames, actualJoints)}</span>
          </div>
          <div>
            <b>Joint tracking error (actual − command, rad)</b>
            <span>{formatJoints(jointNames, jointTrackingErrors)}</span>
          </div>
          <div>
            <b>Max joint tracking error (actual − command)</b>
            <span>
              {maxJointTrackingError === null
                ? '—'
                : `${maxJointTrackingErrorName} ${maxJointTrackingError.toFixed(6)} rad`}
            </span>
          </div>
          <div>
            <b>TCP position tracking error</b>
            <span>
              {tcpPositionTrackingError === null ? '—' : `${tcpPositionTrackingError.toFixed(6)} m`}
            </span>
          </div>
          <div>
            <b>TCP orientation tracking error</b>
            <span>
              {tcpOrientationTrackingError === null
                ? '—'
                : `${tcpOrientationTrackingError.toFixed(6)} rad`}
            </span>
          </div>
        </div>
      </>
    )}

    {error ? <p className="dataset-replay-error">{error}</p> : null}
    <p className="dataset-replay-note">
      SETTLED 要求关节速度 ≤ 0.01 rad/s 且最大跟踪误差 &lt; 0.005 rad；低速但误差超限时显示
      TRACKING_ERROR。
    </p>
  </section>
);
