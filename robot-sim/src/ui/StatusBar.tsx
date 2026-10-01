import type { Pose } from '../core/types';
import type { RobotModelId } from '../app/config';

interface StatusBarProps {
  modelId: RobotModelId;
  isRunning: boolean;
  jointCount: number;
  pose: Pose | null;
}

const formatCoordinate = (value?: number): string => (value === undefined ? '—' : value.toFixed(3));

export const StatusBar = ({ modelId, isRunning, jointCount, pose }: StatusBarProps) => (
  <footer className="status-bar">
    <div className="status-main">
      <span className={isRunning ? 'live-indicator' : 'live-indicator paused'} />
      <strong>{isRunning ? 'RUNNING' : 'PAUSED'}</strong>
      <span className="status-separator" />
      <span>
        Model <b>{modelId}</b>
      </span>
      <span className="status-separator" />
      <span>{jointCount} active joints</span>
    </div>
    <div className="tcp-readout">
      <span>TCP</span>
      <b>{formatCoordinate(pose?.position[0])}</b>
      <b>{formatCoordinate(pose?.position[1])}</b>
      <b>{formatCoordinate(pose?.position[2])}</b>
      <small>m</small>
    </div>
  </footer>
);
