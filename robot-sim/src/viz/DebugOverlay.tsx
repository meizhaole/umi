import type { JointValues, Pose } from '../core/types';

interface DebugOverlayProps {
  jointValues: JointValues;
  pose: Pose | null;
}

export const DebugOverlay = ({ jointValues, pose }: DebugOverlayProps) => {
  const joints = Object.entries(jointValues).filter(
    ([name]) => !name.toLowerCase().includes('fixed'),
  );
  return (
    <div className="debug-overlay">
      <span className="debug-title">
        <i /> LIVE TELEMETRY
      </span>
      <span>
        <b>TCP X</b>
        {pose?.position[0].toFixed(3) ?? '—'} <small>m</small>
      </span>
      <span>
        <b>TCP Y</b>
        {pose?.position[1].toFixed(3) ?? '—'} <small>m</small>
      </span>
      <span>
        <b>TCP Z</b>
        {pose?.position[2].toFixed(3) ?? '—'} <small>m</small>
      </span>
      <span>
        <b>JOINTS</b>
        {joints.length.toString().padStart(2, '0')} <small>active</small>
      </span>
    </div>
  );
};
