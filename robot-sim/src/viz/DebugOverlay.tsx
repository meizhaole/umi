import type { IKResult, JointValues, Pose } from '../core/types';
import type { RobotType } from '../app/config';

interface DebugOverlayProps {
  controllableJointNames: string[];
  ikResult: IKResult | null;
  ikTargetPose: Pose | null;
  jointDeltaNorm: number | null;
  jointValues: JointValues;
  pose: Pose | null;
  robotType: RobotType;
  targetPose: Pose;
  tool0Pose: Pose | null;
}

const formatVector = (values: number[], digits = 3) =>
  `[${values.map((value) => value.toFixed(digits)).join(', ')}]`;

const formatPose = (pose: Pose | null) =>
  pose ? `p ${formatVector(pose.position)} m · qxyzw ${formatVector(pose.orientation)}` : '—';

export const DebugOverlay = ({
  controllableJointNames,
  ikResult,
  ikTargetPose,
  jointDeltaNorm,
  jointValues,
  pose,
  robotType,
  targetPose,
  tool0Pose,
}: DebugOverlayProps) => {
  return (
    <div className={robotType === 'ur5' ? 'debug-overlay debug-overlay-ur5' : 'debug-overlay'}>
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
        {controllableJointNames.length.toString().padStart(2, '0')} <small>active</small>
      </span>
      {robotType === 'ur5' ? (
        <div className="debug-ur5-details">
          <span>
            <b>ROBOT TYPE</b>
            {robotType}
          </span>
          {controllableJointNames.map((jointName) => (
            <span key={jointName}>
              <b>{jointName}</b>
              {(jointValues[jointName] ?? 0).toFixed(4)} <small>rad</small>
            </span>
          ))}
          <span className="debug-pose">
            <b>tool0 POSITION / ORIENTATION</b>
            {formatPose(tool0Pose)}
          </span>
          <span className="debug-pose">
            <b>umi_tcp POSITION / ORIENTATION</b>
            {formatPose(pose)}
          </span>
          <span className="debug-pose">
            <b>IK TARGET POSITION / ORIENTATION</b>
            {formatPose(ikTargetPose ?? targetPose)}
          </span>
          <span>
            <b>IK SUCCESS</b>
            {ikResult ? (ikResult.converged ? 'yes' : 'no') : 'not run'}
          </span>
          <span>
            <b>POSITION RESIDUAL</b>
            {ikResult ? `${ikResult.residual.position.toFixed(6)} m` : '—'}
          </span>
          <span>
            <b>ORIENTATION RESIDUAL</b>
            {ikResult ? `${ikResult.residual.orientation.toFixed(6)} rad` : '—'}
          </span>
          <span>
            <b>JOINT DELTA NORM</b>
            {jointDeltaNorm === null ? '—' : `${jointDeltaNorm.toFixed(6)} rad`}
          </span>
        </div>
      ) : null}
    </div>
  );
};
