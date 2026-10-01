import type { IKResult, Pose } from '../core/types';

interface TaskPanelProps {
  targetPose: Pose;
  ikResult: IKResult | null;
  ikMessage: string;
  onTargetChange: (field: 'position' | 'orientation', index: number, value: number) => void;
  onUseCurrentPose: () => void;
  onSolveIk: () => void;
}

const poseFields = [
  { key: 'position' as const, labels: ['X', 'Y', 'Z'], unit: 'm' },
  { key: 'orientation' as const, labels: ['QX', 'QY', 'QZ', 'QW'], unit: '' },
];

export const TaskPanel = ({
  targetPose,
  ikResult,
  ikMessage,
  onTargetChange,
  onUseCurrentPose,
  onSolveIk,
}: TaskPanelProps) => (
  <section className="panel ik-panel">
    <div className="panel-heading">
      <div>
        <p className="eyebrow">MOTION PLANNING</p>
        <h2>末端位姿 IK</h2>
      </div>
      <span className="ik-chip">DLS</span>
    </div>
    <div className="ik-form">
      {poseFields.map(({ key, labels, unit }) => (
        <div className="pose-field-row" key={key}>
          <span className="pose-field-name">{key === 'position' ? '位置' : '四元数'}</span>
          <div className="pose-inputs">
            {labels.map((label, index) => (
              <label key={label}>
                <span>{label}</span>
                <input
                  aria-label={`${key} ${label}`}
                  onChange={(event) => onTargetChange(key, index, Number(event.target.value))}
                  step={key === 'position' ? 0.01 : 0.05}
                  type="number"
                  value={targetPose[key][index]}
                />
              </label>
            ))}
          </div>
          {unit ? <span className="pose-unit">{unit}</span> : null}
        </div>
      ))}
    </div>
    <div className="ik-actions">
      <button className="secondary-button" onClick={onUseCurrentPose} type="button">
        取当前 TCP
      </button>
      <button className="primary-button" onClick={onSolveIk} type="button">
        求解位姿 <span>↗</span>
      </button>
    </div>
    {ikMessage ? (
      <div className={ikResult?.converged ? 'ik-result success' : 'ik-result'}>
        <span className="result-indicator" />
        <span>{ikMessage}</span>
        {ikResult ? <small>姿态残差 {ikResult.residual.orientation.toFixed(3)} rad</small> : null}
      </div>
    ) : null}
  </section>
);
