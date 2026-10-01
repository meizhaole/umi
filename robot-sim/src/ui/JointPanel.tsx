import type { ChangeEvent } from 'react';
import type { ControlMode, JointCommand, JointDescription, JointValues } from '../core/types';

interface JointPanelProps {
  joints: JointDescription[];
  jointValues: JointValues;
  commands: Record<string, JointCommand>;
  mode: ControlMode;
  onCommand: (jointName: string, value: number) => void;
  onModeChange: (mode: ControlMode) => void;
  disabled: boolean;
}

const MODES: Array<{ id: ControlMode; label: string }> = [
  { id: 'position', label: '位置' },
  { id: 'velocity', label: '速度' },
  { id: 'effort', label: '力矩' },
];

const controlRange = (joint: JointDescription, mode: ControlMode): [number, number] => {
  if (mode === 'position') {
    return [joint.limit?.lower ?? -Math.PI, joint.limit?.upper ?? Math.PI];
  }
  const maximum = mode === 'velocity' ? (joint.limit?.velocity ?? 2) : (joint.limit?.effort ?? 10);
  return [-maximum, maximum];
};

const formatJointName = (name: string): string =>
  name
    .replace(/^joint_/u, '')
    .replaceAll('_', ' ')
    .toUpperCase();

const jointUnit = (joint: JointDescription, mode: ControlMode): string => {
  if (joint.type === 'prismatic') {
    return mode === 'position' ? 'm' : mode === 'velocity' ? 'm/s' : 'N';
  }
  return mode === 'position' ? 'rad' : mode === 'velocity' ? 'rad/s' : 'N·m';
};

const rangeLabel = (
  range: [number, number],
  joint: JointDescription,
  mode: ControlMode,
): string => {
  const unit = jointUnit(joint, mode);
  return `${range[0].toFixed(1)} / ${range[1].toFixed(1)} ${unit}`;
};

export const JointPanel = ({
  joints,
  jointValues,
  commands,
  mode,
  onCommand,
  onModeChange,
  disabled,
}: JointPanelProps) => (
  <section className="panel joint-panel">
    <div className="panel-heading">
      <div>
        <p className="eyebrow">MANUAL CONTROL</p>
        <h2>关节控制</h2>
      </div>
      <span className="count-chip">{joints.length} JOINTS</span>
    </div>
    <div className="mode-switch" role="tablist" aria-label="控制模式">
      {MODES.map((item) => (
        <button
          aria-selected={item.id === mode}
          className={item.id === mode ? 'mode-tab active' : 'mode-tab'}
          key={item.id}
          disabled={disabled}
          onClick={() => onModeChange(item.id)}
          role="tab"
          type="button"
        >
          {item.label}
        </button>
      ))}
    </div>
    <div className="joint-list">
      {joints.length > 0 ? (
        joints.map((joint, index) => {
          const range = controlRange(joint, mode);
          const currentValue = jointValues[joint.name] ?? 0;
          const displayedValue =
            mode === 'position'
              ? commands[joint.name]?.mode === 'position'
                ? commands[joint.name].value
                : currentValue
              : (commands[joint.name]?.value ?? 0);
          const onChange = (event: ChangeEvent<HTMLInputElement>) => {
            onCommand(joint.name, Number(event.target.value));
          };

          return (
            <div className="joint-control" key={joint.name}>
              <div className="joint-topline">
                <div className="joint-name-wrap">
                  <span className={`joint-index index-${index + 1}`}>
                    {String(index + 1).padStart(2, '0')}
                  </span>
                  <span className="joint-name">{formatJointName(joint.name)}</span>
                </div>
                <output>
                  {displayedValue.toFixed(3)}
                  <small>{` ${jointUnit(joint, mode)}`}</small>
                </output>
              </div>
              <input
                aria-label={`${joint.name} ${mode}`}
                className="joint-range"
                disabled={disabled}
                max={range[1]}
                min={range[0]}
                onChange={onChange}
                step={mode === 'position' ? 0.005 : 0.01}
                type="range"
                value={displayedValue}
              />
              <div className="joint-range-labels">
                <span>{rangeLabel(range, joint, mode).split(' / ')[0]}</span>
                <span>{rangeLabel(range, joint, mode).split(' / ')[1]}</span>
              </div>
            </div>
          );
        })
      ) : (
        <p className="empty-note">加载机器人后显示可控关节</p>
      )}
    </div>
    <div className="joint-footer">
      <span className="pulse-dot" />
      命令受 URDF 关节限位约束
    </div>
  </section>
);
