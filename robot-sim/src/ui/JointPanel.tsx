import type { ChangeEvent } from 'react';
import type { ControlMode, JointCommand, JointDescription, JointValues } from '../core/types';
import { rotateVector } from '../utils/math';

interface JointPanelProps {
  joints: JointDescription[];
  jointValues: JointValues;
  commands: Record<string, JointCommand>;
  mode: ControlMode;
  onCommand: (jointName: string, value: number) => void;
  onModeChange: (mode: ControlMode) => void;
  disabled: boolean;
}

interface GripperControl {
  left: JointDescription;
  right: JointDescription;
  rightMultiplier: number;
  range: [number, number];
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

const getGripperControl = (joints: JointDescription[]): GripperControl | null => {
  const left = joints.find((joint) => joint.name === 'left_jaw_joint');
  const right = joints.find((joint) => joint.name === 'right_jaw_joint');
  if (
    !left ||
    !right ||
    left.type !== 'prismatic' ||
    right.type !== 'prismatic' ||
    left.parent !== right.parent ||
    left.limit?.lower === undefined ||
    left.limit.upper === undefined ||
    right.limit?.lower === undefined ||
    right.limit.upper === undefined
  ) {
    return null;
  }

  const leftAxis = rotateVector(left.origin.orientation, left.axis);
  const rightAxis = rotateVector(right.origin.orientation, right.axis);
  const axisProduct = Math.hypot(...leftAxis) * Math.hypot(...rightAxis);
  if (axisProduct === 0) return null;
  const axisAlignment =
    leftAxis.reduce((sum, value, index) => sum + value * rightAxis[index], 0) / axisProduct;
  if (Math.abs(axisAlignment) < 0.999) return null;

  const rightMultiplier = axisAlignment < 0 ? 1 : -1;
  const rightRange: [number, number] =
    rightMultiplier > 0
      ? [right.limit.lower, right.limit.upper]
      : [-right.limit.upper, -right.limit.lower];
  const range: [number, number] = [
    Math.max(left.limit.lower, rightRange[0]),
    Math.min(left.limit.upper, rightRange[1]),
  ];
  if (range[0] > range[1]) return null;

  return { left, right, rightMultiplier, range };
};

export const JointPanel = ({
  joints,
  jointValues,
  commands,
  mode,
  onCommand,
  onModeChange,
  disabled,
}: JointPanelProps) => {
  const gripper = getGripperControl(joints);
  const individualJoints = gripper
    ? joints.filter(
        (joint) => joint.name !== gripper.left.name && joint.name !== gripper.right.name,
      )
    : joints;
  const gripperValue = gripper
    ? commands[gripper.left.name]?.mode === 'position'
      ? commands[gripper.left.name].value
      : (jointValues[gripper.left.name] ?? gripper.range[0])
    : 0;

  return (
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
      {gripper ? (
        <div className="gripper-control">
          <div className="joint-topline">
            <span className="joint-name">Gripper opening</span>
            <output>
              {Math.round(gripperValue * 1000)}
              <small> mm</small>
            </output>
          </div>
          <input
            aria-label="Gripper opening"
            className="joint-range"
            data-testid="gripper-opening-slider"
            disabled={disabled || mode !== 'position'}
            max={gripper.range[1]}
            min={gripper.range[0]}
            onChange={(event: ChangeEvent<HTMLInputElement>) => {
              const value = Number(event.target.value);
              onCommand(gripper.left.name, value);
              onCommand(gripper.right.name, gripper.rightMultiplier * value);
            }}
            step={0.001}
            type="range"
            value={gripperValue}
          />
          <div className="joint-range-labels">
            <span>{Math.round(gripper.range[0] * 1000)} mm</span>
            <span>{Math.round(gripper.range[1] * 1000)} mm</span>
          </div>
          {mode !== 'position' ? (
            <small className="gripper-hint">切换到位置模式以控制开合</small>
          ) : null}
        </div>
      ) : null}
      <div className="joint-list">
        {individualJoints.length > 0 ? (
          individualJoints.map((joint, index) => {
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
};
