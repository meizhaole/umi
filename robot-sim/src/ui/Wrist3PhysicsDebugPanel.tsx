import type { Wrist3PhysicsDebugState } from '../sim/RobotBody';

interface Wrist3PhysicsDebugPanelProps {
  commandValue: number | null;
  state: Wrist3PhysicsDebugState | null;
}

const formatNumber = (value: number | null, digits = 6): string =>
  value === null || !Number.isFinite(value) ? '—' : value.toFixed(digits);

const formatVector = (values: [number, number, number] | undefined): string =>
  values ? `[${values.map((value) => formatNumber(value, 4)).join(', ')}]` : '—';

export const Wrist3PhysicsDebugPanel = ({
  commandValue,
  state,
}: Wrist3PhysicsDebugPanelProps) => {
  const bodyAngle = state?.bodyAngle ?? null;
  const error = commandValue !== null && bodyAngle !== null ? commandValue - bodyAngle : null;
  const following = error !== null && Math.abs(error) < 0.01;
  const primaryReadings = [
    { label: 'COMMAND', value: commandValue, unit: 'rad' },
    { label: 'READBACK', value: state?.readback ?? null, unit: 'rad' },
    { label: 'BODY ANGLE', value: state?.bodyAngle ?? null, unit: 'rad' },
    { label: 'ERROR', value: error, unit: 'rad' },
  ];
  const physicalReadings = [
    {
      label: 'BODY ANGULAR VELOCITY',
      value: formatVector(state?.childAngularVelocity),
      unit: 'rad/s',
    },
    {
      label: 'SWING ERROR',
      value: formatNumber(state?.swingErrorRad ?? null),
      unit: 'rad',
    },
    {
      label: 'ANCHOR ERROR',
      value: formatNumber(state?.anchorErrorM ?? null),
      unit: 'm',
    },
  ];
  const handles = [
    { label: 'JOINT HANDLE', value: state?.jointHandle },
    { label: 'PARENT BODY HANDLE', value: state?.parentBodyHandle },
    { label: 'CHILD BODY HANDLE', value: state?.childBodyHandle },
  ];

  return (
    <section className="wrist3-physics-debug" aria-label="Wrist 3 physics debug">
      <header className="wrist3-debug-header">
        <span>RAPIER LIVE READBACK</span>
        <h2>WRIST 3 PHYSICS DEBUG</h2>
      </header>

      <div className={following ? 'wrist3-following following' : 'wrist3-following'}>
        <span className="wrist3-following-dot" />
        <strong>{following ? 'FOLLOWING' : 'NOT FOLLOWING'}</strong>
        <small>LIMIT 0.010 rad</small>
      </div>

      <div className="wrist3-primary-readings">
        {primaryReadings.map((reading) => (
          <div className="wrist3-primary-reading" key={reading.label}>
            <b>{reading.label}</b>
            <output>
              {formatNumber(reading.value)} <small>{reading.unit}</small>
            </output>
          </div>
        ))}
      </div>

      <div className="wrist3-secondary-readings">
        {physicalReadings.map((reading) => (
          <div className="wrist3-secondary-reading" key={reading.label}>
            <b>{reading.label}</b>
            <output>
              {reading.value} <small>{reading.unit}</small>
            </output>
          </div>
        ))}
      </div>

      <div className="wrist3-handle-readings">
        {handles.map((handle) => (
          <div className="wrist3-handle-reading" key={handle.label}>
            <b>{handle.label}</b>
            <output>{handle.value ?? '—'}</output>
          </div>
        ))}
      </div>
    </section>
  );
};
