import type { InferenceError, InferenceStatus } from '../app/inferenceProtocol';

interface InferencePanelProps {
  status: InferenceStatus;
  progress: string;
  error: InferenceError | null;
  isLocked: boolean;
  canStart: boolean;
  onStart: () => void;
  onStop: () => void;
}

const STATUS_LABELS: Record<InferenceStatus, string> = {
  idle: '未连接',
  connecting: '正在连接',
  loading: '加载权重',
  computing: '求解 IK',
  playing: '动作播放中',
  waiting: '等待下一动作',
  complete: '回放完成',
  stopped: '已停止',
  error: '发生错误',
};

export const InferencePanel = ({
  status,
  progress,
  error,
  isLocked,
  canStart,
  onStart,
  onStop,
}: InferencePanelProps) => (
  <section className="panel inference-panel">
    <div className="panel-heading">
      <div>
        <p className="eyebrow">OFFICIAL UMI CHECKPOINT</p>
        <h2>策略回放</h2>
      </div>
      <span className={'inference-status ' + status}>{STATUS_LABELS[status]}</span>
    </div>
    <p className="inference-progress" aria-live="polite">
      {progress || '使用官方权重逐块订阅动作并驱动当前机械臂'}
    </p>
    <div className="inference-actions">
      <button
        className="primary-button"
        disabled={!canStart || isLocked}
        onClick={onStart}
        type="button"
      >
        开始回放
      </button>
      <button
        className="secondary-button"
        disabled={!isLocked}
        onClick={onStop}
        type="button"
      >
        停止
      </button>
    </div>
    {error ? (
      <div className="inference-error" role="alert">
        <strong>{error.message}</strong>
        <pre>{JSON.stringify(error, null, 2)}</pre>
      </div>
    ) : null}
  </section>
);
