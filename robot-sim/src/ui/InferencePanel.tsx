import type { InferenceError, InferenceStatus } from '../app/inferenceProtocol';
import type { AutoLoopState, AutoLoopStepLimit } from '../app/useInferenceReplay';

interface InferencePanelProps {
  status: InferenceStatus;
  progress: string;
  error: InferenceError | null;
  isLocked: boolean;
  canStep: boolean;
  canExecuteAction: boolean;
  onStep: () => void;
  singleActionDebug: boolean;
  singleActionReview: boolean;
  reviewData: Record<string, unknown> | null;
  onExecuteAction: () => void;
  onStop: () => void;
  autoLoop: AutoLoopState;
  canStartAutoLoop: boolean;
  canStopAutoLoop: boolean;
  onStartAutoLoop: (maxSteps: AutoLoopStepLimit) => void;
  onStopAutoLoop: () => void;
}

const STATUS_LABELS: Record<InferenceStatus, string> = {
  idle: '未连接',
  connecting: '正在连接',
  loading: '加载权重',
  inferring: '模型推理中',
  computing: '求解 IK',
  review: '待验收',
  playing: '动作播放中',
  waiting: '等待点击下一步',
  complete: '推理完成',
  stopped: '已停止',
  error: '发生错误',
};

export const InferencePanel = ({
  status,
  progress,
  error,
  isLocked,
  canStep,
  canExecuteAction,
  onStep,
  singleActionDebug,
  singleActionReview,
  reviewData,
  onExecuteAction,
  onStop,
  autoLoop,
  canStartAutoLoop,
  canStopAutoLoop,
  onStartAutoLoop,
  onStopAutoLoop,
}: InferencePanelProps) => (
  <section className="panel inference-panel">
    <div className="panel-heading">
      <div>
        <p className="eyebrow">OFFICIAL UMI CHECKPOINT</p>
        <h2>相机在线推理</h2>
      </div>
      <span className={'inference-status ' + status}>{STATUS_LABELS[status]}</span>
    </div>
    <p className="inference-progress" aria-live="polite">
      {progress || '每次点击执行一个模型动作，完成后再次点击继续'}
    </p>
    <div className="inference-actions">
      <button className="primary-button" disabled={!canStep} onClick={onStep} type="button">
        {singleActionDebug ? '单动作验证（策略推理与 IK dry-run）' : '推理一步'}
      </button>
      {singleActionDebug ? (
        <>
          <button
            className="primary-button"
            disabled={!canStartAutoLoop}
            onClick={() => onStartAutoLoop(3)}
            type="button"
          >
            自动闭环 3 步
          </button>
          <button
            className="primary-button"
            disabled={!canStartAutoLoop}
            onClick={() => onStartAutoLoop(10)}
            type="button"
          >
            自动闭环 10 步
          </button>
          <p className="inference-progress" aria-live="polite">
            自动闭环：{autoLoop.step || 0} / {autoLoop.maxSteps} · {autoLoop.phase}
          </p>
          <button
            className="secondary-button"
            disabled={!canStopAutoLoop}
            onClick={onStopAutoLoop}
            type="button"
          >
            停止自动闭环
          </button>
        </>
      ) : null}
      {reviewData ? (
        <pre className="inference-review">{JSON.stringify(reviewData, null, 2)}</pre>
      ) : null}
      {singleActionReview ? (
        <button
          className="primary-button"
          disabled={!canExecuteAction}
          onClick={onExecuteAction}
          type="button"
        >
          执行单动作（人工确认）
        </button>
      ) : null}
      <button className="secondary-button" disabled={!isLocked} onClick={onStop} type="button">
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
