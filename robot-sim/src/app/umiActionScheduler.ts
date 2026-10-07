export const OFFICIAL_UMI_FREQUENCY_HZ = 10;
export const OFFICIAL_UMI_STEPS_PER_INFERENCE = 6;
export const OFFICIAL_ACTION_EXEC_LATENCY_SECONDS = 0.01;
export const OFFICIAL_FRAME_LATENCY_SECONDS = 1 / 60;

export interface UmiActionScheduleInput {
  actionCount: number;
  observationTimestampMs: number;
  inferenceRequestSentTimestampMs: number;
  inferenceResponseReceivedTimestampMs: number;
  schedulerCurrentTimestampMs?: number;
  episodeStartTimestampMs: number;
  iterationIndex: number;
  frequencyHz?: number;
  stepsPerInference?: number;
  actionExecLatencySeconds?: number;
  simulatedCommandLatencySeconds?: number;
  frameLatencySeconds?: number;
}

export interface UmiActionTiming {
  actionIndex: number;
  actionTimestampMs: number;
  effectiveScheduledTimestampMs: number | null;
  state: 'EXPIRED' | 'EXECUTABLE' | 'FALLBACK';
}

export interface UmiActionSchedule {
  frequencyHz: number;
  dtMs: number;
  stepsPerInference: number;
  actionExecLatencySeconds: number;
  simulatedCommandLatencySeconds: number;
  actionTimestampsMs: number[];
  expiredActionIndexes: number[];
  executableActionIndexes: number[];
  selectedActionIndexes: number[];
  fallbackActionIndex: number | null;
  fallbackTimestampMs: number | null;
  actionTimings: UmiActionTiming[];
  nextReplanTimestampMs: number;
  timingDiagnostics: {
    clockDomain: 'window.performance.now';
    schedulerCurrentTimestampMs: number;
    staleCutoffTimestampMs: number;
    inferenceRoundTripMs: number;
  };
}

export const scheduleUmiActions = (input: UmiActionScheduleInput): UmiActionSchedule => {
  const frequencyHz = input.frequencyHz ?? OFFICIAL_UMI_FREQUENCY_HZ;
  const stepsPerInference = input.stepsPerInference ?? OFFICIAL_UMI_STEPS_PER_INFERENCE;
  const actionExecLatencySeconds =
    input.actionExecLatencySeconds ?? OFFICIAL_ACTION_EXEC_LATENCY_SECONDS;
  const simulatedCommandLatencySeconds = input.simulatedCommandLatencySeconds ?? 0;
  const frameLatencySeconds = input.frameLatencySeconds ?? OFFICIAL_FRAME_LATENCY_SECONDS;
  const requiredTimestamps = [
    input.observationTimestampMs,
    input.inferenceRequestSentTimestampMs,
    input.inferenceResponseReceivedTimestampMs,
    ...(input.schedulerCurrentTimestampMs === undefined ? [] : [input.schedulerCurrentTimestampMs]),
    input.episodeStartTimestampMs,
  ];

  if (
    !Number.isInteger(input.actionCount) ||
    input.actionCount < 1 ||
    !Number.isFinite(frequencyHz) ||
    frequencyHz <= 0 ||
    !Number.isInteger(stepsPerInference) ||
    stepsPerInference < 1 ||
    !Number.isInteger(input.iterationIndex) ||
    input.iterationIndex < 0 ||
    !Number.isFinite(actionExecLatencySeconds) ||
    actionExecLatencySeconds < 0 ||
    !Number.isFinite(simulatedCommandLatencySeconds) ||
    simulatedCommandLatencySeconds < 0 ||
    !Number.isFinite(frameLatencySeconds) ||
    frameLatencySeconds < 0 ||
    !requiredTimestamps.every(Number.isFinite)
  ) {
    throw new Error('UMI action scheduler received invalid timing input.');
  }

  const dtMs = 1000 / frequencyHz;
  const actionTimestampsMs = Array.from(
    { length: input.actionCount },
    (_, actionIndex) => input.observationTimestampMs + actionIndex * dtMs,
  );
  const schedulerCurrentTimestampMs =
    input.schedulerCurrentTimestampMs ?? input.inferenceResponseReceivedTimestampMs;
  const staleCutoffTimestampMs = schedulerCurrentTimestampMs + actionExecLatencySeconds * 1000;
  const expiredActionIndexes = actionTimestampsMs.flatMap((timestamp, actionIndex) =>
    timestamp > staleCutoffTimestampMs ? [] : [actionIndex],
  );
  const executableActionIndexes = actionTimestampsMs.flatMap((timestamp, actionIndex) =>
    timestamp > staleCutoffTimestampMs ? [actionIndex] : [],
  );

  let fallbackActionIndex: number | null = null;
  let fallbackTimestampMs: number | null = null;
  let selectedActionIndexes = executableActionIndexes;

  if (executableActionIndexes.length === 0) {
    fallbackActionIndex = input.actionCount - 1;
    const dtSeconds = 1 / frequencyHz;
    const nextStepIndex = Math.ceil(
      (schedulerCurrentTimestampMs - input.episodeStartTimestampMs) / dtMs,
    );
    fallbackTimestampMs = input.episodeStartTimestampMs + nextStepIndex * dtSeconds * 1000;
    selectedActionIndexes = [fallbackActionIndex];
  }

  const actionTimings = actionTimestampsMs.map((actionTimestampMs, actionIndex) => {
    if (actionIndex === fallbackActionIndex) {
      return {
        actionIndex,
        actionTimestampMs,
        effectiveScheduledTimestampMs: fallbackTimestampMs,
        state: 'FALLBACK' as const,
      };
    }
    const isExecutable = executableActionIndexes.includes(actionIndex);
    return {
      actionIndex,
      actionTimestampMs,
      effectiveScheduledTimestampMs: isExecutable
        ? actionTimestampMs - simulatedCommandLatencySeconds * 1000
        : null,
      state: isExecutable ? ('EXECUTABLE' as const) : ('EXPIRED' as const),
    };
  });

  const nextReplanTimestampMs =
    input.episodeStartTimestampMs +
    (input.iterationIndex + stepsPerInference) * dtMs -
    frameLatencySeconds * 1000;

  return {
    frequencyHz,
    dtMs,
    stepsPerInference,
    actionExecLatencySeconds,
    simulatedCommandLatencySeconds,
    actionTimestampsMs,
    expiredActionIndexes,
    executableActionIndexes,
    selectedActionIndexes,
    fallbackActionIndex,
    fallbackTimestampMs,
    actionTimings,
    nextReplanTimestampMs,
    timingDiagnostics: {
      clockDomain: 'window.performance.now',
      schedulerCurrentTimestampMs,
      staleCutoffTimestampMs,
      inferenceRoundTripMs:
        input.inferenceResponseReceivedTimestampMs - input.inferenceRequestSentTimestampMs,
    },
  };
};
