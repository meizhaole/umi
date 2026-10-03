export const EXECUTION_HORIZON = 6;

export interface ExecutionSelection {
  actions: number[][];
  predictionCount: number;
  configuredExecutionHorizon: number;
  executionCount: number;
  selectedActionIndexes: number[];
}

export const selectExecutionActions = (
  predictedActions: number[][],
  executionHorizon = EXECUTION_HORIZON,
): ExecutionSelection => {
  const executionCount = Math.min(executionHorizon, predictedActions.length);
  const selectedActionIndexes = Array.from({ length: executionCount }, (_, index) => index);

  return {
    actions: predictedActions.slice(0, executionCount),
    predictionCount: predictedActions.length,
    configuredExecutionHorizon: executionHorizon,
    executionCount,
    selectedActionIndexes,
  };
};
