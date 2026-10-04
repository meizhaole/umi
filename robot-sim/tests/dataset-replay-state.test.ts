import { describe, expect, it } from 'vitest';
import {
  classifyDatasetReplaySettlement,
  DATASET_REPLAY_MAX_TRACKING_ERROR_RAD,
  DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC,
} from '../src/app/datasetReplay';

describe('dataset replay settlement', () => {
  it('continues moving while joint velocity exceeds the settled threshold', () => {
    expect(
      classifyDatasetReplaySettlement(DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC + 0.001, 0),
    ).toBeNull();
  });

  it('reports tracking error when slow joints remain outside the tracking tolerance', () => {
    expect(
      classifyDatasetReplaySettlement(
        DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC,
        DATASET_REPLAY_MAX_TRACKING_ERROR_RAD,
      ),
    ).toBe('TRACKING_ERROR');
  });

  it('settles only when both velocity and tracking error are within thresholds', () => {
    expect(
      classifyDatasetReplaySettlement(
        DATASET_REPLAY_MAX_VELOCITY_RAD_PER_SEC,
        DATASET_REPLAY_MAX_TRACKING_ERROR_RAD - 0.000001,
      ),
    ).toBe('SETTLED');
  });
});
