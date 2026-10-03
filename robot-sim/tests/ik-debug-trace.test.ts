// @vitest-environment jsdom
import { describe, expect, it } from 'vitest';
import type { IkWorkerResponse } from '../src/app/inferenceProtocol';
import type { Pose } from '../src/core/types';
import { Kinematics } from '../src/core/Kinematics';
import { parseUrdf } from '../src/utils/urdfParser';
import { runIkSolve } from '../src/workers/umiIk.worker';

const planarUrdf = `
  <robot name="planar">
    <link name="base" />
    <link name="arm" />
    <link name="tool" />
    <joint name="shoulder" type="revolute">
      <parent link="base" />
      <child link="arm" />
      <axis xyz="0 0 1" />
      <limit lower="-2" upper="2" effort="10" velocity="2" />
    </joint>
    <joint name="elbow" type="revolute">
      <origin xyz="1 0 0" />
      <parent link="arm" />
      <child link="tool" />
      <axis xyz="0 0 1" />
      <limit lower="-2" upper="2" effort="10" velocity="2" />
    </joint>
  </robot>
`;

const description = parseUrdf(planarUrdf);
const seed = { shoulder: 0, elbow: 0 };
const startPose = new Kinematics(description).forwardKinematics(seed);

describe('IK 逐轮 trace', () => {
  it('为成功动作记录关联编号、输入 Seed 和目标位姿摘要', () => {
    const messages: IkWorkerResponse[] = [];
    runIkSolve(
      {
        type: 'solve',
        request_id: 'request-success',
        episode_index: 3,
        frame_index: 1,
        description,
        startPose,
        initialJointValues: seed,
        actions: [
          [0, 0, 0, 0, 0, 0, 0],
          [0, 0, 0, 0, 0, 0, 0],
        ],
        traceAllIterations: false,
      },
      (message) => messages.push(message),
    );

    const record = messages
      .flatMap((message) => (message.type === 'ik_record' ? [message.record] : []))
      .find((item) => item.action_index === 1 && item.record_phase === 'result');
    const startedRecords = messages.flatMap((message) =>
      message.type === 'ik_record' && message.record.record_phase === 'started'
        ? [message.record]
        : [],
    );

    expect(startedRecords).toHaveLength(2);
    expect(record).toMatchObject({
      record_phase: 'result',
      request_id: 'request-success',
      episode_index: 3,
      frame_index: 1,
      action_index: 1,
      status: 'success',
      trace_level: 'summary',
      action_pose_repr: null,
      action_pose_repr_status: 'unavailable',
      raw_action: [0, 0, 0, 0, 0, 0, 0],
      start_pose: startPose,
      target_pose: startPose,
      seed_joint_values: seed,
      termination_reason: 'converged',
    });
    expect(record?.previous_action_output_seed).toEqual(seed);
    expect(record?.final_joint_values).toEqual(seed);
  });

  it('为失败动作持久化退出原因和逐轮信息', () => {
    const messages: IkWorkerResponse[] = [];
    runIkSolve(
      {
        type: 'solve',
        request_id: 'request-failure',
        episode_index: 0,
        frame_index: 1,
        description,
        startPose,
        initialJointValues: seed,
        actions: [[5, 5, 5, 0, 0, 0, 0]],
        traceAllIterations: false,
      },
      (message) => messages.push(message),
    );

    const record = messages
      .flatMap((message) => (message.type === 'ik_record' ? [message.record] : []))
      .find((item) => item.record_phase === 'result');
    expect(record).toMatchObject({
      request_id: 'request-failure',
      frame_index: 1,
      action_index: 0,
      status: 'failure',
      trace_level: 'full',
      seed_joint_values: seed,
    });
    expect(typeof record?.termination_reason).toBe('string');
    expect(record?.iteration_trace?.length).toBeGreaterThan(0);
    expect(record?.iteration_trace?.[0]).toEqual(
      expect.objectContaining({
        iteration_index: 0,
        q_before: expect.any(Object),
        current_position_residual: expect.any(Number),
        current_orientation_residual: expect.any(Number),
        delta_q_raw: expect.any(Object),
        delta_q_after_step_limiting: expect.any(Object),
        q_after: expect.any(Object),
        blocked_joints: expect.any(Array),
        active_joints: expect.any(Array),
        clamp_information: expect.any(Object),
        exit_condition: expect.any(String),
      }),
    );
  });

  it('启用完整 trace 时也记录成功动作的逐轮信息', () => {
    const kinematics = new Kinematics(description);
    const targetPose = kinematics.forwardKinematics({ shoulder: 0.2, elbow: 0 });
    const rotationVector = [0, 0, 0.2];
    const messages: IkWorkerResponse[] = [];
    runIkSolve(
      {
        type: 'solve',
        request_id: 'request-debug',
        episode_index: 0,
        frame_index: 2,
        description,
        startPose,
        initialJointValues: seed,
        actions: [
          [
            targetPose.position[0] - startPose.position[0],
            targetPose.position[1] - startPose.position[1],
            targetPose.position[2] - startPose.position[2],
            ...rotationVector,
            0,
          ],
        ],
        traceAllIterations: true,
      },
      (message) => messages.push(message),
    );

    const record = messages
      .flatMap((message) => (message.type === 'ik_record' ? [message.record] : []))
      .find((item) => item.record_phase === 'result');

    expect(record).toMatchObject({ status: 'success', trace_level: 'full' });
    expect(record?.iteration_trace?.length).toBeGreaterThan(0);
  });

  it('启用逐轮观察时保持 IK 返回结果不变', () => {
    const kinematics = new Kinematics(description);
    const target: Pose = kinematics.forwardKinematics({ shoulder: 0.4, elbow: 0.2 });
    const withoutTrace = kinematics.solveIK(target, seed);
    const iterations: unknown[] = [];
    const withTrace = kinematics.solveIK(target, seed, {}, (iteration) => {
      iterations.push(iteration);
    });

    expect(iterations.length).toBeGreaterThan(0);
    expect(withTrace).toEqual(withoutTrace);
  });
});
