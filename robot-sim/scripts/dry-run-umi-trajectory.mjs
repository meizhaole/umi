// 用当前 UR5 Kinematics 对官方 UMI TCP 轨迹做独立只读 dry-run。
import { spawnSync } from 'node:child_process';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { JSDOM } from 'jsdom';
import { createServer } from 'vite';
import { Matrix4, Quaternion, Vector3 } from 'three';

const SCRIPT_DIRECTORY = dirname(fileURLToPath(import.meta.url));
const PROJECT_DIRECTORY = resolve(SCRIPT_DIRECTORY, '..');
const DATASET_READER = resolve(SCRIPT_DIRECTORY, 'test_umi_trajectory_dataset.py');
const REPORT_PATH = resolve(PROJECT_DIRECTORY, 'reports/phase2a-umi-episode0-frames0-99.jsonl');
const CONSISTENCY_TOLERANCE = 1e-9;
const JOINT_LIMIT_TOLERANCE = 1e-9;

const matrixFromRows = (rows) => new Matrix4().set(...rows.flat());

const matrixToRows = (matrix) => {
  const e = matrix.elements;
  return [
    [e[0], e[4], e[8], e[12]],
    [e[1], e[5], e[9], e[13]],
    [e[2], e[6], e[10], e[14]],
    [e[3], e[7], e[11], e[15]],
  ];
};

const poseToMatrix = (pose) =>
  new Matrix4().compose(
    new Vector3(...pose.position),
    new Quaternion(...pose.orientation).normalize(),
    new Vector3(1, 1, 1),
  );

const matrixToPose = (matrix) => {
  const position = new Vector3();
  const orientation = new Quaternion();
  const scale = new Vector3();
  matrix.decompose(position, orientation, scale);
  orientation.normalize();
  return {
    position: position.toArray(),
    orientation: [orientation.x, orientation.y, orientation.z, orientation.w],
  };
};

const matrixPosition = (matrix) => new Vector3().setFromMatrixPosition(matrix);

const matrixRotation = (matrix) => new Quaternion().setFromRotationMatrix(matrix).normalize();

const maxMatrixDifference = (left, right) =>
  Math.max(...left.elements.map((value, index) => Math.abs(value - right.elements[index])));

const hasNonFinite = (value) => {
  if (typeof value === 'number') return !Number.isFinite(value);
  if (Array.isArray(value)) return value.some(hasNonFinite);
  if (value && typeof value === 'object') return Object.values(value).some(hasNonFinite);
  return false;
};

const jsonSafe = (value) => {
  if (typeof value === 'number' && !Number.isFinite(value)) return String(value);
  if (Array.isArray(value)) return value.map(jsonSafe);
  if (value && typeof value === 'object') {
    return Object.fromEntries(Object.entries(value).map(([key, item]) => [key, jsonSafe(item)]));
  }
  return value;
};

const readDataset = () => {
  const pythonExecutable = process.env.CONDA_PREFIX
    ? resolve(process.env.CONDA_PREFIX, 'bin/python')
    : 'python3';
  const result = spawnSync(pythonExecutable, [DATASET_READER], {
    encoding: 'utf8',
    maxBuffer: 16 * 1024 * 1024,
  });
  if (result.error) throw result.error;
  if (result.status !== 0) {
    throw new Error(`UMI dataset 读取失败。请在 umi Conda 环境运行。${result.stderr || ''}`);
  }
  return JSON.parse(result.stdout);
};

const writeReport = async (report) => {
  const { frames, continuity_checks: continuityChecks, ...metadata } = report;
  const lines = [
    { record_type: 'run', ...metadata },
    ...frames.map((frame) => ({ record_type: 'frame', ...frame })),
    ...continuityChecks.map((check) => ({ record_type: 'continuity_check', ...check })),
    {
      record_type: 'summary',
      status: report.status,
      summary: report.summary,
      failure_diagnostic: report.failure_diagnostic,
    },
  ];
  await mkdir(dirname(REPORT_PATH), { recursive: true });
  await writeFile(
    REPORT_PATH,
    `${lines.map((line) => JSON.stringify(jsonSafe(line))).join('\n')}\n`,
  );
};

const getJointValues = (values, jointNames) =>
  Object.fromEntries(jointNames.map((name) => [name, values[name]]));

const getJointLimits = (description, jointNames) => {
  const joints = new Map(description.joints.map((joint) => [joint.name, joint]));
  return Object.fromEntries(
    jointNames.map((name) => [
      name,
      {
        lower: joints.get(name)?.limit?.lower ?? null,
        upper: joints.get(name)?.limit?.upper ?? null,
      },
    ]),
  );
};

const getLimitViolations = (jointValues, jointLimits) =>
  Object.entries(jointLimits)
    .filter(([name, limit]) => {
      const value = jointValues[name];
      return (
        (limit.lower !== null && value < limit.lower - JOINT_LIMIT_TOLERANCE) ||
        (limit.upper !== null && value > limit.upper + JOINT_LIMIT_TOLERANCE)
      );
    })
    .map(([name, limit]) => ({ joint: name, value: jointValues[name], ...limit }));

const getTranslationReachBound = (description) => {
  const jointsByChild = new Map(description.joints.map((joint) => [joint.child, joint]));
  let link = description.tipLink;
  let distanceBound = 0;
  while (link !== description.rootLink) {
    const joint = jointsByChild.get(link);
    if (!joint) throw new Error(`无法从 ${description.tipLink} 回溯到 URDF root`);
    distanceBound += Math.hypot(...joint.origin.position);
    link = joint.parent;
  }
  return distanceBound;
};

const clampToJointLimits = (values, limits) =>
  Object.fromEntries(
    Object.entries(values).map(([name, value]) => {
      const limit = limits[name];
      const lower = limit?.lower ?? Number.NEGATIVE_INFINITY;
      const upper = limit?.upper ?? Number.POSITIVE_INFINITY;
      return [name, Math.max(lower, Math.min(upper, value))];
    }),
  );

const makeDiagnosticSeeds = (seed, initialJoints, limits) => {
  const candidates = [
    ['configured_initial', initialJoints],
    [
      'shoulder_pan_plus_pi_2',
      { ...seed, shoulder_pan_joint: seed.shoulder_pan_joint + Math.PI / 2 },
    ],
    [
      'shoulder_pan_minus_pi_2',
      { ...seed, shoulder_pan_joint: seed.shoulder_pan_joint - Math.PI / 2 },
    ],
    ['elbow_plus_pi_2', { ...seed, elbow_joint: seed.elbow_joint + Math.PI / 2 }],
    ['elbow_minus_pi_2', { ...seed, elbow_joint: seed.elbow_joint - Math.PI / 2 }],
    ['wrist_1_plus_pi_2', { ...seed, wrist_1_joint: seed.wrist_1_joint + Math.PI / 2 }],
    ['wrist_1_minus_pi_2', { ...seed, wrist_1_joint: seed.wrist_1_joint - Math.PI / 2 }],
    ['wrist_3_plus_pi_2', { ...seed, wrist_3_joint: seed.wrist_3_joint + Math.PI / 2 }],
    ['wrist_3_minus_pi_2', { ...seed, wrist_3_joint: seed.wrist_3_joint - Math.PI / 2 }],
  ];
  const unique = new Set([JSON.stringify(seed)]);
  return candidates.flatMap(([name, offsets]) => {
    const values = clampToJointLimits(offsets, limits);
    const key = JSON.stringify(values);
    if (unique.has(key)) return [];
    unique.add(key);
    return [{ name, values }];
  });
};

const summarizeFailure = (result, targetPose, reachBound, diagnosticResults) => {
  const targetRadius = Math.hypot(...targetPose.position);
  const possibleCauses = [];
  const evidence = [];
  if (targetRadius > reachBound + CONSISTENCY_TOLERANCE) {
    possibleCauses.push('A: target 超出由 URDF joint origins 得到的保守最大半径上界');
    evidence.push({ target_radius_m: targetRadius, maximum_radius_bound_m: reachBound });
  }
  if (result.terminationReason === 'joint_limits_blocked' || result.blockedJoints.length > 0) {
    possibleCauses.push('B: 求解过程中关节限位阻挡了可用步长');
    evidence.push({ blocked_joints: result.blockedJoints });
  }
  if (result.terminationReason === 'linear_solve_failed') {
    possibleCauses.push('C/D: 线性求解失败；可能涉及奇异构型或数值病态');
  } else if (result.terminationReason === 'max_iterations') {
    possibleCauses.push('D: 达到现有最大迭代数仍未满足默认残差容差');
  } else if (result.terminationReason === 'no_joint_motion') {
    possibleCauses.push('B/C/D: 当前 Seed 没有可执行的关节步长，但残差仍未收敛');
  }
  if (diagnosticResults.some((item) => item.ik_success)) {
    possibleCauses.push('Seed 敏感：诊断性 alternate seed 找到解，正式连续 Seed 结果仍保留为失败');
  }
  if (possibleCauses.length === 0) {
    possibleCauses.push('现有结果不足以区分可达性、限位和数值收敛原因');
  }
  return {
    possible_causes: possibleCauses,
    evidence,
    target_radius_m: targetRadius,
    maximum_radius_bound_m: reachBound,
  };
};

const buildContinuityChecks = (datasetFrames, datasetMatrices, targetMatrices) => {
  const checks = [];
  for (let index = 1; index < datasetFrames.length; index += 1) {
    const datasetPrevious = datasetFrames[index - 1];
    const datasetCurrent = datasetFrames[index];
    const datasetPositionDelta = new Vector3(...datasetCurrent.position).sub(
      new Vector3(...datasetPrevious.position),
    );
    const targetPositionDelta = matrixPosition(targetMatrices[index]).sub(
      matrixPosition(targetMatrices[index - 1]),
    );
    const datasetRotationDelta = matrixRotation(datasetMatrices[index - 1]).angleTo(
      matrixRotation(datasetMatrices[index]),
    );
    const targetRotationDelta = matrixRotation(targetMatrices[index - 1]).angleTo(
      matrixRotation(targetMatrices[index]),
    );
    const relativeDataset = datasetMatrices[index - 1]
      .clone()
      .invert()
      .multiply(datasetMatrices[index]);
    const relativeTarget = targetMatrices[index - 1]
      .clone()
      .invert()
      .multiply(targetMatrices[index]);
    checks.push({
      from_frame: datasetPrevious.frame_index,
      to_frame: datasetCurrent.frame_index,
      dataset_position_delta: datasetPositionDelta.toArray(),
      dataset_position_delta_norm_m: datasetPositionDelta.length(),
      aligned_position_delta: targetPositionDelta.toArray(),
      aligned_position_delta_norm_m: targetPositionDelta.length(),
      position_delta_norm_error_m: Math.abs(
        datasetPositionDelta.length() - targetPositionDelta.length(),
      ),
      dataset_rotation_delta_rad: datasetRotationDelta,
      aligned_rotation_delta_rad: targetRotationDelta,
      rotation_delta_error_rad: Math.abs(datasetRotationDelta - targetRotationDelta),
      relative_transform_max_error: maxMatrixDifference(relativeDataset, relativeTarget),
    });
  }
  return checks;
};

const run = async () => {
  const dataset = readDataset();
  const invalidFrames = dataset.frames.filter((frame) => frame.non_finite_fields.length > 0);
  const dom = new JSDOM('');
  globalThis.DOMParser = dom.window.DOMParser;
  globalThis.XMLSerializer = dom.window.XMLSerializer;
  const vite = await createServer({
    configFile: resolve(PROJECT_DIRECTORY, 'vite.config.ts'),
    root: PROJECT_DIRECTORY,
    appType: 'custom',
    logLevel: 'error',
    server: { middlewareMode: true },
  });

  try {
    const [{ findRobotConfig }, { configureRobotUrdf }, { parseUrdf }, { Kinematics }] =
      await Promise.all([
        vite.ssrLoadModule('/src/app/config.ts'),
        vite.ssrLoadModule('/src/utils/configureRobotUrdf.ts'),
        vite.ssrLoadModule('/src/utils/urdfParser.ts'),
        vite.ssrLoadModule('/src/core/Kinematics.ts'),
      ]);
    const robotConfig = findRobotConfig('UR5');
    const urdfPath = resolve(PROJECT_DIRECTORY, 'assets', robotConfig.file);
    const sourceUrdf = await readFile(urdfPath, 'utf8');
    const configuredUrdf = configureRobotUrdf(sourceUrdf, robotConfig);
    const description = parseUrdf(configuredUrdf, { tipLink: robotConfig.tipLink });
    if (description.tipLink !== 'umi_tcp') {
      throw new Error(`URDF parser tipLink 异常：${description.tipLink}`);
    }

    const jointNames = Object.keys(robotConfig.initialJoints);
    const initialJoints = { ...robotConfig.initialJoints };
    const jointLimits = getJointLimits(description, jointNames);
    const kinematics = new Kinematics(description);
    const simulatorPose = kinematics.forwardKinematics(initialJoints, 'umi_tcp');
    const simulatorMatrix = poseToMatrix(simulatorPose);
    const datasetMatrices = dataset.frames.map((frame) =>
      frame.transform ? matrixFromRows(frame.transform) : null,
    );
    const report = {
      phase: '2A',
      dataset_path: dataset.dataset_path,
      dataset_type: dataset.dataset_type,
      episode_index: dataset.episode_index,
      episode_frame_range: [0, dataset.frame_count - 1],
      episode_end_exclusive: dataset.episode_end_exclusive,
      robot_type: robotConfig.type,
      root_link: description.rootLink,
      tip_link: description.tipLink,
      tcp_offset_m: robotConfig.tcpOffset,
      joint_names: jointNames,
      joint_limits: jointLimits,
      initial_joints: initialJoints,
      ik_options: {
        source: 'Kinematics.solveIK defaults; the dry-run passes an empty options object',
        max_iterations: 100,
        damping: 0.05,
        position_tolerance_m: 0.001,
        orientation_tolerance_rad: 0.01,
        max_angular_step_rad: 0.15,
        max_linear_step_m: 0.01,
      },
      alignment: null,
      continuity_tolerance: {
        position_delta_norm_m: CONSISTENCY_TOLERANCE,
        rotation_delta_rad: CONSISTENCY_TOLERANCE,
        relative_transform_component: CONSISTENCY_TOLERANCE,
      },
      continuity_checks: [],
      continuity_summary: null,
      frames: [],
      summary: null,
      failure_diagnostic: null,
      status: 'running',
    };

    if (invalidFrames.length > 0 || datasetMatrices.some((matrix) => matrix === null)) {
      report.status = 'stopped_non_finite_dataset';
      report.non_finite_frame_count = invalidFrames.length;
      report.non_finite_frames = invalidFrames.map((frame) => ({
        frame_index: frame.frame_index,
        fields: frame.non_finite_fields,
      }));
      await writeReport(report);
      console.log(JSON.stringify({ status: report.status, report_path: REPORT_PATH }, null, 2));
      return false;
    }

    const alignMatrix = simulatorMatrix.clone().multiply(datasetMatrices[0].clone().invert());
    const targetMatrices = datasetMatrices.map((matrix) => alignMatrix.clone().multiply(matrix));
    const targetZero = targetMatrices[0];
    const targetZeroPose = matrixToPose(targetZero);
    const simulatorQuaternion = new Quaternion(...simulatorPose.orientation).normalize();
    const targetZeroQuaternion = new Quaternion(...targetZeroPose.orientation).normalize();
    const targetZeroPositionError = new Vector3(...targetZeroPose.position).distanceTo(
      new Vector3(...simulatorPose.position),
    );
    const targetZeroOrientationError = targetZeroQuaternion.angleTo(simulatorQuaternion);
    const targetZeroMatrixError = maxMatrixDifference(targetZero, simulatorMatrix);
    const continuityChecks = buildContinuityChecks(dataset.frames, datasetMatrices, targetMatrices);
    const maxPositionDeltaError = Math.max(
      ...continuityChecks.map((check) => check.position_delta_norm_error_m),
    );
    const maxRotationDeltaError = Math.max(
      ...continuityChecks.map((check) => check.rotation_delta_error_rad),
    );
    const maxRelativeTransformError = Math.max(
      ...continuityChecks.map((check) => check.relative_transform_max_error),
    );
    const continuityPass =
      targetZeroPositionError <= CONSISTENCY_TOLERANCE &&
      targetZeroOrientationError <= CONSISTENCY_TOLERANCE &&
      targetZeroMatrixError <= CONSISTENCY_TOLERANCE &&
      maxPositionDeltaError <= CONSISTENCY_TOLERANCE &&
      maxRotationDeltaError <= CONSISTENCY_TOLERANCE &&
      maxRelativeTransformError <= CONSISTENCY_TOLERANCE;

    report.alignment = {
      definition: 'T_align = S_0 × inverse(D_0); Target_i = T_align × D_i',
      matrix: matrixToRows(alignMatrix),
      transform_position: matrixPosition(alignMatrix).toArray(),
      transform_quaternion: matrixRotation(alignMatrix).toArray(),
      frame_0_dataset_position: dataset.frames[0].position,
      frame_0_dataset_rotation_vector: dataset.frames[0].rotation_vector,
      frame_0_simulator_pose: simulatorPose,
      target_0_pose: targetZeroPose,
      target_0_error: {
        position_m: targetZeroPositionError,
        orientation_rad: targetZeroOrientationError,
        matrix_max_component: targetZeroMatrixError,
      },
    };
    report.continuity_checks = continuityChecks;
    report.continuity_summary = {
      comparison_count: continuityChecks.length,
      passed: continuityPass,
      max_position_delta_norm_error_m: maxPositionDeltaError,
      max_rotation_delta_error_rad: maxRotationDeltaError,
      max_relative_transform_component_error: maxRelativeTransformError,
    };

    if (!continuityPass) {
      report.status = 'stopped_transform_consistency_failure';
      report.failure_diagnostic = {
        possible_cause: 'E: SE(3) alignment or rotation conversion inconsistency',
        checked_tolerance: CONSISTENCY_TOLERANCE,
      };
      await writeReport(report);
      console.log(
        JSON.stringify(
          {
            status: report.status,
            continuity: report.continuity_summary,
            report_path: REPORT_PATH,
          },
          null,
          2,
        ),
      );
      return false;
    }

    const reachBound = getTranslationReachBound(description);
    let seed = { ...initialJoints };
    let failureRecord = null;
    for (let index = 0; index < dataset.frames.length; index += 1) {
      const frame = dataset.frames[index];
      const targetMatrix = targetMatrices[index];
      const targetPose = matrixToPose(targetMatrix);
      const iterationTrace = [];
      const result = kinematics.solveIK(targetPose, seed, {}, (iteration) => {
        iterationTrace.push(iteration);
      });
      const solution = getJointValues(result.jointValues, jointNames);
      const deltas = Object.fromEntries(
        jointNames.map((name) => [name, solution[name] - seed[name]]),
      );
      const deltaEntries = Object.entries(deltas);
      const maxSingleJointEntry = deltaEntries.reduce(
        (maximum, entry) => (Math.abs(entry[1]) > Math.abs(maximum[1]) ? entry : maximum),
        deltaEntries[0],
      );
      const violations = getLimitViolations(solution, jointLimits);
      const nonFiniteFields = [];
      if (hasNonFinite(frame)) nonFiniteFields.push('dataset_frame');
      if (hasNonFinite(targetPose)) nonFiniteFields.push('aligned_target_pose');
      if (hasNonFinite(seed)) nonFiniteFields.push('ik_seed_joints');
      if (hasNonFinite(solution)) nonFiniteFields.push('ik_solution_joints');
      if (hasNonFinite(result.residual)) nonFiniteFields.push('ik_residual');

      const record = {
        frame_index: frame.frame_index,
        dataset_position: frame.position,
        dataset_rotation_vector: frame.rotation_vector,
        aligned_target_position: targetPose.position,
        aligned_target_quaternion: targetPose.orientation,
        ik_seed_joints: getJointValues(seed, jointNames),
        ik_solution_joints: solution,
        ik_success: result.converged,
        ik_termination_reason: result.terminationReason,
        ik_iterations: result.iterations,
        position_residual_m: result.residual.position,
        orientation_residual_rad: result.residual.orientation,
        joint_delta_norm: Math.hypot(...Object.values(deltas)),
        joint_deltas: deltas,
        max_single_joint_delta: {
          joint: maxSingleJointEntry[0],
          value: maxSingleJointEntry[1],
        },
        joint_limit_violations: violations,
        blocked_joints: result.blockedJoints,
        non_finite_fields: nonFiniteFields,
      };
      report.frames.push(record);

      if (!result.converged || nonFiniteFields.length > 0) {
        failureRecord = {
          index,
          frame,
          targetPose,
          seed: { ...seed },
          result,
          record,
          iterationTrace,
        };
        break;
      }
      seed = solution;
    }

    const successfulFrames = report.frames.filter((frame) => frame.ik_success);
    const positionResiduals = report.frames.map((frame) => frame.position_residual_m);
    const orientationResiduals = report.frames.map((frame) => frame.orientation_residual_rad);
    const jointDeltaNorms = report.frames.map((frame) => frame.joint_delta_norm);
    const maximumJump = report.frames.reduce(
      (maximum, frame) => (frame.joint_delta_norm > maximum.joint_delta_norm ? frame : maximum),
      report.frames[0],
    );
    const maximumSingleJointStepFrame = report.frames.reduce(
      (maximum, frame) =>
        Math.abs(frame.max_single_joint_delta.value) >
        Math.abs(maximum.max_single_joint_delta.value)
          ? frame
          : maximum,
      report.frames[0],
    );
    report.summary = {
      tested_frames: report.frames.length,
      ik_success_count: successfulFrames.length,
      ik_failure_count: report.frames.length - successfulFrames.length,
      mean_position_residual_m: positionResiduals.length
        ? positionResiduals.reduce((sum, value) => sum + value, 0) / positionResiduals.length
        : null,
      max_position_residual_m: positionResiduals.length ? Math.max(...positionResiduals) : null,
      mean_orientation_residual_rad: orientationResiduals.length
        ? orientationResiduals.reduce((sum, value) => sum + value, 0) / orientationResiduals.length
        : null,
      max_orientation_residual_rad: orientationResiduals.length
        ? Math.max(...orientationResiduals)
        : null,
      mean_joint_delta_norm: jointDeltaNorms.length
        ? jointDeltaNorms.reduce((sum, value) => sum + value, 0) / jointDeltaNorms.length
        : null,
      max_joint_delta_norm: jointDeltaNorms.length ? Math.max(...jointDeltaNorms) : null,
      max_single_joint_step: maximumSingleJointStepFrame.max_single_joint_delta,
      max_single_joint_step_frame: maximumSingleJointStepFrame.frame_index,
      joint_limit_violation_count: report.frames.filter(
        (frame) => frame.joint_limit_violations.length > 0,
      ).length,
      nan_inf_count: report.frames.filter((frame) => frame.non_finite_fields.length > 0).length,
      first_failure_frame: failureRecord?.frame.frame_index ?? null,
      max_joint_jump_frame: maximumJump?.frame_index ?? null,
    };

    if (failureRecord) {
      const diagnosticResults = makeDiagnosticSeeds(
        failureRecord.seed,
        initialJoints,
        jointLimits,
      ).map(({ name, values }) => {
        const trace = [];
        const diagnostic = kinematics.solveIK(failureRecord.targetPose, values, {}, (iteration) =>
          trace.push(iteration),
        );
        return {
          seed_name: name,
          seed_joints: getJointValues(values, jointNames),
          solution_joints: getJointValues(diagnostic.jointValues, jointNames),
          ik_success: diagnostic.converged,
          termination_reason: diagnostic.terminationReason,
          position_residual_m: diagnostic.residual.position,
          orientation_residual_rad: diagnostic.residual.orientation,
          iterations: diagnostic.iterations,
          blocked_joints: diagnostic.blockedJoints,
        };
      });
      report.failure_diagnostic = {
        frame_index: failureRecord.frame.frame_index,
        dataset_pose: {
          position: failureRecord.frame.position,
          rotation_vector: failureRecord.frame.rotation_vector,
        },
        aligned_target_pose: failureRecord.targetPose,
        previous_target_pose:
          failureRecord.index > 0 ? matrixToPose(targetMatrices[failureRecord.index - 1]) : null,
        seed_joints: getJointValues(failureRecord.seed, jointNames),
        solution_joints: getJointValues(failureRecord.result.jointValues, jointNames),
        position_residual_m: failureRecord.result.residual.position,
        orientation_residual_rad: failureRecord.result.residual.orientation,
        termination_reason: failureRecord.result.terminationReason,
        blocked_joints: failureRecord.result.blockedJoints,
        joint_limits: jointLimits,
        solver_trace: failureRecord.iterationTrace,
        diagnostic_multi_seed: diagnosticResults,
        cause_assessment: summarizeFailure(
          failureRecord.result,
          failureRecord.targetPose,
          reachBound,
          diagnosticResults,
        ),
      };
      report.status = 'stopped_ik_failure';
    } else {
      report.status = 'complete';
    }

    await writeReport(report);
    console.log(
      JSON.stringify(
        {
          status: report.status,
          dataset_path: report.dataset_path,
          episode_index: report.episode_index,
          episode_frame_range: report.episode_frame_range,
          alignment: report.alignment,
          continuity_summary: report.continuity_summary,
          summary: report.summary,
          failure_diagnostic: report.failure_diagnostic,
          report_path: REPORT_PATH,
        },
        null,
        2,
      ),
    );
    return report.status === 'complete';
  } finally {
    await vite.close();
    dom.window.close();
  }
};

run()
  .then((passed) => {
    if (!passed) process.exitCode = 1;
  })
  .catch((error) => {
    console.error(error instanceof Error ? error.stack : String(error));
    process.exitCode = 1;
  });
