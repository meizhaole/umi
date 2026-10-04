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
const BASELINE_REPORT_PATH = resolve(
  PROJECT_DIRECTORY,
  'reports/phase2a-umi-episode0-frames0-99.jsonl',
);
const REPORT_PATH = resolve(PROJECT_DIRECTORY, 'reports/phase2b1-umi-episode0-frames0-399.jsonl');
const CONSISTENCY_TOLERANCE = 1e-9;
const JOINT_LIMIT_TOLERANCE = 1e-9;
const REGRESSION_TOLERANCE = 1e-9;
const REGRESSION_FRAME_INDICES = [0, 65, 99];

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

const getJointLimitMargins = (jointValues, jointLimits) =>
  Object.fromEntries(
    Object.entries(jointLimits).map(([name, limit]) => {
      const value = jointValues[name];
      const lowerMargin = limit.lower === null ? null : value - limit.lower;
      const upperMargin = limit.upper === null ? null : limit.upper - value;
      const availableMargins = [lowerMargin, upperMargin].filter(Number.isFinite);
      const minimumMargin = availableMargins.length ? Math.min(...availableMargins) : null;
      return [
        name,
        {
          lower_margin_rad: lowerMargin,
          upper_margin_rad: upperMargin,
          minimum_margin_rad: minimumMargin,
          nearest_limit:
            minimumMargin === null
              ? null
              : lowerMargin === null || upperMargin < lowerMargin
                ? 'upper'
                : 'lower',
        },
      ];
    }),
  );

const getVectorBounds = (vectors) => {
  const minimum = [Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY];
  const maximum = [Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY];
  vectors.forEach((vector) => {
    vector.forEach((value, axis) => {
      minimum[axis] = Math.min(minimum[axis], value);
      maximum[axis] = Math.max(maximum[axis], value);
    });
  });
  return {
    min_m: minimum,
    max_m: maximum,
    size_m: maximum.map((value, axis) => value - minimum[axis]),
  };
};

const getMaximumFrame = (frames, field, valueSelector = (frame) => frame[field]) => {
  const finiteFrames = frames.filter((frame) => Number.isFinite(valueSelector(frame)));
  if (finiteFrames.length === 0) return null;
  return finiteFrames.reduce((maximum, frame) =>
    valueSelector(frame) > valueSelector(maximum) ? frame : maximum,
  );
};

const getFiniteMean = (values) => {
  const finiteValues = values.filter(Number.isFinite);
  return finiteValues.length
    ? finiteValues.reduce((sum, value) => sum + value, 0) / finiteValues.length
    : null;
};

const getJointStatistics = (frames, jointNames, jointLimits) => {
  const statistics = {};
  jointNames.forEach((name) => {
    const jointFrames = frames.filter((frame) => Number.isFinite(frame.ik_solution_joints[name]));
    if (jointFrames.length === 0) {
      statistics[name] = {
        min_angle_rad: null,
        min_angle_frame: null,
        max_angle_rad: null,
        max_angle_frame: null,
        minimum_limit_margin_rad: null,
        minimum_limit_margin_frame: null,
        nearest_limit: null,
      };
      return;
    }

    const minimumAngleFrame = jointFrames.reduce((minimum, frame) =>
      frame.ik_solution_joints[name] < minimum.ik_solution_joints[name] ? frame : minimum,
    );
    const maximumAngleFrame = jointFrames.reduce((maximum, frame) =>
      frame.ik_solution_joints[name] > maximum.ik_solution_joints[name] ? frame : maximum,
    );
    const minimumMarginFrame = jointFrames.reduce((minimum, frame) =>
      frame.joint_limit_margins[name].minimum_margin_rad <
      minimum.joint_limit_margins[name].minimum_margin_rad
        ? frame
        : minimum,
    );

    statistics[name] = {
      min_angle_rad: minimumAngleFrame.ik_solution_joints[name],
      min_angle_frame: minimumAngleFrame.frame_index,
      max_angle_rad: maximumAngleFrame.ik_solution_joints[name],
      max_angle_frame: maximumAngleFrame.frame_index,
      lower_limit_rad: jointLimits[name].lower,
      upper_limit_rad: jointLimits[name].upper,
      minimum_limit_margin_rad: minimumMarginFrame.joint_limit_margins[name].minimum_margin_rad,
      minimum_limit_margin_frame: minimumMarginFrame.frame_index,
      nearest_limit: minimumMarginFrame.joint_limit_margins[name].nearest_limit,
    };
  });
  return statistics;
};

const getMedian = (values) => {
  const sorted = [...values].sort((left, right) => left - right);
  if (sorted.length === 0) return 0;
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2 === 0 ? (sorted[middle - 1] + sorted[middle]) / 2 : sorted[middle];
};

const readBaselineFrames = async () => {
  const lines = (await readFile(BASELINE_REPORT_PATH, 'utf8')).trim().split('\n');
  return new Map(
    lines
      .map((line) => JSON.parse(line))
      .filter((record) => record.record_type === 'frame')
      .map((frame) => [frame.frame_index, frame]),
  );
};

const compareRegressionFrame = (frame, baseline) => {
  const numericDifferences = [];
  const addDifference = (value, reference) => {
    numericDifferences.push(Math.abs(value - reference));
  };
  const compareJointMap = (values, reference) => {
    Object.keys(reference).forEach((name) => addDifference(values[name], reference[name]));
  };
  const compareVector = (values, reference) =>
    values.forEach((value, index) => addDifference(value, reference[index]));

  compareVector(frame.dataset_position, baseline.dataset_position);
  compareVector(frame.dataset_rotation_vector, baseline.dataset_rotation_vector);
  compareVector(frame.aligned_target_position, baseline.aligned_target_position);
  compareJointMap(frame.ik_seed_joints, baseline.ik_seed_joints);
  compareJointMap(frame.ik_solution_joints, baseline.ik_solution_joints);
  compareJointMap(frame.joint_deltas, baseline.joint_deltas);
  addDifference(frame.position_residual_m, baseline.position_residual_m);
  addDifference(frame.orientation_residual_rad, baseline.orientation_residual_rad);
  addDifference(frame.joint_delta_norm, baseline.joint_delta_norm);

  const quaternion = new Quaternion(...frame.aligned_target_quaternion).normalize();
  const baselineQuaternion = new Quaternion(...baseline.aligned_target_quaternion).normalize();
  const orientationDifferenceRad = quaternion.angleTo(baselineQuaternion);
  const maximumNumericDifference = Math.max(...numericDifferences);
  const stateMatches =
    frame.ik_success === baseline.ik_success &&
    frame.ik_termination_reason === baseline.ik_termination_reason &&
    frame.ik_iterations === baseline.ik_iterations &&
    JSON.stringify(frame.joint_limit_violations) ===
      JSON.stringify(baseline.joint_limit_violations) &&
    JSON.stringify(frame.blocked_joints) === JSON.stringify(baseline.blocked_joints) &&
    JSON.stringify(frame.non_finite_fields) === JSON.stringify(baseline.non_finite_fields);

  return {
    frame_index: frame.frame_index,
    compared: true,
    passed:
      maximumNumericDifference <= REGRESSION_TOLERANCE &&
      orientationDifferenceRad <= REGRESSION_TOLERANCE &&
      stateMatches,
    tolerance: REGRESSION_TOLERANCE,
    maximum_numeric_difference: maximumNumericDifference,
    aligned_target_orientation_difference_rad: orientationDifferenceRad,
    state_matches: stateMatches,
  };
};

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

const summarizeFailure = (result, targetPose, reachBound, diagnosticResults, context) => {
  const targetRadius = Math.hypot(...targetPose.position);
  const possibleCauses = [];
  const evidence = [];
  if (targetRadius > reachBound + CONSISTENCY_TOLERANCE) {
    possibleCauses.push('A: target 超出由 URDF joint origins 得到的保守最大半径上界');
    evidence.push({ target_radius_m: targetRadius, maximum_radius_bound_m: reachBound });
  }
  if (context.transform_consistency_passed === false) {
    possibleCauses.push('F: dataset 到 aligned target 的 SE(3) 一致性检查失败');
    evidence.push({ transform_consistency: context.transform_consistency });
  }
  if (context.incoming_step_is_large) {
    possibleCauses.push('E: 当前帧相对上一帧的 TCP 位移或转角属于大步长');
    evidence.push({
      incoming_step: context.incoming_step,
      large_step_thresholds: context.large_step_thresholds,
    });
  }
  if (context.joint_limit_violations.length > 0 || result.blockedJoints.length > 0) {
    possibleCauses.push('D: 输出关节超限或求解步长被关节限位阻挡');
    evidence.push({ blocked_joints: result.blockedJoints });
    if (context.joint_limit_violations.length > 0) {
      evidence.push({ joint_limit_violations: context.joint_limit_violations });
    }
  }
  if (result.terminationReason === 'linear_solve_failed') {
    possibleCauses.push('C: 线性求解失败，可能涉及奇异构型或数值病态，现有日志不能单独确认奇异性');
  }
  if (result.terminationReason === 'max_iterations') {
    possibleCauses.push('B: 达到现有最大迭代数仍未满足默认残差容差');
  } else if (result.terminationReason === 'no_joint_motion') {
    possibleCauses.push(
      'B: 当前 Seed 没有可执行的关节步长，但残差仍未收敛；需结合限位与数值状态判断',
    );
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
      phase: '2B-1',
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
      seed_policy:
        'frame 0 uses UR5 initial joints; each later frame uses the previous formal solution',
      alignment_policy:
        'Compute T_align = S_0 × inverse(D_0) once; apply Target_i = T_align × D_i to every frame',
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
    const maximumDatasetPositionStep = getMaximumFrame(
      continuityChecks,
      'dataset_position_delta_norm_m',
    );
    const maximumDatasetRotationStep = getMaximumFrame(
      continuityChecks,
      'dataset_rotation_delta_rad',
    );
    const maximumAlignedPositionStep = getMaximumFrame(
      continuityChecks,
      'aligned_position_delta_norm_m',
    );
    const maximumAlignedRotationStep = getMaximumFrame(
      continuityChecks,
      'aligned_rotation_delta_rad',
    );
    const medianPositionStep = getMedian(
      continuityChecks.map((check) => check.dataset_position_delta_norm_m),
    );
    const medianRotationStep = getMedian(
      continuityChecks.map((check) => check.dataset_rotation_delta_rad),
    );
    const largeStepThresholds = {
      translation_m: Math.max(0.02, medianPositionStep * 3),
      rotation_rad: Math.max(0.15, medianRotationStep * 3),
      method:
        'diagnostic only: max(20 mm or 3× episode median translation, 0.15 rad or 3× median rotation)',
    };
    const alignedTcpWorkspace = getVectorBounds(
      targetMatrices.map((matrix) => matrixPosition(matrix).toArray()),
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
      max_dataset_position_delta_norm_m: maximumDatasetPositionStep
        ? maximumDatasetPositionStep.dataset_position_delta_norm_m
        : null,
      max_dataset_position_delta_to_frame: maximumDatasetPositionStep
        ? maximumDatasetPositionStep.to_frame
        : null,
      max_dataset_rotation_delta_rad: maximumDatasetRotationStep
        ? maximumDatasetRotationStep.dataset_rotation_delta_rad
        : null,
      max_dataset_rotation_delta_to_frame: maximumDatasetRotationStep
        ? maximumDatasetRotationStep.to_frame
        : null,
      max_aligned_position_delta_norm_m: maximumAlignedPositionStep
        ? maximumAlignedPositionStep.aligned_position_delta_norm_m
        : null,
      max_aligned_position_delta_to_frame: maximumAlignedPositionStep
        ? maximumAlignedPositionStep.to_frame
        : null,
      max_aligned_rotation_delta_rad: maximumAlignedRotationStep
        ? maximumAlignedRotationStep.aligned_rotation_delta_rad
        : null,
      max_aligned_rotation_delta_to_frame: maximumAlignedRotationStep
        ? maximumAlignedRotationStep.to_frame
        : null,
    };
    report.aligned_tcp_target_workspace_m = alignedTcpWorkspace;
    report.failure_diagnostic_thresholds = largeStepThresholds;

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
      const limitMargins = getJointLimitMargins(solution, jointLimits);
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
        formal_frame_success: result.converged && nonFiniteFields.length === 0,
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
        joint_limit_margins: limitMargins,
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
          incomingContinuityCheck: index > 0 ? continuityChecks[index - 1] : null,
        };
        break;
      }
      seed = solution;
    }

    const successfulFrames = report.frames.filter((frame) => frame.formal_frame_success);
    const positionResiduals = report.frames.map((frame) => frame.position_residual_m);
    const orientationResiduals = report.frames.map((frame) => frame.orientation_residual_rad);
    const jointDeltaNorms = report.frames.map((frame) => frame.joint_delta_norm);
    const maximumPositionResidualFrame = getMaximumFrame(report.frames, 'position_residual_m');
    const maximumOrientationResidualFrame = getMaximumFrame(
      report.frames,
      'orientation_residual_rad',
    );
    const maximumJointDeltaFrame = getMaximumFrame(report.frames, 'joint_delta_norm');
    const maximumSingleJointStepFrame = getMaximumFrame(
      report.frames,
      'max_single_joint_delta.value',
      (frame) => Math.abs(frame.max_single_joint_delta.value),
    );
    const closestJointLimit = report.frames
      .flatMap((frame) =>
        jointNames.map((name) => ({
          frame_index: frame.frame_index,
          joint: name,
          ...frame.joint_limit_margins[name],
        })),
      )
      .filter((item) => Number.isFinite(item.minimum_margin_rad))
      .reduce(
        (closest, item) =>
          !closest || item.minimum_margin_rad < closest.minimum_margin_rad ? item : closest,
        null,
      );
    const perJointStatistics = getJointStatistics(report.frames, jointNames, jointLimits);
    const baselineFrames = await readBaselineFrames();
    const regressionComparison = REGRESSION_FRAME_INDICES.map((frameIndex) => {
      const frame = report.frames.find((item) => item.frame_index === frameIndex);
      const baseline = baselineFrames.get(frameIndex);
      if (!baseline) {
        return {
          frame_index: frameIndex,
          compared: false,
          passed: false,
          reason: 'Phase 2A baseline frame is missing',
        };
      }
      if (!frame) {
        return {
          frame_index: frameIndex,
          compared: false,
          passed: false,
          reason: 'formal sequential run stopped before this frame',
        };
      }
      return compareRegressionFrame(frame, baseline);
    });
    const regressionComplete = regressionComparison.every((item) => item.compared);
    const regressionPassed =
      regressionComplete && regressionComparison.every((item) => item.passed);
    report.summary = {
      tested_frames: report.frames.length,
      ik_success_count: successfulFrames.length,
      ik_failure_count: report.frames.length - successfulFrames.length,
      mean_position_residual_m: getFiniteMean(positionResiduals),
      max_position_residual_m: maximumPositionResidualFrame?.position_residual_m ?? null,
      max_position_residual_frame: maximumPositionResidualFrame?.frame_index ?? null,
      mean_orientation_residual_rad: getFiniteMean(orientationResiduals),
      max_orientation_residual_rad:
        maximumOrientationResidualFrame?.orientation_residual_rad ?? null,
      max_orientation_residual_frame: maximumOrientationResidualFrame?.frame_index ?? null,
      mean_joint_delta_norm: getFiniteMean(jointDeltaNorms),
      max_joint_delta_norm: maximumJointDeltaFrame?.joint_delta_norm ?? null,
      max_joint_delta_norm_frame: maximumJointDeltaFrame?.frame_index ?? null,
      max_single_joint_step: maximumSingleJointStepFrame
        ? {
            ...maximumSingleJointStepFrame.max_single_joint_delta,
            magnitude_rad: Math.abs(maximumSingleJointStepFrame.max_single_joint_delta.value),
            frame_index: maximumSingleJointStepFrame.frame_index,
          }
        : null,
      max_single_joint_step_frame: maximumSingleJointStepFrame?.frame_index ?? null,
      joint_limit_violation_count: report.frames.filter(
        (frame) => frame.joint_limit_violations.length > 0,
      ).length,
      joint_limit_violation_joint_count: report.frames.reduce(
        (count, frame) => count + frame.joint_limit_violations.length,
        0,
      ),
      nan_inf_count: report.frames.filter((frame) => frame.non_finite_fields.length > 0).length,
      first_failure_frame: failureRecord?.frame.frame_index ?? null,
      max_joint_jump_frame: maximumJointDeltaFrame?.frame_index ?? null,
      tcp_target_position_min_m: alignedTcpWorkspace.min_m,
      tcp_target_position_max_m: alignedTcpWorkspace.max_m,
      tcp_workspace_bounding_box_m: alignedTcpWorkspace,
      per_joint_statistics: perJointStatistics,
      closest_joint_limit: closestJointLimit,
      trajectory_transform_consistency: report.continuity_summary,
      regression_comparison: {
        baseline_report: BASELINE_REPORT_PATH,
        frame_indices: REGRESSION_FRAME_INDICES,
        tolerance: REGRESSION_TOLERANCE,
        complete: regressionComplete,
        passed: regressionComplete ? regressionPassed : null,
        frames: regressionComparison,
      },
    };

    if (failureRecord) {
      const incomingStep = failureRecord.incomingContinuityCheck
        ? {
            dataset_translation_m:
              failureRecord.incomingContinuityCheck.dataset_position_delta_norm_m,
            dataset_rotation_rad: failureRecord.incomingContinuityCheck.dataset_rotation_delta_rad,
            aligned_translation_m:
              failureRecord.incomingContinuityCheck.aligned_position_delta_norm_m,
            aligned_rotation_rad: failureRecord.incomingContinuityCheck.aligned_rotation_delta_rad,
            transform_position_delta_error_m:
              failureRecord.incomingContinuityCheck.position_delta_norm_error_m,
            transform_rotation_delta_error_rad:
              failureRecord.incomingContinuityCheck.rotation_delta_error_rad,
          }
        : null;
      const incomingStepIsLarge =
        incomingStep !== null &&
        (incomingStep.dataset_translation_m > largeStepThresholds.translation_m ||
          incomingStep.dataset_rotation_rad > largeStepThresholds.rotation_rad);
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
        formal_sequential_result: 'failure; this frame does not advance the formal seed',
        dataset_pose: {
          position: failureRecord.frame.position,
          rotation_vector: failureRecord.frame.rotation_vector,
        },
        previous_dataset_pose:
          failureRecord.index > 0
            ? {
                position: dataset.frames[failureRecord.index - 1].position,
                rotation_vector: dataset.frames[failureRecord.index - 1].rotation_vector,
              }
            : null,
        aligned_target_pose: failureRecord.targetPose,
        previous_target_pose:
          failureRecord.index > 0 ? matrixToPose(targetMatrices[failureRecord.index - 1]) : null,
        incoming_step: incomingStep,
        incoming_step_is_large: incomingStepIsLarge,
        incoming_step_thresholds: largeStepThresholds,
        seed_joints: getJointValues(failureRecord.seed, jointNames),
        solution_joints: getJointValues(failureRecord.result.jointValues, jointNames),
        joint_delta: failureRecord.record.joint_deltas,
        joint_delta_norm: failureRecord.record.joint_delta_norm,
        max_single_joint_delta: failureRecord.record.max_single_joint_delta,
        position_residual_m: failureRecord.result.residual.position,
        orientation_residual_rad: failureRecord.result.residual.orientation,
        termination_reason: failureRecord.result.terminationReason,
        blocked_joints: failureRecord.result.blockedJoints,
        joint_limit_violations: failureRecord.record.joint_limit_violations,
        seed_joint_limit_margins: getJointLimitMargins(failureRecord.seed, jointLimits),
        solution_joint_limit_margins: failureRecord.record.joint_limit_margins,
        non_finite_fields: failureRecord.record.non_finite_fields,
        joint_limits: jointLimits,
        solver_trace: failureRecord.iterationTrace,
        diagnostic_multi_seed_policy:
          'diagnostic only; results never replace the formal sequential result',
        diagnostic_multi_seed: diagnosticResults,
        cause_assessment: summarizeFailure(
          failureRecord.result,
          failureRecord.targetPose,
          reachBound,
          diagnosticResults,
          {
            transform_consistency_passed: report.continuity_summary.passed,
            transform_consistency: report.continuity_summary,
            incoming_step: incomingStep,
            incoming_step_is_large: incomingStepIsLarge,
            large_step_thresholds: largeStepThresholds,
            joint_limit_violations: failureRecord.record.joint_limit_violations,
          },
        ),
      };
      report.status = 'stopped_ik_failure';
    } else {
      report.status = 'complete';
    }

    if (
      !failureRecord &&
      report.summary.regression_comparison.complete &&
      !report.summary.regression_comparison.passed
    ) {
      report.status = 'stopped_regression_mismatch';
      report.failure_diagnostic = {
        possible_cause:
          '同一输入在 Phase 2A 与 Phase 2B-1 得到不同数值结果；需先调查 deterministic/state 差异',
        regression_comparison: report.summary.regression_comparison,
      };
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
