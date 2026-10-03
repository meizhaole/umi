// 使用最新失败 trace 固定目标，离线比较当前 IK Solver 的多个合法 Seed。
import { createHash } from 'node:crypto';
import { readFile, writeFile } from 'node:fs/promises';
import { dirname, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { JSDOM } from 'jsdom';
import { createServer } from 'vite';

const SCRIPT_DIRECTORY = dirname(fileURLToPath(import.meta.url));
const PROJECT_DIRECTORY = resolve(SCRIPT_DIRECTORY, '..');
const ROOT_DIRECTORY = resolve(PROJECT_DIRECTORY, '..');
const IK_LOG_PATH = resolve(PROJECT_DIRECTORY, 'server/logs/ik-debug.jsonl');
const PREDICTION_LOG_PATH = resolve(PROJECT_DIRECTORY, 'server/logs/inference-predictions.jsonl');
const URDF_PATH = resolve(PROJECT_DIRECTORY, 'public/robot/description/RS/urdf/ReBot_Arm_RS.urdf');
const RESULTS_PATH = resolve(PROJECT_DIRECTORY, 'server/logs/ik-multiseed-results.json');
const REPORT_PATH = resolve(PROJECT_DIRECTORY, 'ik_multiseed_experiment.md');
const REPLAY_TOLERANCE = 1e-10;

const readJsonLines = async (path) => {
  const contents = await readFile(path, 'utf8');
  return contents
    .split(/\r?\n/u)
    .filter((line) => line.trim())
    .map((line) => JSON.parse(line));
};

const findPreviousSuccessfulSeed = (records, failure) =>
  records
    .filter(
      (record) =>
        record.record_phase === 'result' &&
        record.status === 'success' &&
        record.request_id === failure.request_id &&
        record.episode_index === failure.episode_index &&
        record.frame_index === failure.frame_index &&
        record.action_index === failure.action_index - 1 &&
        record.action_output_seed,
    )
    .at(-1)?.action_output_seed ?? null;

const findPredictionRecord = (records, requestId) =>
  records.filter((record) => record.request_id === requestId).at(-1) ?? null;

const formatNumber = (value, digits = 6) =>
  value === null || value === undefined ? 'unavailable' : Number(value).toFixed(digits);

const renderReport = (experiment) => {
  const { target_record: target, fixed_inputs: fixedInputs, seed_results: results } = experiment;
  const tableRows = results
    .map(
      (result) =>
        `| ${result.seed_name} | ${result.status} | ${result.iterations ?? '—'} | ${
          result.final_position_residual === null
            ? '—'
            : `${formatNumber(result.final_position_residual * 1000, 4)} mm`
        } | ${formatNumber(result.final_orientation_residual)} | ${formatNumber(
          result.joint2_initial,
        )} | ${formatNumber(result.joint2_final)} | ${result.blocked_joints.join(', ') || '无'} | ${formatNumber(
          result.minimum_joint_limit_margin,
        )} | ${
          result.termination_reason ?? result.unavailable_reason ?? '—'
        }${result.duplicate_of ? `（重复 ${result.duplicate_of}）` : ''} |`,
    )
    .join('\n');
  const seedInputs = results
    .map(
      (result) =>
        `- ${result.seed_name}：\`${
          result.seed_joint_values ? JSON.stringify(result.seed_joint_values) : 'unavailable'
        }\`${result.duplicate_of ? `（重复 ${result.duplicate_of}）` : ''}`,
    )
    .join('\n');
  const executed = results.filter((result) => result.executed);
  const minimumResidual = executed.reduce(
    (minimum, result) =>
      result.final_position_residual === null
        ? minimum
        : Math.min(minimum, result.final_position_residual),
    Number.POSITIVE_INFINITY,
  );
  const minimumResidualSeed = executed.reduce(
    (minimum, result) =>
      result.final_position_residual === null ||
      (minimum && minimum.final_position_residual <= result.final_position_residual)
        ? minimum
        : result,
    null,
  );
  const successfulAlternatives = results.filter(
    (result) =>
      result.status === 'success' &&
      result.seed_name !== 'Seed A — Original Failure Seed' &&
      !result.duplicate_of,
  );
  const allExecutedFailedNoMotion =
    executed.length > 0 &&
    executed.every((result) => result.termination_reason === 'no_joint_motion');
  const joint2Limit = experiment.robot.joint_limits.find((joint) => joint.joint_name === 'joint2');
  const allJoint2ReachedLowerLimit =
    joint2Limit?.lower !== null &&
    joint2Limit?.lower !== undefined &&
    executed.length > 0 &&
    executed.every(
      (result) =>
        result.joint2_final !== null &&
        Math.abs(result.joint2_final - joint2Limit.lower) <= REPLAY_TOLERANCE,
    );
  let interpretation;
  if (!experiment.baseline_reproduction.matches) {
    interpretation =
      'Case C：原始 Seed 重放未能复现日志结果；本实验在 Seed A 后停止，不比较其他 Seed。需要先核对输入和机器人模型差异。';
  } else if (successfulAlternatives.length > 0) {
    interpretation = `Case A：至少一个不同 Seed 收敛（${successfulAlternatives
      .map((result) => result.seed_name)
      .join('、')}）。在当前 Target 和模型下存在被当前 Solver 找到的合法解，结果对 Seed 敏感。`;
  } else {
    interpretation =
      'Case B：本次执行的有限 Seed 集合没有找到满足 tolerance 的解。不能据此判定 Target 全局不可达。';
  }

  return [
    '# IK Multi-seed 实验报告',
    '',
    `实验状态：${experiment.experiment_status}`,
    '',
    '```mermaid',
    'flowchart LR',
    '  log[最新失败 JSONL 记录] --> fixed[固定 Target、Solver options、RS URDF]',
    '  fixed --> seeds[Seed A 到 H]',
    '  seeds --> solve[现有 Kinematics.solveIK]',
    '  solve --> output[残差、限位、joint2 轨迹]',
    '```',
    '',
    '## Experiment Target',
    '',
    `- request_id：\`${target.request_id}\``,
    `- episode_index：${target.episode_index}`,
    `- frame_index：${target.frame_index}`,
    `- action_index：${target.action_index}（从 0 开始）`,
    `- action_pose_repr：${fixedInputs.action_pose_repr ?? 'unavailable'}`,
    `- raw action：\`${JSON.stringify(fixedInputs.raw_action)}\``,
    `- startPose：\`${JSON.stringify(fixedInputs.start_pose)}\``,
    `- targetPose：\`${JSON.stringify(fixedInputs.target_pose)}\``,
    `- Original Seed：\`${JSON.stringify(fixedInputs.original_seed)}\``,
    `- Solver options：\`${JSON.stringify(fixedInputs.solver_options)}\``,
    `- Robot：\`${experiment.robot.name}\`，URDF SHA-256 \`${experiment.robot.urdf_sha256}\``,
    '- 模型身份说明：IK 请求日志未记录 model_id；joint2 在 0 处的负向步长被 trace 判为越界，与 RS URDF 的下限 0 相符。使用当前浏览器静态目录中的 RS URDF，并用 Seed A 重放结果校验；原运行的 model_id/hash 仍未直接留档。',
    '',
    '### Tested Seed Inputs',
    '',
    '- Seed E 来源：`robot-sim/src/app/App.tsx` 中 RS 启动参考姿态（joint2/joint3 为 0.1 rad，其余 IK 路径关节取初始 0；夹爪保持原失败 Seed 值）。',
    seedInputs,
    '',
    '## Seed Results',
    '',
    '| Seed | Success | Iterations | Position Residual | Orientation Residual | joint2 start | joint2 final | blocked joints | Min limit margin (rad) | Reason |',
    '|---|---:|---:|---:|---:|---:|---:|---|---:|---|',
    tableRows,
    '',
    '### joint2 观察',
    '',
    '| Seed | q2 initial | q2 final | q2 min–max | 首次 active-set 阻挡迭代 | 该轮 raw Δq2 |',
    '|---|---:|---:|---:|---:|---:|',
    ...results.map(
      (result) =>
        `| ${result.seed_name} | ${formatNumber(result.joint2_initial)} | ${formatNumber(
          result.joint2_final,
        )} | ${formatNumber(result.joint2_min)}–${formatNumber(result.joint2_max)} | ${
          result.first_iteration_joint2_blocked ?? '无'
        } | ${formatNumber(result.joint2_raw_delta_when_first_blocked)} |`,
    ),
    '',
    '逐轮 residual 与 joint2 数值轨迹保存在 JSON 结果文件的 `residual_history` 和 `joint2_history`。',
    '',
    '## Interpretation',
    '',
    interpretation,
    '',
    `- Original Seed replay：${experiment.baseline_reproduction.matches ? '匹配日志' : '不匹配日志'}`,
    `- 所有实际失败都以 no_joint_motion 结束：${allExecutedFailedNoMotion ? '是' : '否'}`,
    `- 所有实际 Seed 最终 joint2 都到达 URDF 下限 ${joint2Limit?.lower ?? 'unavailable'}：${allJoint2ReachedLowerLimit ? '是' : '否'}`,
    `- 所有实际 Seed 都出现 joint2 active-set 阻挡：${executed.length > 0 && executed.every((result) => result.joint2_was_blocked) ? '是' : '否'}`,
    `- 最小 Position Residual：${Number.isFinite(minimumResidual) ? `${formatNumber(minimumResidual * 1000, 12)} mm（${minimumResidualSeed?.seed_name ?? 'Seed unavailable'}）` : 'unavailable'}`,
    `- 当前 Position Tolerance：${formatNumber((fixedInputs.solver_options.positionTolerance ?? 0.001) * 1000)} mm；最小残差仍超过 tolerance：${minimumResidual > (fixedInputs.solver_options.positionTolerance ?? 0.001) ? '是' : '否'}`,
    `- 成功 Seed：${successfulAlternatives.map((result) => result.seed_name).join('、') || '无'}`,
    '',
    '实验通过独立入口逐次调用现有 `Kinematics.solveIK`。每次使用同一个 decoded targetPose、solver options、RobotDescription 与关节限位，仅 Seed 变化；未进入正常推理和 playback 路径。',
    '',
  ].join('\n');
};

const run = async () => {
  const dom = new JSDOM('');
  globalThis.DOMParser = dom.window.DOMParser;
  const vite = await createServer({
    configFile: resolve(PROJECT_DIRECTORY, 'vite.config.ts'),
    root: PROJECT_DIRECTORY,
    appType: 'custom',
    logLevel: 'error',
    server: { middlewareMode: true },
  });

  try {
    const [ikRecords, predictionRecords, urdfText] = await Promise.all([
      readJsonLines(IK_LOG_PATH),
      readJsonLines(PREDICTION_LOG_PATH),
      readFile(URDF_PATH, 'utf8'),
    ]);
    const [
      { parseUrdf },
      { buildSeedCases, compareOriginalReplay, runSeedCases, selectLatestFailureRecord },
    ] = await Promise.all([
      vite.ssrLoadModule('/src/utils/urdfParser.ts'),
      vite.ssrLoadModule('/src/diagnostics/ikMultiSeed.ts'),
    ]);
    const failureRecord = selectLatestFailureRecord(ikRecords);
    const previousSeed = findPreviousSuccessfulSeed(ikRecords, failureRecord);
    const prediction = findPredictionRecord(predictionRecords, failureRecord.request_id);
    const description = parseUrdf(urdfText);
    if (!description.name.includes('_RS')) {
      throw new Error(`实验期望 RS URDF，实际解析到 ${description.name}`);
    }

    const seedCases = buildSeedCases(failureRecord, previousSeed, description);
    const seedA = seedCases[0];
    if (seedA.unavailable_reason || !seedA.seed_joint_values) {
      throw new Error(`原始 Seed 不合法，无法开始基线重放：${seedA.unavailable_reason}`);
    }
    const baseline = runSeedCases(
      description,
      failureRecord.target_pose,
      failureRecord.solver_options,
      [seedA],
    )[0];
    const baselineReproduction = compareOriginalReplay(baseline, failureRecord);
    let seedResults = [baseline];
    let experimentStatus = 'baseline_mismatch_stopped';

    if (baselineReproduction.matches) {
      const remainingResults = runSeedCases(
        description,
        failureRecord.target_pose,
        failureRecord.solver_options,
        seedCases.slice(1),
        [baseline],
      );
      seedResults = [baseline, ...remainingResults];
      experimentStatus = 'complete';
    }

    const experiment = {
      schema_version: 1,
      experiment_status: experimentStatus,
      target_record: {
        request_id: failureRecord.request_id,
        episode_index: failureRecord.episode_index,
        frame_index: failureRecord.frame_index,
        action_index: failureRecord.action_index,
        status: failureRecord.status,
        termination_reason: failureRecord.termination_reason,
      },
      fixed_inputs: {
        action_pose_repr: prediction?.action_pose_repr ?? null,
        action_pose_repr_source: prediction
          ? 'inference-predictions.jsonl matched by request_id'
          : 'unavailable',
        raw_action: failureRecord.raw_action,
        start_pose: failureRecord.start_pose,
        target_pose: failureRecord.target_pose,
        original_seed: failureRecord.seed_joint_values,
        solver_options: failureRecord.solver_options,
      },
      robot: {
        name: description.name,
        model_id_status: 'inferred_from_failure_trace',
        urdf_path: relative(ROOT_DIRECTORY, URDF_PATH),
        urdf_sha256: createHash('sha256').update(urdfText).digest('hex'),
        joint_limits: description.joints
          .filter((joint) => joint.limit)
          .map((joint) => ({
            joint_name: joint.name,
            lower: joint.limit?.lower ?? null,
            upper: joint.limit?.upper ?? null,
          })),
      },
      baseline_reproduction: {
        matches: baselineReproduction.matches,
        differences: baselineReproduction.differences,
        absolute_tolerance: REPLAY_TOLERANCE,
        recorded: {
          termination_reason: failureRecord.termination_reason,
          iterations: failureRecord.iterations,
          position_residual: failureRecord.position_residual,
          orientation_residual: failureRecord.orientation_residual,
          final_joint_values: failureRecord.final_joint_values,
          blocked_joints: failureRecord.blocked_joints,
        },
      },
      seed_results: seedResults,
      analysis: {
        classification: baselineReproduction.matches
          ? seedResults.some(
              (result) =>
                result.status === 'success' &&
                result.seed_name !== 'Seed A — Original Failure Seed' &&
                !result.duplicate_of,
            )
            ? 'case_a_seed_sensitive_solution_found'
            : 'case_b_finite_seeds_did_not_find_solution'
          : 'case_c_original_seed_replay_mismatch',
        all_executed_seeds_failed_with_no_joint_motion:
          seedResults.filter((result) => result.executed).length > 0 &&
          seedResults
            .filter((result) => result.executed)
            .every((result) => result.termination_reason === 'no_joint_motion'),
        minimum_position_residual: seedResults
          .filter((result) => result.executed && result.final_position_residual !== null)
          .reduce(
            (minimum, result) => Math.min(minimum, result.final_position_residual),
            Number.POSITIVE_INFINITY,
          ),
        minimum_position_residual_seed: seedResults
          .filter((result) => result.executed && result.final_position_residual !== null)
          .reduce(
            (minimum, result) =>
              minimum === null || result.final_position_residual < minimum.residual
                ? { seed_name: result.seed_name, residual: result.final_position_residual }
                : minimum,
            null,
          ),
        successful_seed_names: seedResults
          .filter(
            (result) =>
              result.status === 'success' &&
              result.seed_name !== 'Seed A — Original Failure Seed' &&
              !result.duplicate_of,
          )
          .map((result) => result.seed_name),
      },
    };

    await writeFile(RESULTS_PATH, `${JSON.stringify(experiment, null, 2)}\n`, 'utf8');
    await writeFile(REPORT_PATH, renderReport(experiment), 'utf8');

    if (!baselineReproduction.matches) {
      console.error(`基线未复现，实验已停止；差异：${baselineReproduction.differences.join(', ')}`);
      process.exitCode = 2;
      return;
    }

    console.log(`成功：实验结果写入 ${relative(PROJECT_DIRECTORY, RESULTS_PATH)}`);
    console.log(`成功：实验报告写入 ${relative(PROJECT_DIRECTORY, REPORT_PATH)}`);
    console.log(
      `基线已复现：request_id=${failureRecord.request_id} frame_index=${failureRecord.frame_index} action_index=${failureRecord.action_index}`,
    );
    seedResults.forEach((result) => {
      console.log(
        `${result.seed_name}: ${result.status}, reason=${result.termination_reason ?? 'unavailable'}, position=${formatNumber(result.final_position_residual === null ? null : result.final_position_residual * 1000, 4)} mm`,
      );
    });
  } finally {
    await vite.close();
    dom.window.close();
  }
};

run().catch((error) => {
  console.error(
    `错误：Multi-seed 实验失败：${error instanceof Error ? error.message : String(error)}`,
  );
  process.exitCode = 1;
});
