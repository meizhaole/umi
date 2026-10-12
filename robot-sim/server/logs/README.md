# 推理关节角日志

`inference-actions.csv` 每行记录一步 IK 结果。第一列 `step` 是连续步号，后续列按关节名记录旋转关节角，单位为 rad。文件保留最近 30 条数据，超出后覆盖较早记录；CSV 表头不计入数据行数。IK 未收敛时也会记录最后一次关节角估计。

旧版 `inference-actions.jsonl` 作为历史日志保留，新推理不再写入该文件。

`inference-predictions.jsonl` 每行记录一次官方策略的完整反归一化预测，包含检查点的动作表示、全部预测点、实际回放的首个动作，以及三毫米阈值统计。`target_displacement_mm` 表示每个目标相对当前末端位置的偏移；`cumulative_path_mm` 表示从当前末端位置开始，沿预测目标序列逐段累加的路径长度。两项均以毫米为单位，超过三毫米的首个点使用从 1 开始的编号。

`ik-debug.jsonl` 每行记录一个 IK action 事件，使用 `request_id`、`episode_index`、`frame_index` 和 `action_index` 关联策略动作。`record_phase=started` 保存进入 IK 时的原始 action、start pose 和 Seed；`record_phase=result` 保存求解摘要。失败结果始终包含完整迭代 trace，成功结果默认只保存摘要。

`ik-multiseed-results.json` 是离线诊断结果，不是在线推理的输入。`pnpm ik:multiseed` 从 `ik-debug.jsonl` 和 `inference-predictions.jsonl` 选取最近的 `no_joint_motion` 失败记录，并读取 `public/robot/description/RS/urdf/ReBot_Arm_RS.urdf`；它会覆盖本文件和 `robot-sim/ik_multiseed_experiment.md`。重跑前应保留现有两份结果及其对应日志。该实验报告记录的 8 个 Seed 均未成功，最小位置残差为 6.992 mm，仍高于 5 mm tolerance；结果不证明目标全局不可达。

完整记录成功 action 的迭代过程时，在仿真页面 URL 加上 `?ikTrace=1`。关闭时移除该参数或使用 `?ikTrace=0`。`action_pose_repr` 从 UMI worker 的动作块传入前端 IK Worker，并原样写入 IK 记录；正常记录值为可用的字符串，状态为 `action_pose_repr_status=available`。该字段指定动作位姿的解释方式。当前浏览器 IK 只支持 `relative`，即将相对观测 TCP 的动作位姿与起始 TCP 位姿组合；不支持的值会返回 IK 错误，不会被当作 `null`。
