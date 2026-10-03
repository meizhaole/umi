# Scripts

`syncRobotAssets.mjs` 读取 ROS 描述目录中的 RS、DM URDF，校验 `package://` 资源路径，并将两个 URDF 与实际引用的 STL 安全覆盖到 `public/robot/description/`。运行 `pnpm dev` 或 `pnpm build` 时会自动执行。

`replay-ik-multiseed.mjs` 从最新 `no_joint_motion` 失败记录读取固定 Target 和 IK options，使用当前 RS URDF 离线调用现有 `Kinematics.solveIK` 比较合法 Seed。原始 Seed 未复现时会停止，不运行其他 Seed。它写入 `server/logs/ik-multiseed-results.json` 和 `ik_multiseed_experiment.md`，可在 `robot-sim/` 下运行 `pnpm ik:multiseed`。
