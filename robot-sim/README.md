# Robot Sim

Robot Sim 在浏览器中仿真 RS、DM、UR5 和 UR5_CAD。默认加载 RS；页面顶部的型号按钮可手动切换到其他型号。官方 UMI 研究主线验证需手动选择 UR5 或 UR5_CAD。配置见 `src/app/config.ts`。

`core/` 提供机器人学模型与 IK，`sim/` 使用 Rapier 执行物理仿真，`viz/` 负责 Three.js 显示。UMI 在线策略由 `server/` 通过 WebSocket 调用独立 worker；`src/agent/` 的接口不是当前在线推理入口。

RS 和 DM 的 URDF 与网格来自同级 `reBotArmController_ROS2/src/rebotarm_bringup/`；标准 UR5 使用 `assets/ur_description/`；UR5_CAD 使用本地 `public/robot/ur_umi_real/` 资源。`pnpm dev` 和 `pnpm build` 会先运行 `scripts/syncRobotAssets.mjs`，把 RS、DM 和标准 UR5 所需的 URDF 与网格复制到 `public/robot/`。运行这两个命令会写入该资源目录；`pnpm build` 还会生成构建产物。

## 启动前后端

使用 Node.js 24 和 pnpm 12.6.0。在项目根目录分别打开两个终端。首次使用前，`pnpm install` 会安装前端依赖并写入 `node_modules/`。

后端终端使用 `umi` Conda 环境：

```sh
cd robot-sim/server
conda activate umi
python main.py
```

前端终端：

```sh
cd robot-sim
pnpm install
pnpm dev
```

`pnpm dev` 启动前会同步机器人资源到 `public/robot/`。推理运行时，后端会在 `server/logs/` 追加策略预测、IK 调试和关节角日志；UR5_CAD 单动作审阅结果写入 `debug/`。

UR5 UMI Dataset Replay 另行读取 `reports/phase2b1-umi-episode0-frames0-399.jsonl` 中的预计算目标和 IK 解，默认使用 `KINEMATIC_REPLAY`。该文件由 `src/app/datasetReplay.ts` 导入，是当前浏览器回放的运行输入；不要删除。离线 dry-run 脚本会覆盖该报告，重跑前应先保留现有结果。

运行检查：`pnpm typecheck`、`pnpm lint`、`pnpm test`、`pnpm build`。其中 `pnpm build` 也会先同步资源并生成构建产物。
