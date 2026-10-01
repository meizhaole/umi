# Robot Sim

RS、DM 机械臂的浏览器仿真样例。`core/` 提供与引擎无关的机器人学接口，`sim/` 连接 Rapier，`viz/` 负责 Three.js 渲染，`agent/` 定义策略接口，`ui/` 提供控制界面。

机械臂 URDF 和网格以 `reBotArmController_ROS2/src/rebotarm_bringup/description/` 为唯一来源。`pnpm dev` 与 `pnpm build` 会先运行资源同步脚本，将 URDF 实际引用的 STL 暂存到被忽略的 `public/robot/description/`。

## 启动前后端

使用 Node.js 24 和 pnpm 12.6.0。在项目根目录分别打开两个终端。

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

运行检查：`pnpm typecheck`、`pnpm lint`、`pnpm test`、`pnpm build`。
