# Scripts

`syncRobotAssets.mjs` 读取 ROS 描述目录中的 RS、DM URDF，校验 `package://` 资源路径，并将两个 URDF 与实际引用的 STL 安全覆盖到 `public/robot/description/`。运行 `pnpm dev` 或 `pnpm build` 时会自动执行。
