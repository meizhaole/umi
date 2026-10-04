# app

应用入口、全局配置和调试消息总线。推理执行时域默认值位于 `executionHorizon.ts`。

UR5 dataset 单帧回放通过 `datasetReplay.ts` 读取 `reports/phase2b1-umi-episode0-frames0-399.jsonl` 中的预计算对齐目标与 IK 解，不重算轨迹或 IK。
