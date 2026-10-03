# tests

Vitest 运动学、URDF 解析和仿真控制接口测试。

`action-pose-repr.test.ts` 覆盖 Browser 动作块校验、relative 解码公式和不支持表示的 fail-fast。

`execution-horizon.test.ts` 覆盖完整预测、执行前缀、IK/播放数量及 ACK 后等待手动下一步。

`ik-multiseed.test.ts` 覆盖最新失败记录选择、A 到 H Seed 构造、重复 Seed 标记、RS 参考初值和夹爪 Seed 保持。
