# 后端测试

使用 `conda run -n umi python tests/test_ik_debug_log.py` 运行日志写入测试。

使用 `conda run -n umi python tests/test_action_pose_repr_protocol.py` 验证 worker 动作块和 WebSocket 序列化保留 `action_pose_repr`。

使用 `conda run -n umi python tests/test_single_action_ack_state.py` 验证 UR5_CAD 单动作审批可无限等待，且普通流程保留 ACK 超时。

使用 `conda run -n umi python -m unittest server.tests.test_observation_episode_protocol server.tests.test_umi_worker_episode_start` 验证 episode_index 协议以及官方/worker wrt_start golden 对照。

使用 `conda run -n umi python -m unittest server.tests.test_umi_worker_warmup` 验证 Worker Warm-up、随机状态隔离与 READY 门禁。
