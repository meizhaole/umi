# UMI 推理 WebSocket

robot-sim 通过 FastAPI 订阅现有 UMI 回放 worker。推理使用官方权重 `universal_manipulation_interface/data/pretrained/cup_wild_vit_l_1img.ckpt` 和本地数据集 `universal_manipulation_interface/data/cup_in_the_wild.zarr.zip`。动作来自验证集回放，不读取浏览器实时相机帧。

## 启动

先更新已有 Conda 环境中的服务依赖：

```bash
cd /home/pan/桌面/umi
conda env update -n umi -f universal_manipulation_interface/conda_environment.yaml
```

启动后端：

```bash
cd /home/pan/桌面/umi/robot-sim/server
conda activate umi
python main.py
```

`main.py` 会组装 FastAPI WebSocket 服务并启动 Uvicorn；推理 worker 由 `load_model.py` 管理。

另开终端启动前端：

```bash
cd /home/pan/桌面/umi/robot-sim
pnpm dev
```

页面中点击“开始回放”后，前端连接 `ws://localhost:8000/ws/inference`。服务端一次只接受一个活跃订阅。回放从 worker 默认验证 episode 和起始帧开始，发送该 episode 的全部动作块。

## 消息协议

连接后服务端先发送加载状态：

```json
{"type":"status","state":"loading"}
```

模型载入后逐块发送动作。每个动作是 `[dx, dy, dz, rx, ry, rz, grip_width]`，位置和夹爪宽度单位为米，旋转是弧度轴角；块内动作都相对于块开始时的 TCP 位姿。

```json
{
  "type":"action_chunk",
  "episode_index": 0,
  "frame_index": 15,
  "last_chunk": false,
  "actions": [[0.01, 0, 0, 0, 0, 0, 0.06]]
}
```

当前策略每块包含 8 个动作，每个动作 7 个数值，序列化后通常只有几 KB。Uvicorn 的 websockets 后端默认消息上限为 16 MiB；此服务显式限制为 1 MiB，并在发送前检查序列化字节数。[Uvicorn WebSocket 设置](https://www.uvicorn.org/settings/)

前端在 Web Worker 中计算整块 IK，本地 watchdog 为 1 秒；超时会终止 Worker 并发送 `stop`。动作块完整播放后前端确认，`frame_index` 必须与当前块匹配：

```json
{"type":"ack","frame_index":15}
```

服务端收到确认后继续读取 worker。前端也可以发送 `{"type":"stop"}` 停止回放。最后一块确认后服务端发送 `complete` 并关闭连接。

失败时服务端发送 `error`，包括 `code`、`stage`、`message`，以及适用时的 `episode_index`、`frame_index`、`action_index`、`iterations` 和位置、姿态残差。ACK 等待上限为 2 秒，超时后服务端先发送错误，再关闭连接并清理 worker。
