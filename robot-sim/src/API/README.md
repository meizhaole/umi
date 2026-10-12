# UMI 推理 WebSocket

robot-sim 通过 FastAPI WebSocket 将仿真腕部相机图像与机械臂状态送给 UMI 推理 worker。推理使用工作区中的权重 `universal_manipulation_interface/data/pretrained/cup_wild_vit_l_1img.ckpt`。该服务从仿真观测在线请求策略预测；Python 服务和 worker 依赖同级的 UMI 仓库及 `umi` Conda 环境。

## 启动

如需按 UMI 环境文件更新已有 `umi` 环境，可执行以下命令。它会修改该 Conda 环境中的依赖；已有环境可用时无需为每次启动重复执行：

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

`main.py` 会组装 FastAPI WebSocket 服务并启动 Uvicorn，监听本机 `127.0.0.1:8000`；推理 worker 由 `load_model.py` 管理。首次收到推理订阅时，后端启动 UMI worker 并加载权重。服务进程同一时刻只接受一个活跃推理订阅。

另开终端启动前端：

```bash
cd /home/pan/桌面/umi/robot-sim
pnpm dev
```

默认型号为 RS。页面顶部可手动选择 RS、DM、UR5 或 UR5_CAD；单动作验证和自动闭环仅适用于 UR5_CAD。点击“推理一步”后，前端采集仿真腕部相机图像、末端状态和夹爪开度，并经 `ws://localhost:8000/ws/inference` 发送观测。

服务端返回完整预测及其转换后的动作数组。预测长度由 worker 输出决定；协议要求 `prediction` 与 `actions` 的步数一致。每条原始策略预测为 10 维（位置、6D 旋转表示、夹爪宽度），`actions` 为 7 维 `[dx, dy, dz, rx, ry, rz, grip_width]`，其中位置和夹爪宽度单位为米，旋转为轴角弧度。普通执行最多选择前 6 条动作；动作块较短时按实际长度执行。前端在 Web Worker 中为选中的动作计算 IK，并自动回放整组已选动作。回放完成后发送带有 `frame_index` 和关节角的 ACK；后端收到 ACK 后等待下一帧观测，用户再次点击“推理一步”继续。

单动作验证模式只对 UR5_CAD 开放。它只对当前动作块的第一条动作计算 IK，随后暂停供用户检查；点击执行后回放这一条动作并采集执行后状态，然后发送 ACK。审阅记录在后台保存；有界自动闭环只有在保存成功后才进入下一步。该人工审批等待不设服务端 ACK 超时。普通动作块的 ACK 等待上限为 20 秒。

UR5_CAD 的有界自动闭环也使用单动作验证路径：当前提供 3 步和 10 步自动闭环，以及依赖杯体检测和接触前安全检查的 20 步杯子接近测试。每一步等待新相机帧，执行一条动作并等待审阅结果保存成功后再进入下一步；运行达到设定步数、停止或发生错误时结束。它不会跳过单动作检查与执行后状态采集。

在线 WebSocket 推理使用当前仿真相机和机器人状态请求策略预测。Phase 2 的 UMI Dataset dry-run 与 Dataset Replay 是独立的离线校验路径：前者读取本地 UMI episode 数据计算对齐目标和 IK，后者读取预计算报告，不经过 WebSocket worker，也不重新计算轨迹或 IK。当前回放输入是 `reports/phase2b1-umi-episode0-frames0-399.jsonl`；应用直接导入该文件，缺失时 Dataset Replay 无法加载。

推理运行会在 `server/logs/` 追加策略预测、IK 调试记录和关节角 CSV；IK 角度日志最多保留最近 30 条数据。UR5_CAD 单动作和自动闭环审阅数据写入 `debug/`。这些是运行时输出，不由启动命令创建。

## 消息协议

虚拟相机安装在 URDF 的末端 link，输出 224×224 RGB 图像；视场角为 60°，相机外参为仿真近似值。连接后服务端先发送加载状态：

```json
{"type":"status","state":"loading"}
```

模型载入后发送 `{"type":"status","state":"ready"}` 并等待观测。前端观测消息示例：

```json
{"type":"observation","episode_index":0,"frame_index":0,"camera0_rgb":"<base64 RGB>","robot0_eef_pos":[[0,0,0],[0,0,0]],"robot0_eef_rot_axis_angle":[[0,0,0],[0,0,0]],"robot0_gripper_width":[[0],[0]]}
```

服务收到观测后发送动作块。每个 `actions` 动作是 `[dx, dy, dz, rx, ry, rz, grip_width]`，位置和夹爪宽度单位为米，旋转是弧度轴角；位姿解释由 `action_pose_repr` 指定。前端将该字段原样传给 IK Worker，用它解码动作目标；当前浏览器 IK 仅支持 `relative`，即将动作相对位姿与本次观测的 TCP 起始位姿组合。缺少该字段会使服务端协议校验失败；使用不支持的值会由 IK Worker 报错。

```json
{
  "type":"action_chunk",
  "episode_index": 0,
  "frame_index": 15,
  "last_chunk": false,
  "action_pose_repr": "relative",
  "prediction": [[0, 0, 0, 1, 0, 0, 0, 1, 0, 0.06]],
  "actions": [[0.01, 0, 0, 0, 0, 0, 0.06]]
}
```

上例只展示一个预测点；实际步数由 UMI worker 输出决定。`prediction` 保存策略预测，`actions` 保存转换后供 IK 使用的目标。前端普通执行最多选择前 6 个动作，单动作验证只选择第一条。Uvicorn WebSocket 消息上限由服务显式设为 1 MiB，并在发送前检查序列化字节数。[Uvicorn WebSocket 设置](https://www.uvicorn.org/settings/)

前端在 Web Worker 中计算已选动作的 IK，本地 watchdog 为 1 秒；超时会终止 Worker 并发送 `stop`。普通模式在所选动作全部播放后确认；单动作验证在用户审阅并执行该动作后确认。`frame_index` 必须与当前块匹配：

```json
{"type":"ack","frame_index":15,"joint_angles":[{"joint1":0.1,"joint2":-0.2}]}
```

`joint_angles` 是可选字段，每个对象对应动作块内的一步；服务端只记录旋转关节，其余字段不写入角度 CSV。`stop` 消息也可以携带相同字段。

服务端收到确认后等待下一帧观测。自动闭环在保存当前审阅结果并等待新相机帧后自动发送下一条观测；普通手动模式则等待用户再次点击。前端也可以发送 `{"type":"stop"}` 停止推理。前端发送 `stop` 后服务端发送 `complete` 并关闭连接。

失败时服务端发送 `error`，包括 `code`、`stage`、`message`，以及适用时的 `episode_index`、`frame_index`、`action_index`、`iterations` 和位置、姿态残差。普通模式 ACK 等待上限为 20 秒；单动作人工审批模式不设 ACK 截止时间。普通模式超时后服务端发送错误、关闭连接并清理 worker。
