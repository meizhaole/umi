# 推理动作日志

`inference-actions.jsonl` 按 JSONL 格式记录服务生成的动作块，每行包含 UTC 时间、episode/frame、末块标记和完整动作数组。文件达到 10 MiB 后轮转，最多保留 5 个备份。
