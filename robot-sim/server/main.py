# 组装并启动 UMI 回放后端服务
# 该服务只暴露一个 WebSocket 推理端点，供 robot-sim 前端连接；HTTP 部分仅用于健康检查。
import asyncio
import logging

import uvicorn
from fastapi import FastAPI

# 兼容两种运行方式：作为包导入（python -m server.main）时用相对导入，
# 直接运行（python main.py）时用绝对导入。
if __package__:
    from .inference import router
    from .inference_protocol import MAX_WEBSOCKET_MESSAGE_BYTES
else:
    from inference import router
    from inference_protocol import MAX_WEBSOCKET_MESSAGE_BYTES


# 只监听本机回环地址，推理服务不需要对外网暴露。
HOST = "127.0.0.1"
PORT = 8000



def create_app() -> FastAPI:
    # 应用状态里放一个全局锁和订阅标记，用于限制同一时刻只有一个推理客户端。
    app = FastAPI()
    app.state.subscription_lock = asyncio.Lock()
    app.state.subscription_active = False
    # 注册 WebSocket 路由（/ws/inference）。
    app.include_router(router)
    return app


# 模块级单例应用，供 uvicorn 直接引用。
app = create_app()


if __name__ == "__main__":
    # 配置 INFO 日志，把 worker 与协议层的日志输出到控制台。
    logging.basicConfig(level=logging.INFO)
    uvicorn.run(
        app,
        host=HOST,
        port=PORT,
        # 使用 websockets 实现，并把单条消息上限与协议层保持一致（1 MiB）。
        ws="websockets",
        ws_max_size=MAX_WEBSOCKET_MESSAGE_BYTES,
    )
