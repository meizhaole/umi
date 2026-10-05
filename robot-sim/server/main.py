# 组装并启动 UMI 回放后端服务
# 该服务只暴露一个 WebSocket 推理端点，供 robot-sim 前端连接；HTTP 部分仅用于健康检查。
#
# 这个文件是后端入口，职责只有三件：
#   1. 创建一个 FastAPI 应用（Web 服务骨架）；
#   2. 把 inference.py 里的 /ws/inference 路由挂上去；
#   3. 直接运行本文件时，用 uvicorn 真正监听端口、收发连接。
# 前端 robot-sim 连接地址是 ws://localhost:8000/ws/inference。
import asyncio
import logging

# uvicorn 是 ASGI 服务器，负责监听端口、维护连接、把消息交给 FastAPI 处理。
import uvicorn
# FastAPI 是 Web 框架，用来组织路由和应用状态。
from fastapi import FastAPI

# 下面这段是 Python 的“双模式导入”写法，目的是让同一个文件既能被当作包的一部分导入，
# 也能直接 python main.py 运行：
#   - 作为包导入时（如 python -m server.main），__package__ 有值，用相对导入 .inference；
#   - 直接运行 main.py 时，__package__ 为空，用同目录下的绝对导入 inference。
# 两条分支加载的是同一份代码，只是写法不同。
if __package__:
    # inference.py 里定义了 router，其中注册了 WebSocket 端点。
    from .inference import router
    # 协议层定义的单条消息字节上限，启动 uvicorn 时要与它保持一致。
    from .inference_protocol import MAX_WEBSOCKET_MESSAGE_BYTES
else:
    from inference import router
    from inference_protocol import MAX_WEBSOCKET_MESSAGE_BYTES


# 只监听本机回环地址。推理服务要加载 GPU 模型，不对外网开放，避免被外部调用。
HOST = "127.0.0.1"
# 监听端口；前端 WebSocket 地址固定为 ws://localhost:8000/ws/inference。
PORT = 8000



def create_app() -> FastAPI:
    # 创建 FastAPI 应用对象，后续所有路由和共享状态都挂在它身上。
    app = FastAPI()

    # app.state 是一个可以随便放属性的命名空间，用来存跨请求共享的状态。这里放两个：
    #   subscription_lock：asyncio 锁，保证“检查是否有人占用 + 标记占用”这两步是原子操作，
    #                      否则两个客户端可能同时通过检查，各起一个 worker 把显存占满；
    #   subscription_active：布尔标记，表示当前是否已有一个推理客户端在跑。
    # 二者配合实现“同一时刻只允许一个客户端推理”。
    app.state.subscription_lock = asyncio.Lock()
    app.state.subscription_active = False

    # 把 inference.py 的 router 挂到应用上。include_router 会把 router 里注册的
    # 所有路由复制进 app 的路由表，于是 /ws/inference 生效。
    app.include_router(router)
    return app


# 模块级创建一次应用实例，uvicorn 启动时直接引用这个对象。
app = create_app()


# 只有“直接运行本文件”时才执行下面代码；被 import 时不会自动启动服务器。
if __name__ == "__main__":
    # 把根日志级别设为 INFO，这样 inference.py / load_model.py 里的 LOGGER.info 才会打印到终端。
    logging.basicConfig(level=logging.INFO)
    # 启动服务器并阻塞运行，直到 Ctrl+C 退出。
    #   app：要运行的 FastAPI 应用；
    #   host / port：监听地址与端口；
    #   ws="websockets"：WebSocket 协议的具体实现库；
    #   ws_max_size：单条 WebSocket 消息的字节上限，与 worker 协议保持一致（1 MiB），
    #                防止超大消息把内存拖爆。
    uvicorn.run(
        app,
        host=HOST,
        port=PORT,
        ws="websockets",
        ws_max_size=MAX_WEBSOCKET_MESSAGE_BYTES,
    )
