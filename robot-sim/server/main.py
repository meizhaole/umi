# 组装并启动 UMI 回放后端服务
import asyncio
import logging

import uvicorn
from fastapi import FastAPI

if __package__:
    from .inference import router
    from .inference_protocol import MAX_WEBSOCKET_MESSAGE_BYTES
else:
    from inference import router
    from inference_protocol import MAX_WEBSOCKET_MESSAGE_BYTES


HOST = "127.0.0.1"
PORT = 8000



def create_app() -> FastAPI:
    app = FastAPI()
    app.state.subscription_lock = asyncio.Lock()
    app.state.subscription_active = False
    app.include_router(router)
    return app


app = create_app()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    uvicorn.run(
        app,
        host=HOST,
        port=PORT,
        ws="websockets",
        ws_max_size=MAX_WEBSOCKET_MESSAGE_BYTES,
    )
