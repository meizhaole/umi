# 组装并启动 UMI 回放后端服务
import asyncio
import json
import logging
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

if __package__:
    from .inference import router
    from .inference_protocol import MAX_WEBSOCKET_MESSAGE_BYTES
else:
    from inference import router
    from inference_protocol import MAX_WEBSOCKET_MESSAGE_BYTES


HOST = "127.0.0.1"
PORT = 8000
SINGLE_ACTION_REVIEW_PATH = (
    Path(__file__).resolve().parents[1] / "debug" / "ur5-cad-single-action-review.json"
)


def create_app() -> FastAPI:
    app = FastAPI()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
        allow_methods=["POST"],
        allow_headers=["content-type"],
    )
    app.state.subscription_lock = asyncio.Lock()
    app.state.subscription_active = False
    app.include_router(router)

    @app.post("/api/single-action-review")
    async def save_single_action_review(review: dict):
        if (
            review.get("robot_id") != "UR5_CAD"
            or review.get("robot_config_id") != "UR5_CAD"
            or review.get("robot_config_file") != "ur_umi_real/urdf/ur5_umi_real.urdf"
        ):
            raise HTTPException(
                status_code=403,
                detail="Single-action validation is only available for UR5_CAD.",
            )

        try:
            persisted_review = {
                **review,
                "persistence_status": "saved",
                "persistence_path": str(SINGLE_ACTION_REVIEW_PATH),
            }
            review_json = json.dumps(
                persisted_review,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            ) + "\n"
            SINGLE_ACTION_REVIEW_PATH.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = SINGLE_ACTION_REVIEW_PATH.with_suffix(".json.tmp")
            temporary_path.write_text(review_json, encoding="utf-8")
            temporary_path.replace(SINGLE_ACTION_REVIEW_PATH)
        except (OSError, TypeError, ValueError) as error:
            raise HTTPException(
                status_code=500,
                detail=f"Review JSON save failed: {error}",
            ) from error

        return {"saved": True, "path": str(SINGLE_ACTION_REVIEW_PATH)}

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
