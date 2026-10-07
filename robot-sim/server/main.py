# 组装并启动 UMI 回放后端服务
import asyncio
import json
import logging
import re
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
AUTO_LOOP_REVIEW_DIR = Path(__file__).resolve().parents[1] / "debug" / "ur5-cad-auto-loop"
AUTO_LOOP_MAX_STEPS = 20
AUTO_LOOP_RUN_ID_PATTERN = re.compile(r"[A-Za-z0-9-]{1,64}")


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

    def persist_single_action_review(review: dict, destination: Path):
        if (
            review.get("robot_id") != "UR5_CAD"
            or review.get("robot_config_id") != "UR5_CAD"
            or review.get("robot_config_file") != "ur_umi_real/urdf/ur5_umi_real.urdf"
        ):
            raise HTTPException(
                status_code=403,
                detail="Single-action validation is only available for UR5_CAD.",
            )

        if review.get("review_status") == "COMPLETED":
            actual_after_action = review.get("actual_after_action")
            actual_tcp = (
                actual_after_action.get("actual_tcp")
                if isinstance(actual_after_action, dict)
                else None
            )
            if (
                not isinstance(actual_after_action, dict)
                or not isinstance(actual_after_action.get("actual_joints"), dict)
                or not actual_after_action.get("actual_joints")
                or not isinstance(actual_tcp, dict)
                or not isinstance(actual_tcp.get("position"), list)
                or not isinstance(actual_tcp.get("orientation"), list)
            ):
                raise HTTPException(
                    status_code=422,
                    detail="A completed review requires actual joints and TCP state.",
                )

        try:
            persisted_review = {
                **review,
                "persistence_status": "saved",
                "persistence_path": str(destination),
            }
            review_json = json.dumps(
                persisted_review,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            ) + "\n"
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = destination.with_suffix(".json.tmp")
            temporary_path.write_text(review_json, encoding="utf-8")
            temporary_path.replace(destination)
        except (OSError, TypeError, ValueError) as error:
            raise HTTPException(
                status_code=500,
                detail=f"Review JSON save failed: {error}",
            ) from error

        return {"saved": True, "path": str(destination)}

    @app.post("/api/single-action-review")
    async def save_single_action_review(review: dict):
        return persist_single_action_review(review, SINGLE_ACTION_REVIEW_PATH)

    @app.post("/api/ur5-cad-auto-loop-review")
    async def save_auto_loop_review(review: dict):
        run_id = review.get("auto_loop_run_id")
        step = review.get("auto_loop_step")
        if (
            not isinstance(run_id, str)
            or AUTO_LOOP_RUN_ID_PATTERN.fullmatch(run_id) is None
            or type(step) is not int
            or not 1 <= step <= AUTO_LOOP_MAX_STEPS
        ):
            raise HTTPException(
                status_code=422,
                detail="A valid auto-loop run ID and step from 1 to 20 are required.",
            )

        destination = AUTO_LOOP_REVIEW_DIR / f"{run_id}-step-{step}.json"
        return persist_single_action_review(review, destination)

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
