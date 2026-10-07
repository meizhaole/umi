# 组装并启动 UMI 回放后端服务
# 这个模块负责创建 FastAPI 应用、挂载推理路由，并保存两类 UR5 CAD 动作审阅结果。
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


# 服务默认只绑定本机回环地址，供本机前端和机器人仿真工具访问。
HOST = "127.0.0.1"
PORT = 8000
# 审阅文件统一写入 robot-sim/debug；使用当前源码文件的位置计算路径，避免依赖启动时的工作目录。
SINGLE_ACTION_REVIEW_PATH = (
    Path(__file__).resolve().parents[1] / "debug" / "ur5-cad-single-action-review.json"
)
AUTO_LOOP_REVIEW_DIR = Path(__file__).resolve().parents[1] / "debug" / "ur5-cad-auto-loop"
# 自动闭环记录最多接受 20 个步骤；run_id 仅允许字母、数字和连字符，且长度为 1 至 64 个字符。
AUTO_LOOP_MAX_STEPS = 20
AUTO_LOOP_RUN_ID_PATTERN = re.compile(r"[A-Za-z0-9-]{1,64}")


def create_app() -> FastAPI:
    # 在函数内创建应用及其状态，便于服务入口和其他调用方复用同一套配置。
    app = FastAPI()
    # 开发前端运行在两个本机来源之一；这里只允许 POST、content-type 请求头，不开放其他跨域方法。
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
        allow_methods=["POST"],
        allow_headers=["content-type"],
    )
    # 路由共享订阅锁和活动标志，用于协调推理模块中的订阅操作。
    app.state.subscription_lock = asyncio.Lock()
    app.state.subscription_active = False
    # 挂载推理模块提供的 API 和 WebSocket 路由。
    app.include_router(router)

    def persist_single_action_review(review: dict, destination: Path):
        # 此审阅接口专用于 UR5 CAD 实机配置；机器人 ID、配置 ID 和 URDF 路径必须同时匹配。
        # 其他配置返回 403，避免误存为本流程的审阅记录。
        if (
            review.get("robot_id") != "UR5_CAD"
            or review.get("robot_config_id") != "UR5_CAD"
            or review.get("robot_config_file") != "ur_umi_real/urdf/ur5_umi_real.urdf"
        ):
            raise HTTPException(
                status_code=403,
                detail="Single-action validation is only available for UR5_CAD.",
            )

        # 状态标记为 COMPLETED 时，要求调用方一并提交动作后的实际关节值，以及 TCP 的位置和方向列表；
        # 实际关节对象还必须非空。未完成的审阅仍可保存，因此只对已完成记录检查这些字段。
        # 缺少所需数据时返回 422；这里检查字段类型和是否为空，不负责验证向量长度或数值范围。
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
            # 保留调用方的审阅字段，并补充服务端确认的保存状态和目标路径。
            persisted_review = {
                **review,
                "persistence_status": "saved",
                "persistence_path": str(destination),
            }
            # 禁止 NaN/Infinity 等非标准 JSON 数值，同时保留中文并缩进，便于人工检查。
            review_json = json.dumps(
                persisted_review,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            ) + "\n"
            # 先确保目录存在，再写入同目录临时文件，最后替换正式文件，避免目标文件留下半截 JSON。
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = destination.with_suffix(".json.tmp")
            temporary_path.write_text(review_json, encoding="utf-8")
            temporary_path.replace(destination)
        except (OSError, TypeError, ValueError) as error:
            # 文件操作失败或审阅内容无法编码为 JSON 时，返回 HTTP 500，并保留底层异常原因。
            raise HTTPException(
                status_code=500,
                detail=f"Review JSON save failed: {error}",
            ) from error

        return {"saved": True, "path": str(destination)}

    @app.post("/api/single-action-review")
    async def save_single_action_review(review: dict):
        # 单动作审阅写入固定文件；校验和序列化统一由持久化函数完成。
        return persist_single_action_review(review, SINGLE_ACTION_REVIEW_PATH)

    @app.post("/api/ur5-cad-auto-loop-review")
    async def save_auto_loop_review(review: dict):
        # run_id 需完整匹配上方的字符集和长度限制；step 必须是 1 到 20 的整数。
        # 使用 type(step) is int 可拒绝 Python 中也属于 int 子类的 bool 值；任一条件不符均返回 422。
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

        # 将运行编号和步骤写入文件名，便于按一次运行和其中的步骤分别查阅记录。
        destination = AUTO_LOOP_REVIEW_DIR / f"{run_id}-step-{step}.json"
        return persist_single_action_review(review, destination)

    return app


# 创建模块级应用实例，供 Uvicorn 以 app 对象启动，也供导入此模块的工具直接使用。
app = create_app()


if __name__ == "__main__":
    # 直接运行本文件时启动本机 HTTP 服务；WebSocket 消息上限与协议模块的配置保持一致。
    logging.basicConfig(level=logging.INFO)
    uvicorn.run(
        app,
        host=HOST,
        port=PORT,
        ws="websockets",
        ws_max_size=MAX_WEBSOCKET_MESSAGE_BYTES,
    )
