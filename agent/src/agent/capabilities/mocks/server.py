import logging
import os
import threading
from dataclasses import dataclass

import uvicorn

from agent.capabilities.mocks.apps import (
    camera_app,
    estimation_app,
    hand_app,
    manipulation_app,
    navigation_app,
    perception_app,
    pose_app,
    vla_app,
)
from agent.observability import configure_logging, log_event

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MockService:
    name: str
    app: object
    port: int


def services() -> tuple[MockService, ...]:
    return (
        MockService(
            "navigation", navigation_app, int(os.getenv("AGENT_MOCK_NAVIGATION_PORT", "28081"))
        ),
        MockService("pose", pose_app, int(os.getenv("AGENT_MOCK_POSE_PORT", "28082"))),
        MockService(
            "perception", perception_app, int(os.getenv("AGENT_MOCK_PERCEPTION_PORT", "28083"))
        ),
        MockService(
            "estimation", estimation_app, int(os.getenv("AGENT_MOCK_ESTIMATION_PORT", "28084"))
        ),
        MockService("camera", camera_app, int(os.getenv("AGENT_MOCK_CAMERA_PORT", "28085"))),
        MockService(
            "manipulation",
            manipulation_app,
            int(os.getenv("AGENT_MOCK_MANIPULATION_PORT", "28086")),
        ),
        MockService("vla", vla_app, int(os.getenv("AGENT_MOCK_VLA_PORT", "28087"))),
        MockService("hand", hand_app, int(os.getenv("AGENT_MOCK_HAND_PORT", "28088"))),
    )


def main() -> None:
    configure_logging(database_path=os.getenv("AGENT_DATABASE_PATH", "agent-tasks.db"))
    servers: list[uvicorn.Server] = []
    threads: list[threading.Thread] = []
    host = os.getenv("AGENT_MOCK_HOST", "127.0.0.1")

    for service in services():
        config = uvicorn.Config(service.app, host=host, port=service.port, log_level="info")
        server = uvicorn.Server(config)
        thread = threading.Thread(target=server.run, name=f"mock-{service.name}", daemon=True)
        servers.append(server)
        threads.append(thread)
        thread.start()
        log_event(
            logger, logging.INFO, "mock.started",
            f"{service.name} Mock 模块已启动", capability=service.name, port=service.port,
            status="SUCCEEDED",
        )

    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        for server in servers:
            server.should_exit = True
        for thread in threads:
            thread.join(timeout=5)


if __name__ == "__main__":
    main()
