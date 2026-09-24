import os

import uvicorn


def main() -> None:
    uvicorn.run(
        "agent.api:create_app",
        factory=True,
        host=os.getenv("AGENT_HOST", "0.0.0.0"),
        port=int(os.getenv("AGENT_PORT", "8090")),
    )


if __name__ == "__main__":
    main()
