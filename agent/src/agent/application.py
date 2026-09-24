from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from agent.capabilities.camera import HttpCameraCapability
from agent.capabilities.common import HealthStatus
from agent.capabilities.estimation import HttpEstimationCapability, load_test_case
from agent.capabilities.hand import HttpHandCapability
from agent.capabilities.http import HttpCapabilityClient
from agent.capabilities.manipulation import HttpManipulationCapability
from agent.capabilities.navigation import HttpNavigationCapability
from agent.capabilities.perception import HttpPerceptionCapability
from agent.capabilities.pose import HttpBodyPoseCapability
from agent.capabilities.vla import HttpVlaCapability
from agent.config import load_pick_policy, load_sku_catalog, load_yaml_config
from agent.runtime import AgentRuntime
from agent.skills.basket_finish import PushBasketSkill
from agent.skills.navigate import NavigateSkill
from agent.skills.perception import RecognizeAndVerifyBarcodeSkill
from agent.skills.pick_sku_hand import PickSkuHandSkill
from agent.skills.pick_sku_standard import PickSkuStandardSkill
from agent.skills.place_sku_in_basket import PlaceSkuInBasketSkill
from agent.skills.pose import PreparePoseSkill
from agent.skills.review import (
    ConfirmReviewBasketEmptySkill,
    PickReviewBasketSkill,
    PickReviewItemStandardSkill,
    PickReviewItemVlaSkill,
    PlaceReviewBasketSkill,
    PlaceReviewItemSkill,
    SummarizeReviewResultSkill,
)
from agent.task_store import SqliteTaskStore
from agent.skus import SkuSpec
from agent.tracing import traced_capabilities, traced_skills
from agent.workflows.policies import PickPolicy

WORKFLOW_CAPABILITY_NAMES = (
    "navigation",
    "pose",
    "perception",
    "estimation",
    "camera",
    "manipulation",
)
from agent.workflows.review import ReviewWorkflow
from agent.workflows.sorting import SortingFinishWorkflow, SortingItemWorkflow


@dataclass
class AgentApplication:
    store: SqliteTaskStore
    runtime: AgentRuntime
    policy: PickPolicy
    workflows: dict[str, Any]
    capabilities: dict[str, Any]
    skills: dict[str, Any]
    _owned_resources: tuple[Any, ...] = field(default_factory=tuple, repr=False)

    def workflow(self, task_type: str) -> Any:
        return self.workflows[task_type]

    def preflight(self, task_type: str) -> None:
        unavailable = [
            name
            for name in WORKFLOW_CAPABILITY_NAMES
            if self.capabilities[name].health().status is not HealthStatus.READY
        ]
        if unavailable:
            raise RuntimeError(f"capabilities are not ready: {', '.join(unavailable)}")

    def close(self) -> None:
        try:
            self.runtime.shutdown()
        finally:
            for resource in self._owned_resources:
                resource.close()


def build_application(
    *,
    database_path: str | Path = "agent-tasks.db",
    modules_path: str | Path = "configs/modules.yaml",
    workflows_path: str | Path = "configs/workflows.yaml",
) -> AgentApplication:
    config = load_yaml_config(modules_path)
    names = {
        "navigation",
        "pose",
        "perception",
        "estimation",
        "camera",
        "manipulation",
        "vla",
        "hand",
    }
    clients = {
        name: HttpCapabilityClient(
            values["base_url"], capability=name, timeout=float(values.get("timeout", 30.0))
        )
        for name, values in config.items()
        if name in names
    }
    capabilities = {
        "navigation": HttpNavigationCapability(clients["navigation"]),
        "pose": HttpBodyPoseCapability(clients["pose"]),
        "perception": HttpPerceptionCapability(clients["perception"]),
        "estimation": HttpEstimationCapability(clients["estimation"]),
        "camera": HttpCameraCapability(clients["camera"]),
        "manipulation": HttpManipulationCapability(clients["manipulation"]),
        "vla": HttpVlaCapability(clients["vla"]),
        "hand": HttpHandCapability(clients["hand"]),
    }
    return build_application_from_capabilities(
        capabilities,
        database_path=database_path,
        policy=load_pick_policy(workflows_path),
        sku_catalog=load_sku_catalog(workflows_path),
        owned_resources=tuple(clients.values()),
    )


def build_application_from_capabilities(
    capabilities: dict[str, Any],
    *,
    database_path: str | Path,
    policy: PickPolicy | None = None,
    owned_resources: tuple[Any, ...] = (),
    calibration_source: Callable[[], Mapping[str, Any]] | None = None,
    sku_catalog: Mapping[str, SkuSpec] | None = None,
) -> AgentApplication:
    policy = policy or PickPolicy()
    calibration_source = calibration_source or load_test_case
    sku_catalog = dict(sku_catalog or {})
    capabilities = traced_capabilities(capabilities)
    p, c, e, m = (
        capabilities["perception"],
        capabilities["camera"],
        capabilities["estimation"],
        capabilities["manipulation"],
    )
    skills = {
        "navigate": NavigateSkill(capabilities["navigation"]),
        "prepare_pose": PreparePoseSkill(capabilities["pose"]),
        "recognize_and_verify_barcode": RecognizeAndVerifyBarcodeSkill(p, c, m),
        "pick_sku_standard": PickSkuStandardSkill(
            c,
            e,
            m,
            capabilities["pose"],
            calibration_source,
            sku_catalog=sku_catalog,
        ),
        "place_sku_in_basket": PlaceSkuInBasketSkill(c, e, m, capabilities["pose"]),
        "push_basket": PushBasketSkill(c, e, m, capabilities["pose"]),
        "pick_review_basket": PickReviewBasketSkill(c, e, m, capabilities["pose"]),
        "place_review_basket": PlaceReviewBasketSkill(m),
        "pick_review_item_standard": PickReviewItemStandardSkill(p, c, e, m),
        "place_review_item": PlaceReviewItemSkill(m),
        "confirm_review_basket_empty": ConfirmReviewBasketEmptySkill(p, c),
        "summarize_review_result": SummarizeReviewResultSkill(),
    }
    if "hand" in capabilities:
        skills["pick_sku_hand"] = PickSkuHandSkill(capabilities["hand"])
    if "vla" in capabilities:
        skills["pick_review_item_vla"] = PickReviewItemVlaSkill(capabilities["vla"])
    skills = traced_skills(skills)
    store = SqliteTaskStore(database_path)
    workflows = {
        "sorting_item": SortingItemWorkflow(
            skills, state_store=store, policy=policy, sku_catalog=sku_catalog
        ),
        "sorting_finish": SortingFinishWorkflow(skills, state_store=store, policy=policy),
        "review": ReviewWorkflow(
            skills, state_store=store, policy=policy, sku_catalog=sku_catalog
        ),
    }
    holder: dict[str, AgentApplication] = {}
    runtime = AgentRuntime(store, lambda name: holder["app"].workflow(name))
    app = AgentApplication(
        store,
        runtime,
        policy,
        workflows,
        capabilities,
        skills,
        owned_resources,
    )
    holder["app"] = app
    return app
