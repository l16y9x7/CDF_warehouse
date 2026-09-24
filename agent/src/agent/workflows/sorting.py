from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Any, ClassVar

from agent.capabilities.common import TaskType
from agent.skus import SkuSpec, sku_spec
from agent.contracts import ExecutionContext
from agent.layouts.basket import (
    agv_nav_id,
    agv_side,
    basket_nav_id,
    validate_agv_position,
    validate_basket_position,
)
from agent.skills.basket_finish import PushBasketInput
from agent.skills.navigate import NavigateInput
from agent.skills.perception import RecognizeBarcodeInput
from agent.skills.pick_sku_standard import PickSkuStandardInput
from agent.skills.place_sku_in_basket import PlaceSkuInBasketInput
from agent.skills.pose import PreparePoseInput
from agent.workflows.base import InMemoryWorkflowStateStore, NodeRunner, WorkflowStateStore
from agent.workflows.policies import BarcodeMismatchMode, PickPolicy
from agent.workflows.state import SortingFinishTaskState, SortingItemTaskState


@dataclass(frozen=True)
class SortingItemInput:
    sku_id: str
    name: str
    agv_row: str
    agv_column: str
    basket_row: str
    basket_column: str


@dataclass(frozen=True)
class SortingFinishInput:
    basket_row: str
    basket_column: str


class _SortingWorkflow:
    waiting_codes: ClassVar[frozenset[str]] = frozenset(
        {
            "ACTION_RESULT_UNKNOWN",
            "SCAN_IMAGE_UNAVAILABLE",
            "SKU_BARCODE_NOT_FOUND",
            "SKU_BARCODE_MISMATCH",
        }
    )

    def __init__(
        self,
        skills: Mapping[str, Any],
        *,
        state_store: WorkflowStateStore | None = None,
        policy: PickPolicy | None = None,
        sku_catalog: Mapping[str, SkuSpec] | None = None,
    ):
        self.skills = dict(skills)
        self.state_store = state_store or InMemoryWorkflowStateStore()
        self.policy = policy or PickPolicy()
        self.sku_catalog = dict(sku_catalog or {})

    def _start(self, context: ExecutionContext, state: Any) -> NodeRunner:
        state.status = "RUNNING"
        context.metadata.update({"workflow_id": self.name, "workflow": self.name})
        context.emit("workflow.started", workflow=self.name)
        self.state_store.save(context.task_id, asdict(state))
        node_attribute = "current_node" if hasattr(state, "current_node") else "last_node"
        return NodeRunner(
            context,
            self.state_store,
            state=state,
            node_attribute=node_attribute,
        )

    def _fail(self, context: ExecutionContext, state: Any, exc: Exception) -> None:
        code = getattr(exc, "code", None)
        state.status = (
            "CANCELLED"
            if code == "CANCELLED"
            else "WAITING_CONFIRMATION"
            if code in self.waiting_codes
            else "FAILED"
        )
        self.state_store.save(context.task_id, asdict(state))
        context.emit(
            "workflow.cancelled" if code == "CANCELLED" else "workflow.failed",
            workflow=self.name,
            status=state.status,
            error_code=code or type(exc).__name__,
        )


class SortingItemWorkflow(_SortingWorkflow):
    name = "sorting_item"

    def __init__(self, skills: Mapping[str, Any], **kwargs: Any):
        super().__init__(skills, **kwargs)
        required = {
            "navigate",
            "prepare_pose",
            "recognize_and_verify_barcode",
            "place_sku_in_basket",
        }
        required.add("pick_sku_standard")
        _require_skills(self.skills, required, self.name)

    def run(self, context: ExecutionContext, data: SortingItemInput) -> dict[str, str]:
        _validate_item(data)
        hand = sku_spec(self.sku_catalog, data.sku_id).hand
        state = SortingItemTaskState(
            context.task_id,
            data.agv_row,
            data.agv_column,
            data.basket_row,
            data.basket_column,
            {"sku_id": data.sku_id, "name": data.name},
            {"hand": hand.value, "barcode_mismatch": self.policy.barcode_mismatch.value},
        )
        runner = self._start(context, state)
        try:
            runner.run(
                "S-I01",
                lambda: self.skills["navigate"].execute(
                    context, NavigateInput(agv_nav_id(data.agv_column, hand))
                ),
            )
            runner.run(
                "S-I02",
                lambda: self.skills["prepare_pose"].execute(
                    context, PreparePoseInput("AGV_carton_item_inspect", data.agv_row)
                ),
            )
            runner.run(
                "S-I03",
                lambda: self.skills["pick_sku_standard"].execute(
                    context,
                    PickSkuStandardInput(
                        data.sku_id,
                        data.name,
                        agv_side(data.agv_column),
                        hand,
                        data.agv_row,
                    ),
                ),
                on_success=lambda result: setattr(
                    state,
                    "pick_result",
                    {"status": result.status, "backend": result.backend},
                ),
            )
            runner.run(
                "S-I04",
                lambda: self.skills["prepare_pose"].execute(
                    context, PreparePoseInput("AGV_item_barcode_scan")
                ),
            )
            try:
                runner.run(
                    "S-I05",
                    lambda: self.skills["recognize_and_verify_barcode"].execute(
                        context,
                        RecognizeBarcodeInput(
                            data.sku_id,
                            data.name,
                            hand,
                            sku_spec(self.sku_catalog, data.sku_id).sku_typ,
                        ),
                    ),
                    on_success=lambda result: setattr(state, "sku_barcode", result.sku_id),
                )
            except Exception as exc:
                if (
                    getattr(exc, "code", None)
                    not in {"SKU_BARCODE_MISMATCH", "SKU_BARCODE_NOT_FOUND"}
                    or self.policy.barcode_mismatch is not BarcodeMismatchMode.CONTINUE
                ):
                    raise
            runner.run(
                "S-I06",
                lambda: self.skills["navigate"].execute(
                    context, NavigateInput(basket_nav_id(data.basket_column))
                ),
            )
            runner.run(
                "S-I07",
                lambda: self.skills["prepare_pose"].execute(
                    context, PreparePoseInput("basket_item_place_prepare", data.basket_row)
                ),
            )

            def finish_item(result: Any) -> None:
                state.place_result = {"status": result.status}
                state.status = "SUCCEEDED"

            runner.run(
                "S-I08",
                lambda: self.skills["place_sku_in_basket"].execute(
                    context,
                    PlaceSkuInBasketInput(
                        sku_spec(self.sku_catalog, data.sku_id).sku_typ,
                        hand,
                    ),
                ),
                on_success=finish_item,
            )
            context.emit("workflow.succeeded", workflow=self.name)
            return {"sku_id": data.sku_id}
        except Exception as exc:
            self._fail(context, state, exc)
            raise


class SortingFinishWorkflow(_SortingWorkflow):
    name = "sorting_finish"

    def __init__(self, skills: Mapping[str, Any], **kwargs: Any):
        super().__init__(skills, **kwargs)
        _require_skills(self.skills, {"navigate", "prepare_pose", "push_basket"}, self.name)

    def run(self, context: ExecutionContext, data: SortingFinishInput) -> dict:
        validate_basket_position(data.basket_row, data.basket_column)
        selection = self.policy.select(TaskType.SORTING)
        state = SortingFinishTaskState(
            context.task_id,
            data.basket_row,
            data.basket_column,
            {"hand": selection.hand.value},
        )
        runner = self._start(context, state)
        try:
            runner.run(
                "S-F01",
                lambda: self.skills["navigate"].execute(
                    context, NavigateInput(basket_nav_id(data.basket_column))
                ),
            )
            runner.run(
                "S-F02",
                lambda: self.skills["prepare_pose"].execute(
                    context, PreparePoseInput("basket_push", data.basket_row)
                ),
            )

            def finish_sorting(_: Any) -> None:
                state.pushed = True
                state.status = "SUCCEEDED"

            runner.run(
                "S-F03",
                lambda: self.skills["push_basket"].execute(
                    context, PushBasketInput(selection.hand)
                ),
                on_success=finish_sorting,
            )
            context.emit("workflow.succeeded", workflow=self.name)
            return {}
        except Exception as exc:
            self._fail(context, state, exc)
            raise


def _validate_item(data: SortingItemInput) -> None:
    if not data.sku_id or not data.name:
        raise ValueError("sku_id and name are required")
    validate_agv_position(data.agv_row, data.agv_column)
    validate_basket_position(data.basket_row, data.basket_column)


def _require_skills(skills: Mapping[str, Any], required: set[str], workflow: str) -> None:
    missing = required.difference(skills)
    if missing:
        raise ValueError(f"missing {workflow} skills: {sorted(missing)}")


__all__ = ["SortingFinishInput", "SortingFinishWorkflow", "SortingItemInput", "SortingItemWorkflow"]
