from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Any, ClassVar

from agent.skus import SkuSpec, shared_working_hand, sku_spec
from agent.contracts import ExecutionContext
from agent.layouts.basket import basket_nav_id, validate_basket_position
from agent.models import InspectedItem, ReviewItemCount
from agent.skills.navigate import NavigateInput
from agent.skills.perception import RecognizeBarcodeInput
from agent.skills.pose import PreparePoseInput
from agent.skills.review import (
    EmptyStatus,
    HandOnlyInput,
    ReviewPickStatus,
    SummarizeReviewInput,
)
from agent.workflows.base import InMemoryWorkflowStateStore, NodeRunner, WorkflowStateStore
from agent.workflows.policies import PickPolicy
from agent.workflows.state import ReviewTaskState


@dataclass(frozen=True)
class ReviewInput:
    order_id: str
    basket_row: str
    basket_column: str
    expected_items: tuple[ReviewItemCount, ...]


class ReviewWorkflow:
    name = "review"
    waiting_codes: ClassVar[frozenset[str]] = frozenset(
        {"ACTION_RESULT_UNKNOWN", "SKU_BARCODE_NOT_FOUND"}
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
        required = {
            "navigate",
            "prepare_pose",
            "pick_review_basket",
            "place_review_basket",
            "recognize_and_verify_barcode",
            "place_review_item",
            "confirm_review_basket_empty",
            "summarize_review_result",
        }
        required.add("pick_review_item_standard")
        missing = required.difference(self.skills)
        if missing:
            raise ValueError(f"missing review skills: {sorted(missing)}")

    def run(self, context: ExecutionContext, data: ReviewInput) -> dict[str, Any]:
        _validate_review(data)
        hand = shared_working_hand(self.sku_catalog, [item.sku_id for item in data.expected_items])
        state = ReviewTaskState(
            context.task_id,
            data.order_id,
            data.basket_row,
            data.basket_column,
            list(data.expected_items),
            review_policy={"hand": hand.value},
        )
        state.status = "RUNNING"
        context.metadata.update({"workflow_id": self.name, "workflow": self.name})
        context.emit("workflow.started", workflow=self.name)
        runner = NodeRunner(
            context,
            self.state_store,
            state=state,
            node_attribute="last_node",
        )
        self.state_store.save(context.task_id, asdict(state))
        try:
            runner.run(
                "R-N01",
                lambda: self.skills["navigate"].execute(
                    context, NavigateInput(basket_nav_id(data.basket_column, review=True))
                ),
            )
            runner.run(
                "R-N02",
                lambda: self.skills["prepare_pose"].execute(
                    context, PreparePoseInput("basket_pick_prepare", data.basket_row)
                ),
            )
            runner.run(
                "R-N03",
                lambda: self.skills["pick_review_basket"].execute(
                    context, HandOnlyInput(hand)
                ),
                on_success=lambda _: state.basket_transfer_state.update(picked=True),
            )
            runner.run(
                "R-N04",
                lambda: self.skills["navigate"].execute(context, NavigateInput("REVIEW_TABLE")),
            )
            runner.run(
                "R-N05",
                lambda: self.skills["prepare_pose"].execute(
                    context, PreparePoseInput("basket_place")
                ),
            )
            runner.run(
                "R-N06",
                lambda: self.skills["place_review_basket"].execute(
                    context, HandOnlyInput(hand)
                ),
                on_success=lambda _: state.basket_transfer_state.update(placed=True),
            )

            while True:
                runner.run(
                    "R-N07",
                    lambda: self.skills["prepare_pose"].execute(
                        context, PreparePoseInput("basket_item_pick_inspect")
                    ),
                )
                runner.run(
                    "R-N08",
                    lambda: self.skills["prepare_pose"].execute(
                        context, PreparePoseInput("basket_item_pick_prepare")
                    ),
                )
                def record_pick(result: Any) -> None:
                    if result.status is ReviewPickStatus.NOT_FOUND:
                        state.empty_observations = 1
                        return
                    state.empty_observations = 0
                    state.current_item = {
                        "sequence": len(state.inspected_items) + 1,
                        "pick_status": "PICKED",
                        "actual_sku_id": None,
                        "place_status": None,
                    }

                picked = runner.run(
                    "R-N09",
                    lambda: self.skills["pick_review_item_standard"].execute(
                        context, HandOnlyInput(hand)
                    ),
                    on_success=record_pick,
                )
                if picked.status is ReviewPickStatus.NOT_FOUND:

                    def record_confirmation(result: Any) -> None:
                        state.empty_observations = (
                            2 if result.status is EmptyStatus.NOT_FOUND else 0
                        )

                    confirmed = runner.run(
                        "R-N10",
                        lambda: self.skills["confirm_review_basket_empty"].execute(
                            context, HandOnlyInput(hand)
                        ),
                        on_success=record_confirmation,
                    )
                    if confirmed.status is EmptyStatus.NOT_FOUND:
                        break
                    continue

                runner.run(
                    "R-N11",
                    lambda: self.skills["prepare_pose"].execute(
                        context, PreparePoseInput("basket_item_barcode_scan")
                    ),
                )

                def record_barcode(result: Any) -> None:
                    assert state.current_item is not None
                    state.current_item["actual_sku_id"] = result.sku_id

                runner.run(
                    "R-N12",
                    lambda: self.skills["recognize_and_verify_barcode"].execute(
                        context,
                        RecognizeBarcodeInput(
                            None,
                            None,
                            hand,
                            sku_spec(self.sku_catalog, data.expected_items[0].sku_id).sku_typ,
                        ),
                    ),
                    on_success=record_barcode,
                )
                runner.run(
                    "R-N13",
                    lambda: self.skills["prepare_pose"].execute(
                        context, PreparePoseInput("review_item_place")
                    ),
                )

                def record_placement(_: Any) -> None:
                    assert state.current_item is not None
                    state.inspected_items.append(
                        InspectedItem(
                            state.current_item["sequence"],
                            state.current_item["actual_sku_id"],
                        )
                    )
                    state.current_item = None

                runner.run(
                    "R-N14",
                    lambda: self.skills["place_review_item"].execute(
                        context, HandOnlyInput(hand)
                    ),
                    on_success=record_placement,
                )

            def finish_review(result: Any) -> None:
                state.review_summary = result
                state.status = "SUCCEEDED"

            summary = runner.run(
                "R-N15",
                lambda: self.skills["summarize_review_result"].execute(
                    context,
                    SummarizeReviewInput(tuple(state.expected_items), tuple(state.inspected_items)),
                ),
                on_success=finish_review,
            )
            context.emit("workflow.succeeded", workflow=self.name)
            return asdict(summary)
        except Exception as exc:
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
            raise


def _validate_review(data: ReviewInput) -> None:
    validate_basket_position(data.basket_row, data.basket_column)
    seen: set[str] = set()
    for item in data.expected_items:
        if not item.sku_id or item.count <= 0 or item.sku_id in seen:
            raise ValueError("expected_items require unique sku_id and positive count")
        seen.add(item.sku_id)


__all__ = ["ReviewInput", "ReviewWorkflow"]
