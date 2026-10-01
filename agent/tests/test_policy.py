import tempfile
import unittest
from pathlib import Path

from agent.capabilities.common import Hand, TaskType
from agent.config import load_pick_policy, load_sku_catalog
from agent.skus import SkuSpec, shared_working_hand, sku_spec
from agent.workflows.policies import BarcodeMismatchMode, PickPolicy


class PickPolicyTest(unittest.TestCase):
    def test_defaults_to_standard_for_both_workflows(self):
        policy = PickPolicy()
        self.assertIs(policy.select(TaskType.SORTING).hand, Hand.RIGHT)
        self.assertIs(policy.select(TaskType.REVIEW).hand, Hand.RIGHT)
        self.assertIs(policy.barcode_mismatch, BarcodeMismatchMode.STOP)

    def test_rejects_unconnected_workflow_backends_from_config(self):
        with self.assertRaisesRegex(ValueError, "not connected"):
            PickPolicy.from_config(
                {"workflows": {"sorting": {"pick_backend": "HAND", "hand": "LEFT"}}}
            )
        with self.assertRaisesRegex(ValueError, "not connected"):
            PickPolicy.from_config(
                {"workflows": {"review": {"pick_backend": "VLA", "hand": "RIGHT"}}}
            )

    def test_accepts_legacy_standard_backend_value(self):
        policy = PickPolicy.from_config(
            {"workflows": {"sorting": {"pick_backend": "STANDARD", "hand": "LEFT"}}}
        )
        self.assertIs(policy.select(TaskType.SORTING).hand, Hand.LEFT)

    def test_loads_repository_sku_catalog(self):
        config = Path(__file__).parents[1] / "configs" / "workflows.yaml"
        policy = load_pick_policy(config)
        self.assertIs(policy.select(TaskType.SORTING).hand, Hand.RIGHT)
        self.assertIs(policy.select(TaskType.REVIEW).hand, Hand.RIGHT)
        self.assertIs(policy.barcode_mismatch, BarcodeMismatchMode.CONTINUE)
        catalog = load_sku_catalog(Path(__file__).parents[1] / "configs" / "products.yaml")
        self.assertEqual(
            catalog,
            {
                "3282779003131": SkuSpec("bottle", Hand.RIGHT, "雅漾舒护活泉水"),
                "887167608641": SkuSpec("box", Hand.LEFT, "修护精华礼盒"),
                "3282770389746": SkuSpec("tube", Hand.RIGHT, "清润洁面乳"),
            },
        )
        self.assertIs(sku_spec(catalog, "887167608641").hand, Hand.LEFT)
        self.assertEqual(sku_spec(catalog, "887167608641").name, "修护精华礼盒")
        self.assertIs(shared_working_hand(catalog, ["3282779003131", "3282770389746"]), Hand.RIGHT)
        with self.assertRaisesRegex(ValueError, "同一只工作手"):
            shared_working_hand(catalog, ["3282779003131", "887167608641"])

    def test_sku_catalog_normalizes_keys_and_rejects_invalid_entries(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "products.yaml"
            self.assertEqual(load_sku_catalog(path), {})
            path.write_text(
                "products:\n  - sku_id: 3282779003131\n    sku_typ: Bottle\n    hand: right\n",
                encoding="utf-8",
            )
            self.assertEqual(
                load_sku_catalog(path),
                {"3282779003131": SkuSpec("bottle", Hand.RIGHT)},
            )
            path.write_text("products:\n  sku-1: bottle\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "products"):
                load_sku_catalog(path)
            path.write_text(
                "products:\n  - sku_id: sku-1\n    sku_typ: \"\"\n    hand: LEFT\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "sku_typ"):
                load_sku_catalog(path)
            path.write_text(
                "products:\n  - sku_id: sku-1\n    sku_typ: cup\n    hand: BOTH\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "hand"):
                load_sku_catalog(path)

    def test_rejects_invalid_hand(self):
        with self.assertRaises(ValueError):
            PickPolicy.from_config({"workflows": {"sorting": {"hand": "BOTH"}}})

    def test_barcode_mismatch_defaults_to_stop_and_rejects_unknown(self):
        policy = PickPolicy.from_config({"workflows": {"sorting": {"hand": "LEFT"}}})
        self.assertIs(policy.barcode_mismatch, BarcodeMismatchMode.STOP)
        policy = PickPolicy.from_config(
            {"workflows": {"sorting": {"barcode_mismatch": "continue"}}}
        )
        self.assertIs(policy.barcode_mismatch, BarcodeMismatchMode.CONTINUE)
        with self.assertRaisesRegex(ValueError, "barcode_mismatch"):
            PickPolicy.from_config(
                {"workflows": {"sorting": {"barcode_mismatch": "RETRY"}}}
            )


if __name__ == "__main__":
    unittest.main()
