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
        catalog = load_sku_catalog(config)
        self.assertEqual(
            catalog,
            {
                "3282779003131": SkuSpec(
                    "bottle", Hand.RIGHT, "Avene 雅漾 雅漾舒泉调理喷雾 300ml"
                ),
                "887167608641": SkuSpec(
                    "box",
                    Hand.LEFT,
                    "Estee Lauder 雅诗兰黛 雅诗兰黛特润修护肌活精华眼霜双支装 15ml*2",
                ),
                "7173342765403": SkuSpec(
                    "tube", Hand.LEFT, "Origins 悦木之源 ORIGINS一举两得泡沫洁面慕斯 30ml"
                ),
            },
        )
        self.assertIs(sku_spec(catalog, "3282779003131").hand, Hand.RIGHT)
        self.assertEqual(
            sku_spec(catalog, "887167608641").name,
            "Estee Lauder 雅诗兰黛 雅诗兰黛特润修护肌活精华眼霜双支装 15ml*2",
        )
        self.assertIs(shared_working_hand(catalog, ["887167608641", "7173342765403"]), Hand.LEFT)
        with self.assertRaisesRegex(ValueError, "同一只工作手"):
            shared_working_hand(catalog, ["3282779003131", "887167608641"])

    def test_sku_catalog_normalizes_keys_and_rejects_invalid_entries(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "workflows.yaml"
            path.write_text(
                "skus:\n  3282779003131:\n    sku_typ: bottle\n    hand: right\n",
                encoding="utf-8",
            )
            self.assertEqual(
                load_sku_catalog(path),
                {"3282779003131": SkuSpec("bottle", Hand.RIGHT)},
            )
            path.write_text("skus:\n  sku-1: bottle\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "skus"):
                load_sku_catalog(path)
            path.write_text(
                "skus:\n  sku-1:\n    sku_typ: cup\n    hand: LEFT\n",
                encoding="utf-8",
            )
            self.assertEqual(
                load_sku_catalog(path),
                {"sku-1": SkuSpec("cup", Hand.LEFT)},
            )
            path.write_text(
                "skus:\n  sku-1:\n    sku_typ: \"\"\n    hand: LEFT\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "sku_typ"):
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
