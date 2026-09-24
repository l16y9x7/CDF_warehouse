import base64
import json
import tempfile
import unittest
from pathlib import Path

from agent.capabilities.estimation import load_test_case, scan_test_cases
from agent.capabilities.estimation.contract import test_case_meta

CAMERA = {
    "cam_K": [612.0, 0.0, 641.0, 0.0, 611.0, 358.0, 0.0, 0.0, 1.0],
    "width": 2,
    "height": 2,
}
SHARED = {
    "_comment": "documentation only",
    "target_type": "sku",
    "sku_typ": "box",
    "side": "RIGHT",
    "rgb_base64": "<BASE64_OF_RGB_JPG_OR_PNG_FILE_BYTES>",
    "depth_npy_base64": "<BASE64_OF_ALIGNED_FLOAT32_DEPTH_NPY_BYTES>",
    "depth_unit": "mm",
    "K": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    "T_chassis_camera": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    "T_unit": "m",
}
RGB_BYTES = b"\xff\xd8jpeg-bytes"


def make_npy(shape=(2, 2), descr="<f4", fortran=False, magic=b"\x93NUMPY"):
    header = repr({"descr": descr, "fortran_order": fortran, "shape": shape}).encode("latin1")
    pad = (64 - (10 + len(header) + 1) % 64) % 64
    header = header + b" " * pad + b"\n"
    itemsize = int(descr[-1]) if descr[-1].isdigit() else 4
    count = 1
    for dim in shape:
        count *= dim
    version_len = 2 if magic == b"\x93NUMPY" else 4
    return (
        magic
        + bytes((1, 0))
        + len(header).to_bytes(version_len, "little")
        + header
        + b"\x00" * count * itemsize
    )


class TestCaseLoaderTest(unittest.TestCase):
    def setUp(self):
        self._temp = tempfile.TemporaryDirectory()
        self.addCleanup(self._temp.cleanup)
        self.root = Path(self._temp.name)

    def write_shared(self, **overrides):
        (self.root / "test_case_request.json").write_text(
            json.dumps({**SHARED, **overrides}), encoding="utf-8"
        )

    def make_case(self, name, *, rgb=RGB_BYTES, npy=None, camera=CAMERA, request=None):
        case_dir = self.root / name
        case_dir.mkdir(parents=True, exist_ok=True)
        if rgb is not None:
            (case_dir / "rgb.jpg").write_bytes(rgb)
        (case_dir / "depth_mm.npy").write_bytes(make_npy() if npy is None else npy)
        if camera is not None:
            (case_dir / "camera.json").write_text(json.dumps(camera), encoding="utf-8")
        if request is not None:
            (case_dir / "request.json").write_text(json.dumps(request), encoding="utf-8")
        return case_dir

    def test_scan_filters_sku_prefixed_dirs(self):
        self.make_case("bottle_120045958")
        self.make_case("BOX_2")
        self.make_case("basket_9")
        (self.root / "tube_3").mkdir()
        (self.root / "avene_120045958").mkdir()
        (self.root / "notes_dir").mkdir()
        (self.root / "loose_file.txt").write_text("x", encoding="utf-8")
        self.assertEqual(
            scan_test_cases(self.root),
            ["BOX_2", "basket_9", "bottle_120045958", "tube_3"],
        )
        self.assertEqual(scan_test_cases(self.root / "absent"), [])

    def test_single_case_autoselected_and_merged(self):
        self.write_shared()
        self.make_case("bottle_120045958")
        data = load_test_case(root=self.root)
        self.assertEqual(data["sku_typ"], "bottle")  # 目录前缀覆盖共享模板
        self.assertEqual(data["side"], "RIGHT")  # 共享模板轻字段保留
        self.assertEqual(
            data["rgb_base64"], base64.b64encode(RGB_BYTES).decode("ascii")
        )  # 占位符被帧文件替换
        self.assertEqual(
            data["depth_npy_base64"], base64.b64encode(make_npy()).decode("ascii")
        )
        self.assertEqual(  # camera.json 逐帧内参覆盖共享模板 K
            data["K"], [[612.0, 0.0, 641.0], [0.0, 611.0, 358.0], [0.0, 0.0, 1.0]]
        )
        self.assertNotIn("_comment", data)

    def test_case_request_json_has_top_priority(self):
        self.write_shared()
        self.make_case(
            "box_9",
            request={"side": "LEFT", "K": [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]},
        )
        data = load_test_case("box_9", root=self.root)
        self.assertEqual(data["side"], "LEFT")
        self.assertEqual(data["K"][0][0], 500.0)
        self.assertEqual(data["sku_typ"], "box")

    def test_multiple_cases_require_explicit_selection(self):
        self.write_shared()
        self.make_case("bottle_1")
        self.make_case("box_2")
        with self.assertRaisesRegex(ValueError, "请选择"):
            load_test_case(root=self.root)
        self.assertEqual(load_test_case("box_2", root=self.root)["sku_typ"], "box")

    def test_unknown_case_rejected(self):
        self.write_shared()
        with self.assertRaisesRegex(ValueError, "不存在"):
            load_test_case("bottle_404", root=self.root)

    def test_basket_prefix_is_loaded_without_sku_mapping(self):
        self.write_shared()
        self.make_case("basket_9")
        data = load_test_case("basket_9", root=self.root)
        self.assertEqual(data["sku_typ"], "box")  # 共享模板保留；basket 前缀不是 sku_typ
        meta = {item["name"]: item for item in test_case_meta(self.root)}
        self.assertIn("basket_9", meta)
        self.assertNotIn("sku_typ", meta["basket_9"])

    def test_meta_exposes_merged_case_for_prefill(self):
        self.write_shared()
        self.make_case("bottle_1")
        self.make_case("tube_2", request={"side": "LEFT"})
        meta = {item["name"]: item for item in test_case_meta(self.root)}
        for name in ("sku_typ", "side", "target_type"):
            self.assertNotIn(name, meta["bottle_1"])
            self.assertNotIn(name, meta["tube_2"])
        self.assertEqual(meta["bottle_1"]["K"][0][0], 612.0)  # 帧文件内参
        self.assertEqual(meta["bottle_1"]["T_unit"], "m")
        self.assertEqual(meta["bottle_1"]["depth_unit"], "mm")
        self.assertNotIn("rgb_base64", meta["bottle_1"])  # 大载荷不进预填元数据

    def test_depth_files_are_encoded_without_npy_geometry_checks(self):
        self.write_shared()
        raw = b'{"json": true}'
        self.make_case("bottle_bad", npy=raw)
        data = load_test_case("bottle_bad", root=self.root)
        self.assertEqual(
            data["depth_npy_base64"], base64.b64encode(raw).decode("ascii")
        )

    def test_root_level_files_without_case_dirs(self):
        self.write_shared()
        (self.root / "rgb.jpg").write_bytes(RGB_BYTES)
        (self.root / "depth_mm.npy").write_bytes(make_npy())
        data = load_test_case(root=self.root)
        self.assertEqual(data["rgb_base64"], base64.b64encode(RGB_BYTES).decode("ascii"))
        self.assertEqual(data["sku_typ"], "box")  # 无用例目录时保留共享模板值


if __name__ == "__main__":
    unittest.main()
