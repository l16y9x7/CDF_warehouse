import io
import json
import os
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.occupancy import encode_pgm_gray, occupancy_from_gray, crop_unknown_border
from navigation.rokae_runtime.matrix_occupancy import (
    occupancy_from_export_bytes,
    occupancy_from_local_path,
    occupancy_from_sros_map,
)


class TestOccupancy(unittest.TestCase):
    def test_gray_top_left_flips_to_ros_origin(self) -> None:
        # 图像顶行白/黑，底行灰/白。ROS 第一行应是图像底行。
        grid = occupancy_from_gray(
            [255, 0, 127, 255],
            width=2,
            height=2,
            resolution=0.05,
            origin={"x": -0.53, "y": -0.33},
        )
        self.assertEqual(grid["width"] * grid["height"], len(grid["data"]))
        self.assertEqual(grid["data"], [-1, 0, 0, 100])
        self.assertEqual(grid["origin"], {"x": -0.53, "y": -0.33})

    def test_crop_unknown_border_shifts_origin(self) -> None:
        grid = {
            "width": 4,
            "height": 3,
            "resolution": 0.1,
            "origin": {"x": -0.2, "y": -0.1},
            "data": [
                -1, -1, -1, -1,
                -1, 0, 100, -1,
                -1, -1, -1, -1,
            ],
        }
        cropped = crop_unknown_border(
            grid,
            extra_xy=[{"x": -0.15, "y": 0.05}],
            pad_cells=0,
        )
        self.assertEqual(cropped["resolution"], 0.1)
        self.assertEqual(cropped["origin"], {"x": -0.2, "y": 0.0})
        self.assertEqual((cropped["width"], cropped["height"]), (3, 2))
        self.assertEqual(cropped["data"][0], -1)

    def test_sros_pgm_and_meta(self) -> None:
        doc = {
            "meta": {
                "length_unit": "mm",
                "resolution": 2,
                "size.x": 742,
                "size.y": 906,
                "zero_offset.x": 265,
                "zero_offset.y": 741,
            }
        }
        pgm = encode_pgm_gray(2, 2, [70, 90, 0, 70])
        grid = occupancy_from_sros_map(doc, pgm, image_name="map.pgm")
        self.assertEqual(grid["resolution"], 0.02)
        self.assertAlmostEqual(grid["origin"]["x"], -5.3)
        self.assertAlmostEqual(grid["origin"]["y"], -3.3)
        # 图像底行 0,70 → 未知/空闲；顶行 70,90 → 空闲/占用
        self.assertEqual(grid["data"], [-1, 0, 0, 100])

    def test_sros_origin_keeps_stations_inside(self) -> None:
        doc = {
            "meta": {
                "length_unit": "mm",
                "resolution": 2,
                "size.x": 742,
                "size.y": 906,
                "zero_offset.x": 265,
                "zero_offset.y": 741,
            }
        }
        pgm = encode_pgm_gray(2, 1, [70, 80])
        grid = occupancy_from_sros_map(doc, pgm, image_name="map.pgm")
        origin_y = grid["origin"]["y"]
        max_y = origin_y + 906 * grid["resolution"]
        for y in (-0.249, 0.148, 0.239, 0.51, 0.655, 0.88):
            self.assertTrue(origin_y <= y <= max_y, y)

    def test_packed_tar_export(self) -> None:
        doc = {
            "meta": {
                "length_unit": "mm",
                "resolution": 50,
                "size.x": 2,
                "size.y": 2,
                "zero_offset.x": 0,
                "zero_offset.y": 0,
            },
            "data": {"station": []},
        }
        pgm = encode_pgm_gray(2, 2, [70, 70, 70, 80])
        buffer = io.BytesIO()
        with tarfile.open(fileobj=buffer, mode="w") as archive:
            json_bytes = json.dumps(doc).encode("utf-8")
            json_info = tarfile.TarInfo("sros/map/SMT_test.json")
            json_info.size = len(json_bytes)
            archive.addfile(json_info, io.BytesIO(json_bytes))
            pgm_info = tarfile.TarInfo("sros/map/SMT_test.pgm")
            pgm_info.size = len(pgm)
            archive.addfile(pgm_info, io.BytesIO(pgm))
        grid = occupancy_from_export_bytes(buffer.getvalue())
        self.assertEqual(grid["data"][1], 100)
        self.assertEqual(grid["width"], 2)

    def test_fms_gzip_skips_empty_pgm_and_uses_level0(self) -> None:
        import gzip

        doc = {
            "meta": {
                "length_unit": "mm",
                "resolution": 50,
                "size.x": 2,
                "size.y": 2,
                "zero_offset.x": 0,
                "zero_offset.y": 0,
            },
            "data": {"station": []},
        }
        full = encode_pgm_gray(2, 2, [70, 70, 70, 80])
        half = encode_pgm_gray(1, 1, [0])
        preview = b"\x89PNG\r\n\x1a\nnot-a-real-png"
        tar_buffer = io.BytesIO()
        with tarfile.open(fileobj=tar_buffer, mode="w") as archive:
            files = {
                "sros/map/demo.pgm": b"",
                "sros/map/demo0.pgm": full,
                "sros/map/demo1.pgm": half,
                "sros/map/demo.json": json.dumps(doc).encode("utf-8"),
                "sros/map/demo.png": preview,
            }
            for name, blob in files.items():
                info = tarfile.TarInfo(name)
                info.size = len(blob)
                archive.addfile(info, io.BytesIO(blob))
        packed = gzip.compress(tar_buffer.getvalue())
        grid = occupancy_from_export_bytes(packed)
        self.assertEqual(grid["width"], 2)
        self.assertEqual(grid["height"], 2)
        self.assertEqual(grid["data"][1], 100)

    def test_local_json_plus_pgm(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            doc = {
                "meta": {
                    "length_unit": "m",
                    "resolution": 0.05,
                    "size.x": 2,
                    "size.y": 1,
                    "zero_offset.x": 0,
                    "zero_offset.y": 0,
                }
            }
            (root / "demo.json").write_text(json.dumps(doc), encoding="utf-8")
            (root / "demo.pgm").write_bytes(encode_pgm_gray(2, 1, [0, 255]))
            grid = occupancy_from_local_path(root / "demo.json")
            self.assertEqual(grid["data"], [100, 0])


if __name__ == "__main__":
    unittest.main()
