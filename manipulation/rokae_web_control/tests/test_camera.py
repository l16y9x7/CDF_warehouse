from __future__ import annotations

import json
import re
import tempfile
import unittest
from pathlib import Path

import numpy as np

from rokae_web.camera import CameraSnapshot, RecordingStore
from rokae_web.camera_manager import CameraManager, MultiCameraRecordingStore


class FakeSnapshotBackend:
    def __init__(self, snapshot: CameraSnapshot) -> None:
        self._snapshot = snapshot
        self.snapshot_count = 0

    def status(self):
        return {"available": True, "enabled": True, "starting": False}

    def snapshot(self):
        self.snapshot_count += 1
        return self._snapshot

    def close(self):
        return


class FakeSwitchableBackend(FakeSnapshotBackend):
    def __init__(self, snapshot: CameraSnapshot) -> None:
        super().__init__(snapshot)
        self.enabled = False
        self.stop_count = 0

    def status(self):
        return {"available": True, "enabled": self.enabled, "starting": False}

    def start(self):
        self.enabled = True
        return self.status()

    def stop(self):
        self.enabled = False
        self.stop_count += 1
        return self.status()


class RecordingStoreTests(unittest.TestCase):
    @staticmethod
    def snapshot(model: str, depth_mm: float) -> CameraSnapshot:
        return CameraSnapshot(
            rgb_bgr=np.zeros((720, 1280, 3), dtype=np.uint8),
            depth_aligned_mm=np.full((720, 1280), depth_mm, dtype=np.float32),
            captured_at="2026-08-28T21:00:00.000+08:00",
            color_timestamp_ms=100.0,
            depth_timestamp_ms=101.0,
            sequence=7,
            camera_info={"model": model, "alignment": "depth_to_color"},
        )

    def test_record_uses_numeric_date_and_millisecond_directories(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "data"
            snapshot = CameraSnapshot(
                rgb_bgr=np.zeros((720, 1280, 3), dtype=np.uint8),
                depth_aligned_mm=np.full((720, 1280), 1234.5, dtype=np.float32),
                captured_at="2026-08-28T21:00:00.000+08:00",
                color_timestamp_ms=100.0,
                depth_timestamp_ms=101.0,
                sequence=7,
                camera_info={"model": "test", "alignment": "depth_to_color"},
            )
            state = {
                "joints_deg": {"left_arm": [0.0] * 7, "right_arm": [0.0] * 7},
                "poses": {"left_arm": [0.0] * 6, "right_arm": [0.0] * 6},
                "arm_elbow_deg": {"left_arm": 0.0, "right_arm": 0.0},
            }

            result = RecordingStore(root).save(snapshot, state)

            target = Path(result["directory"])
            self.assertRegex(target.parent.name, r"^\d{8}$")
            self.assertRegex(target.name, r"^\d{9}$")
            self.assertEqual(
                sorted(path.name for path in target.iterdir()),
                ["camera_metadata.json", "depth_aligned.npy", "rgb.jpg", "robot_state.json"],
            )
            depth = np.load(target / "depth_aligned.npy", allow_pickle=False)
            self.assertEqual(depth.shape, (720, 1280))
            self.assertEqual(depth.dtype, np.dtype("float32"))
            self.assertAlmostEqual(float(depth[0, 0]), 1234.5)
            self.assertEqual(
                json.loads((target / "robot_state.json").read_text(encoding="utf-8")),
                state,
            )
            self.assertTrue(re.fullmatch(r"data/\d{8}/\d{9}", result["relative_directory"]))

    def test_multi_camera_record_saves_only_enabled_snapshots_with_namespaced_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "data"
            snapshots = {
                camera_id: CameraSnapshot(
                    rgb_bgr=np.zeros((720, 1280, 3), dtype=np.uint8),
                    depth_aligned_mm=np.full(
                        (720, 1280),
                        1000.0 if camera_id == "head" else 2000.0,
                        dtype=np.float32,
                    ),
                    captured_at="2026-08-28T21:00:00.000+08:00",
                    color_timestamp_ms=100.0,
                    depth_timestamp_ms=101.0,
                    sequence=7,
                    camera_info={"model": camera_id, "alignment": "depth_to_color"},
                )
                for camera_id in ("head", "left_wrist", "right_wrist")
            }
            state = {
                "joints_deg": {"left_arm": [0.0] * 7, "right_arm": [0.0] * 7},
                "poses": {"left_arm": [0.0] * 6, "right_arm": [0.0] * 6},
                "arm_elbow_deg": {"left_arm": 0.0, "right_arm": 0.0},
            }

            result = MultiCameraRecordingStore(root).save(snapshots, state)

            target = Path(result["directory"])
            self.assertEqual(result["active_cameras"], ["head", "left_wrist", "right_wrist"])
            self.assertEqual(
                sorted(path.name for path in target.iterdir()),
                [
                    "head_camera_metadata.json",
                    "head_depth_aligned.npy",
                    "head_rgb.jpg",
                    "left_wrist_camera_metadata.json",
                    "left_wrist_depth_aligned.npy",
                    "left_wrist_rgb.jpg",
                    "record_metadata.json",
                    "right_wrist_camera_metadata.json",
                    "right_wrist_depth_aligned.npy",
                    "right_wrist_rgb.jpg",
                    "robot_state.json",
                ],
            )
            for camera_id, expected_depth in (
                ("head", 1000.0),
                ("left_wrist", 2000.0),
                ("right_wrist", 2000.0),
            ):
                depth = np.load(target / f"{camera_id}_depth_aligned.npy", allow_pickle=False)
                self.assertEqual(depth.shape, (720, 1280))
                self.assertEqual(depth.dtype, np.dtype("float32"))
                self.assertAlmostEqual(float(depth[0, 0]), expected_depth)
            metadata = json.loads((target / "record_metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(
                metadata["active_cameras"],
                ["head", "left_wrist", "right_wrist"],
            )

    def test_camera_manager_records_only_the_requested_camera(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            head = FakeSnapshotBackend(self.snapshot("head", 1000.0))
            left_wrist = FakeSnapshotBackend(self.snapshot("left_wrist", 1500.0))
            right_wrist = FakeSnapshotBackend(self.snapshot("right_wrist", 2000.0))
            manager = CameraManager(
                {
                    "head": head,
                    "left_wrist": left_wrist,
                    "right_wrist": right_wrist,
                },
                Path(temporary) / "data",
            )

            result = manager.record("right_wrist", {"joints_deg": {}, "poses": {}})

            self.assertEqual(result["active_cameras"], ["right_wrist"])
            self.assertEqual(head.snapshot_count, 0)
            self.assertEqual(left_wrist.snapshot_count, 0)
            self.assertEqual(right_wrist.snapshot_count, 1)
            files = set(result["files"])
            self.assertIn("right_wrist_rgb.jpg", files)
            self.assertNotIn("head_rgb.jpg", files)

    def test_enabling_one_wrist_camera_stops_the_other(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            head = FakeSwitchableBackend(self.snapshot("head", 1000.0))
            left_wrist = FakeSwitchableBackend(self.snapshot("left_wrist", 1500.0))
            right_wrist = FakeSwitchableBackend(self.snapshot("right_wrist", 2000.0))
            manager = CameraManager(
                {
                    "head": head,
                    "left_wrist": left_wrist,
                    "right_wrist": right_wrist,
                },
                Path(temporary) / "data",
            )

            manager.set_enabled("right_wrist", True)
            manager.set_enabled("left_wrist", True)

            self.assertTrue(left_wrist.enabled)
            self.assertFalse(right_wrist.enabled)
            self.assertEqual(right_wrist.stop_count, 1)


if __name__ == "__main__":
    unittest.main()
