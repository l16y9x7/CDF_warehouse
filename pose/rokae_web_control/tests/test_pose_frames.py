from __future__ import annotations

import math
import unittest

from rokae_web.pose_frames import _rotation, ref_pose_to_world, world_pose_to_ref


class PoseFrameTests(unittest.TestCase):
    def assertPoseEquivalent(self, actual, expected) -> None:
        for a, b in zip(actual[:3], expected[:3]):
            self.assertAlmostEqual(a, b, places=10)
        for row_a, row_b in zip(_rotation(actual[3:]), _rotation(expected[3:])):
            for a, b in zip(row_a, row_b):
                self.assertAlmostEqual(a, b, places=10)

    def test_chest_offset_is_removed_for_both_arm_origins(self) -> None:
        for shoulder_y in (-0.0775, 0.0775):
            frame = [0.0, shoulder_y, -0.1429, 0.0, 0.0, 0.0]
            ref_pose = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
            world = ref_pose_to_world(ref_pose, frame)
            self.assertPoseEquivalent(world, [0.1, 0.2 + shoulder_y, 0.1571, 0.4, 0.5, 0.6])
            self.assertPoseEquivalent(world_pose_to_ref(world, frame), ref_pose)

    def test_rotation_applies_to_position_and_orientation(self) -> None:
        frame = [1, 2, 3, 0, 0, math.pi / 2]
        expected = [1, 3, 3, 0, 0, math.pi / 2]
        self.assertPoseEquivalent(ref_pose_to_world([1, 0, 0, 0, 0, 0], frame), expected)
        self.assertPoseEquivalent(world_pose_to_ref(expected, frame), [1, 0, 0, 0, 0, 0])

    def test_general_transform_round_trip(self) -> None:
        frame = [0.1, -0.2, 0.3, 0.4, -0.6, 1.3]
        for pose in ([0.2, 0.6, -0.1, -0.3, 0.2, -0.5], [0, 0, 0, 2.1, -1.2, -2.5]):
            self.assertPoseEquivalent(world_pose_to_ref(ref_pose_to_world(pose, frame), frame), pose)
            self.assertPoseEquivalent(ref_pose_to_world(world_pose_to_ref(pose, frame), frame), pose)

    def test_gimbal_lock_retains_equivalent_rotation(self) -> None:
        for pitch in (-math.pi / 2, math.pi / 2):
            frame = [0.1, 0.2, 0.3, 0.4, pitch, 0.8]
            world = ref_pose_to_world([0] * 6, frame)
            self.assertPoseEquivalent(world, frame)
            self.assertPoseEquivalent(world_pose_to_ref(world, frame), [0] * 6)

    def test_world_z_is_not_the_tilted_arm_base_z(self) -> None:
        for yaw in (-math.pi / 2, math.pi / 2):
            sdk_world_from_base = [0, 0, 0, 0, math.pi / 2, yaw]
            base_pose = world_pose_to_ref([0, 0, 0.1, 0, 0, 0], sdk_world_from_base)
            self.assertAlmostEqual(base_pose[0], -0.1)
            self.assertAlmostEqual(base_pose[1], 0.0)
            self.assertAlmostEqual(base_pose[2], 0.0)

    def test_invalid_sdk_frames_fail_closed(self) -> None:
        for invalid in ([0] * 5, [0] * 5 + [math.nan], [0] * 5 + [math.inf]):
            with self.assertRaises(ValueError):
                ref_pose_to_world([0] * 6, invalid)
            with self.assertRaises(ValueError):
                world_pose_to_ref(invalid, [0] * 6)


if __name__ == "__main__":
    unittest.main()
