"""Frame regression with deliberately nonzero chest/ref/tool rotations."""
import copy
from types import SimpleNamespace
import unittest

import numpy as np

from rokae_web.arm_movel import transform, pose_values
from rokae_web.box_clearance import clearance_in_trunk
from rokae_web.grasp_pose import world_grasp_to_right_shoulder
from rokae_web.grasp_test import flange_targets
from rokae_web.trunk_frame import trunk_reference_in_chassis, stable_trunk_reference, TRUNK_FRAME


def metres(pose):
    T = transform(pose)
    T[:3, 3] /= 1000
    return T


class TrunkFrameTests(unittest.TestCase):
    def test_live_bridge_removes_flange_pose_and_tcp_offset(self):
        world_ref = metres([100, -200, 300, 12, -8, 23])
        ref_flange = metres([80, 0, 700, 0, 15, 57])
        flange_tcp = metres([40, -30, 120, 10, -20, 30])
        model = SimpleNamespace(forward_deg=lambda *a, **kw: world_ref @ ref_flange)
        tcp = ref_flange @ flange_tcp
        tcp_mm = tcp.copy(); tcp_mm[:3, 3] *= 1000
        s = {"pose_frames": {"trunk": TRUNK_FRAME}, "joints_deg": {"trunk": [1, 2, 3, 4]},
             "poses": {"trunk": pose_values(tcp_mm)},
             "toolsets": {"trunk": {"end": [.04, -.03, .12, *np.radians([10, -20, 30])], "ref": [0]*6}}}
        np.testing.assert_allclose(trunk_reference_in_chassis(s, model), world_ref, atol=1e-10)
        altered = copy.deepcopy(s); altered["toolsets"]["trunk"]["ref"][0] = .01
        with self.assertRaisesRegex(ValueError, "参考系改变"):
            stable_trunk_reference(s, altered, model)
        del altered["toolsets"]
        with self.assertRaisesRegex(ValueError, "缺少"):
            trunk_reference_in_chassis(altered, model)

    def test_entire_grasp_uses_trunk_axes_when_chest_is_rotated(self):
        world_ref = metres([100, -200, 300, 8, -12, 23])
        world_shoulder = world_ref @ metres([80, -100, 700, 0, 17, 61])
        obj_ref = np.array([600, -120, 900])
        obj_world = world_ref[:3, :3] @ obj_ref + 1000*world_ref[:3, 3]
        panel = {"valid": True, "frame": "chassis_link",
            "point_mm": (world_ref[:3, :3] @ [400, 0, 0] + 1000*world_ref[:3, 3]).tolist(),
            "normal": (world_ref[:3, :3] @ [1, 0, 0]).tolist()}
        world = {"frame": "chassis_link", "grasp_pose_world_mm_deg": [*obj_world, 0, 0, 0]}
        box = clearance_in_trunk(panel, world, world_ref)
        self.assertAlmostEqual(box["D_mm"], 400)
        self.assertAlmostEqual(box["d_mm"], 200)
        result = world_grasp_to_right_shoulder(world, world_shoulder, box, world_ref)
        targets = dict((key, pose) for key, _, pose in flange_targets(result))
        golden = {"pregrasp": [150, -120, 900], "grasp": [425, -120, 900],
                  "lift": [425, -120, 940], "arm_retreat": [205, -120, 940],
                  "clearance_lift": [205, -120, 1015]}
        ref_shoulder = np.linalg.inv(world_ref) @ world_shoulder
        for key, expected in golden.items():
            pose = ref_shoulder @ metres(targets[key])
            np.testing.assert_allclose(pose[:3, 3]*1000, expected, atol=1e-8)
            np.testing.assert_allclose(pose[:3, :3], metres([0, 0, 0, 180, -90, 0])[:3, :3], atol=1e-8)
        self.assertEqual(result["assigned_orientation_frame"], TRUNK_FRAME)

    def test_chest_frame_or_missing_transform_cannot_silently_fall_back(self):
        world = {"frame": "chassis_link", "grasp_pose_world_mm_deg": [600, 0, 900, 0, 0, 0]}
        with self.assertRaises(ValueError):
            world_grasp_to_right_shoulder(world, np.eye(4),
                {"valid": True, "frame": "Chest_link", "pregrasp_virtual_length_mm": 330}, np.eye(4))


if __name__ == "__main__":
    unittest.main()
