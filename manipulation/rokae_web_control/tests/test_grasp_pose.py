from __future__ import annotations

import unittest

import numpy as np

from rokae_web.grasp_pose import world_grasp_from_4090, world_grasp_to_right_shoulder
from rokae_web.head_kinematics import rpy_rotation
from rokae_web.box_clearance import clearance_in_trunk


class GraspPoseTests(unittest.TestCase):
    def axis_fixture(self, height=800.0):
        trunk=np.eye(4)
        trunk[:3,:3]=rpy_rotation([.3,-.2,1.1])
        trunk[:3,3]=[.2,.1,.7]
        camera=np.eye(4)
        camera[:3,:3]=rpy_rotation([.1,.5,-.3])
        camera[:3,3]=[.17,-.1,.8]
        point=np.array([500.,-100.,720.])
        axis=np.array([.1,-.2,1.]); axis/=np.linalg.norm(axis)
        world=trunk[:3,:3]@point+1000*trunk[:3,3]
        camera_point=camera[:3,:3].T@(world-1000*camera[:3,3])
        request={"base_frame":"chassis_link","camera_frame":"camera","T_unit":"m",
                 "T_chassis_camera":camera.tolist(),"T_chassis_trunk_ref_m":trunk.tolist(),
                 "grasp_height_trunk_mm":height,"sku_typ":"bottle"}
        response={"ok":True,"axis_fit_valid":True,"reference_point_valid":True,
                  "output_frame":"camera","output_unit":"mm",
                  "axis_point_camera_mm":camera_point.tolist(),
                  "axis_direction_camera_up":(camera[:3,:3].T@trunk[:3,:3]@axis).tolist(),
                  "reference_point_camera_mm":camera_point.tolist(),
                  "reference_point_chassis_mm":world.tolist()}
        return request,response,trunk,camera

    def test_fixed_sdk_height_uses_reference_vertical_ignoring_tilted_axis(self):
        request,response,trunk,_=self.axis_fixture()
        result=world_grasp_from_4090(request,response)
        np.testing.assert_allclose(result["axis_intersection_trunk_mm"],[500,-100,800],atol=1e-9)
        np.testing.assert_allclose(result["grasp_point_trunk_mm"],[500,-90,800],atol=1e-9)
        expected=trunk[:3,:3]@np.array([500,-90,800])+1000*trunk[:3,3]
        np.testing.assert_allclose(result["grasp_pose_world_mm_deg"][:3],expected,atol=1e-9)
        self.assertNotAlmostEqual(expected[2],800)
        self.assertEqual(result["grasp_point_rule"],"fixed_trunk_height_vertical_reference_v2")
        self.assertEqual(result["grasp_height_sku_typ"],"bottle")
        response["axis_direction_camera_up"]=[-v for v in response["axis_direction_camera_up"]]
        np.testing.assert_allclose(world_grasp_from_4090(request,response)["grasp_point_trunk_mm"],[500,-90,800],atol=1e-9)
        # A tilted panel changes d at the corrected Y; use that ray for pregrasp.
        panel={"valid":True,"frame":"chassis_link",
               "point_mm":(trunk[:3,:3]@np.array([400,0,0])+1000*trunk[:3,3]).tolist(),
               "normal":(trunk[:3,:3]@np.array([1,.2,0])/np.sqrt(1.04)).tolist()}
        clearance=clearance_in_trunk(panel,result,trunk)
        self.assertAlmostEqual(clearance["d_mm"],82.0)
        shoulder=world_grasp_to_right_shoulder(result,trunk,clearance,trunk)
        np.testing.assert_allclose(shoulder["grasp_pose_right_shoulder_mm_deg"][:3],[325,-90,800],atol=1e-8)
        np.testing.assert_allclose(shoulder["pregrasp_pose_right_shoulder_mm_deg"][:3],[168,-90,800],atol=1e-8)
        np.testing.assert_allclose(rpy_rotation(np.radians(shoulder["grasp_pose_right_shoulder_mm_deg"][3:])),
                                   rpy_rotation(np.radians([180,-90,0])),atol=1e-8)

    def test_bottle_and_tube_use_175mm_with_unchanged_pregrasp(self):
        for kind in ('bottle', 'tube'):
            world = dict(frame='chassis_link', sku_typ=kind,
                         grasp_pose_world_mm_deg=[500,0,800,180,-90,0])
            result = world_grasp_to_right_shoulder(world, np.eye(4),
                dict(valid=True,frame='trunk_controller_ref',d_mm=100,pregrasp_virtual_length_mm=350),np.eye(4))
            self.assertEqual(result['gripper_length_mm'],175.)
            np.testing.assert_allclose(result['grasp_pose_right_shoulder_mm_deg'][:3],[325,0,800])
            np.testing.assert_allclose(result['pregrasp_pose_right_shoulder_mm_deg'][:3],[150,0,800])

    def test_radius_is_not_required_or_used_for_axis_plane_intersection(self):
        request,response,_,_=self.axis_fixture()
        expected=world_grasp_from_4090(request,response)
        for radius in (None, 0, 'unused', 1000):
            response['body_radius_mm']=radius
            self.assertEqual(world_grasp_from_4090(request,response),expected)
        request['body_radius_mm']=28.5
        response['body_radius_mm']=99
        self.assertEqual(world_grasp_from_4090(request,response),expected)

    def test_bad_height_rejected_but_reported_parallel_axis_is_ignored(self):
        for height in (None,True,float('nan'),float('inf'),"800"):
            request,response,_,_=self.axis_fixture(height)
            with self.subTest(height=height), self.assertRaisesRegex(ValueError,"高度"):
                world_grasp_from_4090(request,response)
        request,response,trunk,camera=self.axis_fixture()
        response["axis_direction_camera_up"]=(camera[:3,:3].T@trunk[:3,:3]@np.array([1,0,0])).tolist()
        np.testing.assert_allclose(world_grasp_from_4090(request,response)["grasp_point_trunk_mm"], [500,-90,800])

    def test_world_pose_uses_sdk_shoulder_origin_and_axes(self) -> None:
        shoulder = np.eye(4)
        shoulder[:3, 3] = [-0.1, -0.0775, 1.143]
        world = {
            "frame": "chassis_link",
            "grasp_pose_world_mm_deg": [577.0, -58.0, 853.0, 180.0, -90.0, 0.0],
        }
        result = world_grasp_to_right_shoulder(world, shoulder, {
            "valid": True, "frame": "trunk_controller_ref", "d_mm": 90, "pregrasp_virtual_length_mm": 320}, np.eye(4))
        self.assertEqual(result["frame"], "right_arm_sdk_world")
        self.assertEqual(result["gripper_length_mm"], 175.0)
        self.assertEqual(result["pregrasp_retraction_mm"], 145.0)
        self.assertTrue(np.allclose(
            result["object_grasp_pose_right_shoulder_mm_deg"][:3],
            [677.0, 19.5, -290.0], atol=1e-8,
        ))
        self.assertTrue(np.allclose(
            result["grasp_pose_right_shoulder_mm_deg"][:3],
            [502.0, 19.5, -290.0], atol=1e-8,
        ))
        self.assertTrue(np.allclose(
            result["pregrasp_pose_right_shoulder_mm_deg"][:3],
            [357.0, 19.5, -290.0], atol=1e-8,
        ))
        self.assertTrue(np.allclose(
            rpy_rotation(np.radians(result["grasp_pose_right_shoulder_mm_deg"][3:])),
            rpy_rotation(np.radians([180.0, -90.0, 0.0])), atol=1e-8,
        ))

    def test_rotated_shoulder_preserves_assigned_trunk_orientation(self) -> None:
        shoulder = np.eye(4)
        shoulder[:3, :3] = rpy_rotation([0.0, 0.0, np.pi / 2.0])
        world = {
            "frame": "chassis_link",
            "grasp_pose_world_mm_deg": [100.0, 0.0, 0.0, 180.0, -90.0, 0.0],
        }
        result = world_grasp_to_right_shoulder(world, shoulder, {
            "valid": True, "frame": "trunk_controller_ref", "d_mm": 90, "pregrasp_virtual_length_mm": 320}, np.eye(4))
        pose = result["grasp_pose_right_shoulder_mm_deg"]
        self.assertTrue(np.allclose(
            result["object_grasp_pose_right_shoulder_mm_deg"][:3],
            [0.0, -100.0, 0.0], atol=1e-8,
        ))
        self.assertTrue(np.allclose(pose[:3], [0.0, 75.0, 0.0], atol=1e-8))
        self.assertTrue(np.allclose(
            result["pregrasp_pose_right_shoulder_mm_deg"][:3],
            [0.0, 220.0, 0.0], atol=1e-8,
        ))
        self.assertTrue(np.allclose(
            result["pregrasp_pose_right_shoulder_mm_deg"][3:], pose[3:],
            atol=1e-8,
        ))
        self.assertTrue(np.allclose(
            shoulder[:3, :3] @ rpy_rotation(np.radians(pose[3:])),
            rpy_rotation(np.radians([180, -90, 0])),
            atol=1e-8,
        ))


if __name__ == "__main__":
    unittest.main()
