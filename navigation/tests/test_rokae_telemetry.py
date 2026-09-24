import unittest

from navigation.adapters.rokae_telemetry import (
    chassis_from_state,
    osd_from_vendor,
)


class TestRokaeTelemetry(unittest.TestCase):
    def test_helios_camera_battery_ids(self) -> None:
        fragment = osd_from_vendor(
            {
                "hardware": {
                    "devices": [
                        {"id": 130, "model": "old-cam", "version": "x"},
                        {"id": 240, "model": "old-bat", "version": "x"},
                        {"id": 626, "model": "D435", "version": "1.0"},
                        {"id": 629, "model": "D435-back", "version": "1.0"},
                        {"id": 711, "model": "BAT-48V", "version": "1.0"},
                    ]
                }
            },
            None,
        )
        info = fragment["devices_info"]
        self.assertEqual(info["camera"]["model"], "D435")
        self.assertEqual(info["battery"]["model"], "BAT-48V")
        self.assertNotEqual(info["camera"]["model"], "old-cam")
        self.assertNotEqual(info["battery"]["model"], "old-bat")

    def test_battery_omits_zero_cycle_and_keeps_raw(self) -> None:
        fragment = osd_from_vendor(
            {"location_state": 3, "x": 1.0, "y": 2.0, "yaw": 0.1},
            {
                "capacity_percent": 72.0,
                "temperature": 33.0,
                "charging": False,
                "cycle": 0,
                "voltage": 54.1,
                "soc_raw": 72.0,
                "pack_voltage_raw": 54100,
                "current_raw": -1200,
                "temperature_raw": 33.0,
                "state_raw": 3,
            },
        )
        battery = fragment["battery"]
        self.assertEqual(battery["capacity_percent"], 72.0)
        self.assertEqual(battery["source"], "sros_amr")
        self.assertNotIn("cycle", battery)
        self.assertEqual(battery["pack_voltage_raw"], 54100)
        self.assertEqual(battery["current_raw"], -1200)

    def test_chassis_priority_emergency_error_paused(self) -> None:
        self.assertEqual(
            chassis_from_state({"estop_active": True, "sys_state": 3})["state"],
            "emergency",
        )
        self.assertEqual(chassis_from_state({"sys_state": 3})["state"], "error")
        self.assertEqual(chassis_from_state({"sys_state": 10})["state"], "paused")
        self.assertEqual(chassis_from_state({"sys_state": 1})["state"], "initializing")
        moving = chassis_from_state({"linear_velocity_x": 0.2, "linear_velocity_y": 0.0, "angular_velocity": 0.0})
        self.assertEqual(moving["state"], "moving")
        self.assertAlmostEqual(moving["motion"]["linear_speed_mps"], 0.2)
        self.assertFalse(moving["motion"]["stopped"])
        idle = chassis_from_state(
            {
                "linear_velocity_x": 0.0,
                "linear_velocity_y": 0.0,
                "angular_velocity": 0.0,
                "executing_movement_task": False,
            }
        )
        self.assertEqual(idle["state"], "idle")
        self.assertTrue(idle["motion"]["stopped"])
        still_task = chassis_from_state(
            {
                "linear_velocity_x": 0.0,
                "linear_velocity_y": 0.0,
                "angular_velocity": 0.0,
                "executing_movement_task": True,
            }
        )
        self.assertEqual(still_task["state"], "moving")
        self.assertTrue(still_task["motion"]["stopped"])

    def test_chassis_osd_blocks_from_sros_snapshot(self) -> None:
        state = {
            "measured_at_ms": 1_700_000_000_000,
            "location_state": 3,
            "sys_state": 2,
            "run_state": 1,
            "operation_state": 1,
            "scheduling_mode": 2,
            "fleet_mode": 1,
            "location_type": 1,
            "location_confidence": 90,
            "map_name": "test_zhongmian",
            "station_no": 2,
            "load_state": 1,
            "fresh_state": 2,
            "emergency_state": 1,
            "emergency_source": 0,
            "oba": 1,
            "estop_active": False,
            "x": 1.2,
            "y": -0.4,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.01,
            "yaw": 1.57,
            "linear_velocity_x": 0.0,
            "linear_velocity_y": 0.0,
            "angular_velocity": 0.0,
            "mc_state": 2,
            "path_no": 3,
            "speed_level": 1,
            "new_movement_task_state": 1,
            "faults": [{"code": 11330, "level": 2, "start_time_s": 1700000000}],
            "info": {
                "serial_no": "SN-1",
                "nickname": "helios",
                "vehicle_type": "Oasis-300E",
                "sros_version": "5.0",
            },
            "hardware": {
                "cpu_usage": 11,
                "memory_usage": 40,
                "hardware_state": 2,
                "brake_sw_state": 1,
                "src": {
                    "m1_status_code": 0,
                    "m2_status_code": 1,
                    "src_state": 0,
                    "total_mileage": 88,
                },
                "devices": [
                    {
                        "id": 101,
                        "name": "front_lidar",
                        "state": 1,
                        "model": "Livox",
                        "version": "1.2",
                    },
                    {
                        "id": 121,
                        "name": "IMU",
                        "state": 1,
                        "error_code": 0,
                        "model": "CH040-SR",
                        "version": "1.0",
                    },
                    {
                        "id": 211,
                        "name": "wheel1",
                        "state": 1,
                        "model": "WHEEL",
                        "version": "1",
                    },
                    {
                        "id": 210,
                        "name": "steer1",
                        "state": 128,
                        "error_code": 9,
                        "model": "STEER",
                        "version": "1",
                    },
                    {
                        "id": 231,
                        "name": "SRC",
                        "state": 1,
                        "model": "VC400-SR",
                        "version": "SRTOS",
                    },
                    {
                        "id": 626,
                        "name": "depth_cam_0",
                        "state": 1,
                        "model": "D435",
                        "version": "1.0",
                    },
                    {
                        "id": 711,
                        "name": "pack",
                        "state": 1,
                        "model": "BAT-48V",
                        "version": "1.0",
                    },
                ],
            },
            "collector": {
                "started": True,
                "runtime_sec": 12.0,
                "poll_interval_sec": 0.1,
                "chassis": {
                    "connected": True,
                    "fresh": True,
                    "sample_count": 8,
                    "error_count": 0,
                    "freshness_sec": 2.0,
                    "last_sample_age_ms": 40,
                },
            },
            "transport": {
                "protocol": "srp",
                "host": "192.168.71.50",
                "port": 5001,
                "connected": True,
            },
        }
        fragment = osd_from_vendor(state, {"capacity_percent": 67.0, "cycle": 12, "charging": False})
        self.assertEqual(fragment["robot_mode"]["location_state"], "running")
        self.assertTrue(fragment["robot_mode"]["map_loaded"])
        self.assertEqual(fragment["robot_mode"]["map_name"], "test_zhongmian")
        self.assertTrue(fragment["odometry"]["pose_confirmed"])
        self.assertEqual(fragment["odometry"]["position"]["x"], 1.2)
        self.assertEqual(fragment["locomotion"]["mc_state"], "idle")
        self.assertFalse(fragment["safety"]["certified"])
        self.assertFalse(fragment["safety"]["emergency_stop"])
        self.assertEqual(fragment["alarm_status"]["alarms"][0]["error_code"], 11330)
        self.assertEqual(fragment["devices_info"]["lidar"]["model"], "Livox")
        self.assertEqual(fragment["devices_info"]["imu"]["model"], "CH040-SR")
        self.assertEqual(fragment["devices_info"]["stm32"]["model"], "VC400-SR")
        self.assertEqual(fragment["devices_info"]["camera"]["model"], "D435")
        self.assertEqual(fragment["devices_info"]["battery"]["model"], "BAT-48V")
        self.assertEqual(fragment["motors"]["wheel_motor_count"], 1)
        self.assertEqual(fragment["motors"]["steer_motor_count"], 1)
        self.assertEqual(fragment["motors"]["faulted"][0]["id"], 210)
        self.assertEqual(fragment["mainboard"]["vehicle_type"], "Oasis-300E")
        self.assertEqual(fragment["imu_status"]["euler_angles"]["yaw"], 1.57)
        self.assertTrue(fragment["stm32_status"]["is_connected"])
        self.assertEqual(fragment["collector"]["name"], "rokae_composite_state")
        self.assertEqual(fragment["collector"]["source_count"], 1)
        self.assertTrue(fragment["collector"]["sources"]["chassis"]["fresh"])
        self.assertNotIn("left_arm", fragment["collector"]["sources"])
        self.assertEqual(fragment["transport"]["endpoint_count"], 1)
        self.assertEqual(fragment["transport"]["endpoints"]["chassis"]["host"], "192.168.71.50")
        self.assertNotIn("manipulator_status", fragment)
        self.assertNotIn("left_arm", fragment.get("transport", {}).get("endpoints", {}))


if __name__ == "__main__":
    unittest.main()
