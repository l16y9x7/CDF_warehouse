import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/head-stream-check.py'
SPEC = importlib.util.spec_from_file_location('head_stream_check', SCRIPT)
head_stream_check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(head_stream_check)

ROS_SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/head-stream-ros-check.py'
ROS_SPEC = importlib.util.spec_from_file_location('head_stream_ros_check', ROS_SCRIPT)
head_stream_ros_check = importlib.util.module_from_spec(ROS_SPEC)
ROS_SPEC.loader.exec_module(head_stream_ros_check)


def config():
    return {
        'camera': {'port': 8003},
        'media': {'port': 8005},
        'rokae': {'owner': {'port': 8085, 'source': 'ros'}},
    }


def owner(ready=True):
    return {'cameras': [
        {'id': 'head', 'enabled': True, 'ready': ready, 'error': 'NOT_READY' if not ready else ''},
        {'id': 'hand_left', 'enabled': False, 'ready': False},
        {'id': 'hand_right', 'enabled': False, 'ready': False},
    ]}


def dual_owner(right_ready=True):
    return {'cameras': [
        {'id': 'head', 'enabled': True, 'ready': True, 'error': ''},
        {'id': 'left_wrist', 'enabled': False, 'ready': False},
        {'id': 'hand_wrist', 'enabled': True, 'ready': right_ready,
         'error': '' if right_ready else 'WAITING_FOR_FRAME'},
    ]}


def dual_status(head_frames, right_frames, *, right_online=True):
    return {'streams': [
        {'camera_id': 'head', 'enabled': True, 'desired': True, 'online': True,
         'frames_sent': head_frames, 'progress_age_sec': 0.1},
        {'camera_id': 'hand_right', 'enabled': True, 'desired': True,
         'online': right_online, 'frames_sent': right_frames, 'progress_age_sec': 0.2},
        {'camera_id': 'hand_left', 'enabled': False, 'desired': False,
         'online': False, 'frames_sent': 0},
    ]}


class HealthCheckTests(unittest.TestCase):
    def test_ros_probe_targets_only_enabled_owner_cameras(self):
        cfg = config()
        cfg['rokae']['owner']['cameras'] = {
            'head': {'enabled': True},
            'hand_left': {'enabled': False},
            'hand_right': {'enabled': True},
        }
        self.assertEqual(
            head_stream_ros_check.enabled_ros_ids(cfg),
            ['head', 'right_wrist'],
        )

    def test_ros_probe_does_not_invent_roles_missing_from_owner_config(self):
        cfg = config()
        cfg['rokae']['owner']['cameras'] = {'head': {'enabled': True}}
        self.assertEqual(head_stream_ros_check.enabled_ros_ids(cfg), ['head'])
        self.assertEqual(
            [row['internal_id'] for row in head_stream_ros_check.camera_requirements(cfg)],
            ['head'],
        )

    def test_ros_probe_supports_per_camera_depth_and_synced_requirements(self):
        cfg = config()
        cfg['rokae']['owner']['cameras'] = {
            'head': {'enabled': True},
            'hand_left': {'enabled': False},
            'hand_right': {'enabled': True},
        }
        cfg['health'] = {'ros_requirements': {
            'head': {
                'depth_topic': '/camera/head/depth/image_rect_raw',
                'require_depth': True,
                'require_synced': False,
                'repair_unit': 'head-source',
            },
            # Accept the ROS id as an override key as well.
            'right_wrist': {
                'depth_topic': '/camera/right_wrist/aligned_depth_to_color/image_raw',
                'require_depth': True,
                'require_synced': True,
                'repair_unit': 'hand_right-source',
            },
        }}
        requirements = head_stream_ros_check.camera_requirements(cfg)
        self.assertEqual(requirements, [
            {
                'internal_id': 'head', 'ros_id': 'head',
                'depth_topic': '/camera/head/depth/image_rect_raw',
                'require_depth': True,
                'require_synced': False,
                'repair_unit': 'head-source',
            },
            {
                'internal_id': 'hand_right', 'ros_id': 'right_wrist',
                'depth_topic': '/camera/right_wrist/aligned_depth_to_color/image_raw',
                'require_depth': True,
                'require_synced': True,
                'repair_unit': 'hand_right-source',
            },
        ])

    def test_ros_probe_defaults_to_aligned_depth_and_synced(self):
        cfg = config()
        cfg['rokae']['owner']['cameras'] = {'head': {'enabled': True}}
        self.assertEqual(head_stream_ros_check.camera_requirements(cfg)[0], {
            'internal_id': 'head', 'ros_id': 'head',
            'depth_topic': '/camera/head/aligned_depth_to_color/image_raw',
            'require_depth': True,
            'require_synced': True,
            'repair_unit': 'head-source',
        })

    def test_color_only_wrist_requires_only_color_and_has_role_scoped_repair(self):
        cfg = config()
        cfg['rokae']['owner']['cameras'] = {
            'head': {'enabled': False},
            'hand_left': {
                'enabled': True, 'enable_depth': False, 'require_synced': False,
            },
            'hand_right': {'enabled': False},
        }
        requirement = head_stream_ros_check.camera_requirements(cfg)[0]
        self.assertFalse(requirement['require_depth'])
        self.assertFalse(requirement['require_synced'])
        self.assertEqual(requirement['repair_unit'], 'hand_left-source')

    def test_color_only_rejects_synced_requirement(self):
        cfg = config()
        cfg['rokae']['owner']['cameras'] = {
            'hand_left': {
                'enabled': True, 'enable_depth': False, 'require_synced': True,
            },
        }
        with self.assertRaisesRegex(ValueError, 'requires depth'):
            head_stream_ros_check.camera_requirements(cfg)

    def test_camera_level_require_depth_is_honored_without_health_override(self):
        cfg = config()
        cfg['rokae']['owner']['cameras'] = {
            'hand_left': {
                'enabled': True, 'enable_depth': True,
                'require_depth': False, 'require_synced': False,
            },
        }
        requirement = head_stream_ros_check.camera_requirements(cfg)[0]
        self.assertFalse(requirement['require_depth'])

    def test_health_rejects_required_but_disabled_depth(self):
        cfg = config()
        cfg['rokae']['owner']['cameras'] = {
            'hand_left': {
                'enabled': True, 'enable_depth': False,
                'require_depth': True, 'require_synced': False,
            },
        }
        with self.assertRaisesRegex(ValueError, 'required depth is disabled'):
            head_stream_ros_check.camera_requirements(cfg)

    def test_ros_probe_uses_sensor_data_qos_for_raw_and_synced_topics(self):
        sensor_qos = object()
        subscriptions = []

        class FakeNode:
            def __init__(self, _name):
                pass

            def create_subscription(self, message_type, topic, callback, qos):
                subscriptions.append((message_type, topic, qos))
                callback(object())
                return object()

            def destroy_node(self):
                pass

        rclpy = ModuleType('rclpy')
        rclpy.init = Mock()
        rclpy.shutdown = Mock()
        rclpy.spin_once = Mock()
        rclpy_node = ModuleType('rclpy.node')
        rclpy_node.Node = FakeNode
        rclpy_qos = ModuleType('rclpy.qos')
        rclpy_qos.qos_profile_sensor_data = sensor_qos
        sensor_msgs = ModuleType('sensor_msgs')
        sensor_msgs_msg = ModuleType('sensor_msgs.msg')
        sensor_msgs_msg.CameraInfo = type('CameraInfo', (), {})
        sensor_msgs_msg.Image = type('Image', (), {})

        cfg = config()
        cfg['rokae']['owner']['cameras'] = {
            'head': {'enabled': True},
            'hand_left': {'enabled': False},
            'hand_right': {'enabled': False},
        }
        modules = {
            'rclpy': rclpy,
            'rclpy.node': rclpy_node,
            'rclpy.qos': rclpy_qos,
            'sensor_msgs': sensor_msgs,
            'sensor_msgs.msg': sensor_msgs_msg,
        }
        with patch.dict(sys.modules, modules):
            result = head_stream_ros_check.probe(cfg, timeout_sec=0.1)

        self.assertTrue(result['ok'], result)
        self.assertEqual(len(subscriptions), 5)
        self.assertTrue(all(qos is sensor_qos for _, _, qos in subscriptions))
        self.assertEqual(
            {topic for _, topic, _ in subscriptions},
            {
                '/camera/head/color/image_raw',
                '/camera/head/aligned_depth_to_color/image_raw',
                '/camera/head/synced/color/image_raw',
                '/camera/head/synced/depth/image_raw',
                '/camera/head/synced/color/camera_info',
            },
        )

    def test_ros_probe_raw_only_contract_skips_synced_topics(self):
        sensor_qos = object()
        subscriptions = []

        class FakeNode:
            def __init__(self, _name):
                pass

            def create_subscription(self, message_type, topic, callback, qos):
                subscriptions.append((message_type, topic, qos))
                callback(object())
                return object()

            def destroy_node(self):
                pass

        rclpy = ModuleType('rclpy')
        rclpy.init = Mock()
        rclpy.shutdown = Mock()
        rclpy.spin_once = Mock()
        rclpy_node = ModuleType('rclpy.node')
        rclpy_node.Node = FakeNode
        rclpy_qos = ModuleType('rclpy.qos')
        rclpy_qos.qos_profile_sensor_data = sensor_qos
        sensor_msgs = ModuleType('sensor_msgs')
        sensor_msgs_msg = ModuleType('sensor_msgs.msg')
        sensor_msgs_msg.CameraInfo = type('CameraInfo', (), {})
        sensor_msgs_msg.Image = type('Image', (), {})

        cfg = config()
        cfg['rokae']['owner']['cameras'] = {
            'head': {'enabled': True},
            'hand_left': {'enabled': False},
            'hand_right': {'enabled': False},
        }
        cfg['health'] = {'ros_requirements': {
            'head': {
                'depth_topic': '/camera/head/depth/image_rect_raw',
                'require_synced': False,
            },
        }}
        modules = {
            'rclpy': rclpy,
            'rclpy.node': rclpy_node,
            'rclpy.qos': rclpy_qos,
            'sensor_msgs': sensor_msgs,
            'sensor_msgs.msg': sensor_msgs_msg,
        }
        with patch.dict(sys.modules, modules):
            result = head_stream_ros_check.probe(cfg, timeout_sec=0.1)

        self.assertTrue(result['ok'], result)
        self.assertTrue(result['synced_ok'], result)
        self.assertFalse(result['cameras']['head']['synced_required'])
        self.assertEqual(len(subscriptions), 2)
        self.assertTrue(all(qos is sensor_qos for _, _, qos in subscriptions))
        self.assertEqual(
            {topic for _, topic, _ in subscriptions},
            {
                '/camera/head/color/image_raw',
                '/camera/head/depth/image_rect_raw',
            },
        )

    def test_all_enabled_owner_and_media_streams_must_advance(self):
        statuses = iter([dual_status(10, 20), dual_status(12, 23)])
        def fetch(url, binary=False):
            del binary
            return dual_owner() if url.endswith('/camera/list') else next(statuses)
        with patch.object(head_stream_check, 'fetch', side_effect=fetch), \
             patch.object(head_stream_check.time, 'sleep'):
            result, repair = head_stream_check.check(config())
        self.assertTrue(result['ok'], result)
        self.assertEqual(repair, set())
        self.assertEqual(result['owner_cameras'], ['head', 'hand_wrist'])
        self.assertEqual(set(result['media_streams']), {'head', 'hand_right'})

    def test_one_enabled_media_stream_stalling_fails_whole_media_check(self):
        statuses = iter([dual_status(10, 20), dual_status(12, 20)])
        def fetch(url, binary=False):
            del binary
            return dual_owner() if url.endswith('/camera/list') else next(statuses)
        with patch.object(head_stream_check, 'fetch', side_effect=fetch), \
             patch.object(head_stream_check.time, 'sleep'):
            result, repair = head_stream_check.check(config())
        self.assertFalse(result['ok'])
        self.assertTrue(any('OUTPUT_NOT_ADVANCING:hand_right' in err for err in result['errors']))
        self.assertEqual(repair, {'media'})

    def test_missing_media_progress_counter_fails_closed(self):
        statuses = iter([
            {'streams': [{'camera_id': 'head', 'enabled': True,
                          'desired': True, 'online': True}]},
            {'streams': [{'camera_id': 'head', 'enabled': True,
                          'desired': True, 'online': True}]},
        ])
        def fetch(url, binary=False):
            del binary
            return owner() if url.endswith('/camera/list') else next(statuses)
        with patch.object(head_stream_check, 'fetch', side_effect=fetch), \
             patch.object(head_stream_check.time, 'sleep'):
            result, repair = head_stream_check.check(config())
        self.assertFalse(result['ok'])
        self.assertTrue(any('PUSH_PROGRESS_COUNTER_UNAVAILABLE:head' in err
                            for err in result['errors']))
        self.assertEqual(repair, {'media'})

    def test_one_enabled_owner_camera_not_ready_fails_owner_check(self):
        statuses = iter([dual_status(10, 20), dual_status(12, 23)])
        def fetch(url, binary=False):
            del binary
            return dual_owner(False) if url.endswith('/camera/list') else next(statuses)
        with patch.object(head_stream_check, 'fetch', side_effect=fetch), \
             patch.object(head_stream_check.time, 'sleep'):
            result, repair = head_stream_check.check(config())
        self.assertFalse(result['ok'])
        self.assertTrue(any('hand_wrist:WAITING_FOR_FRAME' in err for err in result['errors']))
        self.assertEqual(repair, set())

    def test_owner_and_media_progress_are_authoritative_without_mjpeg_or_legacy_adapter(self):
        calls = []
        statuses = iter([
            {'streams': [{'camera_id': 'head', 'desired': True, 'online': True, 'frames_sent': 10}]},
            {'streams': [{'camera_id': 'head', 'desired': True, 'online': True,
                          'frames_sent': 12, 'progress_age_sec': 0.04}]},
        ])
        def fetch(url, binary=False):
            del binary
            calls.append(url)
            if url.endswith('/camera/list'):
                return owner()
            if url.endswith('/push'):
                return next(statuses)
            raise AssertionError(f'unexpected periodic HTTP call: {url}')
        with patch.object(head_stream_check, 'fetch', side_effect=fetch), \
             patch.object(head_stream_check.time, 'sleep'):
            result, repair = head_stream_check.check(config())
        self.assertTrue(result['ok'])
        self.assertEqual(repair, set())
        self.assertTrue(result['checks']['owner'])
        self.assertTrue(result['checks']['push_progress'])
        self.assertFalse(any(':8003' in call or '/camera/stream' in call for call in calls))

    def test_owner_unreachable_requests_only_owner_repair(self):
        def fetch(url, binary=False):
            del binary
            if url.endswith('/camera/list'):
                raise OSError('connection refused')
            return {'streams': [{'camera_id': 'head', 'desired': True, 'online': True,
                                 'frames_sent': 1, 'progress_age_sec': 0.1}]}
        with patch.object(head_stream_check, 'fetch', side_effect=fetch), \
             patch.object(head_stream_check.time, 'sleep'):
            result, repair = head_stream_check.check(config())
        self.assertFalse(result['ok'])
        self.assertEqual(repair, {'owner'})

    def test_not_ready_owner_does_not_restart_camera_or_legacy_adapter(self):
        statuses = iter([
            {'streams': [{'camera_id': 'head', 'desired': True, 'online': True, 'frames_sent': 1}]},
            {'streams': [{'camera_id': 'head', 'desired': True, 'online': True, 'frames_sent': 2}]},
        ])
        def fetch(url, binary=False):
            del binary
            return owner(False) if url.endswith('/camera/list') else next(statuses)
        with patch.object(head_stream_check, 'fetch', side_effect=fetch), \
             patch.object(head_stream_check.time, 'sleep'):
            result, repair = head_stream_check.check(config())
        self.assertFalse(result['ok'])
        self.assertEqual(repair, set())

    def test_media_offline_repairs_only_media_when_owner_is_healthy(self):
        def fetch(url, binary=False):
            del binary
            if url.endswith('/camera/list'):
                return owner()
            return {'streams': [{'camera_id': 'head', 'desired': True, 'online': False,
                                 'reason': 'encoder exited', 'frames_sent': 3}]}
        with patch.object(head_stream_check, 'fetch', side_effect=fetch), \
             patch.object(head_stream_check.time, 'sleep'):
            result, repair = head_stream_check.check(config())
        self.assertFalse(result['ok'])
        self.assertEqual(repair, {'media'})

    def test_ros_unhealthy_has_component_specific_repair(self):
        result = {'ok': True, 'errors': []}
        repairs = set()
        probe = SimpleNamespace(stdout=json.dumps({'ok': False, 'raw_ok': False}), stderr='')
        runner = Mock(return_value=probe)
        with patch.object(head_stream_check.os, 'geteuid', return_value=1000):
            head_stream_check.check_ros(
                config(), Path('/tmp/vision/config/vision.json'), result, repairs,
                run=runner,
            )
        self.assertFalse(result['ok'])
        self.assertEqual(repairs, {'rgbd', 'synced-rgbd'})
        command = runner.call_args.args[0]
        self.assertEqual(command[:3], ['/bin/bash', '-c',
                                       'source "$1" && exec "$2" "$3" --config "$4"'])
        self.assertEqual(command[3], 'vision-probe')
        self.assertEqual(command[5], head_stream_check.sys.executable)
        self.assertTrue(command[6].endswith('/scripts/head-stream-ros-check.py'))
        self.assertEqual(command[7], '/tmp/vision/config/vision.json')

    def test_raw_only_ros_failure_does_not_restart_optional_synced_service(self):
        result = {'ok': True, 'errors': []}
        repairs = set()
        probe = SimpleNamespace(stdout=json.dumps({
            'ok': False, 'raw_ok': False, 'synced_ok': True,
        }), stderr='')
        runner = Mock(return_value=probe)
        with patch.object(head_stream_check.os, 'geteuid', return_value=1000):
            head_stream_check.check_ros(
                config(), Path('/tmp/vision/config/vision.json'), result, repairs,
                run=runner,
            )
        self.assertFalse(result['ok'])
        self.assertEqual(repairs, {'rgbd'})

    def test_left_wrist_failure_repairs_only_left_source(self):
        result = {'ok': True, 'errors': []}
        repairs = set()
        probe = SimpleNamespace(stdout=json.dumps({
            'ok': False, 'raw_ok': False, 'synced_ok': True,
            'cameras': {
                'head': {'raw_ok': True, 'synced_ok': True, 'repair_unit': 'head-source'},
                'left_wrist': {
                    'raw_ok': False, 'synced_ok': True,
                    'repair_unit': 'hand_left-source',
                },
            },
        }), stderr='')
        with patch.object(head_stream_check.os, 'geteuid', return_value=1000):
            head_stream_check.check_ros(
                config(), Path('/tmp/vision/config/vision.json'), result, repairs,
                run=Mock(return_value=probe),
            )
        self.assertEqual(repairs, {'hand_left-source'})
        cfg = config()
        cfg['health'] = {
            'service_prefix': 'vision-rokae-preview-',
            'repair_units': {'hand_left-source': 'left-color'},
        }
        self.assertEqual(
            head_stream_check.repair_service_names(cfg, repairs),
            ['vision-rokae-preview-left-color.service'],
        )

    def test_repair_waits_for_three_consecutive_failures_and_resets_on_success(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config' / 'vision.json'
            path.parent.mkdir()
            runner = Mock()
            failed = {'ok': False, 'errors': ['push:down']}
            for expected in (1, 2):
                count, repaired = head_stream_check.update_repair_state(
                    path, dict(failed), {'media'}, now=1000 + expected, run=runner)
                self.assertEqual((count, repaired), (expected, []))
            count, repaired = head_stream_check.update_repair_state(
                path, dict(failed), {'media'}, now=1003, run=runner)
            self.assertEqual((count, repaired), (3, ['media']))
            runner.assert_called_once_with(
                ['systemctl', 'restart', 'vision-head-media.service'], check=True, timeout=20)
            count, repaired = head_stream_check.update_repair_state(
                path, {'ok': True, 'errors': []}, set(), now=1004, run=runner)
            self.assertEqual((count, repaired), (0, []))

    def test_repair_service_prefix_and_unit_mapping_are_configurable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config' / 'vision.json'
            path.parent.mkdir()
            runner = Mock()
            cfg = {'health': {
                'service_prefix': 'vision-rokae-preview-',
                'repair_units': {'rgbd': ['head-rgbd', 'right-rgbd']},
            }}
            state = path.parent.parent / 'runtime/head-stream-health.json'
            state.parent.mkdir()
            state.write_text(json.dumps({'consecutive_failures': 2, 'last_repair': 0}))
            count, repaired = head_stream_check.update_repair_state(
                path, {'ok': False, 'errors': ['ros']}, {'rgbd'},
                config=cfg, now=1003, run=runner,
            )
            self.assertEqual((count, repaired), (3, ['rgbd']))
            self.assertEqual(
                [call.args[0][2] for call in runner.call_args_list],
                ['vision-rokae-preview-head-rgbd.service',
                 'vision-rokae-preview-right-rgbd.service'],
            )


if __name__ == '__main__':
    unittest.main()
