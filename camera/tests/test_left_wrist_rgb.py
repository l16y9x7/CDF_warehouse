import importlib.util
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value

publisher = module('left_rgb', 'vision/rokae_runtime/left_wrist_rgb.py')
probe = module('probe', 'scripts/head-stream-ros-check.py')
health = module('health', 'scripts/head-stream-check.py')

class LeftRGBTests(unittest.TestCase):
    def test_driver_requests_color_only_720p15(self):
        rs = Mock()
        cfg = publisher.configure_pipeline(rs, '262622270751')
        cfg.disable_all_streams.assert_called_once_with()
        cfg.enable_device.assert_called_once_with('262622270751')
        cfg.enable_stream.assert_called_once_with(rs.stream.color,1280,720,rs.format.bgr8,15)

    def test_health_does_not_require_left_depth_or_sync(self):
        cfg=json.loads((ROOT/'config/vision.json').read_text())
        requirements={row['ros_id']:row for row in probe.camera_requirements(cfg)}
        self.assertTrue(requirements['left_wrist']['color_only'])
        self.assertFalse(requirements['left_wrist']['require_synced'])
        self.assertTrue(requirements['head']['require_synced'])
        self.assertFalse(requirements['head']['color_only'])

    def test_left_failure_does_not_restart_head(self):
        cfg=json.loads((ROOT/'config/vision.json').read_text())
        result={'ok':True,'errors':[]}
        repairs=set()
        runner=Mock(return_value=SimpleNamespace(stdout=json.dumps(dict(
            ok=False,raw_ok=False,synced_ok=True,cameras={
                'head':dict(raw_ok=True), 'left_wrist':dict(raw_ok=False,color_only=True)})),stderr=''))
        health.check_ros(cfg,ROOT/'config/vision.json',result,repairs,run=runner)
        self.assertEqual(repairs,{'left-wrist-rgb'})
        self.assertEqual(health.repair_service_names(cfg,repairs),['vision-left-wrist-rgb.service'])

if __name__ == '__main__':
    unittest.main()
