import http.client
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

try:
    from vision.rokae_runtime.agent_contract import query, snapshot
    from vision.rokae_runtime.capture_api import resolve_camera
    from vision.rokae_runtime.ros_bridge import RosCameraOwner, RosImageWorker
    from vision.rokae_runtime.http_app import serve_owner
    VISION = True
except ImportError:
    VISION = False


@unittest.skipUnless(VISION,'isolated vision source required')
class CameraTests(unittest.TestCase):
    def test_canonical_right_and_rgb_only_wrists(self):
        self.assertEqual(resolve_camera('hand_wrist'),('right_wrist','hand_right'))
        for camera in ('left_wrist','right_wrist'):
            with self.assertRaisesRegex(ValueError,'DEPTH_UNSUPPORTED'):
                query({'camera':camera},rgbd=True)
        with self.assertRaisesRegex(ValueError,'PREVIEW'):
            query(dict(camera='head',type='depth',format='raw'),stream=True)
        with self.assertRaisesRegex(ValueError,'INVALID_FORMAT'):
            query(dict(camera='head',type='depth'))

    def test_snapshot_is_json_and_rgbd_has_two_same_capture_paths(self):
        result=dict(ok=True,capture_id='id',camera='head',same_shot=True,
                    color={'path':'/shared/frames/id/rgb.jpg','format':'jpeg','width':2,'height':2},
                    depth={'path':'/shared/frames/id/depth_mm.npy','format':'raw','width':2,'height':2})
        owner=SimpleNamespace(capture=lambda **kw:result)
        self.assertEqual(snapshot(owner,dict(camera='head',type='color'))['image_path'],result['color']['path'])
        pair=snapshot(owner,dict(camera='head'),rgbd=True)
        self.assertEqual(Path(pair['rgb']).parent,Path(pair['depth']).parent)

    def test_fresh_pair_rejects_old_frames_and_matches_source_stamps(self):
        owner=RosCameraOwner({})
        color,depth=RosImageWorker(),RosImageWorker()
        owner._workers={'head':{'worker':color,'stale':2}}
        owner._depth_workers={'head':{'aligned':depth,'stale':2}}
        rgb=np.zeros((2,2,3),np.uint8);z=np.ones((2,2),np.uint16)*800
        color.history.append((1,time.monotonic()-10,rgb))
        depth.history.append((1,time.monotonic()-10,z))
        def publish():
            time.sleep(.05)
            now=time.monotonic()
            with color._lock:color.history.append((12,now,rgb))
            with depth._lock:depth.history.append((12,now,z))
        thread=threading.Thread(target=publish);thread.start()
        with patch('vision.rokae_runtime.capture_api.write_capture_dir',return_value={'color':{},'depth':{}}):
            result=owner.capture(contract='head',internal='head',streams={'color','depth'},format='raw')
        thread.join()
        self.assertTrue(result['same_shot'])
        self.assertEqual(result['timestamps']['color_s'],12)
        self.assertEqual(result['timestamps']['depth_s'],12)

    def test_http_list_array_and_snapshot_routes(self):
        owner=SimpleNamespace(host='127.0.0.1',port=0,listing=lambda:{'cameras':[{'id':'head'}]},
                              capture=lambda **kw:dict(ok=True,capture_id='a',same_shot=True,
                                  color={'path':'/shared/frames/a/rgb.jpg','format':'jpeg','width':2,'height':2}))
        server=serve_owner(owner);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            for path in ('/camera/list','/camera/snapshot?camera=head&type=color'):
                c=http.client.HTTPConnection(*server.server_address);c.request('GET',path)
                response=c.getresponse();self.assertEqual(response.status,200)
                data=json.loads(response.read());c.close()
                if path.endswith('list'):self.assertIsInstance(data,list)
                else:self.assertIn('image_path',data)
        finally:
            server.shutdown();server.server_close();thread.join(2)
