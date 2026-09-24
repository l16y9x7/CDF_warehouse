"""No robot: contract, read-only collector, SDK priority and client lifecycle."""
import copy
import http.client
import json
import math
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from rokae_robot_state import ManipulatorStatePort, ManipulatorStateSnapshot, build_manipulator_state_service
from rokae_web.backends import XCoreRobotBackend
from rokae_web.manipulator_reader import ManipulatorReader, common_state
from rokae_web.manipulator_telemetry import ManipulatorTelemetry
from rokae_web.sdk_broker import BrokerCore
from rokae_web.sdk_wire import SDK
from rokae_web.telemetry import TelemetryBusy
from rokae_web.telemetry_http import TelemetryHTTPServer


class Controller:
    def __init__(self, count):self.count,self.calls = count,[]
    def _record(self, method):self.calls.append(method)
    def jointPos(self, ec):self._record('jointPos');return [math.pi/2]*self.count
    def cartPosture(self, coordinate, ec):
        assert coordinate.name == 'flangeInBase'
        self._record('cartPosture')
        return NS(trans=[.1,.2,.3],rpy=[0,math.pi/2,0],external=[.1,.2])
    def operationState(self, ec):self._record('operationState');return NS(name='idle')
    def powerState(self, ec):self._record('powerState');return NS(name='on')
    def operateMode(self, ec):self._record('operateMode');return NS(name='automatic')
    def jointVel(self, ec):self._record('jointVel');return [math.pi]*self.count
    def jointTorque(self, ec):self._record('jointTorque');return [2.5]*self.count
    def robotInfo(self, ec):self._record('robotInfo');return NS(type='fake',version='v1',id='id',joint_num=self.count)
    def getSoftLimit(self, holder, ec):self._record('getSoftLimit');holder._value=[[-math.pi,math.pi]]*self.count;return True
    def getMechUnit(self, unit, key, holder, ec):self._record('getMechUnit');holder._value=['h1','h2']
    def getExtAxisInfo(self, name, key, holder, ec):self._record('getExtAxisInfo');holder._value=-45 if key.endswith('lower') else 45
    def queryControllerLog(self, count, levels, ec):
        self._record('queryControllerLog');return [NS(id=9,timestamp='old',content='old warning',repair='check')]
    def XPRWModbusRTUReg(self, slave, function, address, kind, count, holder, internal, ec):
        assert (slave,function,address,kind,count,internal) == (9,3,0x07D0,'uint16',3,False)
        self._record('gripper-read');holder._value=[0xF900,0x0011,0x2233]


class StateTests(unittest.TestCase):
    def setUp(self):
        self.backend=XCoreRobotBackend({})
        self.backend._sdk=NS(CoordinateType=SDK.CoordinateType,PyTypeVectorInt=SDK.PyTypeVectorInt,
            PyTypeVectorArrayDouble2=SDK.PyTypeVectorArrayDouble2,PyTypeVectorString=SDK.PyTypeVectorString,
            PyTypeDouble=SDK.PyTypeDouble,BaseRobot=NS(sdkVersion=lambda:'fake-sdk'),
            LogInfoLevel=NS(warning='warning',error='error'))
        self.backend._robots={n:Controller(c) for n,c in [('left_arm',7),('right_arm',7),('trunk',4)]}
        self.reader=ManipulatorReader(self.backend,{})
        self.cache=ManipulatorTelemetry(self.reader,{})

    def poll(self):
        for name in ('left_arm','right_arm','body'):self.cache.poll_once(name)

    def test_full_mapping_units_head_same_sample_and_no_fake_head_torque(self):
        self.poll();data=self.cache.get_state().to_mapping()
        self.assertTrue(data['dual_arm_available'])
        self.assertEqual(set(data['health']),{'left_arm','right_arm','body'})
        self.assertEqual(data['right_arm']['joint_positions_deg'],[90]*7)
        self.assertEqual(data['body']['end_pose'],[100,200,300,0,90,0])
        self.assertEqual(data['body']['measured_at_ms'],data['head']['measured_at_ms'])
        self.assertEqual(data['head']['joint_limits_deg'],[[-45,45],[-45,45]])
        self.assertNotIn('soft_limits_enabled',data['head'])
        self.assertNotIn('joint_torques_nm',data['head'])
        self.assertNotIn('joint_velocities_deg_s',data['head'])
        self.assertNotIn('end_pose',data['head'])
        self.assertEqual(data['right_arm']['gripper']['current_raw'],0x33)
        self.assertNotIn('gripper',data['left_arm'])

    def test_optional_queries_throttled_and_connection_metadata_cached(self):
        self.poll();self.poll()
        for robot in self.backend._robots.values():
            self.assertEqual(robot.calls.count('queryControllerLog'),1)
            self.assertEqual(robot.calls.count('robotInfo'),1)
        self.assertEqual(self.backend._robots['right_arm'].calls.count('gripper-read'),1)

    def test_stale_arm_omitted_and_legacy_requires_both(self):
        self.poll();self.cache._records['right_arm']['monotonic']-=.7
        value=self.cache.get_state();data=value.to_mapping()
        self.assertNotIn('right_arm',data);self.assertIn('left_arm',data)
        self.assertFalse(data['dual_arm_available']);self.assertEqual(value.to_legacy_dual_arm_mapping(),{})
        self.assertTrue(self.cache.snapshot()['modules']['right_arm']['valid']) # old 3-second contract

    def test_stale_body_omits_head_but_keeps_dual_arm_available(self):
        self.poll();self.cache._records['body']['monotonic']-=1
        data=self.cache.get_state().to_mapping()
        self.assertNotIn('body',data);self.assertNotIn('head',data)
        self.assertTrue(data['dual_arm_available'])

    def test_no_head_if_external_axes_missing(self):
        robot=self.backend._robots['trunk'];original=robot.cartPosture
        def cart(*args):
            result=original(*args);result.external=[];return result
        robot.cartPosture=cart
        self.cache.poll_once('body')
        self.assertNotIn('head',self.cache.get_state().to_mapping())

    def test_error_counter_recovery_and_bounded_summary(self):
        self.poll();read=self.reader.sample
        self.reader.sample=lambda _:(_ for _ in ()).throw(RuntimeError('x'*1000))
        self.cache.poll_once('right_arm');d=self.cache.get_state().to_mapping()
        self.assertNotIn('right_arm',d)
        self.assertEqual(d['health']['right_arm']['error_count'],1)
        self.assertLessEqual(len(d['health']['right_arm']['last_error_code']),160)
        self.reader.sample=read;self.cache.poll_once('right_arm')
        health=self.cache.get_state().to_mapping()['health']['right_arm']
        self.assertTrue(health['fresh']);self.assertEqual(health['sample_count'],2)
        self.assertEqual(health['error_count'],1);self.assertIsNone(health['last_error_code'])

    def test_busy_does_not_wait_or_count_as_error(self):
        self.reader.motion_pending=lambda:True
        self.poll();d=self.cache.get_state().to_mapping()
        self.assertEqual(d['health']['left_arm']['sample_count'],0)
        self.assertEqual(d['health']['left_arm']['error_count'],0)
        self.assertEqual(self.backend._robots['left_arm'].calls,[])

    def test_backend_lock_is_not_held_between_getters(self):
        observed=[];original=self.reader._read
        def read(*args):
            value=original(*args)
            def acquire():
                got=self.backend._lock.acquire(blocking=False);observed.append(got)
                if got:self.backend._lock.release()
            t=threading.Thread(target=acquire);t.start();t.join();return value
        self.reader._read=read;self.reader.sample('right_arm')
        self.assertTrue(observed);self.assertTrue(all(observed))

    def test_control_waiter_has_priority_and_counter_recovers_on_error(self):
        core=BrokerCore(self.backend,lambda *a,**k:None)
        self.reader.motion_pending=core.motion_pending
        with self.backend._lock:
            t=threading.Thread(target=lambda:core.call({'module':'right_arm','method':'jointPos','args':[]}))
            t.start()
            deadline=time.monotonic()+1
            while not core.motion_pending() and time.monotonic()<deadline:time.sleep(.001)
            self.assertTrue(core.motion_pending())
            with self.assertRaises(TelemetryBusy):self.reader.sample('left_arm')
        t.join(1);self.assertFalse(t.is_alive());self.assertFalse(core.motion_pending())
        with self.assertRaises(Exception):core.call({'module':'invalid'})
        self.assertFalse(core.motion_pending())

    def test_optional_missing_values_are_omitted_not_zeroed(self):
        self.backend._robots['right_arm'].jointVel=lambda ec:(_ for _ in ()).throw(RuntimeError())
        self.backend._robots['right_arm'].jointTorque=lambda ec:[float('nan')]*7
        self.cache.poll_once('right_arm');d=self.cache.get_state().to_mapping()['right_arm']
        self.assertNotIn('joint_velocities_deg_s',d);self.assertNotIn('joint_torques_nm',d)

    def test_slow_gripper_cache_not_kept_forever_during_motion(self):
        self.reader.sample('right_arm')
        self.reader.last[('right_arm','gripper')]-=3
        with patch.object(self.reader,'_gripper',side_effect=TelemetryBusy):
            self.assertNotIn('gripper',self.reader.sample('right_arm')['right_arm'])

    def test_immutable_snapshot_and_legacy_three_fields(self):
        self.poll();snapshot=self.cache.get_state();d=snapshot.to_mapping();d['right_arm']['state']='changed'
        self.assertEqual(snapshot.to_mapping()['right_arm']['state'],'idle')
        legacy=snapshot.to_legacy_dual_arm_mapping()
        self.assertEqual(set(legacy),{'left_arm','right_arm'})
        self.assertEqual(set(legacy['left_arm']),{'state','joint_positions_deg','end_pose'})

    def test_state_mapping_does_not_treat_old_controller_log_as_active_error(self):
        self.poll();self.assertEqual(self.cache.get_state().to_mapping()['right_arm']['state'],'idle')
        for op,power,expected in [('idle','off','disabled'),('moving','on','moving'),('drag','on','moving'),
                                  ('idle','estop','error'),('idle','gstop','error'),('unknown','on','error')]:
            self.assertEqual(common_state(op,power),expected)

    def test_collector_close_never_disconnects_sdk(self):
        self.backend.close=lambda:self.fail('disconnect')
        self.cache.start();time.sleep(.15);self.cache.close()
        self.assertEqual(len(self.cache._threads),3)
        self.assertTrue(all(not t.is_alive() for t in self.cache._threads))

    def test_http_and_inprocess_getter_are_cache_only_and_client_stop_is_independent(self):
        self.poll()
        self.reader.sample=lambda *_:self.fail('HTTP must not invoke SDK')
        server=TelemetryHTTPServer(('127.0.0.1',0),self.cache)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        port=ManipulatorStatePort(f'http://127.0.0.1:{server.server_port}/api/telemetry/manipulator-state')
        try:
            port._poll()
            with patch('urllib.request.urlopen',side_effect=AssertionError('getter I/O')):
                self.assertTrue(port.get_state().to_mapping()['dual_arm_available'])
            port._received-=1
            self.assertFalse(port.get_state().to_mapping()['dual_arm_available'])
            self.assertEqual(port.get_state().to_legacy_dual_arm_mapping(),{})
            port.stop()
            c=http.client.HTTPConnection('127.0.0.1',server.server_port)
            c.request('GET','/api/telemetry/manipulator-state');r=c.getresponse()
            self.assertEqual(r.status,200);self.assertEqual(json.loads(r.read())['data']['schema_version'],1)
            c.close()
            c=http.client.HTTPConnection('127.0.0.1',server.server_port)
            c.request('POST','/api/telemetry/manipulator-state',body='{}');r=c.getresponse()
            self.assertEqual(r.status,405);r.read();c.close()
        finally:server.shutdown();server.server_close();thread.join()

    def test_transport_failure_omits_components(self):
        port=build_manipulator_state_service({})
        with patch('urllib.request.urlopen',side_effect=OSError('closed')):port._poll()
        data=port.get_state().to_mapping()
        self.assertFalse(data['dual_arm_available']);self.assertNotIn('right_arm',data)
        self.assertEqual(set(data['health']),{'left_arm','right_arm','body'})

    def test_transport_failure_preserves_known_sampler_counters_and_age(self):
        self.poll();port=build_manipulator_state_service({})
        port._mapping=self.cache.get_state().to_mapping();port._received=time.monotonic()-1
        with patch('urllib.request.urlopen',side_effect=OSError('closed')):port._poll()
        health=port.get_state().to_mapping()['health']['right_arm']
        self.assertEqual(health['sample_count'],1)
        self.assertGreaterEqual(health['last_sample_age_ms'],1000)
        self.assertFalse(health['fresh'])

    def test_invalid_collection_config_rejected(self):
        for config in ({'sample_hz':11},{'sample_hz':0},{'freshness_ms':100},
                       {'sample_hz':float('nan')},{'log_query_interval_sec':1},{'gripper_query_interval_sec':.1}):
            with self.assertRaises(ValueError):ManipulatorTelemetry(self.reader,config)


if __name__ == '__main__':unittest.main()
