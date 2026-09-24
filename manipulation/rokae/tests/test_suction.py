import copy
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from rokae_web.suction import set_output, validate_config, SuctionError, frame
from rokae_web.sdk_wire import SDK, encode, decode
from rokae_web.backends import MockRobotBackend, MockChassisBackend
from rokae_web.config import DEFAULT_CONFIG
from rokae_web.service import ControlService


# Protocol fixture only; site wiring is configured separately after confirmation.
CONFIG = dict(enabled=True, slave_id=1,
              open_outputs=[dict(address=0, value=True)],
              close_outputs=[dict(address=0, value=False)])


class SuctionProtocolTests(unittest.TestCase):
    def arm(self, fail=False, corrupt=False, wrong_state=False, padded=True):
        events=[];mask=0
        def raw(sent, received, data, output, ec):
            nonlocal mask
            self.assertEqual(output.content(),[])
            request=bytes(data);events.append(request.hex())
            if fail:
                ec.update(ec=263);return
            if data[1]==5:
                if data[4]:mask |= 1 << data[3]
                else:mask &= ~(1 << data[3])
                reply=request
            else:reply=frame([data[0],1,1,mask ^ (1 if wrong_state else 0)])
            if corrupt:reply=reply[:-1]+bytes([reply[-1]^1])
            output._value=([0]*received if padded else [])+list(reply)
        return SimpleNamespace(XPRS485SendData=raw),events

    def test_documented_frames_padded_and_unpadded_without_power_calls(self):
        for padded in (True,False):
            arm,events=self.arm(padded=padded)
            for enabled in (True,False):
                self.assertTrue(set_output(SDK,arm,CONFIG,enabled)['confirmed'])
            self.assertEqual(events,['01050000ff008c3a','0101000000083dcc',
                                     '010500000000cdca','0101000000083dcc'])

    def test_failure_and_bad_crc_never_retry_or_send_following_coil(self):
        for options in ({'fail':True},{'corrupt':True}):
            arm,events=self.arm(**options)
            cfg=copy.deepcopy(CONFIG);cfg['open_outputs'].append(dict(address=1,value=True))
            with self.assertRaises(SuctionError):set_output(SDK,arm,cfg,True)
            self.assertEqual(len(events),1)

    def test_mismatched_state_remains_unconfirmed(self):
        arm,events=self.arm(wrong_state=True)
        with self.assertRaisesRegex(SuctionError,'回读状态'):set_output(SDK,arm,CONFIG,True)
        self.assertEqual(len(events),2)

    def test_invalid_configuration_never_dispatches(self):
        for cfg in (None,{},dict(CONFIG,slave_id=0),dict(CONFIG,open_outputs=[dict(address=4,value=True)])):
            arm,events=self.arm()
            with self.assertRaises(SuctionError):set_output(SDK,arm,cfg,True)
            self.assertEqual(events,[])

    def test_reject_wrong_echo_and_nonzero_padding(self):
        for response in (frame([1,5,0,1,255,0]),bytes([1]*8)+frame([1,5,0,0,255,0])):
            def raw(n,m,data,out,ec):out._value=list(response)
            with self.assertRaises(SuctionError):set_output(SDK,SimpleNamespace(XPRS485SendData=raw),CONFIG,True)


class SuctionServiceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name)
        cfg=copy.deepcopy(DEFAULT_CONFIG)
        cfg['memory_points']={'file':str(root/'memory.json')}
        cfg['camera']['data_directory']=str(root/'data')
        cfg['suction']=copy.deepcopy(CONFIG)
        self.robot=MockRobotBackend()
        self.robot.suction_set=Mock(return_value=dict(commanded_open=True,confirmed=True))
        self.service=ControlService(cfg,self.robot,MockChassisBackend(),False)
    def tearDown(self):self.service.close();self.tmp.cleanup()

    def test_locked_or_active_sequence_rejects_command(self):
        with self.assertRaises(Exception):self.service.set_suction({'open':True})
        self.service.armed=True;self.service.placement.job['active']=True
        try:
            with self.assertRaises(Exception):self.service.set_suction({'open':True})
        finally:self.service.placement.job['active']=False
        self.robot.suction_set.assert_not_called()

    def test_not_coupled_to_right_gripper_and_no_auto_release_on_disarm(self):
        self.service.armed=True
        self.assertFalse(self.service.gripper_unlocked)
        result=self.service.set_suction({'open':True})
        self.assertTrue(result['confirmed'])
        self.service.disarm()
        self.robot.suction_set.assert_called_once_with(True,CONFIG)

    def test_invalid_request_and_unknown_result_on_failure(self):
        self.service.armed=True
        for payload in ({'open':1},{'open':'false'},{'open':True,'slave_id':2},{}):
            with self.assertRaises(Exception):self.service.set_suction(payload)
        self.robot.suction_set.assert_not_called()
        self.robot.suction_set.side_effect=RuntimeError('lost response')
        with self.assertRaises(RuntimeError):self.service.set_suction({'open':True})
        self.assertIsNone(self.service.suction_status()['commanded_open'])
        self.assertFalse(self.service.suction_status()['confirmed'])


if __name__=='__main__':unittest.main()
