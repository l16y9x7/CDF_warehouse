"""Gripper-first grasp flow, using only mocked controllers."""
import copy
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rokae_web.backends import BackendError
from rokae_web.grasp_gripper import ensure_gripper_open
import test_agent_interfaces as agent_fixtures
import test_grasp_test as web_fixtures


def status(position=200, requested=255, object_state=3, **extra):
    return dict(activation_state=3, fault_code=0, going_to_position=True,
                requested_position=requested, measured_position=position,
                object_state=object_state, **extra)


class OpenTests(unittest.TestCase):
    def setUp(self):
        self.cancel=threading.Event()
        self.robot=SimpleNamespace(gripper_status=Mock(return_value=status()),
            gripper_start_move=Mock(), gripper_stop=Mock(return_value={'going_to_position':False}))
        self.service=SimpleNamespace(robot=self.robot, armed=True, gripper_unlocked=True,
            _lock=threading.RLock(), audit_event=Mock(), agent_actions=SimpleNamespace(stop_unconfirmed=False))

    def check(self):
        if self.cancel.is_set(): raise BackendError('cancelled')

    def run_open(self):
        return ensure_gripper_open(self.service,self.cancel,self.check)

    def test_closed_waits_for_measured_open_endpoint(self):
        self.robot.gripper_status.side_effect=[status(),status(90,0,0),status(3,0)]
        self.assertEqual(self.run_open()['measured_position'],3)
        self.robot.gripper_start_move.assert_called_once_with(0)
        self.robot.gripper_stop.assert_not_called()

    def test_already_open_does_not_issue_another_command(self):
        self.robot.gripper_status.return_value=status(3,0)
        self.run_open()
        self.robot.gripper_start_move.assert_not_called()

    def test_blocked_and_false_arrival_do_not_pass(self):
        for reported in [status(80,0,1),status(80,0,2),status(80,0,3),
                         status(None,0,3),status(float('nan'),0,3)]:
            with self.subTest(reported=reported):
                self.robot.gripper_status.side_effect=[status(),reported]
                with self.assertRaises(BackendError):self.run_open()
        self.assertEqual(self.robot.gripper_stop.call_count,5)

    def test_timeout_fault_and_write_failure_stop_before_planning(self):
        self.robot.gripper_status.side_effect=[status(),status(90,0,0)]
        with patch('rokae_web.grasp_gripper.OPEN_TIMEOUT_SECONDS',0):
            with self.assertRaisesRegex(BackendError,'超时'):self.run_open()
        bad=status(90,0,0);bad['fault_code']=14
        self.robot.gripper_status.side_effect=[status(),bad]
        with self.assertRaisesRegex(BackendError,'未就绪'):self.run_open()
        self.robot.gripper_status.side_effect=[status()]
        self.robot.gripper_start_move.side_effect=BackendError('write timeout')
        with self.assertRaisesRegex(BackendError,'write timeout'):self.run_open()
        self.assertEqual(self.robot.gripper_stop.call_count,3)

    def test_cancel_before_dispatch_or_during_wait(self):
        self.cancel.set()
        with self.assertRaisesRegex(BackendError,'cancelled'):self.run_open()
        self.robot.gripper_start_move.assert_not_called()
        self.cancel.clear()
        self.robot.gripper_start_move.side_effect=lambda _:self.cancel.set()
        with self.assertRaisesRegex(BackendError,'cancelled'):self.run_open()
        self.robot.gripper_stop.assert_called_once()

    def test_unconfirmed_stop_latches_control(self):
        self.robot.gripper_status.side_effect=[status(),status(90,0,1)]
        self.robot.gripper_stop.return_value={'going_to_position':True}
        with self.assertRaises(BackendError) as error:self.run_open()
        self.assertTrue(error.exception.stop_unconfirmed)
        self.assertFalse(self.service.armed)
        self.assertTrue(self.service.agent_actions.stop_unconfirmed)


class GraspIntegrationTests(unittest.TestCase):
    def test_agent_opens_before_freeze_and_full_preflight_and_replay_is_read_only(self):
        f=agent_fixtures.ActionsTests();f.setUp();self.addCleanup(f.doCleanups)
        s,a=f.service,f.actions
        s.robot.gripper_move(200)
        events=[]
        move=s.robot.gripper_start_move
        def open_gripper(position):events.append('open');move(position)
        s.robot.gripper_start_move=Mock(side_effect=open_gripper)
        def freeze(*_):
            self.assertEqual(s.robot.gripper_status()['measured_position'],0)
            events.append('freeze');return {}
        a.geometry.freeze=freeze;a.point=Mock(return_value={'state':{}})
        a.geometry.project=Mock(return_value={'source_result_id':'source','box_clearance':{}})
        s.grasp_test.execute=Mock();a.wait_job=Mock(return_value={'completed_moves':6})
        def plan(*_):events.append('plan');return SimpleNamespace(start={}),{}, {'preplanned_advance_mm':0}
        payload=dict(task_type='SORTING',target_type='sku',sku_typ='bottle',hand='RIGHT',level='L2',localization_result={})
        with patch('rokae_web.agent_actions.preflight_pick',side_effect=plan), \
             patch('rokae_web.agent_actions.execute_trunk',return_value={}):
            first=a.run('/manipulation/pick',payload,'open-once')
            self.assertEqual(first[0],200,first)
            self.assertEqual(a.run('/manipulation/pick',payload,'open-once'),first)
        self.assertEqual(events,['open','freeze','plan'])
        s.robot.gripper_start_move.assert_called_once_with(0)

    def test_agent_blocked_open_prevents_geometry_and_trunk(self):
        f=agent_fixtures.ActionsTests();f.setUp();self.addCleanup(f.doCleanups)
        s,a=f.service,f.actions
        s.robot.gripper_status=Mock(side_effect=[status(),status(),status(80,0,1)])
        s.robot.gripper_start_move=Mock();s.robot.gripper_stop=Mock(return_value={'going_to_position':False})
        a.geometry.freeze=Mock()
        payload=dict(task_type='SORTING',target_type='sku',sku_typ='bottle',hand='RIGHT',level='L2',localization_result={})
        with patch('rokae_web.agent_actions.preflight_pick') as plan, \
             patch('rokae_web.agent_actions.execute_trunk') as trunk:
            result=a.run('/manipulation/pick',payload,'blocked-open')
        self.assertGreaterEqual(result[0],400,result)
        a.geometry.freeze.assert_not_called();plan.assert_not_called();trunk.assert_not_called()

    def test_web_opens_before_first_path_and_existing_close_remains(self):
        f=web_fixtures.SequenceTests();f.setUp();self.addCleanup(f.doCleanups)
        f.gripper.update(requested_position=255,measured_position=200)
        result=f.execute()
        self.assertEqual(result['phase'],'completed',result)
        self.assertLess(f.events.index(('gripper','open')),f.events.index(('right_arm','checkPath')))
        self.assertEqual(f.events.count(('gripper','open')),1)
        self.assertEqual(f.events.count(('gripper','close')),1)

    def test_web_cancel_during_open_has_zero_path_calls(self):
        f=web_fixtures.SequenceTests();f.setUp();self.addCleanup(f.doCleanups)
        f.gripper.update(requested_position=255,measured_position=200)
        def opening(position):
            f.gripper.update(requested_position=0,object_state=0)
            f.service.grasp_test.cancel.set()
        f.backend.gripper_start_move=opening
        result=f.execute()
        self.assertEqual(result['phase'],'cancelled',result)
        self.assertNotIn(('right_arm','checkPath'),f.events)
        self.assertIn(('gripper','stop'),f.events)

    def test_web_stop_failure_latches_job(self):
        f=web_fixtures.SequenceTests();f.setUp();self.addCleanup(f.doCleanups)
        f.gripper.update(requested_position=255,measured_position=200)
        def blocked(position):f.gripper.update(requested_position=0,object_state=1)
        f.backend.gripper_start_move=blocked
        f.backend.gripper_stop=Mock(side_effect=BackendError('stop timeout'))
        result=f.execute()
        self.assertTrue(result['stop_unconfirmed'],result)
        self.assertNotIn(('right_arm','checkPath'),f.events)
        with self.assertRaises(BackendError):f.service.grasp_test.ensure_idle()
