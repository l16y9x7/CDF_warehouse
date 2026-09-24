"""Cross-process boundary tests using fake controllers, never real hardware."""
import copy
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

from rokae_web.backends import BackendError, XCoreRobotBackend
from rokae_web.sdk_broker import BrokerCore, BrokerServer, BrokerRobotBackend
from rokae_web.sdk_wire import SDK, FIELDS, HOLDERS, encode, decode
from rokae_web.telemetry import UpperBodyTelemetry


class FakeController:
    def __init__(self, count):
        self.count, self.calls = count, []
        self.q = [0.1]*count + [0.0]*2
        self.cart = SDK.CartesianPosition([.1,.2,.3,.4,.5,.6])
        self.cart.external = [0.0]*2
        self.cart.confData = [1,2,3,4,5,6,7,8]
        self.cart.elbow = .4
        self.tool = SDK.Toolset()
        self.tool.end.trans = [.01, .02, .03]
        self.tool.load.mass = 1.5

    def jointPos(self, ec): return list(self.q)
    def cartPosture(self, coordinate, ec):
        self.calls.append(('cartPosture', coordinate.name))
        return self.cart
    def posture(self, coordinate, ec): return self.cart.trans + self.cart.rpy
    def toolset(self, ec): return self.tool
    def operationState(self, ec): return SDK.OperationState.idle
    def getSoftLimit(self, out, ec): out._value = [[-180,180]]*self.count
    def getMechUnit(self, unit, key, out, ec):
        out._value = ['h1','h2'] if key == 'axes_info' else True
    def getExtAxisInfo(self, *args):
        args[-2]._value = -180 if 'lower' in str(args) else 180
    def checkPath(self, *args):
        self.calls.append(('checkPath', encode(args)))
        args[-1]['ec'] = 0
        args[-2].confData = [8,7,6,5,4,3,2,1] if hasattr(args[-2], 'confData') else []
        return [.2]*self.count
    def model(self): return self
    def calcIk(self, goal, tool, ec):
        self.calls.append(('calcIk', encode(goal), encode(tool)))
        ec['ec'] = -32
        return [.3]*self.count
    def moveAppend(self, commands, identifier, ec):
        self.calls.append(('moveAppend', encode(commands)))
        identifier._value = 'test-command-123'
    def moveStart(self, ec): self.calls.append(('moveStart',))
    def stop(self, ec): self.calls.append(('stop',))
    def moveReset(self, ec): self.calls.append(('moveReset',))
    def disableDrag(self, ec): self.calls.append(('disableDrag',))
    def XPRWModbusRTUReg(self, slave, function, address, kind, count, out, internal, ec):
        self.calls.append(('modbus', function, out.content()))
        if function == 3: out._value = [0x3100, 0, 123]
    def XPRS485SendData(self, sent, received, data, out, ec):
        from rokae_web.suction import frame
        self.calls.append(('raw',bytes(data).hex()))
        if data[1]==5:
            self.relay_mask=(1 << data[3]) if data[4] else 0
            reply=bytes(data)
        else:reply=frame([data[0],1,1,getattr(self,'relay_mask',0)])
        out._value=[0]*received+list(reply)
    def disconnectFromRobot(self, ec):
        raise AssertionError('Web lifecycle must never disconnect SDK')


@unittest.skipUnless(os.name == 'posix', 'Linux Unix socket integration')
class BrokerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'sdk.sock'
        self.native = XCoreRobotBackend({})
        self.native._sdk = SDK
        self.native._robots = {m: FakeController(n) for m,n in [('left_arm',7),('right_arm',7),('trunk',4)]}
        self.audit = []
        self.core = BrokerCore(self.native, lambda event, **data: self.audit.append((event,data)))
        self.server = BrokerServer(self.path, self.core)
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.client = BrokerRobotBackend({}, self.path)

    def test_suction_commands_cross_broker_only_to_left_arm(self):
        from test_suction import CONFIG
        result=self.client.suction_set(True,CONFIG)
        self.assertTrue(result['confirmed'])
        self.assertEqual(self.native._robots['left_arm'].calls,
                         [('raw','01050000ff008c3a'),('raw','0101000000083dcc')])
        self.assertEqual(self.native._robots['right_arm'].calls,[])
        self.client.close()
        self.assertEqual(len(self.native._robots['left_arm'].calls),2)

    def test_suction_broker_rejects_other_arm_and_power_changes(self):
        from rokae_web.suction import frame
        for module,payload in [('right_arm',[1,5,0,0,255,0]),('left_arm',[1,0xb0,0,0,5,0]),
                               ('left_arm',[1,5,0,4,255,0]),('left_arm',[0,5,0,0,255,0])]:
            data=list(frame(payload))
            with self.assertRaises(BackendError):
                self.client._robot(module).XPRS485SendData(8,8,data,SDK.PyTypeVectorInt([]),{})
        for method in ('setxPanelVout','setxPanelRS485','XPRWModbusRTUCoil'):
            with self.assertRaises(AttributeError):getattr(self.client._robot('left_arm'),method)
        self.assertEqual(self.native._robots['left_arm'].calls,[])
        self.assertEqual(self.native._robots['right_arm'].calls,[])

    def tearDown(self):
        self.client.close()
        self.server.shutdown(); self.server.server_close(); self.thread.join()
        self.temp.cleanup()

    def test_readback_preserves_tool_cartesian_external_and_enum_values(self):
        state = self.client.read_state()
        self.assertEqual(len(state['joints_deg']['trunk']), 4)
        self.assertEqual(len(state['joints_deg']['head']), 2)
        self.assertEqual(state['operation_state']['right_arm'], 'idle')
        self.assertEqual(state['toolsets']['trunk']['end'][:3], [.01,.02,.03])
        robot = self.client._robot('right_arm')
        tool = robot.toolset({})
        self.assertEqual(tool.load.mass, 1.5)
        cart = robot.cartPosture(SDK.CoordinateType.endInRef, {})
        self.assertEqual(cart.external, [0,0])
        self.assertEqual(cart.confData, [1,2,3,4,5,6,7,8])
        self.assertEqual(cart.elbow, .4)

    def queue_arms(self):
        for module in ('left_arm', 'right_arm'):
            self.client._robot(module).moveAppend([SDK.MoveAbsJCommand(SDK.JointPosition([.1]*7), 20, 0)], SDK.PyString(), {})

    def test_synchronized_start_enters_both_native_calls_before_either_returns(self):
        entered = {m: threading.Event() for m in ('left_arm', 'right_arm')}
        def begin(name, ec):
            entered[name].set()
            other = 'right_arm' if name == 'left_arm' else 'left_arm'
            if not entered[other].wait(1):
                raise RuntimeError('other arm was not dispatched concurrently')
        for name in entered:
            self.native._robots[name].moveStart = lambda ec, name=name: begin(name, ec)
        with self.assertRaises(BackendError):
            self.client.start_arms_synchronized(list(entered))
        self.queue_arms()
        result = self.client.start_arms_synchronized(list(entered))
        self.assertEqual(set(result['modules']), set(entered))
        self.assertTrue(all(event.is_set() for event in entered.values()))
        with self.assertRaises(BackendError):
            self.client.start_arms_synchronized(['trunk'])

    def test_right_arm_trunk_start_uses_barrier_and_requires_both_queues(self):
        self.assert_arm_trunk_start('right_arm')

    def test_left_arm_trunk_start_uses_barrier_and_requires_both_queues(self):
        self.assert_arm_trunk_start('left_arm')

    def assert_arm_trunk_start(self, arm):
        entered = {m: threading.Event() for m in (arm, 'trunk')}
        def begin(name, ec):
            entered[name].set()
            other = 'trunk' if name == arm else arm
            if not entered[other].wait(1):
                raise RuntimeError('controllers were started serially')
        for name in entered:
            self.native._robots[name].moveStart = lambda ec, name=name: begin(name, ec)
        self.client._robot(arm).moveAppend([SDK.MoveLCommand(SDK.CartesianPosition([0]*6),50,0)],SDK.PyString(),{})
        with self.assertRaises(BackendError):self.client.start_arms_synchronized(list(entered))
        self.assertFalse(any(e.is_set() for e in entered.values()))
        self.client._robot('trunk').moveAppend([SDK.MoveLCommand(SDK.CartesianPosition([0]*6),50,0)],SDK.PyString(),{})
        result=self.client.start_arms_synchronized(list(entered))
        self.assertEqual(set(result['modules']),set(entered))
        self.assertTrue(all(e.is_set() for e in entered.values()))
        other_arm = 'right_arm' if arm == 'left_arm' else 'left_arm'
        self.assertFalse(self.native._robots[other_arm].calls)
        self.assertFalse(self.core.queued)

    def test_paired_trunk_failure_stops_both_controllers(self):
        self.assert_arm_trunk_failure('right_arm')

    def test_left_arm_trunk_failure_stops_both_controllers(self):
        self.assert_arm_trunk_failure('left_arm')

    def assert_arm_trunk_failure(self, arm):
        for m in (arm,'trunk'):
            self.client._robot(m).moveAppend([SDK.MoveLCommand(SDK.CartesianPosition([0]*6),50,0)],SDK.PyString(),{})
        self.native._robots['trunk'].moveStart=lambda ec:ec.update(ec=123,message='start failed')
        with self.assertRaises(BackendError):self.client.start_arms_synchronized([arm,'trunk'])
        for m in (arm,'trunk'):
            self.assertIn(('stop',),self.native._robots[m].calls)
            self.assertIn(('moveReset',),self.native._robots[m].calls)
        other_arm = 'right_arm' if arm == 'left_arm' else 'left_arm'
        self.assertFalse(self.native._robots[other_arm].calls)
        self.assertFalse(self.core.queued)

    def test_synchronized_failed_start_stops_and_resets_both_arms(self):
        self.queue_arms()
        def fail(ec):
            ec.update(ec=10001, message='failed reply')
        self.native._robots['left_arm'].moveStart = fail
        with self.assertRaisesRegex(BackendError, '同步启动失败'):
            self.client.start_arms_synchronized(['left_arm', 'right_arm'])
        for name in ('left_arm', 'right_arm'):
            self.assertIn(('stop',), self.native._robots[name].calls)
            self.assertIn(('moveReset',), self.native._robots[name].calls)
        self.assertFalse(self.core.queued)

    def test_movel_and_joint_command_speed_angles_and_output_ec_survive(self):
        robot = self.client._robot('right_arm')
        cart = SDK.CartesianPosition([.1,.2,.3,.4,.5,.6])
        cart.hasElbow, cart.elbow, cart.confData, cart.external = True, .8, [1]*8, [.4,.5]
        command = SDK.MoveLCommand(cart, 87, 0)
        command.rotSpeed = .104719755
        identifier, ec = SDK.PyString(), {}
        robot.moveAppend([command], identifier, ec)
        self.assertEqual(identifier.content(), 'test-command-123')
        sent = self.native._robots['right_arm'].calls[-1][1][0]['fields']
        self.assertEqual(sent['speed'], 87)
        self.assertEqual(sent['rotSpeed'], .104719755)
        self.assertEqual(sent['target']['fields']['external'], [.4,.5])
        self.assertEqual(sent['target']['fields']['elbow'], .8)
        for cls in (SDK.MoveJCommand, SDK.MoveAbsJCommand):
            target = SDK.JointPosition([.2]*7) if cls is SDK.MoveAbsJCommand else cart
            target.external = [.1,.2]
            cmd = cls(target, 35, 0); cmd.jointSpeed = .17
            robot.moveAppend([cmd], SDK.PyString(), {})
            actual = self.native._robots['right_arm'].calls[-1][1][0]['fields']
            self.assertEqual(actual['jointSpeed'], .17)
            self.assertEqual(actual['target']['fields']['external'], [.1,.2])
        result = robot.model().calcIk(cart, robot.toolset({}), ec)
        self.assertEqual(ec['ec'], -32)
        self.assertEqual(result, [.3]*7)

    def test_soft_limit_and_gripper_out_parameters_are_updated(self):
        robot = self.client._robot('right_arm')
        out = SDK.PyTypeVectorArrayDouble2()
        robot.getSoftLimit(out,{})
        self.assertEqual(out.content(), [[-180,180]]*7)
        data = SDK.PyTypeVectorInt([0,0,0])
        robot.XPRWModbusRTUReg(9,3,2000,'uint16',3,data,False,{})
        self.assertEqual(data.content(), [0x3100,0,123])
        self.assertFalse(self.core.gripper_dirty)

    def test_checkpath_returns_endpoint_and_mutable_configuration(self):
        robot = self.client._robot('right_arm')
        start, goal = SDK.CartesianPosition([0]*6), SDK.CartesianPosition([.1]*6)
        goal.elbow, goal.hasElbow = .3, True
        ec = {}
        endpoint = robot.checkPath(start, [.2]*7, goal, ec)
        self.assertEqual(endpoint, [.2]*7)
        self.assertEqual(goal.confData, [8,7,6,5,4,3,2,1])
        self.assertEqual(ec, {'ec':0})
        self.assertEqual(len(self.native._robots['right_arm'].calls),1)
        self.assertFalse(self.core.dirty)

    def test_web_restart_keeps_telemetry_running_and_same_sdk_objects(self):
        telemetry = UpperBodyTelemetry(self.native, {'poll_interval_seconds':.1,'stale_after_seconds':1}, hardware=True)
        telemetry.start()
        objects = {k:id(v) for k,v in self.native._robots.items()}
        try:
            self.client.read_state()
            before = telemetry.snapshot()['modules']['right_arm']['sequence']
            robot = self.client._robot('right_arm')
            robot.moveAppend([SDK.MoveLCommand(SDK.CartesianPosition([0]*6),50,0)], SDK.PyString(), {})
            robot.moveStart({})
            self.client.close()  # The hardware/HTTP threads stay alive.
            self.assertIn(('stop',),self.native._robots['right_arm'].calls)
            self.assertIsNone(self.core.session)
            self.client = BrokerRobotBackend({},self.path)
            self.client.read_state()
            deadline = time.monotonic()+1
            while telemetry.snapshot()['modules']['right_arm']['sequence'] <= before and time.monotonic()<deadline:
                time.sleep(.02)
            self.assertGreater(telemetry.snapshot()['modules']['right_arm']['sequence'],before)
            self.assertEqual(objects,{k:id(v) for k,v in self.native._robots.items()})
            self.assertTrue(telemetry.snapshot()['all_valid'])
        finally:
            telemetry.close()

    def test_only_one_command_client_and_no_disconnect_method(self):
        self.client.read_state()
        other = BrokerRobotBackend({},self.path)
        try:
            with self.assertRaisesRegex(BackendError,'已有网页'):
                other.read_state()
        finally:
            other.close()
        with self.assertRaises(AttributeError):
            self.client._robot('right_arm').disconnectFromRobot({})


class WireTests(unittest.TestCase):
    def test_all_supported_value_types_round_trip(self):
        values=[SDK.Frame(),SDK.CartesianPosition([.1]*6),SDK.JointPosition([.2]*7),SDK.Load(),SDK.Toolset()]
        values += [SDK.MoveLCommand(values[1],42,0), SDK.MoveJCommand(values[1],33,0), SDK.MoveAbsJCommand(values[2],12,0)]
        values += [getattr(SDK,n)() for n in HOLDERS]
        values += [SDK.CoordinateType.flangeInBase, SDK.MotionControlMode.NrtCommandMode]
        for value in values:
            with self.subTest(type=type(value).__name__):
                self.assertEqual(encode(decode(encode(value))),encode(value))

    def test_unsupported_native_object_is_rejected(self):
        with self.assertRaises(TypeError): encode(object())
        with self.assertRaises(ValueError): decode({'$type':'ArRobot','fields':{}})


if __name__=='__main__': unittest.main()
