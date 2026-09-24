from __future__ import annotations

import unittest

from rokae_web.gripper import Robotiq2F85


class Vector:
    def __init__(self, values):
        self.values = list(values)

    def content(self):
        return self.values


class SDK:
    PyTypeVectorInt = Vector


class FakeArm:
    def __init__(self):
        self.requests = []
        self.status_words = [0xF900, 0x0000, 0x0300]

    def XPRWModbusRTUReg(self, slave, function, address, dtype, count, data, crc_reverse, ec):
        self.requests.append((slave, function, address, dtype, count, data.values[:], crc_reverse))
        if function == 0x03:
            data.values = self.status_words[:]


class GripperProtocolTests(unittest.TestCase):
    def test_decodes_status_bytes_from_three_16_bit_registers(self):
        arm = FakeArm()
        arm.status_words = [0xB900, 0x00FF, 0xBD0A]
        status = Robotiq2F85(SDK, arm).status()
        self.assertEqual(status["activation_state"], 3)
        self.assertEqual(status["object_state"], 2)
        self.assertEqual(status["requested_position"], 255)
        self.assertEqual(status["measured_position"], 189)
        self.assertEqual(status["motor_current_ma"], 100)

    def test_move_uses_verified_right_xpanel_modbus_words(self):
        arm = FakeArm()
        arm.status_words = [0xF900, 0x001E, 0x1E00]
        status = Robotiq2F85(SDK, arm).move(30)
        self.assertEqual(status["measured_position"], 30)
        self.assertEqual(arm.requests[0][:5], (9, 3, 0x07D0, "uint16", 3))
        self.assertIn((9, 0x10, 0x03E8, "uint16", 3, [0x0900, 30, 0x140A], False), arm.requests)


if __name__ == "__main__":
    unittest.main()
