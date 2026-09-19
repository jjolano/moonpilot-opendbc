#!/usr/bin/env python3
import unittest
from opendbc.car.chrysler.values import ChryslerSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
import opendbc.safety.tests.lateral_engage_common as lateral_engage_common  # moonpilot: by module, never by name — see AGENTS.md


class TestChryslerCuswSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x1F6, 0], [0x2FA, 0], [0x5DC, 0]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (0x1F6, 0x5DC)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1F6, 0x5DC]}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 4
  MAX_TORQUE_LOOKUP = [0], [250]
  MAX_RT_DELTA = 150
  MAX_TORQUE_ERROR = 80

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_cusw")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chryslerCusw, 0)
    self.safety.init_tests()

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": 1 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_CONTROL", 0, values)

  def _speed_msg(self, speed):
    values = {"VEHICLE_SPEED": speed}
    return self.packer.make_can_msg_safety("BRAKE_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_HUMAN": gas}
    return self.packer.make_can_msg_safety("ACCEL_GAS", 0, values)

  def _user_brake_msg(self, brake):
    values = {"DRIVER_BRAKE_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("BRAKE_3", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("EPS_STATUS", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": steer_req}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_meas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")


# moonpilot: lateral engagement, CUSW. Host-armed: the mode never sets `acc_main_on` (ACC_CONTROL
# carries ACC active, which is not the main switch), so the arm is openpilot's own engaged heartbeat
# — see lateral_engage_common. `chrysler_cusw_init` reads nothing else from the param, so the flag
# rides alone.
class TestChryslerCuswLateralEngageBase(TestChryslerCuswSafety):
  def _set_lateral_engage_hooks(self, enabled):
    param = int(ChryslerSafetyFlags.LATERAL_ENGAGE) if enabled else 0
    self.safety.set_safety_hooks(CarParams.SafetyModel.chryslerCusw, param)

  def _set_lat_engage_hooks(self):
    """The brand base's own setup: install with the permission on, then let init run."""
    self._set_lateral_engage_hooks(True)
    self.safety.init_tests()

  def _steer_tx_msg(self):
    """A steering request that is only legal while steering is permitted. Seeded so the rate and
    measured-torque checks pass, leaving the permit as the only variable."""
    self.safety.set_torque_meas(4, 4)
    self._set_prev_torque(4)
    return self._torque_cmd_msg(4)

  def _accel_tx_msg(self):
    """ACC_CONTROL: the car's own gas request, which this mode never sends and the permission must
    not open"""
    return self.packer.make_can_msg_safety("ACC_CONTROL", 0, {"GAS_VALID": 1, "GAS_VALUE": 100})


class TestChryslerCuswLateralEngage(lateral_engage_common.LateralEngageSafetyTest, TestChryslerCuswLateralEngageBase):
  LATERAL_ENGAGE_ARM = "host"

  def setUp(self):
    super().setUp()
    self._set_lat_engage_hooks()


if __name__ == "__main__":
  unittest.main()
