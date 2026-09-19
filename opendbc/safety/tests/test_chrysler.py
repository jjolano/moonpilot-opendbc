#!/usr/bin/env python3
import unittest

from opendbc.car.chrysler.values import ChryslerSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
import opendbc.safety.tests.lateral_engage_common as lateral_engage_common  # moonpilot: by module, never by name — see AGENTS.md


class TestChryslerSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x292, 0x2A6)}
  FWD_BLACKLISTED_ADDRS = {2: [0x292, 0x2A6]}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [261]
  MAX_RT_DELTA = 112
  MAX_TORQUE_ERROR = 80

  LKAS_ACTIVE_VALUE = 1

  DAS_BUS = 0

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_pacifica_2017_hybrid_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, 0)
    self.safety.init_tests()

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.DAS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable}
    return self.packer.make_can_msg_safety("DAS_3", self.DAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"SPEED_LEFT": speed, "SPEED_RIGHT": speed}
    return self.packer.make_can_msg_safety("SPEED_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_safety("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ESP_1", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"EPS_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": self.LKAS_ACTIVE_VALUE if steer_req else 0}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

      # only one button at a time
      self.assertFalse(self._tx(self._button_msg(cancel=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=False, resume=False)))


class TestChryslerRamDTSafety(TestChryslerSafety):
  TX_MSGS = [[0xB1, 2], [0xA6, 0], [0xFA, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xA6, 0xFA)}
  FWD_BLACKLISTED_ADDRS = {2: [0xA6, 0xFA]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [350]

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_ram_dt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, ChryslerSafetyFlags.RAM_DT)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_safety("ESP_8", 0, values)


class TestChryslerRamHDSafety(TestChryslerSafety):
  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23A, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x276, 0x275)}
  FWD_BLACKLISTED_ADDRS = {2: [0x275, 0x276]}

  MAX_TORQUE_LOOKUP = [0], [361]
  MAX_RATE_UP = 14
  MAX_RATE_DOWN = 14
  MAX_RT_DELTA = 182

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_ram_hd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, ChryslerSafetyFlags.RAM_HD)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_safety("ESP_8", 0, values)


# moonpilot: lateral engagement, Pacifica/Jeep — the default platform, where the car's own ACC holds
# speed. `chrysler_init` reads the enable once for all three platforms, so one class covers the
# mode. Host-armed: the mode never sets `acc_main_on` (DAS_3 carries cruise engaged, which is not
# the main switch), so the arm is openpilot's own engaged heartbeat — see lateral_engage_common.
class TestChryslerLateralEngageBase(TestChryslerSafety):
  def _set_lateral_engage_hooks(self, enabled):
    param = int(ChryslerSafetyFlags.LATERAL_ENGAGE) if enabled else 0
    self.safety.set_safety_hooks(CarParams.SafetyModel.chrysler, param)

  def _set_lat_engage_hooks(self):
    """The brand base's own setup: install with the permission on, then let init run."""
    self._set_lateral_engage_hooks(True)
    self.safety.init_tests()

  def _steer_tx_msg(self):
    """A steering request that is only legal while steering is permitted. Seeded so the rate and
    measured-torque checks pass, leaving the permit as the only variable."""
    self.safety.set_torque_meas(3, 3)
    self._set_prev_torque(3)
    return self._torque_cmd_msg(3)

  def _accel_tx_msg(self):
    """ACCEL_RELATED_2FC: the DASM's acceleration message. This mode has no longitudinal command at
    all — its ACC is the DASM's — and the permission must not open this one."""
    return self.packer.make_can_msg_safety("ACCEL_RELATED_2FC", 0, {"ACCEL_2FC": 1})


class TestChryslerLateralEngage(lateral_engage_common.LateralEngageSafetyTest, TestChryslerLateralEngageBase):
  LATERAL_ENGAGE_ARM = "host"

  def setUp(self):
    super().setUp()
    self._set_lat_engage_hooks()


if __name__ == "__main__":
  unittest.main()
