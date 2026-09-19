#!/usr/bin/env python3
import unittest

from opendbc.car.nissan.values import NissanSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
import opendbc.safety.tests.lateral_engage_common as lateral_engage_common  # moonpilot: by module, never by name -- see AGENTS.md


class TestNissanSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):

  TX_MSGS = [[0x169, 0], [0x2b1, 0], [0x4cc, 0], [0x20b, 2], [0x280, 2]]
  GAS_PRESSED_THRESHOLD = 3
  RELAY_MALFUNCTION_ADDRS = {0: (0x169, 0x2b1, 0x4cc), 2: (0x280,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x280], 2: [0x169, 0x2b1, 0x4cc]}

  EPS_BUS = 0
  CRUISE_BUS = 2

  # Angle control limits
  STEER_ANGLE_MAX = 600  # deg, reasonable limit
  DEG_TO_CAN = 100

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [5., .8, .15]  # windup limit
  ANGLE_RATE_DOWN = [5., 3.5, .4]  # unwind limit

  def setUp(self):
    self.packer = CANPackerSafety("nissan_x_trail_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.nissan, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"DESIRED_ANGLE": angle, "LKA_ACTIVE": 1 if enabled else 0}
    return self.packer.make_can_msg_safety("LKAS", 0, values)

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE": angle}
    return self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_ENABLED": enable}
    return self.packer.make_can_msg_safety("CRUISE_STATE", self.CRUISE_BUS, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_%s" % s: speed * 3.6 for s in ["RR", "RL"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS_REAR", self.EPS_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"USER_BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("DOORS_LIGHTS", self.EPS_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_safety("GAS_PEDAL", self.EPS_BUS, values)

  def _acc_button_cmd(self, cancel=0, propilot=0, flw_dist=0, _set=0, res=0):
    no_button = not any([cancel, propilot, flw_dist, _set, res])
    values = {"CANCEL_BUTTON": cancel, "PROPILOT_BUTTON": propilot,
              "FOLLOW_DISTANCE_BUTTON": flw_dist, "SET_BUTTON": _set,
              "RES_BUTTON": res, "NO_BUTTON_PRESSED": no_button}
    return self.packer.make_can_msg_safety("CRUISE_THROTTLE", 2, values)

  def test_acc_buttons(self):
    btns = [
      ("cancel", True),
      ("propilot", False),
      ("flw_dist", False),
      ("_set", False),
      ("res", False),
      (None, False),
    ]
    for controls_allowed in (True, False):
      for btn, should_tx in btns:
        self.safety.set_controls_allowed(controls_allowed)
        args = {} if btn is None else {btn: 1}
        tx = self._tx(self._acc_button_cmd(**args))
        self.assertEqual(tx, should_tx)


class TestNissanSafetyAltEpsBus(TestNissanSafety):
  """Altima uses different buses"""

  EPS_BUS = 1
  CRUISE_BUS = 1

  def setUp(self):
    self.packer = CANPackerSafety("nissan_x_trail_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.nissan, NissanSafetyFlags.ALT_EPS_BUS)
    self.safety.init_tests()


class TestNissanLeafSafety(TestNissanSafety):

  def setUp(self):
    self.packer = CANPackerSafety("nissan_leaf_2018_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.nissan, 0)
    self.safety.init_tests()

  def _user_brake_msg(self, brake):
    values = {"USER_BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("CRUISE_THROTTLE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_safety("CRUISE_THROTTLE", 0, values)

  # TODO: leaf should use its own safety param
  def test_acc_buttons(self):
    pass


# moonpilot: lateral engagement. Stock ACC only -- nissan's mode enters controls on CRUISE_STATE's
# rising edge and sends the car no longitudinal command -- and host-armed: that message is the ACC
# state, not the cruise main switch, so openpilot's heartbeat is the arm.
class TestNissanLateralEngageBase(TestNissanSafety):
  def _steer_tx_msg(self):
    """A steering message that is only legal while steering is permitted: the angle command's
    LKA_ACTIVE bit is the request, and the angle itself stays at the current one."""
    self._reset_angle_measurement(0)
    self._set_prev_desired_angle(0)
    return self._angle_cmd_msg(0, True)

  def _accel_tx_msg(self):
    """A resume press: the car's own ACC buttons are the only longitudinal command nissan's tx set
    carries, and upstream allows cancel alone."""
    return self._acc_button_cmd(res=1)

  def _set_lateral_engage_hooks(self, enabled):
    flags = int(NissanSafetyFlags.LATERAL_ENGAGE) if enabled else 0
    self.safety.set_safety_hooks(CarParams.SafetyModel.nissan, flags)

  def _set_lat_engage_hooks(self):
    """The brand base's own setup: install with the permission on, then let init run."""
    self._set_lateral_engage_hooks(True)
    self.safety.init_tests()


class TestNissanLateralEngage(lateral_engage_common.LateralEngageSafetyTest, TestNissanLateralEngageBase):
  LATERAL_ENGAGE_ARM = "host"

  def setUp(self):
    super().setUp()
    self._set_lat_engage_hooks()


if __name__ == "__main__":
  unittest.main()
