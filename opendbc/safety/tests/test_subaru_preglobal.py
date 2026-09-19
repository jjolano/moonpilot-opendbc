#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
import opendbc.safety.tests.lateral_engage_common as lateral_engage_common  # moonpilot: by module, never by name — see AGENTS.md


class TestSubaruPreglobalSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  FLAGS = 0
  DBC = "subaru_outback_2015_generated"
  TX_MSGS = [[0x161, 0], [0x164, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x164, 0x161)}
  FWD_BLACKLISTED_ADDRS = {2: [0x161, 0x164]}

  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 75
  DRIVER_TORQUE_FACTOR = 10

  def setUp(self):
    self.packer = CANPackerSafety(self.DBC)
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaruPreglobal, self.FLAGS)
    self.safety.init_tests()

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    # subaru safety doesn't use the scaled value, so undo the scaling
    values = {s: speed*0.0592 for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal": brake}
    return self.packer.make_can_msg_safety("Brake_Pedal", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Command": torque, "LKAS_Active": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", 0, values)


class TestSubaruPreglobalReversedDriverTorqueSafety(TestSubaruPreglobalSafety):
  FLAGS = SubaruSafetyFlags.PREGLOBAL_REVERSED_DRIVER_TORQUE
  DBC = "subaru_outback_2019_generated"


# moonpilot: lateral engagement, pre-global. Host-armed: the mode never sets `acc_main_on`
# (CruiseControl carries cruise engaged, which is not the main switch), so the arm is openpilot's own
# engaged heartbeat — see lateral_engage_common.
class TestSubaruPreglobalLateralEngageBase(TestSubaruPreglobalSafety):
  def _set_lateral_engage_hooks(self, enabled):
    flags = int(self.FLAGS) | (int(SubaruSafetyFlags.LATERAL_ENGAGE) if enabled else 0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaruPreglobal, flags)

  def _set_lat_engage_hooks(self):
    """The brand base's own setup: install with the permission on, then let init run."""
    self._set_lateral_engage_hooks(True)
    self.safety.init_tests()

  def _steer_tx_msg(self):
    """A steering request that is only legal while steering is permitted. The driver torque sample
    is seeded at zero so the driver limit is the full max torque, leaving the permit as the only
    variable."""
    self.safety.set_torque_driver(0, 0)
    self._set_prev_torque(50)
    return self._torque_cmd_msg(50)

  def _accel_tx_msg(self):
    """The car's own throttle message. The one acceleration-adjacent message this mode may send,
    ES_Distance, carries the car-follow distance alone and upstream gates neither permission on it,
    so a message the permission *can* open is the only one that can make the claim here."""
    return self._user_gas_msg(1)


class TestSubaruPreglobalLateralEngage(lateral_engage_common.LateralEngageSafetyTest, TestSubaruPreglobalLateralEngageBase):
  LATERAL_ENGAGE_ARM = "host"

  def setUp(self):
    super().setUp()
    self._set_lat_engage_hooks()


if __name__ == "__main__":
  unittest.main()
